/*-
 *   BSD LICENSE
 *
 *   Copyright (c) Intel Corporation.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "spdk/stdinc.h"

#include "spdk/bdev.h"
#include "spdk/conf.h"

#include "spdk/config.h"
#include "spdk/env.h"
#include "spdk/event.h"
#include "spdk/thread.h"
#include "spdk/likely.h"
#include "spdk/queue.h"
#include "spdk/nvme_spec.h"
#include "spdk/scsi_spec.h"
#include "spdk/notify.h"
#include "spdk/util.h"
#include "spdk/trace.h"

#include "spdk/bdev_module.h"
#include "spdk_internal/log.h"
#include "spdk/string.h"

#ifdef SPDK_CONFIG_VTUNE
#include "ittnotify.h"
#include "ittnotify_types.h"
int __itt_init_ittlib(const char *, __itt_group_id);
#endif

#define SPDK_BDEV_IO_POOL_SIZE			(64 * 1024 - 1)
#define SPDK_BDEV_IO_CACHE_SIZE			256
#define BUF_SMALL_POOL_SIZE			8191
#define BUF_LARGE_POOL_SIZE			1023
#define NOMEM_THRESHOLD_COUNT			8
#define ZERO_BUFFER_SIZE			0x100000

#define OWNER_BDEV		0x2

#define OBJECT_BDEV_IO		0x2

#define TRACE_GROUP_BDEV	0x3
#define TRACE_BDEV_IO_START	SPDK_TPOINT_ID(TRACE_GROUP_BDEV, 0x0)
#define TRACE_BDEV_IO_DONE	SPDK_TPOINT_ID(TRACE_GROUP_BDEV, 0x1)

// zhou: QoS slice is 1ms
#define SPDK_BDEV_QOS_TIMESLICE_IN_USEC		1000

#define SPDK_BDEV_QOS_MIN_IO_PER_TIMESLICE	1
#define SPDK_BDEV_QOS_MIN_BYTE_PER_TIMESLICE	512

// zhou: limit set should be multiply of these value.
#define SPDK_BDEV_QOS_MIN_IOS_PER_SEC		10000
#define SPDK_BDEV_QOS_MIN_BYTES_PER_SEC		(10 * 1024 * 1024)

// zhou: when used in RPC and Config file,
//           1. NOT defined, means not provided by client;
//           2. Zero, means unlimited
//       when used in "internal.qos",
//           1. NOT defined, means unlimited.
//           2. Zero is impossible.
#define SPDK_BDEV_QOS_LIMIT_NOT_DEFINED		UINT64_MAX

#define SPDK_BDEV_POOL_ALIGNMENT 512

// zhou: Limit IOPS, Throughput, Read Thp, Write Thp.
static const char *qos_conf_type[] = {"Limit_IOPS",
				      "Limit_BPS", "Limit_Read_BPS", "Limit_Write_BPS"
				     };
static const char *qos_rpc_type[] = {"rw_ios_per_sec",
				     "rw_mbytes_per_sec", "r_mbytes_per_sec", "w_mbytes_per_sec"
				    };

TAILQ_HEAD(spdk_bdev_list, spdk_bdev);

// zhou:
struct spdk_bdev_mgr {
    // zhou: "struct spdk_bdev_io" resource shared between threads.
    //       Only can be used when per thread cache exhausted.
	struct spdk_mempool *bdev_io_pool;

    // zhou: general purpose for small and large object, to hold READ/WRITE buffer?
	struct spdk_mempool *buf_small_pool;
	struct spdk_mempool *buf_large_pool;

    // zhou: 1 MB memory
	void *zero_buffer;

    // zhou: manage all types of underlying device.
	TAILQ_HEAD(bdev_module_list, spdk_bdev_module) bdev_modules;

    // zhou: no mattach which type, here just a list of underlying device disks.
    //       Pay attention, may be more than one disk of one type underlying device.
	struct spdk_bdev_list bdevs;

    // zhou: bdev init completed.
	bool init_complete;
    // zhou: all types of underlying device module init completed
	bool module_init_complete;

#ifdef SPDK_CONFIG_VTUNE
	__itt_domain	*domain;
#endif
};

// zhou: IO device for lib bdev itself. The only place to spdk_get_io_channel()
//       get its channel, is when creating underlying disk's I/O channel
//       spdk_bdev_channel_create().
static struct spdk_bdev_mgr g_bdev_mgr = {
	.bdev_modules = TAILQ_HEAD_INITIALIZER(g_bdev_mgr.bdev_modules),
	.bdevs = TAILQ_HEAD_INITIALIZER(g_bdev_mgr.bdevs),
	.init_complete = false,
	.module_init_complete = false,
};

static struct spdk_bdev_opts	g_bdev_opts = {
    // zhou: by default, 64*1024 - 1
	.bdev_io_pool_size = SPDK_BDEV_IO_POOL_SIZE,
    // zhou: 256
	.bdev_io_cache_size = SPDK_BDEV_IO_CACHE_SIZE,
};

static spdk_bdev_init_cb	g_init_cb_fn = NULL;
static void			*g_init_cb_arg = NULL;

static spdk_bdev_fini_cb	g_fini_cb_fn = NULL;
static void			*g_fini_cb_arg = NULL;
static struct spdk_thread	*g_fini_thread = NULL;

struct spdk_bdev_qos_limit {
	/** IOs or bytes allowed per second (i.e., 1s). */
	uint64_t limit;

	/** Remaining IOs or bytes allowed in current timeslice (e.g., 1ms).
	 *  For remaining bytes, allowed to run negative if an I/O is submitted when
	 *  some bytes are remaining, but the I/O is bigger than that amount. The
	 *  excess will be deducted from the next timeslice.
	 */
	int64_t remaining_this_timeslice;

	/** Minimum allowed IOs or bytes to be issued in one timeslice (e.g., 1ms). */
	uint32_t min_per_timeslice;

	/** Maximum allowed IOs or bytes to be issued in one timeslice (e.g., 1ms). */
	uint32_t max_per_timeslice;

    // zhou:
	/** Function to check whether to queue the IO. */
	bool (*queue_io)(const struct spdk_bdev_qos_limit *limit, struct spdk_bdev_io *io);

	/** Function to update for the submitted IO. */
	void (*update_quota)(struct spdk_bdev_qos_limit *limit, struct spdk_bdev_io *io);
};

// zhou:
struct spdk_bdev_qos {
	/** Types of structure of rate limits. */
	struct spdk_bdev_qos_limit rate_limits[SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES];

    // zhou: all IO will go one I/O channel.
	/** The channel that all I/O are funneled through. */
	struct spdk_bdev_channel *ch;

	/** The thread on which the poller is running. */
	struct spdk_thread *thread;

	/** Queue of I/O waiting to be issued. */
	bdev_io_tailq_t queued;

    // zhou: time slice in ticks.
	/** Size of a timeslice in tsc ticks. */
	uint64_t timeslice_size;

	/** Timestamp of start of last timeslice. */
	uint64_t last_timeslice;

	/** Poller that processes queued I/O commands each time slice. */
	struct spdk_poller *poller;
};

// zhou: I/O channel for I/O device "g_bdev_mgr", which used to manage shared resouce
//       between all bdev within each thread.
//       The shared resouces includes buffer for READ/WRITE
//       "struct spdk_bdev_io".
struct spdk_bdev_mgmt_channel {

    // zhou: waiting list for such resource, used when pool is empty.
	bdev_io_stailq_t need_buf_small;
	bdev_io_stailq_t need_buf_large;

    // zhou: list of "struct spdk_bdev_io"
	/*
	 * Each thread keeps a cache of bdev_io - this allows
	 *  bdev threads which are *not* DPDK threads to still
	 *  benefit from a per-thread bdev_io cache.  Without
	 *  this, non-DPDK threads fetching from the mempool
	 *  incur a cmpxchg on get and put.
	 */
	bdev_io_stailq_t per_thread_cache;

    // zhou: the number of "struct spdk_bdev_io" in list "per_thread_cache"
	uint32_t	per_thread_cache_count;
    // zhou: MAX number of list "per_thread_cache".
	uint32_t	bdev_io_cache_size;

    // zhou: each type of underlying controller's I/O device represent with a
    //       "struct spdk_bdev_shared_resource". All I/O channel binding with
    //       this thread will link with lib bdev I/O channel of this thread.
    //
    //       Then more than one underlying disk I/O device over one underlying device,
    //       can be identified in this list.
	TAILQ_HEAD(, spdk_bdev_shared_resource)	shared_resources;

    // zhou: wait queue due to no memory resource of this thread.
    //       Not all spdk_bdev_io alloc failure will register here. When client invoke
    //       READ/WRITE api, failured will be returned to client. Client may decide to
    //       register this queue or abort.
    //
    //       Refer to spdk_bdev_queue_io_wait() description:
    //       "Add an entry into the calling thread's queue to be notified when an
    //       spdk_bdev_io becomes available.
    //       When one of the @ref bdev_io_submit_functions returns -ENOMEM, it means
    //       the spdk_bdev_io buffer pool has no available buffers. This function may
    //       be called to register a callback to be notified when a buffer becomes
    //       available on the calling thread.
    //       The callback function will always be called on the same thread as this
    //       function was called."
	TAILQ_HEAD(, spdk_bdev_io_wait_entry)	io_wait_queue;
};

// zhou: in case of more than device (struct spdk_bdev) based on underlying disk
//       (corresponding a I/O device).
//       In this case, Block Device Layer understands these bdev share same
//       underlying disk, so manage these bdev with same resource.
//       By this method, give underlying module a way to utilize bdev framework
//       to expose one underlying disk to more than one client (each with a bdev).

/*
 * Per-module (or per-io_device) data. Multiple bdevs built on the same io_device
 * will queue here their IO that awaits retry. It makes it possible to retry sending
 * IO to one bdev after IO from other bdev completes.
 */
struct spdk_bdev_shared_resource {
	/* The bdev management channel */
	struct spdk_bdev_mgmt_channel *mgmt_ch;

	/*
	 * Count of I/O submitted to bdev module and waiting for completion.
	 * Incremented before submit_request() is called on an spdk_bdev_io.
	 */
	uint64_t		io_outstanding;

    // zhou: queued IO due to underlying disk completed with no memory.
    //       We should wait until some I/O compeleted success.
    //
    //       If retry pending I/O, should wait for outstanding I/O number less than
    //       "nomem_threshold".
	/*
	 * Queue of IO awaiting retry because of a previous NOMEM status returned
	 *  on this channel.
	 */
	bdev_io_tailq_t		nomem_io;

	/*
	 * Threshold which io_outstanding must drop to before retrying nomem_io.
	 */
	uint64_t		nomem_threshold;

    // zhou: underlying disk's I/O channel, created by "fn_table->get_io_channel()".
	/* I/O channel allocated by a bdev module */
	struct spdk_io_channel	*shared_ch;

	/* Refcount of bdev channels using this resource */
	uint32_t		ref;

    // zhou: link to "struct spdk_bdev_mgmt_channel.shared_resources"
	TAILQ_ENTRY(spdk_bdev_shared_resource) link;
};

#define BDEV_CH_RESET_IN_PROGRESS	(1 << 0)
#define BDEV_CH_QOS_ENABLED		(1 << 1)

// zhou: bdev I/O channel private workspace, for each backing disk.
struct spdk_bdev_channel {

	struct spdk_bdev	*bdev;

    // zhou: underlying disk's I/O channel, created by "fn_table->get_io_channel()"
	/* The channel for the underlying device */
	struct spdk_io_channel	*channel;

    // zhou: each bdev I/O channel will manage via "shared_resource" which linked in
    //       "g_bdev_mgr" I/O channel "struct spdk_bdev_mgmt_channel.shared_resources".
	/* Per io_device per thread data */
	struct spdk_bdev_shared_resource *shared_resource;

	struct spdk_bdev_io_stat stat;

    // zhou: only for this bdev I/O channel, will be accounted in
    //       "shared_resource.io_outstanding" at the same time.
	/*
	 * Count of I/O submitted through this channel and waiting for completion.
	 * Incremented before submit_request() is called on an spdk_bdev_io.
	 */
	uint64_t		io_outstanding;

    // zhou: RESET will be treated as a special IO.
	bdev_io_tailq_t		queued_resets;

    // zhou: BDEV_CH_QOS_ENABLED and BDEV_CH_RESET_IN_PROGRESS. Used to indicate the IO operation
    //       on this channel should go special path.
    //       In normal case, all IO handled by this channel will not care about bdev global state.
	uint32_t		flags;

	struct spdk_histogram_data *histogram;

#ifdef SPDK_CONFIG_VTUNE
	uint64_t		start_tsc;
	uint64_t		interval_tsc;
	__itt_string_handle	*handle;
	struct spdk_bdev_io_stat prev_stat;
#endif

};

// zhou: represents a handler to a given block device. Not too much information.
//       "Descriptors are used to establish and track permissions to use the underlying
//       block device, much like a file descriptor on UNIX systems."
struct spdk_bdev_desc {
    // zhou: refer to backend storage disk.
	struct spdk_bdev		*bdev;

	struct spdk_thread		*thread;

    // zhou: callback to handle backing device removed, passed by client
	spdk_bdev_remove_cb_t		remove_cb;
	void				*remove_ctx;

	bool				remove_scheduled;
	bool				closed;
	bool				write;

	TAILQ_ENTRY(spdk_bdev_desc)	link;
};

struct spdk_bdev_iostat_ctx {
	struct spdk_bdev_io_stat *stat;
	spdk_bdev_get_device_stat_cb cb;
	void *cb_arg;
};

struct set_qos_limit_ctx {
	void (*cb_fn)(void *cb_arg, int status);
	void *cb_arg;
	struct spdk_bdev *bdev;
};

// zhou: in case of nvme, "struct nvme_bdev{}" which first element is "struct spdk_bdev	disk"
//       Previous struct will be passed to spdk_io_device_register() as, and later is same.
//       So although two memory address will be passed to create IO device, but they are
//       accidently share the same memery address.
//
//       So, by this way, the IO device passed to lib bdev in creating backing device, will
//       minus 1 to differentiate IO device when link to thread.
#define __bdev_to_io_dev(bdev)		(((char *)bdev) + 1)
#define __bdev_from_io_dev(io_dev)	((struct spdk_bdev *)(((char *)io_dev) - 1))

static void _spdk_bdev_write_zero_buffer_done(struct spdk_bdev_io *bdev_io, bool success,
		void *cb_arg);
static void _spdk_bdev_write_zero_buffer_next(void *_bdev_io);

static void _spdk_bdev_enable_qos_msg(struct spdk_io_channel_iter *i);
static void _spdk_bdev_enable_qos_done(struct spdk_io_channel_iter *i, int status);

void
spdk_bdev_get_opts(struct spdk_bdev_opts *opts)
{
	*opts = g_bdev_opts;
}

// zhou: make sure each thread cache size will no more than pool size.
int
spdk_bdev_set_opts(struct spdk_bdev_opts *opts)
{
	uint32_t min_pool_size;

	/*
	 * Add 1 to the thread count to account for the extra mgmt_ch that gets created during subsystem
	 *  initialization.  A second mgmt_ch will be created on the same thread when the application starts
	 *  but before the deferred put_io_channel event is executed for the first mgmt_ch.
	 */
	min_pool_size = opts->bdev_io_cache_size * (spdk_thread_get_count() + 1);

	if (opts->bdev_io_pool_size < min_pool_size) {

		SPDK_ERRLOG("bdev_io_pool_size %" PRIu32 " is not compatible with bdev_io_cache_size %" PRIu32
			    " and %" PRIu32 " threads\n", opts->bdev_io_pool_size, opts->bdev_io_cache_size,
			    spdk_thread_get_count());
		SPDK_ERRLOG("bdev_io_pool_size must be at least %" PRIu32 "\n", min_pool_size);
		return -1;
	}

	g_bdev_opts = *opts;

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// zhou: START of finding backing storage

struct spdk_bdev *
spdk_bdev_first(void)
{
	struct spdk_bdev *bdev;

	bdev = TAILQ_FIRST(&g_bdev_mgr.bdevs);
	if (bdev) {
		SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Starting bdev iteration at %s\n", bdev->name);
	}

	return bdev;
}

struct spdk_bdev *
spdk_bdev_next(struct spdk_bdev *prev)
{
	struct spdk_bdev *bdev;

	bdev = TAILQ_NEXT(prev, internal.link);
	if (bdev) {
		SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Continuing bdev iteration at %s\n", bdev->name);
	}

	return bdev;
}

static struct spdk_bdev *
_bdev_next_leaf(struct spdk_bdev *bdev)
{
	while (bdev != NULL) {
		if (bdev->internal.claim_module == NULL) {
			return bdev;
		} else {
			bdev = TAILQ_NEXT(bdev, internal.link);
		}
	}

	return bdev;
}

struct spdk_bdev *
spdk_bdev_first_leaf(void)
{
	struct spdk_bdev *bdev;

	bdev = _bdev_next_leaf(TAILQ_FIRST(&g_bdev_mgr.bdevs));

	if (bdev) {
		SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Starting bdev iteration at %s\n", bdev->name);
	}

	return bdev;
}

struct spdk_bdev *
spdk_bdev_next_leaf(struct spdk_bdev *prev)
{
	struct spdk_bdev *bdev;

	bdev = _bdev_next_leaf(TAILQ_NEXT(prev, internal.link));

	if (bdev) {
		SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Continuing bdev iteration at %s\n", bdev->name);
	}

	return bdev;
}

// zhou: search already created disk by name from "g_bdev_mgr.bdevs"
struct spdk_bdev *
spdk_bdev_get_by_name(const char *bdev_name)
{
	struct spdk_bdev_alias *tmp;
	struct spdk_bdev *bdev = spdk_bdev_first();

	while (bdev != NULL) {
		if (strcmp(bdev_name, bdev->name) == 0) {
			return bdev;
		}

		TAILQ_FOREACH(tmp, &bdev->aliases, tailq) {
			if (strcmp(bdev_name, tmp->alias) == 0) {
				return bdev;
			}
		}

		bdev = spdk_bdev_next(bdev);
	}

	return NULL;
}

// zhou: END of finding backing storage
////////////////////////////////////////////////////////////////////////////////
// zhou: START of IO Request's buffer operations.

// zhou: set allocated single buffer to "bdev_io->u.bdev.iocnt".
void
spdk_bdev_io_set_buf(struct spdk_bdev_io *bdev_io, void *buf, size_t len)
{
	struct iovec *iovs;

	if (bdev_io->u.bdev.iovs == NULL) {
		bdev_io->u.bdev.iovs = &bdev_io->iov;
		bdev_io->u.bdev.iovcnt = 1;
	}

	iovs = bdev_io->u.bdev.iovs;

	assert(iovs != NULL);
	assert(bdev_io->u.bdev.iovcnt >= 1);

	iovs[0].iov_base = buf;
	iovs[0].iov_len = len;
}

static bool
_is_buf_allocated(struct iovec *iovs)
{
	if (iovs == NULL) {
		return false;
	}

	return iovs[0].iov_base != NULL;
}

// zhou: all buffer in vector, align with "1 << struct spdk_bdev.required_alignment".
static bool
_are_iovs_aligned(struct iovec *iovs, int iovcnt, uint32_t alignment)
{
	int i;
	uintptr_t iov_base;

	if (spdk_likely(alignment == 1)) {
		return true;
	}

	for (i = 0; i < iovcnt; i++) {
		iov_base = (uintptr_t)iovs[i].iov_base;
		if ((iov_base & (alignment - 1)) != 0) {
			return false;
		}
	}

	return true;
}

static void
_copy_iovs_to_buf(void *buf, size_t buf_len, struct iovec *iovs, int iovcnt)
{
	int i;
	size_t len;

	for (i = 0; i < iovcnt; i++) {
		len = spdk_min(iovs[i].iov_len, buf_len);
		memcpy(buf, iovs[i].iov_base, len);
		buf += len;
		buf_len -= len;
	}
}

static void
_copy_buf_to_iovs(struct iovec *iovs, int iovcnt, void *buf, size_t buf_len)
{
	int i;
	size_t len;

	for (i = 0; i < iovcnt; i++) {
		len = spdk_min(iovs[i].iov_len, buf_len);
		memcpy(iovs[i].iov_base, buf, len);
		buf += len;
		buf_len -= len;
	}
}

// zhou: In case buffer already provided by client, but it not meet the Memory
//       Requirement. Then preserve original iovec, and use a allocated buffer
//       as "bounce buf".
//       In case of WRITE, have to copy content to bounce buf;
//       In case of READ, use it directly.
static void
_bdev_io_set_bounce_buf(struct spdk_bdev_io *bdev_io, void *buf, size_t len)
{
	/* save original iovec */
	bdev_io->internal.orig_iovs = bdev_io->u.bdev.iovs;
	bdev_io->internal.orig_iovcnt = bdev_io->u.bdev.iovcnt;

	/* set bounce iov */
	bdev_io->u.bdev.iovs = &bdev_io->internal.bounce_iov;
	bdev_io->u.bdev.iovcnt = 1;

	/* set bounce buffer for this operation */
	bdev_io->u.bdev.iovs[0].iov_base = buf;
	bdev_io->u.bdev.iovs[0].iov_len = len;

	/* if this is write path, copy data from original buffer to bounce buffer */
	if (bdev_io->type == SPDK_BDEV_IO_TYPE_WRITE) {
        // zhou: copy IOV to a single buffer.
		_copy_iovs_to_buf(buf, len, bdev_io->internal.orig_iovs, bdev_io->internal.orig_iovcnt);
	}
}

// zhou: release object to pool, or assign it to who in waiting list.
static void
spdk_bdev_io_put_buf(struct spdk_bdev_io *bdev_io)
{
	struct spdk_mempool *pool;
	struct spdk_bdev_io *tmp;
	void *buf, *aligned_buf;
	bdev_io_stailq_t *stailq;
	struct spdk_bdev_mgmt_channel *ch;
	uint64_t buf_len;
	uint64_t alignment;
	bool buf_allocated;

	buf = bdev_io->internal.buf;
	buf_len = bdev_io->internal.buf_len;

	alignment = spdk_bdev_get_buf_align(bdev_io->bdev);
	ch = bdev_io->internal.ch->shared_resource->mgmt_ch;

	bdev_io->internal.buf = NULL;

    // zhou: found out which pool the "bdev_io->internal.buf" belongs to.
	if (buf_len + alignment <= SPDK_BDEV_BUF_SIZE_WITH_MD(SPDK_BDEV_SMALL_BUF_MAX_SIZE) +
	    SPDK_BDEV_POOL_ALIGNMENT) {
		pool = g_bdev_mgr.buf_small_pool;
		stailq = &ch->need_buf_small;
	} else {
		pool = g_bdev_mgr.buf_large_pool;
		stailq = &ch->need_buf_large;
	}

    // zhou: once there is someone in waiting list, reuse it immediately.
	if (STAILQ_EMPTY(stailq)) {
		spdk_mempool_put(pool, buf);
	} else {
		tmp = STAILQ_FIRST(stailq);

		alignment = spdk_bdev_get_buf_align(tmp->bdev);
		buf_allocated = _is_buf_allocated(tmp->u.bdev.iovs);

		aligned_buf = (void *)(((uintptr_t)buf +
					(alignment - 1)) & ~(alignment - 1));
		if (buf_allocated) {
			_bdev_io_set_bounce_buf(tmp, aligned_buf, tmp->internal.buf_len);
		} else {
			spdk_bdev_io_set_buf(tmp, aligned_buf, tmp->internal.buf_len);
		}

		STAILQ_REMOVE_HEAD(stailq, internal.buf_link);

		tmp->internal.buf = buf;

        // zhou: notify that its buffer allocation completion.
		tmp->internal.get_buf_cb(tmp->internal.ch->channel, tmp, true);
	}
}

// zhou: In case of READ, need copy data from bounce memory to client's memory, when
//       IO completed success.
static void
_bdev_io_unset_bounce_buf(struct spdk_bdev_io *bdev_io)
{
    // zhou: WRITE, copy will be performed when submit IO.
    //       READ, copy will be performed when IO completed

	/* if this is read path, copy data from bounce buffer to original buffer */
	if (bdev_io->type == SPDK_BDEV_IO_TYPE_READ &&
	    bdev_io->internal.status == SPDK_BDEV_IO_STATUS_SUCCESS) {

		_copy_buf_to_iovs(bdev_io->internal.orig_iovs, bdev_io->internal.orig_iovcnt,
				  bdev_io->internal.bounce_iov.iov_base, bdev_io->internal.bounce_iov.iov_len);

	}

	/* set orignal buffer for this io */
	bdev_io->u.bdev.iovcnt = bdev_io->internal.orig_iovcnt;
	bdev_io->u.bdev.iovs = bdev_io->internal.orig_iovs;
	/* disable bouncing buffer for this io */
	bdev_io->internal.orig_iovcnt = 0;
	bdev_io->internal.orig_iovs = NULL;

	/* return bounce buffer to the pool */
	spdk_bdev_io_put_buf(bdev_io);
}

// zhou: fetch buffer before submit IO.
//       "cb" in parameters, will be invoked to notify driver.
void
spdk_bdev_io_get_buf(struct spdk_bdev_io *bdev_io, spdk_bdev_io_get_buf_cb cb, uint64_t len)
{
	struct spdk_mempool *pool;
	bdev_io_stailq_t *stailq;
	void *buf, *aligned_buf;
	struct spdk_bdev_mgmt_channel *mgmt_ch;
	uint64_t alignment;
	bool buf_allocated;

	assert(cb != NULL);

    // zhou: Memory Requirement "struct spdk_bdev.required_alignment".
	alignment = spdk_bdev_get_buf_align(bdev_io->bdev);

    // zhou: "iovs[0].iov_base != NULL", buffer provided by client when it submits
    //       I/O using,  e.g. spdk_bdev_read(), spdk_bdev_read_blocks(),
    //       spdk_bdev_write(), spdk_bdev_write_blocks().
	buf_allocated = _is_buf_allocated(bdev_io->u.bdev.iovs);

    // zhou: buffer exist already align with memory requirement.
	if (buf_allocated &&
	    _are_iovs_aligned(bdev_io->u.bdev.iovs, bdev_io->u.bdev.iovcnt, alignment)) {

		/* Buffer already present and aligned */
		cb(bdev_io->internal.ch->channel, bdev_io, true);
		return;
	}


    // zhou: > 64 KB, which is the object size in "buf_large_pool".
    //       Why "+ alignment" ? Because the object in pool may not meet the Memory
    //       Requirment. We may need add offset to meet it.
	if (len + alignment > SPDK_BDEV_BUF_SIZE_WITH_MD(SPDK_BDEV_LARGE_BUF_MAX_SIZE) +
	    SPDK_BDEV_POOL_ALIGNMENT) {

		SPDK_ERRLOG("Length + alignment %" PRIu64 " is larger than allowed\n",
			    len + alignment);
		cb(bdev_io->internal.ch->channel, bdev_io, false);
		return;
	}

    // zhou: start to allocate READ/WRITE buffer.

	mgmt_ch = bdev_io->internal.ch->shared_resource->mgmt_ch;

    // zhou: due to buffer allocation may be not completed right now. This will be
    //       used in async callback.
	bdev_io->internal.buf_len = len;
	bdev_io->internal.get_buf_cb = cb;

    // zhou: <= 8 KB
	if (len + alignment <= SPDK_BDEV_BUF_SIZE_WITH_MD(SPDK_BDEV_SMALL_BUF_MAX_SIZE) +
	    SPDK_BDEV_POOL_ALIGNMENT) {
		pool = g_bdev_mgr.buf_small_pool;
		stailq = &mgmt_ch->need_buf_small;
	} else {
		pool = g_bdev_mgr.buf_large_pool;
		stailq = &mgmt_ch->need_buf_large;
	}

	buf = spdk_mempool_get(pool);

	if (!buf) {
        // zhou: pool is out of resource.
		STAILQ_INSERT_TAIL(stailq, bdev_io, internal.buf_link);

	} else {
        // zhou: adjust start of buffer according Memory Requirement.
		aligned_buf = (void *)(((uintptr_t)buf + (alignment - 1)) & ~(alignment - 1));

        // zhou: already allocate buffer, but not be used, since allocate again.
		if (buf_allocated) {
            // zhou: move data from client allocated buffer to our alignment buffer.
			_bdev_io_set_bounce_buf(bdev_io, aligned_buf, len);
		} else {
			spdk_bdev_io_set_buf(bdev_io, aligned_buf, len);
		}

        // zhou: don't forget buffer room address.
		bdev_io->internal.buf = buf;

        // zhou: just function parameter "spdk_bdev_io_get_buf_cb cb"
		bdev_io->internal.get_buf_cb(bdev_io->internal.ch->channel, bdev_io, true);
	}
}


// zhou: END of IO Request's buffer operations.
////////////////////////////////////////////////////////////////////////////////
// zhou: START of subsystem related operations, includes ".init", ".fini", ".config", .write_config_json"

// zhou: although the backing storage will be installed by RPC, but the adapter layer
//       of each supporting backing storage will be init with library bdev.
//
//       Each "struct spdk_bdev_io" will allocated with underlying disk private work space.
//       For this function, will find out enough room to hold private space size of all
//       supporting bakcing storage.
//       Then, we can allocate reserve enough space when create mempool for
//       "struct spdk_bdev_io"
static int
spdk_bdev_module_get_max_ctx_size(void)
{
	struct spdk_bdev_module *bdev_module;
	int max_bdev_module_size = 0;

	TAILQ_FOREACH(bdev_module, &g_bdev_mgr.bdev_modules, internal.tailq) {

		if (bdev_module->get_ctx_size && bdev_module->get_ctx_size() > max_bdev_module_size) {
			max_bdev_module_size = bdev_module->get_ctx_size();
		}
	}

	return max_bdev_module_size;
}

// zhou: read config file.
void
spdk_bdev_config_text(FILE *fp)
{
	struct spdk_bdev_module *bdev_module;

	TAILQ_FOREACH(bdev_module, &g_bdev_mgr.bdev_modules, internal.tailq) {
		if (bdev_module->config_text) {
			bdev_module->config_text(fp);
		}
	}
}

static void
spdk_bdev_qos_config_json(struct spdk_bdev *bdev, struct spdk_json_write_ctx *w)
{
	int i;
	struct spdk_bdev_qos *qos = bdev->internal.qos;
	uint64_t limits[SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES];

	if (!qos) {
		return;
	}

	spdk_bdev_get_qos_rate_limits(bdev, limits);

	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "method", "set_bdev_qos_limit");

	spdk_json_write_named_object_begin(w, "params");
	spdk_json_write_named_string(w, "name", bdev->name);
	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
		if (limits[i] > 0) {
			spdk_json_write_named_uint64(w, qos_rpc_type[i], limits[i]);
		}
	}
	spdk_json_write_object_end(w);

	spdk_json_write_object_end(w);
}

// zhou: write current using config in json.
void
spdk_bdev_subsystem_config_json(struct spdk_json_write_ctx *w)
{
	struct spdk_bdev_module *bdev_module;
	struct spdk_bdev *bdev;

	assert(w != NULL);

	spdk_json_write_array_begin(w);

	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "method", "set_bdev_options");
	spdk_json_write_named_object_begin(w, "params");
	spdk_json_write_named_uint32(w, "bdev_io_pool_size", g_bdev_opts.bdev_io_pool_size);
	spdk_json_write_named_uint32(w, "bdev_io_cache_size", g_bdev_opts.bdev_io_cache_size);
	spdk_json_write_object_end(w);
	spdk_json_write_object_end(w);

	TAILQ_FOREACH(bdev_module, &g_bdev_mgr.bdev_modules, internal.tailq) {
		if (bdev_module->config_json) {
			bdev_module->config_json(w);
		}
	}

	TAILQ_FOREACH(bdev, &g_bdev_mgr.bdevs, internal.link) {
		if (bdev->fn_table->write_config_json) {
			bdev->fn_table->write_config_json(bdev, w);
		}

		spdk_bdev_qos_config_json(bdev, w);
	}

	spdk_json_write_array_end(w);
}

// zhou: create I/O channel for I/O device "struct spdk_bdev_mgr".
//       used when backing disk (I/O device "struct spdk_bdev") I/O channel created,
static int
spdk_bdev_mgmt_channel_create(void *io_device, void *ctx_buf)
{
	struct spdk_bdev_mgmt_channel *ch = ctx_buf;
	struct spdk_bdev_io *bdev_io;
	uint32_t i;

	STAILQ_INIT(&ch->need_buf_small);
	STAILQ_INIT(&ch->need_buf_large);

	STAILQ_INIT(&ch->per_thread_cache);

	ch->bdev_io_cache_size = g_bdev_opts.bdev_io_cache_size;

	/* Pre-populate bdev_io cache to ensure this thread cannot be starved. */
	ch->per_thread_cache_count = 0;

	for (i = 0; i < ch->bdev_io_cache_size; i++) {

		bdev_io = spdk_mempool_get(g_bdev_mgr.bdev_io_pool);
		assert(bdev_io != NULL);

		ch->per_thread_cache_count++;

		STAILQ_INSERT_HEAD(&ch->per_thread_cache, bdev_io, internal.buf_link);
	}

	TAILQ_INIT(&ch->shared_resources);
	TAILQ_INIT(&ch->io_wait_queue);

	return 0;
}

static void
spdk_bdev_mgmt_channel_destroy(void *io_device, void *ctx_buf)
{
	struct spdk_bdev_mgmt_channel *ch = ctx_buf;
	struct spdk_bdev_io *bdev_io;

	if (!STAILQ_EMPTY(&ch->need_buf_small) || !STAILQ_EMPTY(&ch->need_buf_large)) {
		SPDK_ERRLOG("Pending I/O list wasn't empty on mgmt channel free\n");
	}

	if (!TAILQ_EMPTY(&ch->shared_resources)) {
		SPDK_ERRLOG("Module channel list wasn't empty on mgmt channel free\n");
	}

	while (!STAILQ_EMPTY(&ch->per_thread_cache)) {
		bdev_io = STAILQ_FIRST(&ch->per_thread_cache);
		STAILQ_REMOVE_HEAD(&ch->per_thread_cache, internal.buf_link);
		ch->per_thread_cache_count--;
		spdk_mempool_put(g_bdev_mgr.bdev_io_pool, (void *)bdev_io);
	}

	assert(ch->per_thread_cache_count == 0);
}

static void
spdk_bdev_init_complete(int rc)
{
	spdk_bdev_init_cb cb_fn = g_init_cb_fn;
	void *cb_arg = g_init_cb_arg;
	struct spdk_bdev_module *m;

	g_bdev_mgr.init_complete = true;
	g_init_cb_fn = NULL;
	g_init_cb_arg = NULL;

	/*
	 * For modules that need to know when subsystem init is complete,
	 * inform them now.
	 */
	if (rc == 0) {
		TAILQ_FOREACH(m, &g_bdev_mgr.bdev_modules, internal.tailq) {
			if (m->init_complete) {
				m->init_complete();
			}
		}
	}

	cb_fn(cb_arg, rc);
}

static void
spdk_bdev_module_action_complete(void)
{
	struct spdk_bdev_module *m;

	/*
	 * Don't finish bdev subsystem initialization if
	 * module pre-initialization is still in progress, or
	 * the subsystem been already initialized.
	 */
	if (!g_bdev_mgr.module_init_complete || g_bdev_mgr.init_complete) {
		return;
	}

	/*
	 * Check all bdev modules for inits/examinations in progress. If any
	 * exist, return immediately since we cannot finish bdev subsystem
	 * initialization until all are completed.
	 */
	TAILQ_FOREACH(m, &g_bdev_mgr.bdev_modules, internal.tailq) {
		if (m->internal.action_in_progress > 0) {
			return;
		}
	}

	/*
	 * Modules already finished initialization - now that all
	 * the bdev modules have finished their asynchronous I/O
	 * processing, the entire bdev layer can be marked as complete.
	 */
	spdk_bdev_init_complete(0);
}

static void
spdk_bdev_module_action_done(struct spdk_bdev_module *module)
{
	assert(module->internal.action_in_progress > 0);
	module->internal.action_in_progress--;
	spdk_bdev_module_action_complete();
}

void
spdk_bdev_module_init_done(struct spdk_bdev_module *module)
{
	spdk_bdev_module_action_done(module);
}

void
spdk_bdev_module_examine_done(struct spdk_bdev_module *module)
{
	spdk_bdev_module_action_done(module);
}

/** The last initialized bdev module */
static struct spdk_bdev_module *g_resume_bdev_module = NULL;

// zhou: init backing storage adapter layer one by one.
static int
spdk_bdev_modules_init(void)
{
	struct spdk_bdev_module *module;
	int rc = 0;

	TAILQ_FOREACH(module, &g_bdev_mgr.bdev_modules, internal.tailq) {

		g_resume_bdev_module = module;

        // zhou: e.g. bdev_aio_initialize(), bdev_nvme_library_init()
		rc = module->module_init();
		if (rc != 0) {
			return rc;
		}
	}

	g_resume_bdev_module = NULL;
	return 0;
}

static void
spdk_bdev_init_failed(void *cb_arg)
{
	spdk_bdev_init_complete(-1);
}

// zhou: from an allocated thread, the bdev library may be initialized by calling
//       this function, which is an asynchronous operation. Uuntil the completion
//       callback is called, no other bdev library functions may be invoked.
//
//       "cb_fn" will invoke next subsytem's initialize.
void
spdk_bdev_initialize(spdk_bdev_init_cb cb_fn, void *cb_arg)
{
	struct spdk_conf_section *sp;
	struct spdk_bdev_opts bdev_opts;
	int32_t bdev_io_pool_size, bdev_io_cache_size;
	int cache_size;
	int rc = 0;
	char mempool_name[32];

	assert(cb_fn != NULL);

	sp = spdk_conf_find_section(NULL, "Bdev");
	if (sp != NULL) {
		spdk_bdev_get_opts(&bdev_opts);

		bdev_io_pool_size = spdk_conf_section_get_intval(sp, "BdevIoPoolSize");
		if (bdev_io_pool_size >= 0) {
			bdev_opts.bdev_io_pool_size = bdev_io_pool_size;
		}

		bdev_io_cache_size = spdk_conf_section_get_intval(sp, "BdevIoCacheSize");
		if (bdev_io_cache_size >= 0) {
			bdev_opts.bdev_io_cache_size = bdev_io_cache_size;
		}

		if (spdk_bdev_set_opts(&bdev_opts)) {
			spdk_bdev_init_complete(-1);
			return;
		}

		assert(memcmp(&bdev_opts, &g_bdev_opts, sizeof(bdev_opts)) == 0);
	}

	g_init_cb_fn = cb_fn;
	g_init_cb_arg = cb_arg;

	spdk_notify_type_register("bdev_register");
	spdk_notify_type_register("bdev_unregister");

	snprintf(mempool_name, sizeof(mempool_name), "bdev_io_%d", getpid());

	g_bdev_mgr.bdev_io_pool = spdk_mempool_create(mempool_name,
				  g_bdev_opts.bdev_io_pool_size,
				  sizeof(struct spdk_bdev_io) +
				  spdk_bdev_module_get_max_ctx_size(),
				  0,
				  SPDK_ENV_SOCKET_ID_ANY);

	if (g_bdev_mgr.bdev_io_pool == NULL) {
		SPDK_ERRLOG("could not allocate spdk_bdev_io pool\n");
		spdk_bdev_init_complete(-1);
		return;
	}

    // zhou: general purpose for small and large object

	/**
	 * Ensure no more than half of the total buffers end up local caches, by
	 *   using spdk_thread_get_count() to determine how many local caches we need
	 *   to account for.
	 */
	cache_size = BUF_SMALL_POOL_SIZE / (2 * spdk_thread_get_count());
	snprintf(mempool_name, sizeof(mempool_name), "buf_small_pool_%d", getpid());

	g_bdev_mgr.buf_small_pool = spdk_mempool_create(mempool_name,
				    BUF_SMALL_POOL_SIZE,
				    SPDK_BDEV_BUF_SIZE_WITH_MD(SPDK_BDEV_SMALL_BUF_MAX_SIZE) +
				    SPDK_BDEV_POOL_ALIGNMENT,
				    cache_size,
				    SPDK_ENV_SOCKET_ID_ANY);
	if (!g_bdev_mgr.buf_small_pool) {
		SPDK_ERRLOG("create rbuf small pool failed\n");
		spdk_bdev_init_complete(-1);
		return;
	}


	cache_size = BUF_LARGE_POOL_SIZE / (2 * spdk_thread_get_count());
	snprintf(mempool_name, sizeof(mempool_name), "buf_large_pool_%d", getpid());

	g_bdev_mgr.buf_large_pool = spdk_mempool_create(mempool_name,
				    BUF_LARGE_POOL_SIZE,
				    SPDK_BDEV_BUF_SIZE_WITH_MD(SPDK_BDEV_LARGE_BUF_MAX_SIZE) +
				    SPDK_BDEV_POOL_ALIGNMENT,
				    cache_size,
				    SPDK_ENV_SOCKET_ID_ANY);
	if (!g_bdev_mgr.buf_large_pool) {
		SPDK_ERRLOG("create rbuf large pool failed\n");
		spdk_bdev_init_complete(-1);
		return;
	}


    // zhou: 1MB
	g_bdev_mgr.zero_buffer = spdk_zmalloc(ZERO_BUFFER_SIZE, ZERO_BUFFER_SIZE,
					      NULL, SPDK_ENV_LCORE_ID_ANY, SPDK_MALLOC_DMA);
	if (!g_bdev_mgr.zero_buffer) {
		SPDK_ERRLOG("create bdev zero buffer failed\n");
		spdk_bdev_init_complete(-1);
		return;
	}

#ifdef SPDK_CONFIG_VTUNE
	g_bdev_mgr.domain = __itt_domain_create("spdk_bdev");
#endif


    // zhou: no matter lib bdev, each backing storag device, each frontend device...
    //       all of them looks as a IO device from lib/thread perspective.
	spdk_io_device_register(&g_bdev_mgr, spdk_bdev_mgmt_channel_create,
				spdk_bdev_mgmt_channel_destroy,
				sizeof(struct spdk_bdev_mgmt_channel),
				"bdev_mgr");


    // zhou: init all types backing storage module, but no backing disk are
    //       created/attached. Just make them ready to handle RPC.
    //       Then create disk RPC will trigger disk creation.
	rc = spdk_bdev_modules_init();
	g_bdev_mgr.module_init_complete = true;

	if (rc != 0) {
		SPDK_ERRLOG("bdev modules init failed\n");
        // zhou: will be completed in async way same thread.(like deferred queue)
		spdk_thread_send_msg(spdk_get_thread(), spdk_bdev_init_failed, NULL);
		return;
	}

	spdk_bdev_module_action_complete();
}

static void
spdk_bdev_mgr_unregister_cb(void *io_device)
{
	spdk_bdev_fini_cb cb_fn = g_fini_cb_fn;

	if (spdk_mempool_count(g_bdev_mgr.bdev_io_pool) != g_bdev_opts.bdev_io_pool_size) {
		SPDK_ERRLOG("bdev IO pool count is %zu but should be %u\n",
			    spdk_mempool_count(g_bdev_mgr.bdev_io_pool),
			    g_bdev_opts.bdev_io_pool_size);
	}

	if (spdk_mempool_count(g_bdev_mgr.buf_small_pool) != BUF_SMALL_POOL_SIZE) {
		SPDK_ERRLOG("Small buffer pool count is %zu but should be %u\n",
			    spdk_mempool_count(g_bdev_mgr.buf_small_pool),
			    BUF_SMALL_POOL_SIZE);
		assert(false);
	}

	if (spdk_mempool_count(g_bdev_mgr.buf_large_pool) != BUF_LARGE_POOL_SIZE) {
		SPDK_ERRLOG("Large buffer pool count is %zu but should be %u\n",
			    spdk_mempool_count(g_bdev_mgr.buf_large_pool),
			    BUF_LARGE_POOL_SIZE);
		assert(false);
	}

	spdk_mempool_free(g_bdev_mgr.bdev_io_pool);
	spdk_mempool_free(g_bdev_mgr.buf_small_pool);
	spdk_mempool_free(g_bdev_mgr.buf_large_pool);
	spdk_free(g_bdev_mgr.zero_buffer);

	cb_fn(g_fini_cb_arg);
	g_fini_cb_fn = NULL;
	g_fini_cb_arg = NULL;
	g_bdev_mgr.init_complete = false;
	g_bdev_mgr.module_init_complete = false;
}

static void
spdk_bdev_module_finish_iter(void *arg)
{
	struct spdk_bdev_module *bdev_module;

	/* Start iterating from the last touched module */
	if (!g_resume_bdev_module) {
		bdev_module = TAILQ_LAST(&g_bdev_mgr.bdev_modules, bdev_module_list);
	} else {
		bdev_module = TAILQ_PREV(g_resume_bdev_module, bdev_module_list,
					 internal.tailq);
	}

	while (bdev_module) {
		if (bdev_module->async_fini) {
			/* Save our place so we can resume later. We must
			 * save the variable here, before calling module_fini()
			 * below, because in some cases the module may immediately
			 * call spdk_bdev_module_finish_done() and re-enter
			 * this function to continue iterating. */
			g_resume_bdev_module = bdev_module;
		}

		if (bdev_module->module_fini) {
			bdev_module->module_fini();
		}

		if (bdev_module->async_fini) {
			return;
		}

		bdev_module = TAILQ_PREV(bdev_module, bdev_module_list,
					 internal.tailq);
	}

	g_resume_bdev_module = NULL;
	spdk_io_device_unregister(&g_bdev_mgr, spdk_bdev_mgr_unregister_cb);
}

void
spdk_bdev_module_finish_done(void)
{
	if (spdk_get_thread() != g_fini_thread) {
		spdk_thread_send_msg(g_fini_thread, spdk_bdev_module_finish_iter, NULL);
	} else {
		spdk_bdev_module_finish_iter(NULL);
	}
}

static void
_spdk_bdev_finish_unregister_bdevs_iter(void *cb_arg, int bdeverrno)
{
	struct spdk_bdev *bdev = cb_arg;

	if (bdeverrno && bdev) {
		SPDK_WARNLOG("Unable to unregister bdev '%s' during spdk_bdev_finish()\n",
			     bdev->name);

		/*
		 * Since the call to spdk_bdev_unregister() failed, we have no way to free this
		 *  bdev; try to continue by manually removing this bdev from the list and continue
		 *  with the next bdev in the list.
		 */
		TAILQ_REMOVE(&g_bdev_mgr.bdevs, bdev, internal.link);
	}

	if (TAILQ_EMPTY(&g_bdev_mgr.bdevs)) {
		SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Done unregistering bdevs\n");
		/*
		 * Bdev module finish need to be deferred as we might be in the middle of some context
		 * (like bdev part free) that will use this bdev (or private bdev driver ctx data)
		 * after returning.
		 */
		spdk_thread_send_msg(spdk_get_thread(), spdk_bdev_module_finish_iter, NULL);
		return;
	}

	/*
	 * Unregister last unclaimed bdev in the list, to ensure that bdev subsystem
	 * shutdown proceeds top-down. The goal is to give virtual bdevs an opportunity
	 * to detect clean shutdown as opposed to run-time hot removal of the underlying
	 * base bdevs.
	 *
	 * Also, walk the list in the reverse order.
	 */
	for (bdev = TAILQ_LAST(&g_bdev_mgr.bdevs, spdk_bdev_list);
	     bdev; bdev = TAILQ_PREV(bdev, spdk_bdev_list, internal.link)) {
		if (bdev->internal.claim_module != NULL) {
			SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Skipping claimed bdev '%s'(<-'%s').\n",
				      bdev->name, bdev->internal.claim_module->name);
			continue;
		}

		SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Unregistering bdev '%s'\n", bdev->name);
		spdk_bdev_unregister(bdev, _spdk_bdev_finish_unregister_bdevs_iter, bdev);
		return;
	}

	/*
	 * If any bdev fails to unclaim underlying bdev properly, we may face the
	 * case of bdev list consisting of claimed bdevs only (if claims are managed
	 * correctly, this would mean there's a loop in the claims graph which is
	 * clearly impossible). Warn and unregister last bdev on the list then.
	 */
	for (bdev = TAILQ_LAST(&g_bdev_mgr.bdevs, spdk_bdev_list);
	     bdev; bdev = TAILQ_PREV(bdev, spdk_bdev_list, internal.link)) {
		SPDK_ERRLOG("Unregistering claimed bdev '%s'!\n", bdev->name);
		spdk_bdev_unregister(bdev, _spdk_bdev_finish_unregister_bdevs_iter, bdev);
		return;
	}
}

// zhou: tear down the bdev library.
void
spdk_bdev_finish(spdk_bdev_fini_cb cb_fn, void *cb_arg)
{
	struct spdk_bdev_module *m;

	assert(cb_fn != NULL);

	g_fini_thread = spdk_get_thread();

	g_fini_cb_fn = cb_fn;
	g_fini_cb_arg = cb_arg;

	TAILQ_FOREACH(m, &g_bdev_mgr.bdev_modules, internal.tailq) {
		if (m->fini_start) {
			m->fini_start();
		}
	}

	_spdk_bdev_finish_unregister_bdevs_iter(NULL, 0);
}

// zhou: END of subsystem related operations, includes ".init", ".fini", ".config", .write_config_json"
////////////////////////////////////////////////////////////////////////////////
// zhou: START of "struct spdk_bdev_io"

// zhou: get a empty IO request, there is per thread cache and global pool.
static struct spdk_bdev_io *
spdk_bdev_get_io(struct spdk_bdev_channel *channel)
{
	struct spdk_bdev_mgmt_channel *ch = channel->shared_resource->mgmt_ch;
	struct spdk_bdev_io *bdev_io;

	if (ch->per_thread_cache_count > 0) {

		bdev_io = STAILQ_FIRST(&ch->per_thread_cache);
		STAILQ_REMOVE_HEAD(&ch->per_thread_cache, internal.buf_link);

		ch->per_thread_cache_count--;

	} else if (spdk_unlikely(!TAILQ_EMPTY(&ch->io_wait_queue))) {
        // zhou: there are already some one waiting on "io_wait_queue" for
        //       "struct spdk_bdev_io" becomes available.
        //       In this case, should use spdk_bdev_queue_io_wait() to register callback
        //       be notified when resource becomes available.
		/*
		 * Don't try to look for bdev_ios in the global pool if there are
		 * waiters on bdev_ios - we don't want this caller to jump the line.
		 */
		bdev_io = NULL;
	} else {
		bdev_io = spdk_mempool_get(g_bdev_mgr.bdev_io_pool);
	}

	return bdev_io;
}

// zhou: there are two kinds resource, "struct spdk_bdev_io" and Large/Small buffer.
void
spdk_bdev_free_io(struct spdk_bdev_io *bdev_io)
{
	struct spdk_bdev_mgmt_channel *ch;

	assert(bdev_io != NULL);
	assert(bdev_io->internal.status != SPDK_BDEV_IO_STATUS_PENDING);

	ch = bdev_io->internal.ch->shared_resource->mgmt_ch;

    // zhou: this spdk_bdev_io owns a buffer allocated from pool,
	if (bdev_io->internal.buf != NULL) {
		spdk_bdev_io_put_buf(bdev_io);
	}

    // zhou: up to channel's "struct spdk_bdev_io" cache state.
	if (ch->per_thread_cache_count < ch->bdev_io_cache_size) {
		ch->per_thread_cache_count++;

		STAILQ_INSERT_HEAD(&ch->per_thread_cache, bdev_io, internal.buf_link);

        // zhou: someone is warting for "struct spdk_bdev_io" resource, wait them up.
		while (ch->per_thread_cache_count > 0 && !TAILQ_EMPTY(&ch->io_wait_queue)) {
			struct spdk_bdev_io_wait_entry *entry;

			entry = TAILQ_FIRST(&ch->io_wait_queue);
			TAILQ_REMOVE(&ch->io_wait_queue, entry, link);

			entry->cb_fn(entry->cb_arg);
		}
	} else {
		/* We should never have a full cache with entries on the io wait queue. */
		assert(TAILQ_EMPTY(&ch->io_wait_queue));
		spdk_mempool_put(g_bdev_mgr.bdev_io_pool, (void *)bdev_io);
	}
}

// zhou: END of "struct spdk_bdev_io"
////////////////////////////////////////////////////////////////////////////////
// zhou: START of QoS

static bool
_spdk_bdev_qos_is_iops_rate_limit(enum spdk_bdev_qos_rate_limit_type limit)
{
	assert(limit != SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES);

	switch (limit) {
	case SPDK_BDEV_QOS_RW_IOPS_RATE_LIMIT:
		return true;
	case SPDK_BDEV_QOS_RW_BPS_RATE_LIMIT:
	case SPDK_BDEV_QOS_R_BPS_RATE_LIMIT:
	case SPDK_BDEV_QOS_W_BPS_RATE_LIMIT:
		return false;
	case SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES:
	default:
		return false;
	}
}

static bool
_spdk_bdev_qos_io_to_limit(struct spdk_bdev_io *bdev_io)
{
	switch (bdev_io->type) {
	case SPDK_BDEV_IO_TYPE_NVME_IO:
	case SPDK_BDEV_IO_TYPE_NVME_IO_MD:
	case SPDK_BDEV_IO_TYPE_READ:
	case SPDK_BDEV_IO_TYPE_WRITE:
		return true;
	default:
		return false;
	}
}

static bool
_spdk_bdev_is_read_io(struct spdk_bdev_io *bdev_io)
{
	switch (bdev_io->type) {
	case SPDK_BDEV_IO_TYPE_NVME_IO:
	case SPDK_BDEV_IO_TYPE_NVME_IO_MD:
		/* Bit 1 (0x2) set for read operation */
		if (bdev_io->u.nvme_passthru.cmd.opc & SPDK_NVME_OPC_READ) {
			return true;
		} else {
			return false;
		}
	case SPDK_BDEV_IO_TYPE_READ:
		return true;
	default:
		return false;
	}
}

static uint64_t
_spdk_bdev_get_io_size_in_byte(struct spdk_bdev_io *bdev_io)
{
	struct spdk_bdev	*bdev = bdev_io->bdev;

	switch (bdev_io->type) {
	case SPDK_BDEV_IO_TYPE_NVME_IO:
	case SPDK_BDEV_IO_TYPE_NVME_IO_MD:
		return bdev_io->u.nvme_passthru.nbytes;
	case SPDK_BDEV_IO_TYPE_READ:
	case SPDK_BDEV_IO_TYPE_WRITE:
		return bdev_io->u.bdev.num_blocks * bdev->blocklen;
	default:
		return 0;
	}
}

static bool
_spdk_bdev_qos_rw_queue_io(const struct spdk_bdev_qos_limit *limit, struct spdk_bdev_io *io)
{
	if (limit->max_per_timeslice > 0 && limit->remaining_this_timeslice <= 0) {
		return true;
	} else {
		return false;
	}
}

static bool
_spdk_bdev_qos_r_queue_io(const struct spdk_bdev_qos_limit *limit, struct spdk_bdev_io *io)
{
	if (_spdk_bdev_is_read_io(io) == false) {
		return false;
	}

	return _spdk_bdev_qos_rw_queue_io(limit, io);
}

static bool
_spdk_bdev_qos_w_queue_io(const struct spdk_bdev_qos_limit *limit, struct spdk_bdev_io *io)
{
	if (_spdk_bdev_is_read_io(io) == true) {
		return false;
	}

	return _spdk_bdev_qos_rw_queue_io(limit, io);
}

static void
_spdk_bdev_qos_rw_iops_update_quota(struct spdk_bdev_qos_limit *limit, struct spdk_bdev_io *io)
{
	limit->remaining_this_timeslice--;
}

static void
_spdk_bdev_qos_rw_bps_update_quota(struct spdk_bdev_qos_limit *limit, struct spdk_bdev_io *io)
{
	limit->remaining_this_timeslice -= _spdk_bdev_get_io_size_in_byte(io);
}

static void
_spdk_bdev_qos_r_bps_update_quota(struct spdk_bdev_qos_limit *limit, struct spdk_bdev_io *io)
{
	if (_spdk_bdev_is_read_io(io) == false) {
		return;
	}

	return _spdk_bdev_qos_rw_bps_update_quota(limit, io);
}

static void
_spdk_bdev_qos_w_bps_update_quota(struct spdk_bdev_qos_limit *limit, struct spdk_bdev_io *io)
{
	if (_spdk_bdev_is_read_io(io) == true) {
		return;
	}

	return _spdk_bdev_qos_rw_bps_update_quota(limit, io);
}

// zhou: Just one of them will be used ?
static void
_spdk_bdev_qos_set_ops(struct spdk_bdev_qos *qos)
{
	int i;

	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
		if (qos->rate_limits[i].limit == SPDK_BDEV_QOS_LIMIT_NOT_DEFINED) {
			qos->rate_limits[i].queue_io = NULL;
			qos->rate_limits[i].update_quota = NULL;
			continue;
		}

		switch (i) {
		case SPDK_BDEV_QOS_RW_IOPS_RATE_LIMIT:
            // zhou: Function to check whether to queue the IO
			qos->rate_limits[i].queue_io = _spdk_bdev_qos_rw_queue_io;
            // zhou: Function to update for the submitted IO.
			qos->rate_limits[i].update_quota = _spdk_bdev_qos_rw_iops_update_quota;
			break;
		case SPDK_BDEV_QOS_RW_BPS_RATE_LIMIT:
			qos->rate_limits[i].queue_io = _spdk_bdev_qos_rw_queue_io;
			qos->rate_limits[i].update_quota = _spdk_bdev_qos_rw_bps_update_quota;
			break;
		case SPDK_BDEV_QOS_R_BPS_RATE_LIMIT:
			qos->rate_limits[i].queue_io = _spdk_bdev_qos_r_queue_io;
			qos->rate_limits[i].update_quota = _spdk_bdev_qos_r_bps_update_quota;
			break;
		case SPDK_BDEV_QOS_W_BPS_RATE_LIMIT:
			qos->rate_limits[i].queue_io = _spdk_bdev_qos_w_queue_io;
			qos->rate_limits[i].update_quota = _spdk_bdev_qos_w_bps_update_quota;
			break;
		default:
			break;
		}
	}
}

// zhou: submit or queued IO depending on QoS quota. "spdk_bdev_channel_poll_qos()"
//       is another chance to submit IO which still in queue.
static int
_spdk_bdev_qos_io_submit(struct spdk_bdev_channel *ch, struct spdk_bdev_qos *qos)
{
	struct spdk_bdev_io		*bdev_io = NULL, *tmp = NULL;
	struct spdk_bdev		*bdev = ch->bdev;
	struct spdk_bdev_shared_resource *shared_resource = ch->shared_resource;
	int				i, submitted_ios = 0;

	TAILQ_FOREACH_SAFE(bdev_io, &qos->queued, internal.link, tmp) {
		if (_spdk_bdev_qos_io_to_limit(bdev_io) == true) {
			for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
				if (!qos->rate_limits[i].queue_io) {
					continue;
				}

                // zhou: no quota to send, just queue.
				if (qos->rate_limits[i].queue_io(&qos->rate_limits[i],
								 bdev_io) == true) {
					return submitted_ios;
				}
			}

			for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
				if (!qos->rate_limits[i].update_quota) {
					continue;
				}
                // zhou: decrease quota
				qos->rate_limits[i].update_quota(&qos->rate_limits[i], bdev_io);
			}
		}

        // zhou: pick up a queued IO and send out.
		TAILQ_REMOVE(&qos->queued, bdev_io, internal.link);
		ch->io_outstanding++;
		shared_resource->io_outstanding++;

		bdev_io->internal.in_submit_request = true;
		bdev->fn_table->submit_request(ch->channel, bdev_io);
		bdev_io->internal.in_submit_request = false;
		submitted_ios++;
	}

	return submitted_ios;
}

// zhou: END of QoS
////////////////////////////////////////////////////////////////////////////////
// zhou: START of split IO

static void
_spdk_bdev_queue_io_wait_with_cb(struct spdk_bdev_io *bdev_io, spdk_bdev_io_wait_cb cb_fn)
{
	int rc;

	bdev_io->internal.waitq_entry.bdev = bdev_io->bdev;
	bdev_io->internal.waitq_entry.cb_fn = cb_fn;
	bdev_io->internal.waitq_entry.cb_arg = bdev_io;
	rc = spdk_bdev_queue_io_wait(bdev_io->bdev, spdk_io_channel_from_ctx(bdev_io->internal.ch),
				     &bdev_io->internal.waitq_entry);
	if (rc != 0) {
		SPDK_ERRLOG("Queue IO failed, rc=%d\n", rc);
		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_FAILED;
		bdev_io->internal.cb(bdev_io, false, bdev_io->internal.caller_ctx);
	}
}

// zhou: bdev framework just split READ and WRITE I/O.
//       Because other IO like UNMAP, I/O size may over entire block device,
//       and without a payload.
//       Underlying disk may handle it as a single I/O more easy and efficient.
static bool
_spdk_bdev_io_type_can_split(uint8_t type)
{
	assert(type != SPDK_BDEV_IO_TYPE_INVALID);
	assert(type < SPDK_BDEV_NUM_IO_TYPES);

	/* Only split READ and WRITE I/O.  Theoretically other types of I/O like
	 * UNMAP could be split, but these types of I/O are typically much larger
	 * in size (sometimes the size of the entire block device), and the bdev
	 * module can more efficiently split these types of I/O.  Plus those types
	 * of I/O do not have a payload, which makes the splitting process simpler.
	 */
	if (type == SPDK_BDEV_IO_TYPE_READ || type == SPDK_BDEV_IO_TYPE_WRITE) {
		return true;
	} else {
		return false;
	}
}

// zhou: judge the current IO Request should be split or not.
static bool
_spdk_bdev_io_should_split(struct spdk_bdev_io *bdev_io)
{
	uint64_t start_stripe, end_stripe;
    // zhou: number in blocks.
	uint32_t io_boundary = bdev_io->bdev->optimal_io_boundary;

	if (io_boundary == 0) {
		return false;
	}

	if (!_spdk_bdev_io_type_can_split(bdev_io->type)) {
		return false;
	}

	start_stripe = bdev_io->u.bdev.offset_blocks;
	end_stripe = start_stripe + bdev_io->u.bdev.num_blocks - 1;

	/* Avoid expensive div operations if possible.  These spdk_u32 functions are very cheap. */
	if (spdk_likely(spdk_u32_is_pow2(io_boundary))) {
		start_stripe >>= spdk_u32log2(io_boundary);
		end_stripe >>= spdk_u32log2(io_boundary);
	} else {
		start_stripe /= io_boundary;
		end_stripe /= io_boundary;
	}

    // zhou: when start block number is not in same "io_boundary" as end block number,
    //       return true == need split.
	return (start_stripe != end_stripe);
}

static uint32_t
_to_next_boundary(uint64_t offset, uint32_t boundary)
{
	return (boundary - (offset % boundary));
}

static void
_spdk_bdev_io_split_done(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);

// zhou: Interation way to split payload.
static void
_spdk_bdev_io_split_with_payload(void *_bdev_io)
{
	struct spdk_bdev_io *bdev_io = _bdev_io;
	uint64_t current_offset, remaining;
	uint32_t blocklen, to_next_boundary, to_next_boundary_bytes;
	struct iovec *parent_iov, *iov;
	uint64_t parent_iov_offset, iov_len;
	uint32_t parent_iovpos, parent_iovcnt, child_iovcnt, iovcnt;
	int rc;

    // zhou: unit in block.
	remaining = bdev_io->u.bdev.split_remaining_num_blocks;
	current_offset = bdev_io->u.bdev.split_current_offset_blocks;

	blocklen = bdev_io->bdev->blocklen;


	parent_iov_offset = (current_offset - bdev_io->u.bdev.offset_blocks) * blocklen;
	parent_iovcnt = bdev_io->u.bdev.iovcnt;

    // zhou: find out cursor of buffer in form of IOV.
	for (parent_iovpos = 0; parent_iovpos < parent_iovcnt; parent_iovpos++) {
		parent_iov = &bdev_io->u.bdev.iovs[parent_iovpos];
		if (parent_iov_offset < parent_iov->iov_len) {
			break;
		}
		parent_iov_offset -= parent_iov->iov_len;
	}

    // zhou: element "bdev_io->u.bdev.iovs[parent_iovpos]", offset is "parent_iov_offset"

	child_iovcnt = 0;
	while (remaining > 0 && parent_iovpos < parent_iovcnt && child_iovcnt < BDEV_IO_NUM_CHILD_IOV) {

        // zhou: how many blocks need to handle before next strip boundary.
		to_next_boundary = _to_next_boundary(current_offset, bdev_io->bdev->optimal_io_boundary);
		to_next_boundary = spdk_min(remaining, to_next_boundary);
		to_next_boundary_bytes = to_next_boundary * blocklen;

		iov = &bdev_io->child_iov[child_iovcnt];
		iovcnt = 0;

		while (to_next_boundary_bytes > 0 && parent_iovpos < parent_iovcnt &&
		       child_iovcnt < BDEV_IO_NUM_CHILD_IOV) {

			parent_iov = &bdev_io->u.bdev.iovs[parent_iovpos];
			iov_len = spdk_min(to_next_boundary_bytes, parent_iov->iov_len - parent_iov_offset);
			to_next_boundary_bytes -= iov_len;

			bdev_io->child_iov[child_iovcnt].iov_base = parent_iov->iov_base + parent_iov_offset;
			bdev_io->child_iov[child_iovcnt].iov_len = iov_len;

			if (iov_len < parent_iov->iov_len - parent_iov_offset) {
				parent_iov_offset += iov_len;
			} else {
				parent_iovpos++;
				parent_iov_offset = 0;
			}
			child_iovcnt++;
			iovcnt++;
		}

        // zhou: we don't want meet this case, BDEV_IO_NUM_CHILD_IOV suppose big enough
        //       to hold all pieces of splitted IO.

        //       Unless "u.bdev.iovs[]" is composited with so many small elements.
		if (to_next_boundary_bytes > 0) {

            // zhou: have to pray for good luck.

			/* We had to stop this child I/O early because we ran out of
			 *  child_iov space.  Make sure the iovs collected are valid and
			 *  then adjust to_next_boundary before starting the child I/O.
			 */
			if ((to_next_boundary_bytes % blocklen) != 0) {
				SPDK_ERRLOG("Remaining %" PRIu32 " is not multiple of block size %" PRIu32 "\n",
					    to_next_boundary_bytes, blocklen);
				bdev_io->internal.status = SPDK_BDEV_IO_STATUS_FAILED;
				if (bdev_io->u.bdev.split_outstanding == 0) {
					bdev_io->internal.cb(bdev_io, false, bdev_io->internal.caller_ctx);
				}
				return;
			}

			to_next_boundary -= to_next_boundary_bytes / blocklen;
		}

        // zhou: in normal case, we will submit all child IO not to wait for completion.
		bdev_io->u.bdev.split_outstanding++;

        // zhou: spdk_bdev_readv_blocks() like recursion function.

		if (bdev_io->type == SPDK_BDEV_IO_TYPE_READ) {
			rc = spdk_bdev_readv_blocks(bdev_io->internal.desc,
						    spdk_io_channel_from_ctx(bdev_io->internal.ch),
						    iov, iovcnt, current_offset, to_next_boundary,
						    _spdk_bdev_io_split_done, bdev_io);
		} else {
			rc = spdk_bdev_writev_blocks(bdev_io->internal.desc,
						     spdk_io_channel_from_ctx(bdev_io->internal.ch),
						     iov, iovcnt, current_offset, to_next_boundary,
						     _spdk_bdev_io_split_done, bdev_io);
		}

		if (rc == 0) {
            // zhou: update for split next piece if resource is enough for this while loop.
            //       In rare case, we have to wait for resource by _spdk_bdev_io_split_done().
			current_offset += to_next_boundary;
			remaining -= to_next_boundary;

			bdev_io->u.bdev.split_current_offset_blocks = current_offset;
			bdev_io->u.bdev.split_remaining_num_blocks = remaining;

		} else {
            // zhou: otherwise, failed.
			bdev_io->u.bdev.split_outstanding--;

			if (rc == -ENOMEM) {
				if (bdev_io->u.bdev.split_outstanding == 0) {
					/* No I/O is outstanding. Hence we should wait here. */
					_spdk_bdev_queue_io_wait_with_cb(bdev_io,
									 _spdk_bdev_io_split_with_payload);
				}
			} else {
				bdev_io->internal.status = SPDK_BDEV_IO_STATUS_FAILED;
				if (bdev_io->u.bdev.split_outstanding == 0) {
					bdev_io->internal.cb(bdev_io, false, bdev_io->internal.caller_ctx);
				}
			}

			return;
		}
	}
}

// zhou: a piece of slitted IO completed.
static void
_spdk_bdev_io_split_done(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct spdk_bdev_io *parent_io = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		parent_io->internal.status = SPDK_BDEV_IO_STATUS_FAILED;
	}
	parent_io->u.bdev.split_outstanding--;
	if (parent_io->u.bdev.split_outstanding != 0) {
		return;
	}

	/*
	 * Parent I/O finishes when all blocks are consumed or there is any failure of
	 * child I/O and no outstanding child I/O.
	 */
	if (parent_io->u.bdev.split_remaining_num_blocks == 0 ||
	    parent_io->internal.status != SPDK_BDEV_IO_STATUS_SUCCESS) {

        // zhou: completed the parent I/O success or failure.
		parent_io->internal.cb(parent_io, parent_io->internal.status == SPDK_BDEV_IO_STATUS_SUCCESS,
				       parent_io->internal.caller_ctx);
		return;
	}

    // zhou: This is rare case, that resource is not enough to split IO in one time.
	/*
	 * Continue with the splitting process.  This function will complete the parent I/O if the
	 * splitting is done.
	 */
	_spdk_bdev_io_split_with_payload(parent_io);
}

// zhou: prepare to start split.
static void
_spdk_bdev_io_split(struct spdk_io_channel *ch, struct spdk_bdev_io *bdev_io)
{
    // zhou: only READ/WRITE
	assert(_spdk_bdev_io_type_can_split(bdev_io->type));

	bdev_io->u.bdev.split_current_offset_blocks = bdev_io->u.bdev.offset_blocks;
	bdev_io->u.bdev.split_remaining_num_blocks = bdev_io->u.bdev.num_blocks;
	bdev_io->u.bdev.split_outstanding = 0;

	bdev_io->internal.status = SPDK_BDEV_IO_STATUS_SUCCESS;

	_spdk_bdev_io_split_with_payload(bdev_io);
}

// zhou: get buf completed for READ, then start to split IO.
static void
_spdk_bdev_io_split_get_buf_cb(struct spdk_io_channel *ch, struct spdk_bdev_io *bdev_io,
			       bool success)
{
	if (!success) {
		spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	_spdk_bdev_io_split(ch, bdev_io);
}

// zhou: END of split IO
////////////////////////////////////////////////////////////////////////////////
// zhou: START of submit IO

// zhou:
/* Explicitly mark this inline, since it's used as a function pointer and otherwise won't
 *  be inlined, at least on some compilers.
 */
static inline void
_spdk_bdev_io_submit(void *ctx)
{
	struct spdk_bdev_io *bdev_io = ctx;
	struct spdk_bdev *bdev = bdev_io->bdev;
	struct spdk_bdev_channel *bdev_ch = bdev_io->internal.ch;
	struct spdk_io_channel *ch = bdev_ch->channel;
	struct spdk_bdev_shared_resource *shared_resource = bdev_ch->shared_resource;
	uint64_t tsc;

	tsc = spdk_get_ticks();

	bdev_io->internal.submit_tsc = tsc;

	spdk_trace_record_tsc(tsc, TRACE_BDEV_IO_START, 0, 0, (uintptr_t)bdev_io, bdev_io->type);

	bdev_ch->io_outstanding++;
	shared_resource->io_outstanding++;
	bdev_io->internal.in_submit_request = true;

	if (spdk_likely(bdev_ch->flags == 0)) {

		if (spdk_likely(TAILQ_EMPTY(&shared_resource->nomem_io))) {
            // zhou: e.g. bdev_aio_submit_request(), bdev_nvme_submit_request()
            //       bdev_uring_submit_request(),
			bdev->fn_table->submit_request(ch, bdev_io);

		} else {

			bdev_ch->io_outstanding--;
			shared_resource->io_outstanding--;
			TAILQ_INSERT_TAIL(&shared_resource->nomem_io, bdev_io, internal.link);
		}

	} else if (bdev_ch->flags & BDEV_CH_RESET_IN_PROGRESS) {

		spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);

	} else if (bdev_ch->flags & BDEV_CH_QOS_ENABLED) {
        // zhou: will not submit immediately, but queued in QoS queue for schedule.
		bdev_ch->io_outstanding--;
		shared_resource->io_outstanding--;
		TAILQ_INSERT_TAIL(&bdev->internal.qos->queued, bdev_io, internal.link);

		_spdk_bdev_qos_io_submit(bdev_ch, bdev->internal.qos);

	} else {
		SPDK_ERRLOG("unknown bdev_ch flag %x found\n", bdev_ch->flags);
		spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
	}

	bdev_io->internal.in_submit_request = false;
}

// zhou:
static void
spdk_bdev_io_submit(struct spdk_bdev_io *bdev_io)
{
	struct spdk_bdev *bdev = bdev_io->bdev;

    // zhou: get thread which the specified I/O channed binded.
	struct spdk_thread *thread = spdk_io_channel_get_thread(bdev_io->internal.ch->channel);

	assert(thread != NULL);
	assert(bdev_io->internal.status == SPDK_BDEV_IO_STATUS_PENDING);

	if (bdev->split_on_optimal_io_boundary && _spdk_bdev_io_should_split(bdev_io)) {

        // zhou: only allocate buffer for READ, split IO for WRITE directly.
		if (bdev_io->type == SPDK_BDEV_IO_TYPE_READ) {
            // zhou: _spdk_bdev_io_split_get_buf_cb() will invoke _spdk_bdev_io_split().
			spdk_bdev_io_get_buf(bdev_io, _spdk_bdev_io_split_get_buf_cb,
					     bdev_io->u.bdev.num_blocks * bdev_io->bdev->blocklen);
		} else {
            // zhou: underlying disk will invoke spdk_bdev_io_get_buf() later.
			_spdk_bdev_io_split(NULL, bdev_io);
		}

        // zhou: _spdk_bdev_io_split_with_payload() will first split IO then submit IO.
		return;
	}

    // zhou: QoS enabled bdev processing.
	if (bdev_io->internal.ch->flags & BDEV_CH_QOS_ENABLED) {

		if ((thread == bdev->internal.qos->thread) || !bdev->internal.qos->thread) {
			_spdk_bdev_io_submit(bdev_io);
		} else {
            // zhou: should be handled by QoS owner channel

			bdev_io->internal.io_submit_ch = bdev_io->internal.ch;
			bdev_io->internal.ch = bdev->internal.qos->ch;

			spdk_thread_send_msg(bdev->internal.qos->thread, _spdk_bdev_io_submit, bdev_io);
		}

	} else {

		_spdk_bdev_io_submit(bdev_io);
	}
}

// zhou: submit RESET to underlying device in this thread/channel.
static void
spdk_bdev_io_submit_reset(struct spdk_bdev_io *bdev_io)
{
	struct spdk_bdev *bdev = bdev_io->bdev;
	struct spdk_bdev_channel *bdev_ch = bdev_io->internal.ch;
	struct spdk_io_channel *ch = bdev_ch->channel;

	assert(bdev_io->internal.status == SPDK_BDEV_IO_STATUS_PENDING);

	bdev_io->internal.in_submit_request = true;
	bdev->fn_table->submit_request(ch, bdev_io);
	bdev_io->internal.in_submit_request = false;
}

// zhou: set other private data of a IO request.
static void
spdk_bdev_io_init(struct spdk_bdev_io *bdev_io,
		  struct spdk_bdev *bdev, void *cb_arg,
		  spdk_bdev_io_completion_cb cb)
{
	bdev_io->bdev = bdev;

	bdev_io->internal.caller_ctx = cb_arg;
	bdev_io->internal.cb = cb;
	bdev_io->internal.status = SPDK_BDEV_IO_STATUS_PENDING;
	bdev_io->internal.in_submit_request = false;
	bdev_io->internal.buf = NULL;
	bdev_io->internal.io_submit_ch = NULL;
	bdev_io->internal.orig_iovs = NULL;
	bdev_io->internal.orig_iovcnt = 0;
}

static bool
_spdk_bdev_io_type_supported(struct spdk_bdev *bdev, enum spdk_bdev_io_type io_type)
{
	return bdev->fn_table->io_type_supported(bdev->ctxt, io_type);
}

// zhou: "Some I/O request types are optional and may not be supported by a given bdev.
//       To query a bdev for the I/O request types it supports, call
//       spdk_bdev_io_type_supported()."
bool
spdk_bdev_io_type_supported(struct spdk_bdev *bdev, enum spdk_bdev_io_type io_type)
{
	bool supported;

	supported = _spdk_bdev_io_type_supported(bdev, io_type);

	if (!supported) {
		switch (io_type) {
		case SPDK_BDEV_IO_TYPE_WRITE_ZEROES:
			/* The bdev layer will emulate write zeroes as long as write is supported. */
			supported = _spdk_bdev_io_type_supported(bdev, SPDK_BDEV_IO_TYPE_WRITE);
			break;
		case SPDK_BDEV_IO_TYPE_ZCOPY:
			/* Zero copy can be emulated with regular read and write */
			supported = _spdk_bdev_io_type_supported(bdev, SPDK_BDEV_IO_TYPE_READ) &&
				    _spdk_bdev_io_type_supported(bdev, SPDK_BDEV_IO_TYPE_WRITE);
			break;
		default:
			break;
		}
	}

	return supported;
}

// zhou: END of IO Request operation
////////////////////////////////////////////////////////////////////////////////
// zhou: START of

// zhou: handle RPC "get_bdevs"
int
spdk_bdev_dump_info_json(struct spdk_bdev *bdev, struct spdk_json_write_ctx *w)
{
	if (bdev->fn_table->dump_info_json) {
		return bdev->fn_table->dump_info_json(bdev->ctxt, w);
	}

	return 0;
}

// zhou: used by QoS owner channel/thread only.
static void
spdk_bdev_qos_update_max_quota_per_timeslice(struct spdk_bdev_qos *qos)
{
	uint32_t max_per_timeslice = 0;
	int i;

	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
		if (qos->rate_limits[i].limit == SPDK_BDEV_QOS_LIMIT_NOT_DEFINED) {
			qos->rate_limits[i].max_per_timeslice = 0;
			continue;
		}

		max_per_timeslice = qos->rate_limits[i].limit *
				    SPDK_BDEV_QOS_TIMESLICE_IN_USEC / SPDK_SEC_TO_USEC;

		qos->rate_limits[i].max_per_timeslice = spdk_max(max_per_timeslice,
							qos->rate_limits[i].min_per_timeslice);

		qos->rate_limits[i].remaining_this_timeslice = qos->rate_limits[i].max_per_timeslice;
	}

	_spdk_bdev_qos_set_ops(qos);
}

// zhou: all IO that has not been send immediately due to quota limit in
//       _spdk_bdev_qos_io_submit(), still has chance to be sent by this poller.
//       Then this poller will schedule the queued IO according to slice and rate limit.
//       By this method, all IO in queue will be submited, even if
//       _spdk_bdev_qos_io_submit() has no chance to run.
static int
spdk_bdev_channel_poll_qos(void *arg)
{
	struct spdk_bdev_qos *qos = arg;
	uint64_t now = spdk_get_ticks();
	int i;

	if (now < (qos->last_timeslice + qos->timeslice_size)) {
		/* We received our callback earlier than expected - return
		 *  immediately and wait to do accounting until at least one
		 *  timeslice has actually expired.  This should never happen
		 *  with a well-behaved timer implementation.
		 */
		return 0;
	}

	/* Reset for next round of rate limiting */
	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
		/* We may have allowed the IOs or bytes to slightly overrun in the last
		 * timeslice. remaining_this_timeslice is signed, so if it's negative
		 * here, we'll account for the overrun so that the next timeslice will
		 * be appropriately reduced.
		 */
		if (qos->rate_limits[i].remaining_this_timeslice > 0) {
			qos->rate_limits[i].remaining_this_timeslice = 0;
		}
	}

	while (now >= (qos->last_timeslice + qos->timeslice_size)) {
		qos->last_timeslice += qos->timeslice_size;
		for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
			qos->rate_limits[i].remaining_this_timeslice +=
				qos->rate_limits[i].max_per_timeslice;
		}
	}

	return _spdk_bdev_qos_io_submit(qos->ch, qos);
}

static void
_spdk_bdev_channel_destroy_resource(struct spdk_bdev_channel *ch)
{
	struct spdk_bdev_shared_resource *shared_resource;

	spdk_put_io_channel(ch->channel);

	shared_resource = ch->shared_resource;

	assert(ch->io_outstanding == 0);
	assert(shared_resource->ref > 0);
	shared_resource->ref--;
	if (shared_resource->ref == 0) {
		assert(shared_resource->io_outstanding == 0);
		TAILQ_REMOVE(&shared_resource->mgmt_ch->shared_resources, shared_resource, link);
		spdk_put_io_channel(spdk_io_channel_from_ctx(shared_resource->mgmt_ch));
		free(shared_resource);
	}
}

// zhou:
/* Caller must hold bdev->internal.mutex. */
static void
_spdk_bdev_enable_qos(struct spdk_bdev *bdev, struct spdk_bdev_channel *ch)
{
	struct spdk_bdev_qos	*qos = bdev->internal.qos;
	int			i;

	/* Rate limiting on this bdev enabled */
	if (qos) {
        // zhou: only the first channel will be take in charge of IO Throttling.
        //       All channels will be set flag "BDEV_CH_QOS_ENABLED"
		if (qos->ch == NULL) {
			struct spdk_io_channel *io_ch;

			SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Selecting channel %p as QoS channel for bdev %s on thread %p\n", ch,
				      bdev->name, spdk_get_thread());

			/* No qos channel has been selected, so set one up */

			/* Take another reference to ch */
			io_ch = spdk_get_io_channel(__bdev_to_io_dev(bdev));
			assert(io_ch != NULL);

            // zhou: bdev channel
			qos->ch = ch;
            // zhou: the corresponding thread of the bdev channel.
			qos->thread = spdk_io_channel_get_thread(io_ch);

			TAILQ_INIT(&qos->queued);

			for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
				if (_spdk_bdev_qos_is_iops_rate_limit(i) == true) {
                    // zhou: 1 IOPS
					qos->rate_limits[i].min_per_timeslice =
						SPDK_BDEV_QOS_MIN_IO_PER_TIMESLICE;
				} else {
                    // zhou: 512 Bytes, just a sector.
					qos->rate_limits[i].min_per_timeslice =
						SPDK_BDEV_QOS_MIN_BYTE_PER_TIMESLICE;
				}

				if (qos->rate_limits[i].limit == 0) {
					qos->rate_limits[i].limit = SPDK_BDEV_QOS_LIMIT_NOT_DEFINED;
				}
			}

			spdk_bdev_qos_update_max_quota_per_timeslice(qos);

            // zhou: convert time slice in ticks.
			qos->timeslice_size =
				SPDK_BDEV_QOS_TIMESLICE_IN_USEC * spdk_get_ticks_hz() / SPDK_SEC_TO_USEC;

			qos->last_timeslice = spdk_get_ticks();

            // zhou:
			qos->poller = spdk_poller_register(spdk_bdev_channel_poll_qos,
							   qos,
							   SPDK_BDEV_QOS_TIMESLICE_IN_USEC);
		}

		ch->flags |= BDEV_CH_QOS_ENABLED;
	}
}

// zhou: bdev I/O channel create callback function.
//       "Descriptors may be passed to and used from multiple threads simultaneously.
//       However, for each thread a separate I/O channel must be obtained by
//       calling spdk_bdev_get_io_channel(). This will allocate the necessary
//       per-thread resources to submit I/O requests to the bdev without taking locks."
static int
spdk_bdev_channel_create(void *io_device, void *ctx_buf)
{
	struct spdk_bdev		*bdev = __bdev_from_io_dev(io_device);
	struct spdk_bdev_channel	*ch = ctx_buf;
	struct spdk_io_channel		*mgmt_io_ch;
	struct spdk_bdev_mgmt_channel	*mgmt_ch;
	struct spdk_bdev_shared_resource *shared_resource;

	ch->bdev = bdev;

    // zhou: set underlying disk's I/O channel, by e.g. bdev_aio_get_io_channel(),
	ch->channel = bdev->fn_table->get_io_channel(bdev->ctxt);
	if (!ch->channel) {
		return -1;
	}

	assert(ch->histogram == NULL);

	if (bdev->internal.histogram_enabled) {
		ch->histogram = spdk_histogram_data_alloc();
		if (ch->histogram == NULL) {
			SPDK_ERRLOG("Could not allocate histogram\n");
		}
	}

    // zhou: get I/O device "struct spdk_bdev_mgr" I/O channel of this thread.
	mgmt_io_ch = spdk_get_io_channel(&g_bdev_mgr);
	if (!mgmt_io_ch) {
		spdk_put_io_channel(ch->channel);
		return -1;
	}

    // zhou: get "g_bdev_mgr" I/O channel's private workspace.
	mgmt_ch = spdk_io_channel_get_ctx(mgmt_io_ch);

	TAILQ_FOREACH(shared_resource, &mgmt_ch->shared_resources, link) {

        // zhou: underlying device's I/O channel
		if (shared_resource->shared_ch == ch->channel) {
            // zhou: here means "I will not keep 'mgmt_io_ch' it anymore in this case."
            //       Because it's enough to keep it in "shared_resource->mgmt_ch" at
            //       first time.
			spdk_put_io_channel(mgmt_io_ch);
			shared_resource->ref++;
			break;
		}
	}

    // zhou:
	if (shared_resource == NULL) {

		shared_resource = calloc(1, sizeof(*shared_resource));
		if (shared_resource == NULL) {
			spdk_put_io_channel(ch->channel);
			spdk_put_io_channel(mgmt_io_ch);
			return -1;
		}

		shared_resource->mgmt_ch = mgmt_ch;
		shared_resource->io_outstanding = 0;

		TAILQ_INIT(&shared_resource->nomem_io);

		shared_resource->nomem_threshold = 0;
		shared_resource->shared_ch = ch->channel;
		shared_resource->ref = 1;

		TAILQ_INSERT_TAIL(&mgmt_ch->shared_resources, shared_resource, link);
	}

	memset(&ch->stat, 0, sizeof(ch->stat));
	ch->stat.ticks_rate = spdk_get_ticks_hz();
	ch->io_outstanding = 0;

	TAILQ_INIT(&ch->queued_resets);
	ch->flags = 0;
	ch->shared_resource = shared_resource;

#ifdef SPDK_CONFIG_VTUNE
	{
		char *name;
		__itt_init_ittlib(NULL, 0);
		name = spdk_sprintf_alloc("spdk_bdev_%s_%p", ch->bdev->name, ch);
		if (!name) {
			_spdk_bdev_channel_destroy_resource(ch);
			return -1;
		}
		ch->handle = __itt_string_handle_create(name);
		free(name);
		ch->start_tsc = spdk_get_ticks();
		ch->interval_tsc = spdk_get_ticks_hz() / 100;
		memset(&ch->prev_stat, 0, sizeof(ch->prev_stat));
	}
#endif

	pthread_mutex_lock(&bdev->internal.mutex);
	_spdk_bdev_enable_qos(bdev, ch);
	pthread_mutex_unlock(&bdev->internal.mutex);

	return 0;
}

// zhou:
/*
 * Abort I/O that are waiting on a data buffer.  These types of I/O are
 *  linked using the spdk_bdev_io internal.buf_link TAILQ_ENTRY.
 */
static void
_spdk_bdev_abort_buf_io(bdev_io_stailq_t *queue, struct spdk_bdev_channel *ch)
{
	bdev_io_stailq_t tmp;
	struct spdk_bdev_io *bdev_io;

	STAILQ_INIT(&tmp);

	while (!STAILQ_EMPTY(queue)) {
		bdev_io = STAILQ_FIRST(queue);
		STAILQ_REMOVE_HEAD(queue, internal.buf_link);
		if (bdev_io->internal.ch == ch) {
			spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		} else {
			STAILQ_INSERT_TAIL(&tmp, bdev_io, internal.buf_link);
		}
	}

	STAILQ_SWAP(&tmp, queue, spdk_bdev_io);
}

// zhou: ABORT IO in queue.
/*
 * Abort I/O that are queued waiting for submission.  These types of I/O are
 *  linked using the spdk_bdev_io link TAILQ_ENTRY.
 */
static void
_spdk_bdev_abort_queued_io(bdev_io_tailq_t *queue, struct spdk_bdev_channel *ch)
{
	struct spdk_bdev_io *bdev_io, *tmp;

	TAILQ_FOREACH_SAFE(bdev_io, queue, internal.link, tmp) {
		if (bdev_io->internal.ch == ch) {
			TAILQ_REMOVE(queue, bdev_io, internal.link);
			/*
			 * spdk_bdev_io_complete() assumes that the completed I/O had
			 *  been submitted to the bdev module.  Since in this case it
			 *  hadn't, bump io_outstanding to account for the decrement
			 *  that spdk_bdev_io_complete() will do.
			 */
			if (bdev_io->type != SPDK_BDEV_IO_TYPE_RESET) {
				ch->io_outstanding++;
				ch->shared_resource->io_outstanding++;
			}
			spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		}
	}
}

static void
spdk_bdev_qos_channel_destroy(void *cb_arg)
{
	struct spdk_bdev_qos *qos = cb_arg;

	spdk_put_io_channel(spdk_io_channel_from_ctx(qos->ch));
	spdk_poller_unregister(&qos->poller);

	SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Free QoS %p.\n", qos);

	free(qos);
}

static int
spdk_bdev_qos_destroy(struct spdk_bdev *bdev)
{
	int i;

	/*
	 * Cleanly shutting down the QoS poller is tricky, because
	 * during the asynchronous operation the user could open
	 * a new descriptor and create a new channel, spawning
	 * a new QoS poller.
	 *
	 * The strategy is to create a new QoS structure here and swap it
	 * in. The shutdown path then continues to refer to the old one
	 * until it completes and then releases it.
	 */
	struct spdk_bdev_qos *new_qos, *old_qos;

	old_qos = bdev->internal.qos;

	new_qos = calloc(1, sizeof(*new_qos));
	if (!new_qos) {
		SPDK_ERRLOG("Unable to allocate memory to shut down QoS.\n");
		return -ENOMEM;
	}

	/* Copy the old QoS data into the newly allocated structure */
	memcpy(new_qos, old_qos, sizeof(*new_qos));

	/* Zero out the key parts of the QoS structure */
	new_qos->ch = NULL;
	new_qos->thread = NULL;
	new_qos->poller = NULL;
	TAILQ_INIT(&new_qos->queued);
	/*
	 * The limit member of spdk_bdev_qos_limit structure is not zeroed.
	 * It will be used later for the new QoS structure.
	 */
	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
		new_qos->rate_limits[i].remaining_this_timeslice = 0;
		new_qos->rate_limits[i].min_per_timeslice = 0;
		new_qos->rate_limits[i].max_per_timeslice = 0;
	}

	bdev->internal.qos = new_qos;

	if (old_qos->thread == NULL) {
		free(old_qos);
	} else {
		spdk_thread_send_msg(old_qos->thread, spdk_bdev_qos_channel_destroy,
				     old_qos);
	}

	/* It is safe to continue with destroying the bdev even though the QoS channel hasn't
	 * been destroyed yet. The destruction path will end up waiting for the final
	 * channel to be put before it releases resources. */

	return 0;
}

static void
_spdk_bdev_io_stat_add(struct spdk_bdev_io_stat *total, struct spdk_bdev_io_stat *add)
{
	total->bytes_read += add->bytes_read;
	total->num_read_ops += add->num_read_ops;
	total->bytes_written += add->bytes_written;
	total->num_write_ops += add->num_write_ops;
	total->bytes_unmapped += add->bytes_unmapped;
	total->num_unmap_ops += add->num_unmap_ops;
	total->read_latency_ticks += add->read_latency_ticks;
	total->write_latency_ticks += add->write_latency_ticks;
	total->unmap_latency_ticks += add->unmap_latency_ticks;
}

static void
spdk_bdev_channel_destroy(void *io_device, void *ctx_buf)
{
	struct spdk_bdev_channel	*ch = ctx_buf;
	struct spdk_bdev_mgmt_channel	*mgmt_ch;
	struct spdk_bdev_shared_resource *shared_resource = ch->shared_resource;

	SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Destroying channel %p for bdev %s on thread %p\n", ch, ch->bdev->name,
		      spdk_get_thread());

	/* This channel is going away, so add its statistics into the bdev so that they don't get lost. */
	pthread_mutex_lock(&ch->bdev->internal.mutex);
	_spdk_bdev_io_stat_add(&ch->bdev->internal.stat, &ch->stat);
	pthread_mutex_unlock(&ch->bdev->internal.mutex);

	mgmt_ch = shared_resource->mgmt_ch;

	_spdk_bdev_abort_queued_io(&ch->queued_resets, ch);
	_spdk_bdev_abort_queued_io(&shared_resource->nomem_io, ch);

	_spdk_bdev_abort_buf_io(&mgmt_ch->need_buf_small, ch);
	_spdk_bdev_abort_buf_io(&mgmt_ch->need_buf_large, ch);

	if (ch->histogram) {
		spdk_histogram_data_free(ch->histogram);
	}

	_spdk_bdev_channel_destroy_resource(ch);
}

// zhou: "Add alias to block device names list.
//        Aliases can be add only to registered bdev."
int
spdk_bdev_alias_add(struct spdk_bdev *bdev, const char *alias)
{
	struct spdk_bdev_alias *tmp;

	if (alias == NULL) {
		SPDK_ERRLOG("Empty alias passed\n");
		return -EINVAL;
	}

	if (spdk_bdev_get_by_name(alias)) {
		SPDK_ERRLOG("Bdev name/alias: %s already exists\n", alias);
		return -EEXIST;
	}

	tmp = calloc(1, sizeof(*tmp));
	if (tmp == NULL) {
		SPDK_ERRLOG("Unable to allocate alias\n");
		return -ENOMEM;
	}

	tmp->alias = strdup(alias);
	if (tmp->alias == NULL) {
		free(tmp);
		SPDK_ERRLOG("Unable to allocate alias\n");
		return -ENOMEM;
	}

	TAILQ_INSERT_TAIL(&bdev->aliases, tmp, tailq);

	return 0;
}

int
spdk_bdev_alias_del(struct spdk_bdev *bdev, const char *alias)
{
	struct spdk_bdev_alias *tmp;

	TAILQ_FOREACH(tmp, &bdev->aliases, tailq) {
		if (strcmp(alias, tmp->alias) == 0) {
			TAILQ_REMOVE(&bdev->aliases, tmp, tailq);
			free(tmp->alias);
			free(tmp);
			return 0;
		}
	}

	SPDK_INFOLOG(SPDK_LOG_BDEV, "Alias %s does not exists\n", alias);

	return -ENOENT;
}

void
spdk_bdev_alias_del_all(struct spdk_bdev *bdev)
{
	struct spdk_bdev_alias *p, *tmp;

	TAILQ_FOREACH_SAFE(p, &bdev->aliases, tailq, tmp) {
		TAILQ_REMOVE(&bdev->aliases, p, tailq);
		free(p->alias);
		free(p);
	}
}

// zhou: get bdev I/O channel
struct spdk_io_channel *
spdk_bdev_get_io_channel(struct spdk_bdev_desc *desc)
{
    // zhou: fetch bdev underlying disk's I/O channel.
	return spdk_get_io_channel(__bdev_to_io_dev(desc->bdev));
}

const char *
spdk_bdev_get_name(const struct spdk_bdev *bdev)
{
	return bdev->name;
}

const char *
spdk_bdev_get_product_name(const struct spdk_bdev *bdev)
{
	return bdev->product_name;
}

const struct spdk_bdev_aliases_list *
spdk_bdev_get_aliases(const struct spdk_bdev *bdev)
{
	return &bdev->aliases;
}

uint32_t
spdk_bdev_get_block_size(const struct spdk_bdev *bdev)
{
	return bdev->blocklen;
}

// zhou: bdev size in block number
uint64_t
spdk_bdev_get_num_blocks(const struct spdk_bdev *bdev)
{
	return bdev->blockcnt;
}

const char *
spdk_bdev_get_qos_rpc_type(enum spdk_bdev_qos_rate_limit_type type)
{
	return qos_rpc_type[type];
}

void
spdk_bdev_get_qos_rate_limits(struct spdk_bdev *bdev, uint64_t *limits)
{
	int i;

	memset(limits, 0, sizeof(*limits) * SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES);

	pthread_mutex_lock(&bdev->internal.mutex);
	if (bdev->internal.qos) {
		for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
			if (bdev->internal.qos->rate_limits[i].limit !=
			    SPDK_BDEV_QOS_LIMIT_NOT_DEFINED) {
				limits[i] = bdev->internal.qos->rate_limits[i].limit;
				if (_spdk_bdev_qos_is_iops_rate_limit(i) == false) {
					/* Change from Byte to Megabyte which is user visible. */
					limits[i] = limits[i] / 1024 / 1024;
				}
			}
		}
	}
	pthread_mutex_unlock(&bdev->internal.mutex);
}

size_t
spdk_bdev_get_buf_align(const struct spdk_bdev *bdev)
{
	return 1 << bdev->required_alignment;
}

uint32_t
spdk_bdev_get_optimal_io_boundary(const struct spdk_bdev *bdev)
{
	return bdev->optimal_io_boundary;
}

bool
spdk_bdev_has_write_cache(const struct spdk_bdev *bdev)
{
	return bdev->write_cache;
}

const struct spdk_uuid *
spdk_bdev_get_uuid(const struct spdk_bdev *bdev)
{
	return &bdev->uuid;
}

uint32_t
spdk_bdev_get_md_size(const struct spdk_bdev *bdev)
{
	return bdev->md_len;
}

bool
spdk_bdev_is_md_interleaved(const struct spdk_bdev *bdev)
{
	return (bdev->md_len != 0) && bdev->md_interleave;
}

uint32_t
spdk_bdev_get_data_block_size(const struct spdk_bdev *bdev)
{
	if (spdk_bdev_is_md_interleaved(bdev)) {
		return bdev->blocklen - bdev->md_len;
	} else {
		return bdev->blocklen;
	}
}

enum spdk_dif_type spdk_bdev_get_dif_type(const struct spdk_bdev *bdev)
{
	if (bdev->md_len != 0) {
		return bdev->dif_type;
	} else {
		return SPDK_DIF_DISABLE;
	}
}

bool
spdk_bdev_is_dif_head_of_md(const struct spdk_bdev *bdev)
{
	if (spdk_bdev_get_dif_type(bdev) != SPDK_DIF_DISABLE) {
		return bdev->dif_is_head_of_md;
	} else {
		return false;
	}
}

bool
spdk_bdev_is_dif_check_enabled(const struct spdk_bdev *bdev,
			       enum spdk_dif_check_type check_type)
{
	if (spdk_bdev_get_dif_type(bdev) == SPDK_DIF_DISABLE) {
		return false;
	}

	switch (check_type) {
	case SPDK_DIF_CHECK_TYPE_REFTAG:
		return (bdev->dif_check_flags & SPDK_DIF_FLAGS_REFTAG_CHECK) != 0;
	case SPDK_DIF_CHECK_TYPE_APPTAG:
		return (bdev->dif_check_flags & SPDK_DIF_FLAGS_APPTAG_CHECK) != 0;
	case SPDK_DIF_CHECK_TYPE_GUARD:
		return (bdev->dif_check_flags & SPDK_DIF_FLAGS_GUARD_CHECK) != 0;
	default:
		return false;
	}
}

uint64_t
spdk_bdev_get_qd(const struct spdk_bdev *bdev)
{
	return bdev->internal.measured_queue_depth;
}

uint64_t
spdk_bdev_get_qd_sampling_period(const struct spdk_bdev *bdev)
{
	return bdev->internal.period;
}

uint64_t
spdk_bdev_get_weighted_io_time(const struct spdk_bdev *bdev)
{
	return bdev->internal.weighted_io_time;
}

uint64_t
spdk_bdev_get_io_time(const struct spdk_bdev *bdev)
{
	return bdev->internal.io_time;
}

static void
_calculate_measured_qd_cpl(struct spdk_io_channel_iter *i, int status)
{
	struct spdk_bdev *bdev = spdk_io_channel_iter_get_ctx(i);

	bdev->internal.measured_queue_depth = bdev->internal.temporary_queue_depth;

	if (bdev->internal.measured_queue_depth) {
		bdev->internal.io_time += bdev->internal.period;
		bdev->internal.weighted_io_time += bdev->internal.period * bdev->internal.measured_queue_depth;
	}
}

static void
_calculate_measured_qd(struct spdk_io_channel_iter *i)
{
	struct spdk_bdev *bdev = spdk_io_channel_iter_get_ctx(i);
	struct spdk_io_channel *io_ch = spdk_io_channel_iter_get_channel(i);
	struct spdk_bdev_channel *ch = spdk_io_channel_get_ctx(io_ch);

    // zhou: collect outstanding IO number in different I/O channel.
	bdev->internal.temporary_queue_depth += ch->io_outstanding;
	spdk_for_each_channel_continue(i, 0);
}

static int
spdk_bdev_calculate_measured_queue_depth(void *ctx)
{
	struct spdk_bdev *bdev = ctx;
	bdev->internal.temporary_queue_depth = 0;

	spdk_for_each_channel(__bdev_to_io_dev(bdev), _calculate_measured_qd, bdev,
			      _calculate_measured_qd_cpl);
	return 0;
}

void
spdk_bdev_set_qd_sampling_period(struct spdk_bdev *bdev, uint64_t period)
{
	bdev->internal.period = period;

	if (bdev->internal.qd_poller != NULL) {
		spdk_poller_unregister(&bdev->internal.qd_poller);
		bdev->internal.measured_queue_depth = UINT64_MAX;
	}

	if (period != 0) {
		bdev->internal.qd_poller = spdk_poller_register(spdk_bdev_calculate_measured_queue_depth, bdev,
					   period);
	}
}

int
spdk_bdev_notify_blockcnt_change(struct spdk_bdev *bdev, uint64_t size)
{
	int ret;

	pthread_mutex_lock(&bdev->internal.mutex);

	/* bdev has open descriptors */
	if (!TAILQ_EMPTY(&bdev->internal.open_descs) &&
	    bdev->blockcnt > size) {
		ret = -EBUSY;
	} else {
		bdev->blockcnt = size;
		ret = 0;
	}

	pthread_mutex_unlock(&bdev->internal.mutex);

	return ret;
}


////////////////////////////////////////////////////////////////////////////////

// zhou: "offset_blocks", "num_blocks" are output parameters
/*
 * Convert I/O offset and length from bytes to blocks.
 *
 * Returns zero on success or non-zero if the byte parameters aren't divisible by the block size.
 */
static uint64_t
spdk_bdev_bytes_to_blocks(struct spdk_bdev *bdev, uint64_t offset_bytes, uint64_t *offset_blocks,
			  uint64_t num_bytes, uint64_t *num_blocks)
{
	uint32_t block_size = bdev->blocklen;
	uint8_t shift_cnt;

	/* Avoid expensive div operations if possible. These spdk_u32 functions are very cheap. */
	if (spdk_likely(spdk_u32_is_pow2(block_size))) {

		shift_cnt = spdk_u32log2(block_size);
		*offset_blocks = offset_bytes >> shift_cnt;
		*num_blocks = num_bytes >> shift_cnt;
		return (offset_bytes - (*offset_blocks << shift_cnt)) |
		       (num_bytes - (*num_blocks << shift_cnt));

	} else {

		*offset_blocks = offset_bytes / block_size;
		*num_blocks = num_bytes / block_size;

		return (offset_bytes % block_size) | (num_bytes % block_size);
	}
}

// zhou: check whether the device has so many blocks.
static bool
spdk_bdev_io_valid_blocks(struct spdk_bdev *bdev, uint64_t offset_blocks, uint64_t num_blocks)
{
	/* Return failure if offset_blocks + num_blocks is less than offset_blocks; indicates there
	 * has been an overflow and hence the offset has been wrapped around */
	if (offset_blocks + num_blocks < offset_blocks) {
		return false;
	}

	/* Return failure if offset_blocks + num_blocks exceeds the size of the bdev */
	if (offset_blocks + num_blocks > bdev->blockcnt) {
		return false;
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////////
// zhou: START of READ/WRITE/UNMAP/RESET API

// zhou:
/*
  "Once a descriptor and a channel have been obtained, I/O may be sent by
  calling the various I/O submission functions such as spdk_bdev_read(). These
  calls each take a callback as an argument which will be called some time
  later with a handle to an spdk_bdev_io object.
  In response to that completion, the user must call spdk_bdev_free_io() to
  release the resources. Within this callback, the user may also use the
  functions spdk_bdev_io_get_nvme_status() and spdk_bdev_io_get_scsi_status()
  to obtain error information in the format of their choosing."

  "I/O submission is performed by calling functions such as spdk_bdev_read()
  or spdk_bdev_write(). These functions take as an argument a pointer to a region
  of memory or a scatter gather list describing memory that will be transferred to
  the block device. This memory must be allocated through spdk_dma_malloc() or its
  variants. For a full explanation of why the memory must come from a special
  allocation pool, see Memory Management for User Space Drivers. Where possible,
  data in memory will be directly transferred to the block device using Direct
  Memory Access. That means it is not copied."

  "All I/O submission functions are asynchronous and non-blocking. They will not
  block or stall the thread for any reason. However, the I/O submission functions
  may fail in one of two ways. First, they may fail immediately and return an error
  code. In that case, the provided callback will not be called. Second, they may
  fail asynchronously. In that case, the associated spdk_bdev_io will be passed to
  the callback and it will report error information."
*/

// zhou: buffer provided by client in form of pointer.
//       Target information, should be convert from byte to block.
int
spdk_bdev_read(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
	       void *buf, uint64_t offset, uint64_t nbytes,
	       spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	uint64_t offset_blocks, num_blocks;

    // zhou: convert from byte to block num, just consider block size of the device.
	if (spdk_bdev_bytes_to_blocks(desc->bdev, offset, &offset_blocks, nbytes, &num_blocks) != 0) {
		return -EINVAL;
	}

	return spdk_bdev_read_blocks(desc, ch, buf, offset_blocks, num_blocks, cb, cb_arg);
}

// zhou: buffer provided by client in form of pointer.
//
//       spdk_bdev_io_submit() -> _spdk_bdev_io_submit() -> fn_table->submit_request()
//       -> invoke each kind of backend storage registered callback function.
//
//       AIO: bdev_aio_submit_request()
//       NVMe: bdev_nvme_submit_request()
int
spdk_bdev_read_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		      void *buf, uint64_t offset_blocks, uint64_t num_blocks,
		      spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;

    // zhou: "ch" is a bdev I/O channel
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!spdk_bdev_io_valid_blocks(bdev, offset_blocks, num_blocks)) {
		return -EINVAL;
	}

    // zhou: get a empty IO request, once no resource right now, give up immediately.
	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_READ;

	bdev_io->u.bdev.iovs = &bdev_io->iov;
	bdev_io->u.bdev.iovs[0].iov_base = buf;
	bdev_io->u.bdev.iovs[0].iov_len = num_blocks * bdev->blocklen;
	bdev_io->u.bdev.iovcnt = 1;

	bdev_io->u.bdev.num_blocks = num_blocks;
	bdev_io->u.bdev.offset_blocks = offset_blocks;

	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	spdk_bdev_io_submit(bdev_io);

	return 0;
}

// zhou: buffer provided by client in form of IOV.
//       Target information, should be convert from byte to block.
int
spdk_bdev_readv(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		struct iovec *iov, int iovcnt,
		uint64_t offset, uint64_t nbytes,
		spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	uint64_t offset_blocks, num_blocks;

	if (spdk_bdev_bytes_to_blocks(desc->bdev, offset, &offset_blocks, nbytes, &num_blocks) != 0) {
		return -EINVAL;
	}

	return spdk_bdev_readv_blocks(desc, ch, iov, iovcnt, offset_blocks, num_blocks, cb, cb_arg);
}

// zhou: buffer provided by client in form of IOV.
int spdk_bdev_readv_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
			   struct iovec *iov, int iovcnt,
			   uint64_t offset_blocks, uint64_t num_blocks,
			   spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!spdk_bdev_io_valid_blocks(bdev, offset_blocks, num_blocks)) {
		return -EINVAL;
	}

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;

	bdev_io->type = SPDK_BDEV_IO_TYPE_READ;

	bdev_io->u.bdev.iovs = iov;
	bdev_io->u.bdev.iovcnt = iovcnt;

	bdev_io->u.bdev.num_blocks = num_blocks;
	bdev_io->u.bdev.offset_blocks = offset_blocks;

	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	spdk_bdev_io_submit(bdev_io);
	return 0;
}


int
spdk_bdev_write(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		void *buf, uint64_t offset, uint64_t nbytes,
		spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	uint64_t offset_blocks, num_blocks;

	if (spdk_bdev_bytes_to_blocks(desc->bdev, offset, &offset_blocks, nbytes, &num_blocks) != 0) {
		return -EINVAL;
	}

	return spdk_bdev_write_blocks(desc, ch, buf, offset_blocks, num_blocks, cb, cb_arg);
}

// zhou: "buf", the data to be wrote -> "offset_blocks", "num_blocks", LBA
int
spdk_bdev_write_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		       void *buf, uint64_t offset_blocks, uint64_t num_blocks,
		       spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

    // zhou: this bdev is readonly.
	if (!desc->write) {
		return -EBADF;
	}

    // zhou: targe LBA is valid for this bdev.
	if (!spdk_bdev_io_valid_blocks(bdev, offset_blocks, num_blocks)) {
		return -EINVAL;
	}

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_WRITE;

	bdev_io->u.bdev.iovs = &bdev_io->iov;

	bdev_io->u.bdev.iovs[0].iov_base = buf;
	bdev_io->u.bdev.iovs[0].iov_len = num_blocks * bdev->blocklen;
	bdev_io->u.bdev.iovcnt = 1;

	bdev_io->u.bdev.num_blocks = num_blocks;
	bdev_io->u.bdev.offset_blocks = offset_blocks;

	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	spdk_bdev_io_submit(bdev_io);
	return 0;
}

int
spdk_bdev_writev(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		 struct iovec *iov, int iovcnt,
		 uint64_t offset, uint64_t len,
		 spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	uint64_t offset_blocks, num_blocks;

	if (spdk_bdev_bytes_to_blocks(desc->bdev, offset, &offset_blocks, len, &num_blocks) != 0) {
		return -EINVAL;
	}

	return spdk_bdev_writev_blocks(desc, ch, iov, iovcnt, offset_blocks, num_blocks, cb, cb_arg);
}

int
spdk_bdev_writev_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
			struct iovec *iov, int iovcnt,
			uint64_t offset_blocks, uint64_t num_blocks,
			spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!desc->write) {
		return -EBADF;
	}

	if (!spdk_bdev_io_valid_blocks(bdev, offset_blocks, num_blocks)) {
		return -EINVAL;
	}

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_WRITE;
	bdev_io->u.bdev.iovs = iov;
	bdev_io->u.bdev.iovcnt = iovcnt;
	bdev_io->u.bdev.num_blocks = num_blocks;
	bdev_io->u.bdev.offset_blocks = offset_blocks;
	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	spdk_bdev_io_submit(bdev_io);
	return 0;
}


// zhou: END of READ/WRITE API
////////////////////////////////////////////////////////////////////////////////
// zhou: START of ZCOPY API

static void
bdev_zcopy_get_buf(struct spdk_io_channel *ch, struct spdk_bdev_io *bdev_io, bool success)
{
	if (!success) {
		/* Don't use spdk_bdev_io_complete here - this bdev_io was never actually submitted. */
		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_NOMEM;
		bdev_io->internal.cb(bdev_io, success, bdev_io->internal.caller_ctx);
		return;
	}

	if (bdev_io->u.bdev.zcopy.populate) {
		/* Read the real data into the buffer */
		bdev_io->type = SPDK_BDEV_IO_TYPE_READ;
		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_PENDING;
		spdk_bdev_io_submit(bdev_io);
		return;
	}

	/* Don't use spdk_bdev_io_complete here - this bdev_io was never actually submitted. */
	bdev_io->internal.status = SPDK_BDEV_IO_STATUS_SUCCESS;
	bdev_io->internal.cb(bdev_io, success, bdev_io->internal.caller_ctx);
}

int
spdk_bdev_zcopy_start(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		      uint64_t offset_blocks, uint64_t num_blocks,
		      bool populate,
		      spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!desc->write) {
		return -EBADF;
	}

	if (!spdk_bdev_io_valid_blocks(bdev, offset_blocks, num_blocks)) {
		return -EINVAL;
	}

	if (!spdk_bdev_io_type_supported(bdev, SPDK_BDEV_IO_TYPE_ZCOPY)) {
		return -ENOTSUP;
	}

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_ZCOPY;
	bdev_io->u.bdev.num_blocks = num_blocks;
	bdev_io->u.bdev.offset_blocks = offset_blocks;
	bdev_io->u.bdev.iovs = NULL;
	bdev_io->u.bdev.iovcnt = 0;
	bdev_io->u.bdev.zcopy.populate = populate ? 1 : 0;
	bdev_io->u.bdev.zcopy.commit = 0;
	bdev_io->u.bdev.zcopy.start = 1;
	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	if (_spdk_bdev_io_type_supported(bdev, SPDK_BDEV_IO_TYPE_ZCOPY)) {
		spdk_bdev_io_submit(bdev_io);
	} else {
		/* Emulate zcopy by allocating a buffer */
		spdk_bdev_io_get_buf(bdev_io, bdev_zcopy_get_buf,
				     bdev_io->u.bdev.num_blocks * bdev->blocklen);
	}

	return 0;
}

int
spdk_bdev_zcopy_end(struct spdk_bdev_io *bdev_io, bool commit,
		    spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = bdev_io->bdev;

	if (bdev_io->type == SPDK_BDEV_IO_TYPE_READ) {
		/* This can happen if the zcopy was emulated in start */
		if (bdev_io->u.bdev.zcopy.start != 1) {
			return -EINVAL;
		}
		bdev_io->type = SPDK_BDEV_IO_TYPE_ZCOPY;
	}

	if (bdev_io->type != SPDK_BDEV_IO_TYPE_ZCOPY) {
		return -EINVAL;
	}

	bdev_io->u.bdev.zcopy.commit = commit ? 1 : 0;
	bdev_io->u.bdev.zcopy.start = 0;
	bdev_io->internal.caller_ctx = cb_arg;
	bdev_io->internal.cb = cb;
	bdev_io->internal.status = SPDK_BDEV_IO_STATUS_PENDING;

	if (_spdk_bdev_io_type_supported(bdev, SPDK_BDEV_IO_TYPE_ZCOPY)) {
		spdk_bdev_io_submit(bdev_io);
		return 0;
	}

	if (!bdev_io->u.bdev.zcopy.commit) {
		/* Don't use spdk_bdev_io_complete here - this bdev_io was never actually submitted. */
		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_SUCCESS;
		bdev_io->internal.cb(bdev_io, true, bdev_io->internal.caller_ctx);
		return 0;
	}

	bdev_io->type = SPDK_BDEV_IO_TYPE_WRITE;
	spdk_bdev_io_submit(bdev_io);

	return 0;
}

// zhou: END of ZCOPY API
////////////////////////////////////////////////////////////////////////////////
// zhou: START of WRITE ZERO API

int
spdk_bdev_write_zeroes(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		       uint64_t offset, uint64_t len,
		       spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	uint64_t offset_blocks, num_blocks;

	if (spdk_bdev_bytes_to_blocks(desc->bdev, offset, &offset_blocks, len, &num_blocks) != 0) {
		return -EINVAL;
	}

	return spdk_bdev_write_zeroes_blocks(desc, ch, offset_blocks, num_blocks, cb, cb_arg);
}

int
spdk_bdev_write_zeroes_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
			      uint64_t offset_blocks, uint64_t num_blocks,
			      spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!desc->write) {
		return -EBADF;
	}

	if (!spdk_bdev_io_valid_blocks(bdev, offset_blocks, num_blocks)) {
		return -EINVAL;
	}

	bdev_io = spdk_bdev_get_io(channel);

	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->type = SPDK_BDEV_IO_TYPE_WRITE_ZEROES;
	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->u.bdev.offset_blocks = offset_blocks;
	bdev_io->u.bdev.num_blocks = num_blocks;
	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	if (_spdk_bdev_io_type_supported(bdev, SPDK_BDEV_IO_TYPE_WRITE_ZEROES)) {
		spdk_bdev_io_submit(bdev_io);
		return 0;
	} else if (_spdk_bdev_io_type_supported(bdev, SPDK_BDEV_IO_TYPE_WRITE)) {
		assert(spdk_bdev_get_block_size(bdev) <= ZERO_BUFFER_SIZE);
		bdev_io->u.bdev.split_remaining_num_blocks = num_blocks;
		bdev_io->u.bdev.split_current_offset_blocks = offset_blocks;
		_spdk_bdev_write_zero_buffer_next(bdev_io);
		return 0;
	} else {
		spdk_bdev_free_io(bdev_io);
		return -ENOTSUP;
	}
}

// zhou: END of WRITE ZERO API
////////////////////////////////////////////////////////////////////////////////
// zhou: START of UNMAP API

int
spdk_bdev_unmap(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		uint64_t offset, uint64_t nbytes,
		spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	uint64_t offset_blocks, num_blocks;

	if (spdk_bdev_bytes_to_blocks(desc->bdev, offset, &offset_blocks, nbytes, &num_blocks) != 0) {
		return -EINVAL;
	}

	return spdk_bdev_unmap_blocks(desc, ch, offset_blocks, num_blocks, cb, cb_arg);
}

int
spdk_bdev_unmap_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		       uint64_t offset_blocks, uint64_t num_blocks,
		       spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!desc->write) {
		return -EBADF;
	}

	if (!spdk_bdev_io_valid_blocks(bdev, offset_blocks, num_blocks)) {
		return -EINVAL;
	}

	if (num_blocks == 0) {
		SPDK_ERRLOG("Can't unmap 0 bytes\n");
		return -EINVAL;
	}

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_UNMAP;

	bdev_io->u.bdev.iovs = &bdev_io->iov;
	bdev_io->u.bdev.iovs[0].iov_base = NULL;
	bdev_io->u.bdev.iovs[0].iov_len = 0;
	bdev_io->u.bdev.iovcnt = 1;

	bdev_io->u.bdev.offset_blocks = offset_blocks;
	bdev_io->u.bdev.num_blocks = num_blocks;
	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	spdk_bdev_io_submit(bdev_io);
	return 0;
}

// zhou: END of UNMAP API
////////////////////////////////////////////////////////////////////////////////
// zhou: START of FLUSH API

int
spdk_bdev_flush(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		uint64_t offset, uint64_t length,
		spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	uint64_t offset_blocks, num_blocks;

	if (spdk_bdev_bytes_to_blocks(desc->bdev, offset, &offset_blocks, length, &num_blocks) != 0) {
		return -EINVAL;
	}

	return spdk_bdev_flush_blocks(desc, ch, offset_blocks, num_blocks, cb, cb_arg);
}

// zhou:
int
spdk_bdev_flush_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		       uint64_t offset_blocks, uint64_t num_blocks,
		       spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!desc->write) {
		return -EBADF;
	}

	if (!spdk_bdev_io_valid_blocks(bdev, offset_blocks, num_blocks)) {
		return -EINVAL;
	}

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_FLUSH;
	bdev_io->u.bdev.iovs = NULL;
	bdev_io->u.bdev.iovcnt = 0;
	bdev_io->u.bdev.offset_blocks = offset_blocks;
	bdev_io->u.bdev.num_blocks = num_blocks;

	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	spdk_bdev_io_submit(bdev_io);
	return 0;
}

// zhou: END of FLUSH API
////////////////////////////////////////////////////////////////////////////////
// zhou: START of RESET API


// zhou: all thread/channel be notified that IO was froze.
static void
_spdk_bdev_reset_dev(struct spdk_io_channel_iter *i, int status)
{
	struct spdk_bdev_channel *ch = spdk_io_channel_iter_get_ctx(i);
	struct spdk_bdev_io *bdev_io;

	bdev_io = TAILQ_FIRST(&ch->queued_resets);
	TAILQ_REMOVE(&ch->queued_resets, bdev_io, internal.link);

    // zhou: unlike other IO which could be performed on all channel in parallal.
    //       RESET will be performed on one thread/channel only.
	spdk_bdev_io_submit_reset(bdev_io);
}

// zhou: freeze to prepare reset underlying device in this channel.
static void
_spdk_bdev_reset_freeze_channel(struct spdk_io_channel_iter *i)
{
	struct spdk_io_channel		*ch;
	struct spdk_bdev_channel	*channel;
	struct spdk_bdev_mgmt_channel	*mgmt_channel;
	struct spdk_bdev_shared_resource *shared_resource;
	bdev_io_tailq_t			tmp_queued;

	TAILQ_INIT(&tmp_queued);

	ch = spdk_io_channel_iter_get_channel(i);
	channel = spdk_io_channel_get_ctx(ch);
	shared_resource = channel->shared_resource;n
	mgmt_channel = shared_resource->mgmt_ch;

	channel->flags |= BDEV_CH_RESET_IN_PROGRESS;

	if ((channel->flags & BDEV_CH_QOS_ENABLED) != 0) {
		/* The QoS object is always valid and readable while
		 * the channel flag is set, so the lock here should not
		 * be necessary. We're not in the fast path though, so
		 * just take it anyway. */
		pthread_mutex_lock(&channel->bdev->internal.mutex);
		if (channel->bdev->internal.qos->ch == channel) {
			TAILQ_SWAP(&channel->bdev->internal.qos->queued, &tmp_queued, spdk_bdev_io, internal.link);
		}
		pthread_mutex_unlock(&channel->bdev->internal.mutex);
	}

    // zhou:
	_spdk_bdev_abort_queued_io(&shared_resource->nomem_io, channel);
	_spdk_bdev_abort_buf_io(&mgmt_channel->need_buf_small, channel);
	_spdk_bdev_abort_buf_io(&mgmt_channel->need_buf_large, channel);
	_spdk_bdev_abort_queued_io(&tmp_queued, channel);

	spdk_for_each_channel_continue(i, 0);
}

// zhou: notify each thread/channel to freeze IO, no more new IO could be submited.
static void
_spdk_bdev_start_reset(void *ctx)
{
	struct spdk_bdev_channel *ch = ctx;

	spdk_for_each_channel(__bdev_to_io_dev(ch->bdev), _spdk_bdev_reset_freeze_channel,
			      ch, _spdk_bdev_reset_dev);
}

static void
_spdk_bdev_channel_start_reset(struct spdk_bdev_channel *ch)
{
	struct spdk_bdev *bdev = ch->bdev;

	assert(!TAILQ_EMPTY(&ch->queued_resets));

	pthread_mutex_lock(&bdev->internal.mutex);
	if (bdev->internal.reset_in_progress == NULL) {
		bdev->internal.reset_in_progress = TAILQ_FIRST(&ch->queued_resets);
		/*
		 * Take a channel reference for the target bdev for the life of this
		 *  reset.  This guards against the channel getting destroyed while
		 *  spdk_for_each_channel() calls related to this reset IO are in
		 *  progress.  We will release the reference when this reset is
		 *  completed.
		 */
		bdev->internal.reset_in_progress->u.reset.ch_ref = spdk_get_io_channel(__bdev_to_io_dev(bdev));
		_spdk_bdev_start_reset(ch);
	}
	pthread_mutex_unlock(&bdev->internal.mutex);
}

// zhou:
/*
  In order to handle unexpected failure conditions, the bdev library provides a
  mechanism to perform a device reset by calling spdk_bdev_reset(). This will pass
  a message to every other thread for which an I/O channel exists for the bdev,
  pause it, then forward a reset request to the underlying bdev module and wait for
  completion. Upon completion, the I/O channels will resume and the reset will
  complete. The specific behavior inside the bdev module is module-specific.
  For example, NVMe devices will delete all queue pairs, perform an NVMe reset,
  then recreate the queue pairs and continue. Most importantly, regardless of
  device type, all I/O outstanding to the block device will be completed prior to
  the reset completing.
*/
int
spdk_bdev_reset(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_RESET;
	bdev_io->u.reset.ch_ref = NULL;

	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	pthread_mutex_lock(&bdev->internal.mutex);
	TAILQ_INSERT_TAIL(&channel->queued_resets, bdev_io, internal.link);
	pthread_mutex_unlock(&bdev->internal.mutex);

	_spdk_bdev_channel_start_reset(channel);

	return 0;
}

// zhou: END of RESET API
////////////////////////////////////////////////////////////////////////////////
// zhou: START of stats

void
spdk_bdev_get_io_stat(struct spdk_bdev *bdev, struct spdk_io_channel *ch,
		      struct spdk_bdev_io_stat *stat)
{
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	*stat = channel->stat;
}

static void
_spdk_bdev_get_device_stat_done(struct spdk_io_channel_iter *i, int status)
{
	void *io_device = spdk_io_channel_iter_get_io_device(i);
	struct spdk_bdev_iostat_ctx *bdev_iostat_ctx = spdk_io_channel_iter_get_ctx(i);

	bdev_iostat_ctx->cb(__bdev_from_io_dev(io_device), bdev_iostat_ctx->stat,
			    bdev_iostat_ctx->cb_arg, 0);
	free(bdev_iostat_ctx);
}

static void
_spdk_bdev_get_each_channel_stat(struct spdk_io_channel_iter *i)
{
	struct spdk_bdev_iostat_ctx *bdev_iostat_ctx = spdk_io_channel_iter_get_ctx(i);
	struct spdk_io_channel *ch = spdk_io_channel_iter_get_channel(i);
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	_spdk_bdev_io_stat_add(bdev_iostat_ctx->stat, &channel->stat);
	spdk_for_each_channel_continue(i, 0);
}

void
spdk_bdev_get_device_stat(struct spdk_bdev *bdev, struct spdk_bdev_io_stat *stat,
			  spdk_bdev_get_device_stat_cb cb, void *cb_arg)
{
	struct spdk_bdev_iostat_ctx *bdev_iostat_ctx;

	assert(bdev != NULL);
	assert(stat != NULL);
	assert(cb != NULL);

	bdev_iostat_ctx = calloc(1, sizeof(struct spdk_bdev_iostat_ctx));
	if (bdev_iostat_ctx == NULL) {
		SPDK_ERRLOG("Unable to allocate memory for spdk_bdev_iostat_ctx\n");
		cb(bdev, stat, cb_arg, -ENOMEM);
		return;
	}

	bdev_iostat_ctx->stat = stat;
	bdev_iostat_ctx->cb = cb;
	bdev_iostat_ctx->cb_arg = cb_arg;

	/* Start with the statistics from previously deleted channels. */
	pthread_mutex_lock(&bdev->internal.mutex);
	_spdk_bdev_io_stat_add(bdev_iostat_ctx->stat, &bdev->internal.stat);
	pthread_mutex_unlock(&bdev->internal.mutex);

	/* Then iterate and add the statistics from each existing channel. */
	spdk_for_each_channel(__bdev_to_io_dev(bdev),
			      _spdk_bdev_get_each_channel_stat,
			      bdev_iostat_ctx,
			      _spdk_bdev_get_device_stat_done);
}

// zhou: END of stats
////////////////////////////////////////////////////////////////////////////////
// zhou: START of NVMe Passthrough

int
spdk_bdev_nvme_admin_passthru(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
			      const struct spdk_nvme_cmd *cmd, void *buf, size_t nbytes,
			      spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!desc->write) {
		return -EBADF;
	}

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_NVME_ADMIN;
	bdev_io->u.nvme_passthru.cmd = *cmd;
	bdev_io->u.nvme_passthru.buf = buf;
	bdev_io->u.nvme_passthru.nbytes = nbytes;
	bdev_io->u.nvme_passthru.md_buf = NULL;
	bdev_io->u.nvme_passthru.md_len = 0;

	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	spdk_bdev_io_submit(bdev_io);
	return 0;
}

int
spdk_bdev_nvme_io_passthru(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
			   const struct spdk_nvme_cmd *cmd, void *buf, size_t nbytes,
			   spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!desc->write) {
		/*
		 * Do not try to parse the NVMe command - we could maybe use bits in the opcode
		 *  to easily determine if the command is a read or write, but for now just
		 *  do not allow io_passthru with a read-only descriptor.
		 */
		return -EBADF;
	}

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_NVME_IO;
	bdev_io->u.nvme_passthru.cmd = *cmd;
	bdev_io->u.nvme_passthru.buf = buf;
	bdev_io->u.nvme_passthru.nbytes = nbytes;
	bdev_io->u.nvme_passthru.md_buf = NULL;
	bdev_io->u.nvme_passthru.md_len = 0;

	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	spdk_bdev_io_submit(bdev_io);
	return 0;
}

int
spdk_bdev_nvme_io_passthru_md(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
			      const struct spdk_nvme_cmd *cmd, void *buf, size_t nbytes, void *md_buf, size_t md_len,
			      spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	struct spdk_bdev *bdev = desc->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);

	if (!desc->write) {
		/*
		 * Do not try to parse the NVMe command - we could maybe use bits in the opcode
		 *  to easily determine if the command is a read or write, but for now just
		 *  do not allow io_passthru with a read-only descriptor.
		 */
		return -EBADF;
	}

	bdev_io = spdk_bdev_get_io(channel);
	if (!bdev_io) {
		return -ENOMEM;
	}

	bdev_io->internal.ch = channel;
	bdev_io->internal.desc = desc;
	bdev_io->type = SPDK_BDEV_IO_TYPE_NVME_IO_MD;
	bdev_io->u.nvme_passthru.cmd = *cmd;
	bdev_io->u.nvme_passthru.buf = buf;
	bdev_io->u.nvme_passthru.nbytes = nbytes;
	bdev_io->u.nvme_passthru.md_buf = md_buf;
	bdev_io->u.nvme_passthru.md_len = md_len;

	spdk_bdev_io_init(bdev_io, bdev, cb_arg, cb);

	spdk_bdev_io_submit(bdev_io);
	return 0;
}

// zhou: END of NVMe Passthrough
////////////////////////////////////////////////////////////////////////////////
// zhou: START of IO management

// zhou: register a callback in "mgmt_ch->io_wait_queue", want to be wake up when
//       some "struct spdk_bdev_io" is avaiable on this thread.
int
spdk_bdev_queue_io_wait(struct spdk_bdev *bdev, struct spdk_io_channel *ch,
			struct spdk_bdev_io_wait_entry *entry)
{
	struct spdk_bdev_channel *channel = spdk_io_channel_get_ctx(ch);
	struct spdk_bdev_mgmt_channel *mgmt_ch = channel->shared_resource->mgmt_ch;

	if (bdev != entry->bdev) {
		SPDK_ERRLOG("bdevs do not match\n");
		return -EINVAL;
	}

	if (mgmt_ch->per_thread_cache_count > 0) {
		SPDK_ERRLOG("Cannot queue io_wait if spdk_bdev_io available in per-thread cache\n");
		return -EINVAL;
	}

	TAILQ_INSERT_TAIL(&mgmt_ch->io_wait_queue, entry, link);
	return 0;
}

// zhou: handle pending I/O in list "shared_resource->nomem_io" when other I/O
//       compeleted without reporting "SPDK_BDEV_IO_STATUS_NOMEM".
static void
_spdk_bdev_ch_retry_io(struct spdk_bdev_channel *bdev_ch)
{
	struct spdk_bdev *bdev = bdev_ch->bdev;
	struct spdk_bdev_shared_resource *shared_resource = bdev_ch->shared_resource;
	struct spdk_bdev_io *bdev_io;

    // zhou: outstanding I/O decreased less than "nomem_threshold"
	if (shared_resource->io_outstanding > shared_resource->nomem_threshold) {
		/*
		 * Allow some more I/O to complete before retrying the nomem_io queue.
		 *  Some drivers (such as nvme) cannot immediately take a new I/O in
		 *  the context of a completion, because the resources for the I/O are
		 *  not released until control returns to the bdev poller.  Also, we
		 *  may require several small I/O to complete before a larger I/O
		 *  (that requires splitting) can be submitted.
		 */
		return;
	}


	while (!TAILQ_EMPTY(&shared_resource->nomem_io)) {

		bdev_io = TAILQ_FIRST(&shared_resource->nomem_io);
		TAILQ_REMOVE(&shared_resource->nomem_io, bdev_io, internal.link);

		bdev_io->internal.ch->io_outstanding++;
		shared_resource->io_outstanding++;

		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_PENDING;

		bdev->fn_table->submit_request(bdev_io->internal.ch->channel, bdev_io);

		if (bdev_io->internal.status == SPDK_BDEV_IO_STATUS_NOMEM) {
			break;
		}
	}
}

// zhou:
static inline void
_spdk_bdev_io_complete(void *ctx)
{
	struct spdk_bdev_io *bdev_io = ctx;
n	uint64_t tsc, tsc_diff;

    // zhou: completion process need be deferred or by another channel.
	if (spdk_unlikely(bdev_io->internal.in_submit_request || bdev_io->internal.io_submit_ch)) {
		/*
		 * Send the completion to the thread that originally submitted the I/O,
		 * which may not be the current thread in the case of QoS.
		 */
		if (bdev_io->internal.io_submit_ch) {
			bdev_io->internal.ch = bdev_io->internal.io_submit_ch;
			bdev_io->internal.io_submit_ch = NULL;
		}

        // zhou: good points.
		/*
		 * Defer completion to avoid potential infinite recursion if the
		 * user's completion callback issues a new I/O.
		 */
		spdk_thread_send_msg(spdk_io_channel_get_thread(bdev_io->internal.ch->channel),
				     _spdk_bdev_io_complete, bdev_io);
		return;
	}

	tsc = spdk_get_ticks();
	tsc_diff = tsc - bdev_io->internal.submit_tsc;
	spdk_trace_record_tsc(tsc, TRACE_BDEV_IO_DONE, 0, 0, (uintptr_t)bdev_io, 0);

	if (bdev_io->internal.ch->histogram) {
		spdk_histogram_data_tally(bdev_io->internal.ch->histogram, tsc_diff);
	}

	if (bdev_io->internal.status == SPDK_BDEV_IO_STATUS_SUCCESS) {
		switch (bdev_io->type) {
		case SPDK_BDEV_IO_TYPE_READ:
			bdev_io->internal.ch->stat.bytes_read += bdev_io->u.bdev.num_blocks * bdev_io->bdev->blocklen;
			bdev_io->internal.ch->stat.num_read_ops++;
			bdev_io->internal.ch->stat.read_latency_ticks += tsc_diff;
			break;
		case SPDK_BDEV_IO_TYPE_WRITE:
			bdev_io->internal.ch->stat.bytes_written += bdev_io->u.bdev.num_blocks * bdev_io->bdev->blocklen;
			bdev_io->internal.ch->stat.num_write_ops++;
			bdev_io->internal.ch->stat.write_latency_ticks += tsc_diff;
			break;
		case SPDK_BDEV_IO_TYPE_UNMAP:
			bdev_io->internal.ch->stat.bytes_unmapped += bdev_io->u.bdev.num_blocks * bdev_io->bdev->blocklen;
			bdev_io->internal.ch->stat.num_unmap_ops++;
			bdev_io->internal.ch->stat.unmap_latency_ticks += tsc_diff;
		default:
			break;
		}
	}

#ifdef SPDK_CONFIG_VTUNE
	uint64_t now_tsc = spdk_get_ticks();
	if (now_tsc > (bdev_io->internal.ch->start_tsc + bdev_io->internal.ch->interval_tsc)) {
		uint64_t data[5];

		data[0] = bdev_io->internal.ch->stat.num_read_ops - bdev_io->internal.ch->prev_stat.num_read_ops;
		data[1] = bdev_io->internal.ch->stat.bytes_read - bdev_io->internal.ch->prev_stat.bytes_read;
		data[2] = bdev_io->internal.ch->stat.num_write_ops - bdev_io->internal.ch->prev_stat.num_write_ops;
		data[3] = bdev_io->internal.ch->stat.bytes_written - bdev_io->internal.ch->prev_stat.bytes_written;
		data[4] = bdev_io->bdev->fn_table->get_spin_time ?
			  bdev_io->bdev->fn_table->get_spin_time(bdev_io->internal.ch->channel) : 0;

		__itt_metadata_add(g_bdev_mgr.domain, __itt_null, bdev_io->internal.ch->handle,
				   __itt_metadata_u64, 5, data);

		bdev_io->internal.ch->prev_stat = bdev_io->internal.ch->stat;
		bdev_io->internal.ch->start_tsc = now_tsc;
	}
#endif

	assert(bdev_io->internal.cb != NULL);
	assert(spdk_get_thread() == spdk_io_channel_get_thread(bdev_io->internal.ch->channel));

    // zhou: invoke client I/O completion callback.
	bdev_io->internal.cb(bdev_io, bdev_io->internal.status == SPDK_BDEV_IO_STATUS_SUCCESS,
			     bdev_io->internal.caller_ctx);
}

static void
_spdk_bdev_reset_complete(struct spdk_io_channel_iter *i, int status)
{
	struct spdk_bdev_io *bdev_io = spdk_io_channel_iter_get_ctx(i);

	if (bdev_io->u.reset.ch_ref != NULL) {
		spdk_put_io_channel(bdev_io->u.reset.ch_ref);
		bdev_io->u.reset.ch_ref = NULL;
	}

	_spdk_bdev_io_complete(bdev_io);
}

static void
_spdk_bdev_unfreeze_channel(struct spdk_io_channel_iter *i)
{
	struct spdk_io_channel *_ch = spdk_io_channel_iter_get_channel(i);
	struct spdk_bdev_channel *ch = spdk_io_channel_get_ctx(_ch);

	ch->flags &= ~BDEV_CH_RESET_IN_PROGRESS;
	if (!TAILQ_EMPTY(&ch->queued_resets)) {
		_spdk_bdev_channel_start_reset(ch);
	}

	spdk_for_each_channel_continue(i, 0);
}

// zhou: when bdev IO completed success or failed. Always invoked by underlying disk
//       when completed success.
void
spdk_bdev_io_complete(struct spdk_bdev_io *bdev_io, enum spdk_bdev_io_status status)
{
	struct spdk_bdev *bdev = bdev_io->bdev;
	struct spdk_bdev_channel *bdev_ch = bdev_io->internal.ch;
	struct spdk_bdev_shared_resource *shared_resource = bdev_ch->shared_resource;

	bdev_io->internal.status = status;

	if (spdk_unlikely(bdev_io->type == SPDK_BDEV_IO_TYPE_RESET)) {

		bool unlock_channels = false;

		if (status == SPDK_BDEV_IO_STATUS_NOMEM) {
			SPDK_ERRLOG("NOMEM returned for reset\n");
		}

		pthread_mutex_lock(&bdev->internal.mutex);

		if (bdev_io == bdev->internal.reset_in_progress) {
			bdev->internal.reset_in_progress = NULL;
			unlock_channels = true;
		}
		pthread_mutex_unlock(&bdev->internal.mutex);

		if (unlock_channels) {
			spdk_for_each_channel(__bdev_to_io_dev(bdev), _spdk_bdev_unfreeze_channel,
					      bdev_io, _spdk_bdev_reset_complete);
			return;
		}

	} else {

		if (spdk_unlikely(bdev_io->internal.orig_iovcnt > 0)) {
			_bdev_io_unset_bounce_buf(bdev_io);
		}

		assert(bdev_ch->io_outstanding > 0);
		assert(shared_resource->io_outstanding > 0);

		bdev_ch->io_outstanding--;

		shared_resource->io_outstanding--;

		if (spdk_unlikely(status == SPDK_BDEV_IO_STATUS_NOMEM)) {

			assert(shared_resource->io_outstanding > 0);

			TAILQ_INSERT_HEAD(&shared_resource->nomem_io, bdev_io, internal.link);

			/*
			 * Wait for some of the outstanding I/O to complete before we
			 *  retry any of the nomem_io.  Normally we will wait for
			 *  NOMEM_THRESHOLD_COUNT I/O to complete but for low queue
			 *  depth channels we will instead wait for half to complete.
			 */
			shared_resource->nomem_threshold = spdk_max((int64_t)shared_resource->io_outstanding / 2,
							   (int64_t)shared_resource->io_outstanding - NOMEM_THRESHOLD_COUNT);
			return;
		}

        // zhou: submit I/O Request immediately, keep underlying disk busy.
		if (spdk_unlikely(!TAILQ_EMPTY(&shared_resource->nomem_io))) {
			_spdk_bdev_ch_retry_io(bdev_ch);
		}
	}

	_spdk_bdev_io_complete(bdev_io);
}

void
spdk_bdev_io_complete_scsi_status(struct spdk_bdev_io *bdev_io, enum spdk_scsi_status sc,
				  enum spdk_scsi_sense sk, uint8_t asc, uint8_t ascq)
{
	if (sc == SPDK_SCSI_STATUS_GOOD) {
		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_SUCCESS;
	} else {
		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_SCSI_ERROR;
		bdev_io->internal.error.scsi.sc = sc;
		bdev_io->internal.error.scsi.sk = sk;
		bdev_io->internal.error.scsi.asc = asc;
		bdev_io->internal.error.scsi.ascq = ascq;
	}

	spdk_bdev_io_complete(bdev_io, bdev_io->internal.status);
}

void
spdk_bdev_io_get_scsi_status(const struct spdk_bdev_io *bdev_io,
			     int *sc, int *sk, int *asc, int *ascq)
{
	assert(sc != NULL);
	assert(sk != NULL);
	assert(asc != NULL);
	assert(ascq != NULL);

	switch (bdev_io->internal.status) {
	case SPDK_BDEV_IO_STATUS_SUCCESS:
		*sc = SPDK_SCSI_STATUS_GOOD;
		*sk = SPDK_SCSI_SENSE_NO_SENSE;
		*asc = SPDK_SCSI_ASC_NO_ADDITIONAL_SENSE;
		*ascq = SPDK_SCSI_ASCQ_CAUSE_NOT_REPORTABLE;
		break;
	case SPDK_BDEV_IO_STATUS_NVME_ERROR:
		spdk_scsi_nvme_translate(bdev_io, sc, sk, asc, ascq);
		break;
	case SPDK_BDEV_IO_STATUS_SCSI_ERROR:
		*sc = bdev_io->internal.error.scsi.sc;
		*sk = bdev_io->internal.error.scsi.sk;
		*asc = bdev_io->internal.error.scsi.asc;
		*ascq = bdev_io->internal.error.scsi.ascq;
		break;
	default:
		*sc = SPDK_SCSI_STATUS_CHECK_CONDITION;
		*sk = SPDK_SCSI_SENSE_ABORTED_COMMAND;
		*asc = SPDK_SCSI_ASC_NO_ADDITIONAL_SENSE;
		*ascq = SPDK_SCSI_ASCQ_CAUSE_NOT_REPORTABLE;
		break;
	}
}

void
spdk_bdev_io_complete_nvme_status(struct spdk_bdev_io *bdev_io, int sct, int sc)
{
	if (sct == SPDK_NVME_SCT_GENERIC && sc == SPDK_NVME_SC_SUCCESS) {
		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_SUCCESS;
	} else {
		bdev_io->internal.error.nvme.sct = sct;
		bdev_io->internal.error.nvme.sc = sc;
		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_NVME_ERROR;
	}

	spdk_bdev_io_complete(bdev_io, bdev_io->internal.status);
}

void
spdk_bdev_io_get_nvme_status(const struct spdk_bdev_io *bdev_io, int *sct, int *sc)
{
	assert(sct != NULL);
	assert(sc != NULL);

	if (bdev_io->internal.status == SPDK_BDEV_IO_STATUS_NVME_ERROR) {
		*sct = bdev_io->internal.error.nvme.sct;
		*sc = bdev_io->internal.error.nvme.sc;
	} else if (bdev_io->internal.status == SPDK_BDEV_IO_STATUS_SUCCESS) {
		*sct = SPDK_NVME_SCT_GENERIC;
		*sc = SPDK_NVME_SC_SUCCESS;
	} else {
		*sct = SPDK_NVME_SCT_GENERIC;
		*sc = SPDK_NVME_SC_INTERNAL_DEVICE_ERROR;
	}
}

// zhou: END of IO management
////////////////////////////////////////////////////////////////////////////////
// zhou: START of

struct spdk_thread *
spdk_bdev_io_get_thread(struct spdk_bdev_io *bdev_io)
{
	return spdk_io_channel_get_thread(bdev_io->internal.ch->channel);
}

struct spdk_io_channel *
spdk_bdev_io_get_io_channel(struct spdk_bdev_io *bdev_io)
{
	return bdev_io->internal.ch->channel;
}

// zhou: set QoS according to config file.
static void
_spdk_bdev_qos_config_limit(struct spdk_bdev *bdev, uint64_t *limits)
{
	uint64_t	min_qos_set;
	int		i;

	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
		if (limits[i] != SPDK_BDEV_QOS_LIMIT_NOT_DEFINED) {
			break;
		}
	}

	if (i == SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES) {
		SPDK_ERRLOG("Invalid rate limits set.\n");
		return;
	}

	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
		if (limits[i] == SPDK_BDEV_QOS_LIMIT_NOT_DEFINED) {
			continue;
		}

		if (_spdk_bdev_qos_is_iops_rate_limit(i) == true) {
			min_qos_set = SPDK_BDEV_QOS_MIN_IOS_PER_SEC;
		} else {
			min_qos_set = SPDK_BDEV_QOS_MIN_BYTES_PER_SEC;
		}

		if (limits[i] == 0 || limits[i] % min_qos_set) {
			SPDK_ERRLOG("Assigned limit %" PRIu64 " on bdev %s is not multiple of %" PRIu64 "\n",
				    limits[i], bdev->name, min_qos_set);
			SPDK_ERRLOG("Failed to enable QoS on this bdev %s\n", bdev->name);
			return;
		}
	}

    // zhou: enable QoS
	if (!bdev->internal.qos) {
		bdev->internal.qos = calloc(1, sizeof(*bdev->internal.qos));
		if (!bdev->internal.qos) {
			SPDK_ERRLOG("Unable to allocate memory for QoS tracking\n");
			return;
		}
	}

	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
		bdev->internal.qos->rate_limits[i].limit = limits[i];
		SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Bdev:%s QoS type:%d set:%lu\n",
			      bdev->name, i, limits[i]);
	}

	return;
}

// zhou: read config section "QoS"
static void
_spdk_bdev_qos_config(struct spdk_bdev *bdev)
{
	struct spdk_conf_section	*sp = NULL;
	const char			*val = NULL;
	int				i = 0, j = 0;
	uint64_t			limits[SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES] = {};
	bool				config_qos = false;

	sp = spdk_conf_find_section(NULL, "QoS");
	if (!sp) {
		return;
	}

	while (j < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES) {

		limits[j] = SPDK_BDEV_QOS_LIMIT_NOT_DEFINED;

		i = 0;
		while (true) {
			val = spdk_conf_section_get_nmval(sp, qos_conf_type[j], i, 0);
			if (!val) {
				break;
			}

			if (strcmp(bdev->name, val) != 0) {
				i++;
				continue;
			}

			val = spdk_conf_section_get_nmval(sp, qos_conf_type[j], i, 1);
			if (val) {
				if (_spdk_bdev_qos_is_iops_rate_limit(j) == true) {
					limits[j] = strtoull(val, NULL, 10);
				} else {
					limits[j] = strtoull(val, NULL, 10) * 1024 * 1024;
				}
				config_qos = true;
			}

			break;
		}

		j++;
	}

	if (config_qos == true) {
		_spdk_bdev_qos_config_limit(bdev, limits);
	}

	return;
}

// zhou: init "struct spdk_bdev" which is abstract object of backing disk.
static int
spdk_bdev_init(struct spdk_bdev *bdev)
{
	char *bdev_name;

	assert(bdev->module != NULL);

	if (!bdev->name) {
		SPDK_ERRLOG("Bdev name is NULL\n");
		return -EINVAL;
	}

	if (spdk_bdev_get_by_name(bdev->name)) {
		SPDK_ERRLOG("Bdev name:%s already exists\n", bdev->name);
		return -EEXIST;
	}

	/* Users often register their own I/O devices using the bdev name. In
	 * order to avoid conflicts, prepend bdev_. */
	bdev_name = spdk_sprintf_alloc("bdev_%s", bdev->name);
	if (!bdev_name) {
		SPDK_ERRLOG("Unable to allocate memory for internal bdev name.\n");
		return -ENOMEM;
	}

	bdev->internal.status = SPDK_BDEV_STATUS_READY;
	bdev->internal.measured_queue_depth = UINT64_MAX;
	bdev->internal.claim_module = NULL;
	bdev->internal.qd_poller = NULL;
	bdev->internal.qos = NULL;

    // zhou: why need adjust "optimal_io_boundary" when "required_alignment" > 0 ?
    //       Because, we have to allocate bounce memory to meet the "required_alignment".
    //       And at this time, our mbuf pool object size may not big enough, have to
    //       split IO. Then the "optimal_io_boundary" is useful.
	if (spdk_bdev_get_buf_align(bdev) > 1) {

		if (bdev->split_on_optimal_io_boundary) {
            // zhou: can't exceed the resource limitation.
			bdev->optimal_io_boundary = spdk_min(bdev->optimal_io_boundary,
							     SPDK_BDEV_LARGE_BUF_MAX_SIZE / bdev->blocklen);
		} else {
            // zhou: when underlying disk has no contraint, we want the split IO
            //       as big as possible.
			bdev->split_on_optimal_io_boundary = true;
			bdev->optimal_io_boundary = SPDK_BDEV_LARGE_BUF_MAX_SIZE / bdev->blocklen;
		}
	}


	TAILQ_INIT(&bdev->internal.open_descs);

	TAILQ_INIT(&bdev->aliases);

	bdev->internal.reset_in_progress = NULL;

    // zhou: set QoS
	_spdk_bdev_qos_config(bdev);

    // zhou:
	spdk_io_device_register(__bdev_to_io_dev(bdev),
				spdk_bdev_channel_create, spdk_bdev_channel_destroy,
				sizeof(struct spdk_bdev_channel),
				bdev_name);

	free(bdev_name);

	pthread_mutex_init(&bdev->internal.mutex, NULL);
	return 0;
}

static void
spdk_bdev_destroy_cb(void *io_device)
{
	int			rc;
	struct spdk_bdev	*bdev;
	spdk_bdev_unregister_cb	cb_fn;
	void			*cb_arg;

	bdev = __bdev_from_io_dev(io_device);
	cb_fn = bdev->internal.unregister_cb;
	cb_arg = bdev->internal.unregister_ctx;

	rc = bdev->fn_table->destruct(bdev->ctxt);
	if (rc < 0) {
		SPDK_ERRLOG("destruct failed\n");
	}
	if (rc <= 0 && cb_fn != NULL) {
		cb_fn(cb_arg, rc);
	}
}


static void
spdk_bdev_fini(struct spdk_bdev *bdev)
{
	pthread_mutex_destroy(&bdev->internal.mutex);

	free(bdev->internal.qos);

	spdk_io_device_unregister(__bdev_to_io_dev(bdev), spdk_bdev_destroy_cb);
}

// zhou: then this backing disk could be used by client.
static void
spdk_bdev_start(struct spdk_bdev *bdev)
{
	struct spdk_bdev_module *module;
	uint32_t action;

	SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Inserting bdev %s into list\n", bdev->name);

	TAILQ_INSERT_TAIL(&g_bdev_mgr.bdevs, bdev, internal.link);

	/* Examine configuration before initializing I/O */
	TAILQ_FOREACH(module, &g_bdev_mgr.bdev_modules, internal.tailq) {

        // zhou: some vdev backing storage type need extra steps.
		if (module->examine_config) {
			action = module->internal.action_in_progress;
			module->internal.action_in_progress++;
			module->examine_config(bdev);
			if (action != module->internal.action_in_progress) {
				SPDK_ERRLOG("examine_config for module %s did not call spdk_bdev_module_examine_done()\n",
					    module->name);
			}
		}
	}

	if (bdev->internal.claim_module) {
		if (bdev->internal.claim_module->examine_disk) {
			bdev->internal.claim_module->internal.action_in_progress++;
			bdev->internal.claim_module->examine_disk(bdev);
		}
		return;
	}

	TAILQ_FOREACH(module, &g_bdev_mgr.bdev_modules, internal.tailq) {
		if (module->examine_disk) {
			module->internal.action_in_progress++;
			module->examine_disk(bdev);
		}
	}
}

// zhou: used when backing storage creating a disk, and register to a to bdev
//       framework.  e.g. "create_aio_bdev()"
int
spdk_bdev_register(struct spdk_bdev *bdev)
{
    // zhou: init abstract object of backing disk.
	int rc = spdk_bdev_init(bdev);

	if (rc == 0) {
		spdk_bdev_start(bdev);
	}

    // zhou: just recorded in "g_events[]", user could query by RPC.
	spdk_notify_send("bdev_register", spdk_bdev_get_name(bdev));
	return rc;
}

int
spdk_vbdev_register(struct spdk_bdev *vbdev, struct spdk_bdev **base_bdevs, int base_bdev_count)
{
	SPDK_ERRLOG("This function is deprecated.  Use spdk_bdev_register() instead.\n");
	return spdk_bdev_register(vbdev);
}

void
spdk_bdev_destruct_done(struct spdk_bdev *bdev, int bdeverrno)
{
	if (bdev->internal.unregister_cb != NULL) {
		bdev->internal.unregister_cb(bdev->internal.unregister_ctx, bdeverrno);
	}
}

static void
_remove_notify(void *arg)
{
	struct spdk_bdev_desc *desc = arg;

	desc->remove_scheduled = false;

	if (desc->closed) {
		free(desc);
	} else {
		desc->remove_cb(desc->remove_ctx);
	}
}

/* Must be called while holding bdev->internal.mutex.
 * returns: 0 - bdev removed and ready to be destructed.
 *          -EBUSY - bdev can't be destructed yet.  */
static int
spdk_bdev_unregister_unsafe(struct spdk_bdev *bdev)
{
	struct spdk_bdev_desc	*desc, *tmp;
	int			rc = 0;

	/* Notify each descriptor about hotremoval */
	TAILQ_FOREACH_SAFE(desc, &bdev->internal.open_descs, link, tmp) {
		rc = -EBUSY;
		if (desc->remove_cb) {
			/*
			 * Defer invocation of the remove_cb to a separate message that will
			 *  run later on its thread.  This ensures this context unwinds and
			 *  we don't recursively unregister this bdev again if the remove_cb
			 *  immediately closes its descriptor.
			 */
			if (!desc->remove_scheduled) {
				/* Avoid scheduling removal of the same descriptor multiple times. */
				desc->remove_scheduled = true;
				spdk_thread_send_msg(desc->thread, _remove_notify, desc);
			}
		}
	}

	/* If there are no descriptors, proceed removing the bdev */
	if (rc == 0) {
		TAILQ_REMOVE(&g_bdev_mgr.bdevs, bdev, internal.link);
		SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Removing bdev %s from list done\n", bdev->name);
		spdk_notify_send("bdev_unregister", spdk_bdev_get_name(bdev));
	}

	return rc;
}

void
spdk_bdev_unregister(struct spdk_bdev *bdev, spdk_bdev_unregister_cb cb_fn, void *cb_arg)
{
	struct spdk_thread	*thread;
	int			rc;

	SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Removing bdev %s from list\n", bdev->name);

	thread = spdk_get_thread();
	if (!thread) {
		/* The user called this from a non-SPDK thread. */
		if (cb_fn != NULL) {
			cb_fn(cb_arg, -ENOTSUP);
		}
		return;
	}

	pthread_mutex_lock(&bdev->internal.mutex);
	if (bdev->internal.status == SPDK_BDEV_STATUS_REMOVING) {
		pthread_mutex_unlock(&bdev->internal.mutex);
		if (cb_fn) {
			cb_fn(cb_arg, -EBUSY);
		}
		return;
	}

	bdev->internal.status = SPDK_BDEV_STATUS_REMOVING;
	bdev->internal.unregister_cb = cb_fn;
	bdev->internal.unregister_ctx = cb_arg;

	/* Call under lock. */
	rc = spdk_bdev_unregister_unsafe(bdev);
	pthread_mutex_unlock(&bdev->internal.mutex);

	if (rc == 0) {
		spdk_bdev_fini(bdev);
	}
}

// zhou: already found "struct spdk_bdev", open block device, just like open a file.
//
//       "Multiple users may have a bdev open at the same time, and coordination
//       of reads and writes between users must be handled by some higher level
//       mechanism outside of the bdev layer. Opening a bdev with write permission
//       may fail if a virtual bdev module has claimed the bdev. Virtual bdev modules
//       implement logic like RAID or logical volume management and forward their
//       I/O to lower level bdevs, so they mark these lower level bdevs as claimed
//       to prevent outside users from issuing writes."
int
spdk_bdev_open(struct spdk_bdev *bdev, bool write, spdk_bdev_remove_cb_t remove_cb,
	       void *remove_ctx, struct spdk_bdev_desc **_desc)
{
	struct spdk_bdev_desc *desc;
	struct spdk_thread *thread;
	struct set_qos_limit_ctx *ctx;

	thread = spdk_get_thread();
	if (!thread) {
		SPDK_ERRLOG("Cannot open bdev from non-SPDK thread.\n");
		return -ENOTSUP;
	}

    // zhou: each open operation will creater a new descriptor.
	desc = calloc(1, sizeof(*desc));
	if (desc == NULL) {
		SPDK_ERRLOG("Failed to allocate memory for bdev descriptor\n");
		return -ENOMEM;
	}

	SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Opening descriptor %p for bdev %s on thread %p\n", desc, bdev->name,
		      spdk_get_thread());

	desc->bdev = bdev;
	desc->thread = thread;
	desc->remove_cb = remove_cb;
	desc->remove_ctx = remove_ctx;
	desc->write = write;
	*_desc = desc;

	pthread_mutex_lock(&bdev->internal.mutex);

	if (write && bdev->internal.claim_module) {
		SPDK_ERRLOG("Could not open %s - %s module already claimed it\n",
			    bdev->name, bdev->internal.claim_module->name);
		pthread_mutex_unlock(&bdev->internal.mutex);
		free(desc);
		*_desc = NULL;
		return -EPERM;
	}

	/* Enable QoS */
	if (bdev->internal.qos && bdev->internal.qos->thread == NULL) {
		ctx = calloc(1, sizeof(*ctx));
		if (ctx == NULL) {
			SPDK_ERRLOG("Failed to allocate memory for QoS context\n");
			pthread_mutex_unlock(&bdev->internal.mutex);
			free(desc);
			*_desc = NULL;
			return -ENOMEM;
		}
		ctx->bdev = bdev;
		spdk_for_each_channel(__bdev_to_io_dev(bdev),
				      _spdk_bdev_enable_qos_msg, ctx,
				      _spdk_bdev_enable_qos_done);
	}

    // zhou: add FD to this bdev's list.
	TAILQ_INSERT_TAIL(&bdev->internal.open_descs, desc, link);

	pthread_mutex_unlock(&bdev->internal.mutex);

	return 0;
}

void
spdk_bdev_close(struct spdk_bdev_desc *desc)
{
	struct spdk_bdev *bdev = desc->bdev;
	int rc;

	SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Closing descriptor %p for bdev %s on thread %p\n", desc, bdev->name,
		      spdk_get_thread());

	if (desc->thread != spdk_get_thread()) {
		SPDK_ERRLOG("Descriptor %p for bdev %s closed on wrong thread (%p, expected %p)\n",
			    desc, bdev->name, spdk_get_thread(), desc->thread);
	}

	pthread_mutex_lock(&bdev->internal.mutex);

	TAILQ_REMOVE(&bdev->internal.open_descs, desc, link);

	desc->closed = true;

	if (!desc->remove_scheduled) {
		free(desc);
	}

	/* If no more descriptors, kill QoS channel */
	if (bdev->internal.qos && TAILQ_EMPTY(&bdev->internal.open_descs)) {
		SPDK_DEBUGLOG(SPDK_LOG_BDEV, "Closed last descriptor for bdev %s on thread %p. Stopping QoS.\n",
			      bdev->name, spdk_get_thread());

		if (spdk_bdev_qos_destroy(bdev)) {
			/* There isn't anything we can do to recover here. Just let the
			 * old QoS poller keep running. The QoS handling won't change
			 * cores when the user allocates a new channel, but it won't break. */
			SPDK_ERRLOG("Unable to shut down QoS poller. It will continue running on the current thread.\n");
		}
	}

	spdk_bdev_set_qd_sampling_period(bdev, 0);

	if (bdev->internal.status == SPDK_BDEV_STATUS_REMOVING && TAILQ_EMPTY(&bdev->internal.open_descs)) {
		rc = spdk_bdev_unregister_unsafe(bdev);
		pthread_mutex_unlock(&bdev->internal.mutex);

		if (rc == 0) {
			spdk_bdev_fini(bdev);
		}
	} else {
		pthread_mutex_unlock(&bdev->internal.mutex);
	}
}

int
spdk_bdev_module_claim_bdev(struct spdk_bdev *bdev, struct spdk_bdev_desc *desc,
			    struct spdk_bdev_module *module)
{
	if (bdev->internal.claim_module != NULL) {
		SPDK_ERRLOG("bdev %s already claimed by module %s\n", bdev->name,
			    bdev->internal.claim_module->name);
		return -EPERM;
	}

	if (desc && !desc->write) {
		desc->write = true;
	}

	bdev->internal.claim_module = module;
	return 0;
}

void
spdk_bdev_module_release_bdev(struct spdk_bdev *bdev)
{
	assert(bdev->internal.claim_module != NULL);
	bdev->internal.claim_module = NULL;
}

struct spdk_bdev *
spdk_bdev_desc_get_bdev(struct spdk_bdev_desc *desc)
{
	return desc->bdev;
}

// zhou: END of init backing storage.
////////////////////////////////////////////////////////////////////////////////
// zhou: START of config backing storage

void
spdk_bdev_io_get_iovec(struct spdk_bdev_io *bdev_io, struct iovec **iovp, int *iovcntp)
{
	struct iovec *iovs;
	int iovcnt;

	if (bdev_io == NULL) {
		return;
	}

	switch (bdev_io->type) {
	case SPDK_BDEV_IO_TYPE_READ:
	case SPDK_BDEV_IO_TYPE_WRITE:
	case SPDK_BDEV_IO_TYPE_ZCOPY:
		iovs = bdev_io->u.bdev.iovs;
		iovcnt = bdev_io->u.bdev.iovcnt;
		break;
	default:
		iovs = NULL;
		iovcnt = 0;
		break;
	}

	if (iovp) {
		*iovp = iovs;
	}
	if (iovcntp) {
		*iovcntp = iovcnt;
	}
}

void
spdk_bdev_module_list_add(struct spdk_bdev_module *bdev_module)
{

	if (spdk_bdev_module_list_find(bdev_module->name)) {
		SPDK_ERRLOG("ERROR: module '%s' already registered.\n", bdev_module->name);
		assert(false);
	}

	if (bdev_module->async_init) {
		bdev_module->internal.action_in_progress = 1;
	}

	/*
	 * Modules with examine callbacks must be initialized first, so they are
	 *  ready to handle examine callbacks from later modules that will
	 *  register physical bdevs.
	 */
	if (bdev_module->examine_config != NULL || bdev_module->examine_disk != NULL) {
		TAILQ_INSERT_HEAD(&g_bdev_mgr.bdev_modules, bdev_module, internal.tailq);
	} else {
		TAILQ_INSERT_TAIL(&g_bdev_mgr.bdev_modules, bdev_module, internal.tailq);
	}
}

struct spdk_bdev_module *
spdk_bdev_module_list_find(const char *name)
{
	struct spdk_bdev_module *bdev_module;

	TAILQ_FOREACH(bdev_module, &g_bdev_mgr.bdev_modules, internal.tailq) {
		if (strcmp(name, bdev_module->name) == 0) {
			break;
		}
	}

	return bdev_module;
}

static void
_spdk_bdev_write_zero_buffer_next(void *_bdev_io)
{
	struct spdk_bdev_io *bdev_io = _bdev_io;
	uint64_t num_bytes, num_blocks;
	int rc;

	num_bytes = spdk_min(spdk_bdev_get_block_size(bdev_io->bdev) *
			     bdev_io->u.bdev.split_remaining_num_blocks,
			     ZERO_BUFFER_SIZE);
	num_blocks = num_bytes / spdk_bdev_get_block_size(bdev_io->bdev);

	rc = spdk_bdev_write_blocks(bdev_io->internal.desc,
				    spdk_io_channel_from_ctx(bdev_io->internal.ch),
				    g_bdev_mgr.zero_buffer,
				    bdev_io->u.bdev.split_current_offset_blocks, num_blocks,
				    _spdk_bdev_write_zero_buffer_done, bdev_io);
	if (rc == 0) {
		bdev_io->u.bdev.split_remaining_num_blocks -= num_blocks;
		bdev_io->u.bdev.split_current_offset_blocks += num_blocks;
	} else if (rc == -ENOMEM) {
		_spdk_bdev_queue_io_wait_with_cb(bdev_io, _spdk_bdev_write_zero_buffer_next);
	} else {
		bdev_io->internal.status = SPDK_BDEV_IO_STATUS_FAILED;
		bdev_io->internal.cb(bdev_io, false, bdev_io->internal.caller_ctx);
	}
}

static void
_spdk_bdev_write_zero_buffer_done(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct spdk_bdev_io *parent_io = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		parent_io->internal.status = SPDK_BDEV_IO_STATUS_FAILED;
		parent_io->internal.cb(parent_io, false, parent_io->internal.caller_ctx);
		return;
	}

	if (parent_io->u.bdev.split_remaining_num_blocks == 0) {
		parent_io->internal.status = SPDK_BDEV_IO_STATUS_SUCCESS;
		parent_io->internal.cb(parent_io, true, parent_io->internal.caller_ctx);
		return;
	}

	_spdk_bdev_write_zero_buffer_next(parent_io);
}

static void
_spdk_bdev_set_qos_limit_done(struct set_qos_limit_ctx *ctx, int status)
{
	pthread_mutex_lock(&ctx->bdev->internal.mutex);
	ctx->bdev->internal.qos_mod_in_progress = false;
	pthread_mutex_unlock(&ctx->bdev->internal.mutex);

	if (ctx->cb_fn) {
		ctx->cb_fn(ctx->cb_arg, status);
	}
	free(ctx);
}

static void
_spdk_bdev_disable_qos_done(void *cb_arg)
{
	struct set_qos_limit_ctx *ctx = cb_arg;
	struct spdk_bdev *bdev = ctx->bdev;
	struct spdk_bdev_io *bdev_io;
	struct spdk_bdev_qos *qos;

	pthread_mutex_lock(&bdev->internal.mutex);
	qos = bdev->internal.qos;
	bdev->internal.qos = NULL;
	pthread_mutex_unlock(&bdev->internal.mutex);

	while (!TAILQ_EMPTY(&qos->queued)) {
		/* Send queued I/O back to their original thread for resubmission. */
		bdev_io = TAILQ_FIRST(&qos->queued);
		TAILQ_REMOVE(&qos->queued, bdev_io, internal.link);

		if (bdev_io->internal.io_submit_ch) {
			/*
			 * Channel was changed when sending it to the QoS thread - change it back
			 *  before sending it back to the original thread.
			 */
			bdev_io->internal.ch = bdev_io->internal.io_submit_ch;
			bdev_io->internal.io_submit_ch = NULL;
		}

		spdk_thread_send_msg(spdk_io_channel_get_thread(bdev_io->internal.ch->channel),
				     _spdk_bdev_io_submit, bdev_io);
	}

	if (qos->thread != NULL) {
		spdk_put_io_channel(spdk_io_channel_from_ctx(qos->ch));
		spdk_poller_unregister(&qos->poller);
	}

	free(qos);

	_spdk_bdev_set_qos_limit_done(ctx, 0);
}

// zhou: notify owner channel to stop QoS
static void
_spdk_bdev_disable_qos_msg_done(struct spdk_io_channel_iter *i, int status)
{
	void *io_device = spdk_io_channel_iter_get_io_device(i);
	struct spdk_bdev *bdev = __bdev_from_io_dev(io_device);
	struct set_qos_limit_ctx *ctx = spdk_io_channel_iter_get_ctx(i);
	struct spdk_thread *thread;

	pthread_mutex_lock(&bdev->internal.mutex);
	thread = bdev->internal.qos->thread;
	pthread_mutex_unlock(&bdev->internal.mutex);

	if (thread != NULL) {
		spdk_thread_send_msg(thread, _spdk_bdev_disable_qos_done, ctx);
	} else {
		_spdk_bdev_disable_qos_done(ctx);
	}
}

// zhou: clear up QoS flag for each channel
static void
_spdk_bdev_disable_qos_msg(struct spdk_io_channel_iter *i)
{
	struct spdk_io_channel *ch = spdk_io_channel_iter_get_channel(i);
	struct spdk_bdev_channel *bdev_ch = spdk_io_channel_get_ctx(ch);

	bdev_ch->flags &= ~BDEV_CH_QOS_ENABLED;

	spdk_for_each_channel_continue(i, 0);
}

// zhou: QoS ower thread
static void
_spdk_bdev_update_qos_rate_limit_msg(void *cb_arg)
{
	struct set_qos_limit_ctx *ctx = cb_arg;
	struct spdk_bdev *bdev = ctx->bdev;

	pthread_mutex_lock(&bdev->internal.mutex);
	spdk_bdev_qos_update_max_quota_per_timeslice(bdev->internal.qos);
	pthread_mutex_unlock(&bdev->internal.mutex);

	_spdk_bdev_set_qos_limit_done(ctx, 0);
}

static void
_spdk_bdev_enable_qos_msg(struct spdk_io_channel_iter *i)
{
	void *io_device = spdk_io_channel_iter_get_io_device(i);
	struct spdk_bdev *bdev = __bdev_from_io_dev(io_device);
	struct spdk_io_channel *ch = spdk_io_channel_iter_get_channel(i);

	struct spdk_bdev_channel *bdev_ch = spdk_io_channel_get_ctx(ch);

	pthread_mutex_lock(&bdev->internal.mutex);
	_spdk_bdev_enable_qos(bdev, bdev_ch);
	pthread_mutex_unlock(&bdev->internal.mutex);

	spdk_for_each_channel_continue(i, 0);
}

static void
_spdk_bdev_enable_qos_done(struct spdk_io_channel_iter *i, int status)
{
	struct set_qos_limit_ctx *ctx = spdk_io_channel_iter_get_ctx(i);

	_spdk_bdev_set_qos_limit_done(ctx, status);
}

// zhou: set value to bdev
static void
_spdk_bdev_set_qos_rate_limits(struct spdk_bdev *bdev, uint64_t *limits)
{
	int i;

	assert(bdev->internal.qos != NULL);

	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
		if (limits[i] != SPDK_BDEV_QOS_LIMIT_NOT_DEFINED) {
            // zhou: just change valued provided by client.
			bdev->internal.qos->rate_limits[i].limit = limits[i];

			if (limits[i] == 0) {
				bdev->internal.qos->rate_limits[i].limit =
					SPDK_BDEV_QOS_LIMIT_NOT_DEFINED;
			}
		}
	}
}

// zhou: set QoS by RPC.
void
spdk_bdev_set_qos_rate_limits(struct spdk_bdev *bdev, uint64_t *limits,
			      void (*cb_fn)(void *cb_arg, int status), void *cb_arg)
{
	struct set_qos_limit_ctx	*ctx;
	uint32_t			limit_set_complement;
	uint64_t			min_limit_per_sec;
	int				i;
	bool				disable_rate_limit = true;

	for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {

        // zhou: NOT provided by client.
		if (limits[i] == SPDK_BDEV_QOS_LIMIT_NOT_DEFINED) {
			continue;
		}

        // zhou: QoS, some value enabled/disabled -> enabled
		if (limits[i] > 0) {
			disable_rate_limit = false;
		}

		if (_spdk_bdev_qos_is_iops_rate_limit(i) == true) {
			min_limit_per_sec = SPDK_BDEV_QOS_MIN_IOS_PER_SEC;
		} else {
			/* Change from megabyte to byte rate limit */
			limits[i] = limits[i] * 1024 * 1024;
			min_limit_per_sec = SPDK_BDEV_QOS_MIN_BYTES_PER_SEC;
		}

		limit_set_complement = limits[i] % min_limit_per_sec;

        // zhou: limit should be multiply of
        //       SPDK_BDEV_QOS_MIN_IOS_PER_SEC/SPDK_BDEV_QOS_MIN_BYTES_PER_SEC.
		if (limit_set_complement) {
			SPDK_ERRLOG("Requested rate limit %" PRIu64 " is not a multiple of %" PRIu64 "\n",
				    limits[i], min_limit_per_sec);
			limits[i] += min_limit_per_sec - limit_set_complement;
			SPDK_ERRLOG("Round up the rate limit to %" PRIu64 "\n", limits[i]);
		}
	}

    // zhou: once "disable_rate_limit == true", means all rate values provided by client
    //       are 0. But we need continue to check the values not provided by client this time.
    //       If they are still valid value for rate limit, we can't disable whole rate limit
    //       of this bdev.

	ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL) {
		cb_fn(cb_arg, -ENOMEM);
		return;
	}

	ctx->cb_fn = cb_fn;
	ctx->cb_arg = cb_arg;
	ctx->bdev = bdev;

	pthread_mutex_lock(&bdev->internal.mutex);
    // zhou: already someone is updating QoS, give up with EAGAIN
	if (bdev->internal.qos_mod_in_progress) {
		pthread_mutex_unlock(&bdev->internal.mutex);
		free(ctx);
		cb_fn(cb_arg, -EAGAIN);
		return;
	}

    // zhou: set flag "is updating QoS"
	bdev->internal.qos_mod_in_progress = true;

    // zhou:
	if (disable_rate_limit == true && bdev->internal.qos) {

		for (i = 0; i < SPDK_BDEV_QOS_NUM_RATE_LIMIT_TYPES; i++) {
            // zhou: rate values not provided by client, when they are valid value,
            //       still can't
			if (limits[i] == SPDK_BDEV_QOS_LIMIT_NOT_DEFINED &&
			    (bdev->internal.qos->rate_limits[i].limit > 0 &&
			     bdev->internal.qos->rate_limits[i].limit !=
			     SPDK_BDEV_QOS_LIMIT_NOT_DEFINED)) {

                // zhou: we can't disable QoS for this bdev, since still have some limit rate.
				disable_rate_limit = false;
				break;
			}
		}
	}

	if (disable_rate_limit == false) {
        // zhou: we want QoS enabled

		if (bdev->internal.qos == NULL) {
            // zhou: QoS disalbed -> enabled
			bdev->internal.qos = calloc(1, sizeof(*bdev->internal.qos));

			if (!bdev->internal.qos) {
				pthread_mutex_unlock(&bdev->internal.mutex);
				SPDK_ERRLOG("Unable to allocate memory for QoS tracking\n");
				free(ctx);
				cb_fn(cb_arg, -ENOMEM);
				return;
			}
		}
        // zhou: else, maybe just update some values.

		if (bdev->internal.qos->thread == NULL) {
			/* Enabling */
			_spdk_bdev_set_qos_rate_limits(bdev, limits);


			spdk_for_each_channel(__bdev_to_io_dev(bdev),
					      _spdk_bdev_enable_qos_msg, ctx,
					      _spdk_bdev_enable_qos_done);
		} else {
			/* Updating */
			_spdk_bdev_set_qos_rate_limits(bdev, limits);

            // zhou: just need to notify the channel/thread who take in charge of QoS.
			spdk_thread_send_msg(bdev->internal.qos->thread,
					     _spdk_bdev_update_qos_rate_limit_msg, ctx);
		}


	} else {
        // zhou: disabling QoS

		if (bdev->internal.qos != NULL) {
			_spdk_bdev_set_qos_rate_limits(bdev, limits);

			/* Disabling */
			spdk_for_each_channel(__bdev_to_io_dev(bdev),
					      _spdk_bdev_disable_qos_msg, ctx,
					      _spdk_bdev_disable_qos_msg_done);
		} else {
			pthread_mutex_unlock(&bdev->internal.mutex);
			_spdk_bdev_set_qos_limit_done(ctx, 0);
			return;
		}
	}

	pthread_mutex_unlock(&bdev->internal.mutex);
}

struct spdk_bdev_histogram_ctx {
	spdk_bdev_histogram_status_cb cb_fn;
	void *cb_arg;
	struct spdk_bdev *bdev;
	int status;
};

static void
_spdk_bdev_histogram_disable_channel_cb(struct spdk_io_channel_iter *i, int status)
{
	struct spdk_bdev_histogram_ctx *ctx = spdk_io_channel_iter_get_ctx(i);

	pthread_mutex_lock(&ctx->bdev->internal.mutex);
	ctx->bdev->internal.histogram_in_progress = false;
	pthread_mutex_unlock(&ctx->bdev->internal.mutex);
	ctx->cb_fn(ctx->cb_arg, ctx->status);
	free(ctx);
}

static void
_spdk_bdev_histogram_disable_channel(struct spdk_io_channel_iter *i)
{
	struct spdk_io_channel *_ch = spdk_io_channel_iter_get_channel(i);
	struct spdk_bdev_channel *ch = spdk_io_channel_get_ctx(_ch);

	if (ch->histogram != NULL) {
		spdk_histogram_data_free(ch->histogram);
		ch->histogram = NULL;
	}
	spdk_for_each_channel_continue(i, 0);
}

static void
_spdk_bdev_histogram_enable_channel_cb(struct spdk_io_channel_iter *i, int status)
{
	struct spdk_bdev_histogram_ctx *ctx = spdk_io_channel_iter_get_ctx(i);

	if (status != 0) {
		ctx->status = status;
		ctx->bdev->internal.histogram_enabled = false;
		spdk_for_each_channel(__bdev_to_io_dev(ctx->bdev), _spdk_bdev_histogram_disable_channel, ctx,
				      _spdk_bdev_histogram_disable_channel_cb);
	} else {
		pthread_mutex_lock(&ctx->bdev->internal.mutex);
		ctx->bdev->internal.histogram_in_progress = false;
		pthread_mutex_unlock(&ctx->bdev->internal.mutex);
		ctx->cb_fn(ctx->cb_arg, ctx->status);
		free(ctx);
	}
}

static void
_spdk_bdev_histogram_enable_channel(struct spdk_io_channel_iter *i)
{
	struct spdk_io_channel *_ch = spdk_io_channel_iter_get_channel(i);
	struct spdk_bdev_channel *ch = spdk_io_channel_get_ctx(_ch);
	int status = 0;

	if (ch->histogram == NULL) {
		ch->histogram = spdk_histogram_data_alloc();
		if (ch->histogram == NULL) {
			status = -ENOMEM;
		}
	}

	spdk_for_each_channel_continue(i, status);
}

// zhou: README,
void
spdk_bdev_histogram_enable(struct spdk_bdev *bdev, spdk_bdev_histogram_status_cb cb_fn,
			   void *cb_arg, bool enable)
{
	struct spdk_bdev_histogram_ctx *ctx;

	ctx = calloc(1, sizeof(struct spdk_bdev_histogram_ctx));
	if (ctx == NULL) {
		cb_fn(cb_arg, -ENOMEM);
		return;
	}

	ctx->bdev = bdev;
	ctx->status = 0;
	ctx->cb_fn = cb_fn;
	ctx->cb_arg = cb_arg;

	pthread_mutex_lock(&bdev->internal.mutex);
	if (bdev->internal.histogram_in_progress) {
		pthread_mutex_unlock(&bdev->internal.mutex);
		free(ctx);
		cb_fn(cb_arg, -EAGAIN);
		return;
	}

	bdev->internal.histogram_in_progress = true;
	pthread_mutex_unlock(&bdev->internal.mutex);

	bdev->internal.histogram_enabled = enable;

	if (enable) {
		/* Allocate histogram for each channel */
		spdk_for_each_channel(__bdev_to_io_dev(bdev), _spdk_bdev_histogram_enable_channel, ctx,
				      _spdk_bdev_histogram_enable_channel_cb);
	} else {
		spdk_for_each_channel(__bdev_to_io_dev(bdev), _spdk_bdev_histogram_disable_channel, ctx,
				      _spdk_bdev_histogram_disable_channel_cb);
	}
}

struct spdk_bdev_histogram_data_ctx {
	spdk_bdev_histogram_data_cb cb_fn;
	void *cb_arg;
	struct spdk_bdev *bdev;
	/** merged histogram data from all channels */
	struct spdk_histogram_data	*histogram;
};

static void
_spdk_bdev_histogram_get_channel_cb(struct spdk_io_channel_iter *i, int status)
{
	struct spdk_bdev_histogram_data_ctx *ctx = spdk_io_channel_iter_get_ctx(i);

	ctx->cb_fn(ctx->cb_arg, status, ctx->histogram);
	free(ctx);
}

static void
_spdk_bdev_histogram_get_channel(struct spdk_io_channel_iter *i)
{
	struct spdk_io_channel *_ch = spdk_io_channel_iter_get_channel(i);
	struct spdk_bdev_channel *ch = spdk_io_channel_get_ctx(_ch);
	struct spdk_bdev_histogram_data_ctx *ctx = spdk_io_channel_iter_get_ctx(i);
	int status = 0;

	if (ch->histogram == NULL) {
		status = -EFAULT;
	} else {
		spdk_histogram_data_merge(ctx->histogram, ch->histogram);
	}

	spdk_for_each_channel_continue(i, status);
}

void
spdk_bdev_histogram_get(struct spdk_bdev *bdev, struct spdk_histogram_data *histogram,
			spdk_bdev_histogram_data_cb cb_fn,
			void *cb_arg)
{
	struct spdk_bdev_histogram_data_ctx *ctx;

	ctx = calloc(1, sizeof(struct spdk_bdev_histogram_data_ctx));
	if (ctx == NULL) {
		cb_fn(cb_arg, -ENOMEM, NULL);
		return;
	}

	ctx->bdev = bdev;
	ctx->cb_fn = cb_fn;
	ctx->cb_arg = cb_arg;

	ctx->histogram = histogram;

	spdk_for_each_channel(__bdev_to_io_dev(bdev), _spdk_bdev_histogram_get_channel, ctx,
			      _spdk_bdev_histogram_get_channel_cb);
}

SPDK_LOG_REGISTER_COMPONENT("bdev", SPDK_LOG_BDEV)

SPDK_TRACE_REGISTER_FN(bdev_trace, "bdev", TRACE_GROUP_BDEV)
{
	spdk_trace_register_owner(OWNER_BDEV, 'b');
	spdk_trace_register_object(OBJECT_BDEV_IO, 'i');
	spdk_trace_register_description("BDEV_IO_START", TRACE_BDEV_IO_START, OWNER_BDEV,
					OBJECT_BDEV_IO, 1, 0, "type:   ");
	spdk_trace_register_description("BDEV_IO_DONE", TRACE_BDEV_IO_DONE, OWNER_BDEV,
					OBJECT_BDEV_IO, 0, 0, "");
}

// zhou: END of config backing storage
////////////////////////////////////////////////////////////////////////////////
