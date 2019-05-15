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

#ifndef SPDK_BLOBSTORE_H
#define SPDK_BLOBSTORE_H

#include "spdk/assert.h"
#include "spdk/blob.h"
#include "spdk/queue.h"
#include "spdk/util.h"

#include "request.h"

/* In Memory Data Structures
 *
 * The following data structures exist only in memory.
 */

#define SPDK_BLOB_OPTS_CLUSTER_SZ (1024 * 1024)
#define SPDK_BLOB_OPTS_NUM_MD_PAGES UINT32_MAX
#define SPDK_BLOB_OPTS_MAX_MD_OPS 32
#define SPDK_BLOB_OPTS_DEFAULT_CHANNEL_OPS 512
#define SPDK_BLOB_BLOBID_HIGH_BIT (1ULL << 32)

struct spdk_xattr {
	uint32_t	index;
	uint16_t	value_len;
	char		*name;
	void		*value;
	TAILQ_ENTRY(spdk_xattr)	link;
};

// zhou: info about the blob, which could be parsed from on disk metadata.
//
//      "This is a per blob structure, included the struct spdk_blob struct that
//       actually defines the blob itself. It has the specific information on size
//       and makeup of the blob (ie how many clusters are allocated for this blob
//       and which ones.)"

/* The mutable part of the blob data that is sync'd to
 * disk. The data in here is both mutable and persistent.
 */
struct spdk_blob_mut_data {
	/* Number of data clusters in the blob */
	uint64_t	num_clusters;


    // zhou: when sector size is 512 Bytes, uint32_t is not big enough to
    //       hold all number of sectors when disk size is greater than 2T.
	/* Array LBAs that are the beginning of a cluster, in
	 * the order they appear in the blob.
	 */
	uint64_t	*clusters;
    // zhou: "clusters" array szie
	/* The size of the clusters array. This is greater than or
	 * equal to 'num_clusters'.
	 */
	size_t		cluster_array_size;


	/* Number of metadata pages */
	uint32_t	num_pages;

    // zhou:
	/* Array of page offsets into the metadata region, in
	 * the order of the metadata page sequence.
	 */
	uint32_t	*pages;
};

enum spdk_blob_state {
	/* The blob in-memory version does not match the on-disk
	 * version.
	 */
	SPDK_BLOB_STATE_DIRTY,

	/* The blob in memory version of the blob matches the on disk
	 * version.
	 */
	SPDK_BLOB_STATE_CLEAN,

    // zhou: reading from disk, nothing could be done.
	/* The in-memory state being synchronized with the on-disk
	 * blob state. */
	SPDK_BLOB_STATE_LOADING,
};

TAILQ_HEAD(spdk_xattr_tailq, spdk_xattr);

struct spdk_blob_list {
	spdk_blob_id id;
	size_t clone_count;
	TAILQ_HEAD(, spdk_blob_list) clones;
	TAILQ_ENTRY(spdk_blob_list) link;
};

// zhou: object blob,
//       "This is an in-memory data structure that contains key elements like
//        the blob identifier, it's current state and two copies of the mutable
//        metadata for the blob; one copy is the current metadata and the other
//        is the last copy written to disk."
struct spdk_blob {
	struct spdk_blob_store *bs;

	uint32_t	open_ref;

	spdk_blob_id	id;
	spdk_blob_id	parent_id;

	enum spdk_blob_state		state;

	/* Two copies of the mutable data. One is a version
	 * that matches the last known data on disk (clean).
	 * The other (active) is the current data. Syncing
	 * a blob makes the clean match the active.
	 */
	struct spdk_blob_mut_data	clean;
	struct spdk_blob_mut_data	active;


	bool		invalid;

    // zhou: client data readonly
	bool		data_ro;
    // zhou: metadata readonly
	bool		md_ro;

	uint64_t	invalid_flags;

	uint64_t	data_ro_flags;
	uint64_t	md_ro_flags;

	struct spdk_bs_dev *back_bs_dev;

	/* TODO: The xattrs are mutable, but we don't want to be
	 * copying them unnecessarily. Figure this out.
	 */
	struct spdk_xattr_tailq xattrs;
	struct spdk_xattr_tailq xattrs_internal;

	TAILQ_ENTRY(spdk_blob) link;

    // zhou: frozen request count, will be read/write in Metadata Thread, and
    //       be read IO channel when processing READ/WRITE request.
    //       Should be shared between threads. How to protect ?
    //       In one cache line, and go through each thread which will using
    //       "spp_ring_enqueue()", which should include a memory fence.
	uint32_t frozen_refcnt;

    // zhou: avoid several requests conflict, only one Request be handled in
    //       Metadata Thread.
	bool locked_operation_in_progress;

	enum blob_clear_method clear_method;
};

// zhou: core data for BlobStore.
//       "This is the main in-memory structure for the entire Blobstore. It defines
//       the global on disk metadata region and maintains information relevant to
//       the entire system - initialization options such as cluster size, etc."
struct spdk_blob_store {
	uint64_t			md_start; /* Offset from beginning of disk, in pages */
	uint32_t			md_len; /* Count, in pages */

    // zhou: IO Channel which metadata maintenance thread.
	struct spdk_io_channel		*md_channel;
	uint32_t			max_channel_ops;

	struct spdk_thread		*md_thread;

    // zhou: operations related  "struct spdk_bdev",
    //       "struct blob_bdev" includes "struct spdk_bdev"
	struct spdk_bs_dev		*dev;

    // zhou: all of bitmap below will be wrote into disk.

    // zhou: bitmap for meta-data pages
	struct spdk_bit_array		*used_md_pages;
    // zhou: bitmap for clusters.
	struct spdk_bit_array		*used_clusters;
    // zhou: bitmap for blob ID
	struct spdk_bit_array		*used_blobids;

    // zhou: used to protect clusters allocation. Why we need the this lock,
    //       all metadata should be done in metadata thread.
	pthread_mutex_t			used_clusters_mutex;

	uint32_t			cluster_sz;
	uint64_t			total_clusters;
	uint64_t			total_data_clusters;
	uint64_t			num_free_clusters;
	uint64_t			pages_per_cluster;
	uint32_t			io_unit_size;

	spdk_blob_id			super_blob;
	struct spdk_bs_type		bstype;

	struct spdk_bs_cpl		unload_cpl;
	int				unload_err;

	TAILQ_HEAD(, spdk_blob)		blobs;
	TAILQ_HEAD(, spdk_blob_list)	snapshots;

	bool                            clean;
};

// zhou: context with IO channel.
struct spdk_bs_channel {
	struct spdk_bs_request_set	*req_mem;

	TAILQ_HEAD(, spdk_bs_request_set) reqs;

	struct spdk_blob_store		*bs;

	struct spdk_bs_dev		*dev;
	struct spdk_io_channel		*dev_channel;

	TAILQ_HEAD(, spdk_bs_request_set) need_cluster_alloc;
	TAILQ_HEAD(, spdk_bs_request_set) queued_io;
};

/** operation type */
enum spdk_blob_op_type {
	SPDK_BLOB_WRITE,
	SPDK_BLOB_READ,
	SPDK_BLOB_UNMAP,
	SPDK_BLOB_WRITE_ZEROES,
	SPDK_BLOB_WRITEV,
	SPDK_BLOB_READV,
};

/* back bs_dev */

#define BLOB_SNAPSHOT "SNAP"
#define SNAPSHOT_IN_PROGRESS "SNAPTMP"

struct spdk_blob_bs_dev {
	struct spdk_bs_dev bs_dev;
	struct spdk_blob *blob;
};

/* On-Disk Data Structures
 *
 * The following data structures exist on disk.
 */
#define SPDK_BS_INITIAL_VERSION 1
#define SPDK_BS_VERSION 3 /* current version */

#pragma pack(push, 1)

#define SPDK_MD_MASK_TYPE_USED_PAGES 0
#define SPDK_MD_MASK_TYPE_USED_CLUSTERS 1
#define SPDK_MD_MASK_TYPE_USED_BLOBIDS 2

// zhou: will be stored with bitmap, as bitmap's meta-data.
struct spdk_bs_md_mask {
	uint8_t		type;
	uint32_t	length; /* In bits */
	uint8_t		mask[0];
};

#define SPDK_MD_DESCRIPTOR_TYPE_PADDING 0
#define SPDK_MD_DESCRIPTOR_TYPE_EXTENT 1
#define SPDK_MD_DESCRIPTOR_TYPE_XATTR 2
#define SPDK_MD_DESCRIPTOR_TYPE_FLAGS 3
#define SPDK_MD_DESCRIPTOR_TYPE_XATTR_INTERNAL 4

struct spdk_blob_md_descriptor_xattr {
	uint8_t		type;
	uint32_t	length;

	uint16_t	name_length;
	uint16_t	value_length;

	char		name[0];
	/* String name immediately followed by string value. */
};

struct spdk_blob_md_descriptor_extent {
	uint8_t		type;
	uint32_t	length;

	struct {
		uint32_t        cluster_idx;
		uint32_t        length; /* In units of clusters */
	} extents[0];
};

#define SPDK_BLOB_THIN_PROV (1ULL << 0)
#define SPDK_BLOB_INTERNAL_XATTR (1ULL << 1)
#define SPDK_BLOB_INVALID_FLAGS_MASK	(SPDK_BLOB_THIN_PROV | SPDK_BLOB_INTERNAL_XATTR)

#define SPDK_BLOB_READ_ONLY (1ULL << 0)
#define SPDK_BLOB_DATA_RO_FLAGS_MASK	SPDK_BLOB_READ_ONLY
#define SPDK_BLOB_MD_RO_FLAGS_MASK	0

struct spdk_blob_md_descriptor_flags {
	uint8_t		type;
	uint32_t	length;

	/*
	 * If a flag in invalid_flags is set that the application is not aware of,
	 *  it will not allow the blob to be opened.
	 */
	uint64_t	invalid_flags;

	/*
	 * If a flag in data_ro_flags is set that the application is not aware of,
	 *  allow the blob to be opened in data_read_only and md_read_only mode.
	 */
	uint64_t	data_ro_flags;

	/*
	 * If a flag in md_ro_flags is set the the application is not aware of,
	 *  allow the blob to be opened in md_read_only mode.
	 */
	uint64_t	md_ro_flags;
};

struct spdk_blob_md_descriptor {
	uint8_t		type;
	uint32_t	length;
};

#define SPDK_INVALID_MD_PAGE UINT32_MAX

struct spdk_blob_md_page {
	spdk_blob_id     id;

	uint32_t        sequence_num;
	uint32_t	reserved0;

	/* Descriptors here */
	uint8_t		descriptors[4072];

	uint32_t	next;
	uint32_t	crc;
};

// zhou: 4 KByte
#define SPDK_BS_PAGE_SIZE 0x1000
SPDK_STATIC_ASSERT(SPDK_BS_PAGE_SIZE == sizeof(struct spdk_blob_md_page), "Invalid md page size");

#define SPDK_BS_SUPER_BLOCK_SIG "SPDKBLOB"

// zhou: Just 4096 bytes, Cluster 0, Page 0, Super Block on disk structure.
struct spdk_bs_super_block {
	uint8_t		signature[8];
	uint32_t        version;
    // zhou: lenth of "struct spdk_bs_super_block"
	uint32_t        length;

    // zhou: graceful shutdown, will be changed when create/resize blob.
	uint32_t	clean; /* If there was a clean shutdown, this is 1. */

    // zhou: ???
	spdk_blob_id	super_blob;
    // zhou: cluster size could be set when format. Page size is fixed, 4KByte.
	uint32_t	cluster_size; /* In bytes */

    // zhou: the Page 0 is used by Super Block.
    //       The Page begins at used_page_mask_start, for Used Page Bitmap.
	uint32_t	used_page_mask_start; /* Offset from beginning of disk, in pages */
    // zhou: how many Pages used by "used page mask"
	uint32_t	used_page_mask_len; /* Count, in pages */

    // zhou: The Page begins at used_cluster_mask_start, for Used Cluster Bitmap.
	uint32_t	used_cluster_mask_start; /* Offset from beginning of disk, in pages */
    // zhou: how many Pages used by "used cluster mask"
	uint32_t	used_cluster_mask_len; /* Count, in pages */

    // zhou: The Page begins at md_start, for meta-date page.
	uint32_t	md_start; /* Offset from beginning of disk, in pages */
	uint32_t	md_len; /* Count, in pages */

    // zhou: string 16 bytes, such as "LVOLSTORE", set by "struct spdk_bs_opts"
	struct spdk_bs_type	bstype; /* blobstore type */

    // zhou: The Page begins at used_blobid_mask_start, for Used Blob ID Bitmap.
	uint32_t	used_blobid_mask_start; /* Offset from beginning of disk, in pages */
    // zhou: how many Pages used by "used blobid mask"
	uint32_t	used_blobid_mask_len; /* Count, in pages */

	uint64_t        size; /* size of blobstore in bytes */
	uint32_t        io_unit_size; /* Size of io unit in bytes */

	uint8_t         reserved[4000];
	uint32_t	crc;
};

// zhou: static_assert, introduced in C++0x. Make sure the struct size is 1 page.
SPDK_STATIC_ASSERT(sizeof(struct spdk_bs_super_block) == 0x1000, "Invalid super block size");



#pragma pack(pop)

struct spdk_bs_dev *spdk_bs_create_zeroes_dev(void);
struct spdk_bs_dev *spdk_bs_create_blob_bs_dev(struct spdk_blob *blob);

/* Unit Conversions
 *
 * The blobstore works with several different units:
 * - Byte: Self explanatory
 * - LBA: The logical blocks on the backing storage device.
 * - Page: The read/write units of blobs and metadata. This is
 *         an offset into a blob in units of 4KiB.
 * - Cluster Index: The disk is broken into a sequential list of
 *		    clusters. This is the offset from the beginning.
 *
 * NOTE: These conversions all act on simple magnitudes, not with any sort
 *        of knowledge about the blobs themselves. For instance, converting
 *        a page to an lba with the conversion function below simply converts
 *        a number of pages to an equivalent number of lbas, but that
 *        lba certainly isn't the right lba that corresponds to a page offset
 *        for a particular blob.
 */

// zhou: convert to number of block/sector.
static inline uint64_t
_spdk_bs_byte_to_lba(struct spdk_blob_store *bs, uint64_t length)
{
	assert(length % bs->dev->blocklen == 0);

	return length / bs->dev->blocklen;
}

static inline uint64_t
_spdk_bs_dev_byte_to_lba(struct spdk_bs_dev *bs_dev, uint64_t length)
{
	assert(length % bs_dev->blocklen == 0);

	return length / bs_dev->blocklen;
}

// zhou: convert to sector/block
static inline uint64_t
_spdk_bs_page_to_lba(struct spdk_blob_store *bs, uint64_t page)
{
	return page * SPDK_BS_PAGE_SIZE / bs->dev->blocklen;
}

static inline uint64_t
_spdk_bs_dev_page_to_lba(struct spdk_bs_dev *bs_dev, uint64_t page)
{
	return page * SPDK_BS_PAGE_SIZE / bs_dev->blocklen;
}

static inline uint64_t
_spdk_bs_io_unit_per_page(struct spdk_blob_store *bs)
{
	return SPDK_BS_PAGE_SIZE / bs->io_unit_size;
}

static inline uint64_t
_spdk_bs_io_unit_to_page(struct spdk_blob_store *bs, uint64_t io_unit)
{
	return io_unit / _spdk_bs_io_unit_per_page(bs);
}

static inline uint64_t
_spdk_bs_cluster_to_page(struct spdk_blob_store *bs, uint32_t cluster)
{
	return (uint64_t)cluster * bs->pages_per_cluster;
}

static inline uint32_t
_spdk_bs_page_to_cluster(struct spdk_blob_store *bs, uint64_t page)
{
	assert(page % bs->pages_per_cluster == 0);

	return page / bs->pages_per_cluster;
}

static inline uint64_t
_spdk_bs_cluster_to_lba(struct spdk_blob_store *bs, uint32_t cluster)
{
	return (uint64_t)cluster * (bs->cluster_sz / bs->dev->blocklen);
}

static inline uint32_t
_spdk_bs_lba_to_cluster(struct spdk_blob_store *bs, uint64_t lba)
{
	assert(lba % (bs->cluster_sz / bs->dev->blocklen) == 0);

	return lba / (bs->cluster_sz / bs->dev->blocklen);
}

static inline uint64_t
_spdk_bs_io_unit_to_back_dev_lba(struct spdk_blob *blob, uint64_t io_unit)
{
	return io_unit * (blob->bs->io_unit_size / blob->back_bs_dev->blocklen);
}

static inline uint64_t
_spdk_bs_back_dev_lba_to_io_unit(struct spdk_blob *blob, uint64_t lba)
{
	return lba * (blob->back_bs_dev->blocklen / blob->bs->io_unit_size);
}

/* End basic conversions */

static inline uint64_t
_spdk_bs_blobid_to_page(spdk_blob_id id)
{
	return id & 0xFFFFFFFF;
}

/* The blob id is a 64 bit number. The lower 32 bits are the page_idx. The upper
 * 32 bits are not currently used. Stick a 1 there just to catch bugs where the
 * code assumes blob id == page_idx.
 */
static inline spdk_blob_id
_spdk_bs_page_to_blobid(uint64_t page_idx)
{
	if (page_idx > UINT32_MAX) {
		return SPDK_BLOBID_INVALID;
	}
	return SPDK_BLOB_BLOBID_HIGH_BIT | page_idx;
}

/* Given an io unit offset into a blob, look up the LBA for the
 * start of that io unit.
 */
static inline uint64_t
_spdk_bs_blob_io_unit_to_lba(struct spdk_blob *blob, uint64_t io_unit)
{
	uint64_t	lba;
	uint64_t	pages_per_cluster;
	uint64_t	io_units_per_cluster;
	uint64_t	io_units_per_page;
	uint64_t	page;

	page = _spdk_bs_io_unit_to_page(blob->bs, io_unit);

	pages_per_cluster = blob->bs->pages_per_cluster;
	io_units_per_page = _spdk_bs_io_unit_per_page(blob->bs);
	io_units_per_cluster = io_units_per_page * pages_per_cluster;

	assert(page < blob->active.num_clusters * pages_per_cluster);

	lba = blob->active.clusters[page / pages_per_cluster];
	lba += io_unit % io_units_per_cluster;
	return lba;
}

/* Given an io_unit offset into a blob, look up the number of io_units until the
 * next cluster boundary.
 */
static inline uint32_t
_spdk_bs_num_io_units_to_cluster_boundary(struct spdk_blob *blob, uint64_t io_unit)
{
	uint64_t	io_units_per_cluster;

	io_units_per_cluster = _spdk_bs_io_unit_per_page(blob->bs) * blob->bs->pages_per_cluster;

	return io_units_per_cluster - (io_unit % io_units_per_cluster);
}

/* Given a page offset into a blob, look up the number of pages until the
 * next cluster boundary.
 */
static inline uint32_t
_spdk_bs_num_pages_to_cluster_boundary(struct spdk_blob *blob, uint64_t page)
{
	uint64_t	pages_per_cluster;

	pages_per_cluster = blob->bs->pages_per_cluster;

	return pages_per_cluster - (page % pages_per_cluster);
}

/* Given an io_unit offset into a blob, look up the number of pages into blob to beginning of current cluster */
static inline uint32_t
_spdk_bs_io_unit_to_cluster_start(struct spdk_blob *blob, uint64_t io_unit)
{
	uint64_t	pages_per_cluster;
	uint64_t	page;

	pages_per_cluster = blob->bs->pages_per_cluster;
	page = _spdk_bs_io_unit_to_page(blob->bs, io_unit);

	return page - (page % pages_per_cluster);
}

/* Given an io_unit offset into a blob, look up the number of pages into blob to beginning of current cluster */
static inline uint32_t
_spdk_bs_io_unit_to_cluster_number(struct spdk_blob *blob, uint64_t io_unit)
{
	return (io_unit / _spdk_bs_io_unit_per_page(blob->bs)) / blob->bs->pages_per_cluster;
}

/* Given an io unit offset into a blob, look up if it is from allocated cluster. */
static inline bool
_spdk_bs_io_unit_is_allocated(struct spdk_blob *blob, uint64_t io_unit)
{
	uint64_t	lba;
	uint64_t	page;
	uint64_t	pages_per_cluster;

	pages_per_cluster = blob->bs->pages_per_cluster;
	page = _spdk_bs_io_unit_to_page(blob->bs, io_unit);

	assert(page < blob->active.num_clusters * pages_per_cluster);

	lba = blob->active.clusters[page / pages_per_cluster];

	if (lba == 0) {
		assert(spdk_blob_is_thin_provisioned(blob));
		return false;
	} else {
		return true;
	}
}

#endif
