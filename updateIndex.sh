#!/bin/sh


date

find lib -type f \( -name "*.[ch]" -o -name "*.cpp" \) -print > cscope.files
find app -type f \( -name "*.[ch]" -o -name "*.cpp" \) -print >> cscope.files
#find examples -type f \( -name "*.[ch]" -o -name "*.cpp" \) -print >> cscope.files
find go -type f \( -name "*.[ch]" -o -name "*.cpp" \) -print >> cscope.files
find include -type f \( -name "*.[ch]" -o -name "*.cpp" \) -print >> cscope.files

cscope -b
date

cd examples
find . -type f \( -name "*.[ch]" -o -name "*.cpp" \) -print > cscope.files
cscope -b
cd -
date

#rsync -avr --exclude=.git/* --exclude=cscope.* dpdk-read orange:~/opensource/
