#!/bin/bash

date
find . -type f \( -name "*.[ch]" -o -name "*.cpp" \) -print > cscope.files

cscope -b


find  . -name \*.py -print | xargs etags

date

