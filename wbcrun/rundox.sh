#!/bin/sh

echo "removing html directory and Doxyfile"
rm -rf html Doxyfile

REVISION=`svn info | awk '/^Revision:/{print $2}'`

echo "substituting revision r${REVISION}"
cat Doxyfile.in | sed s:@REVISION@:${REVISION}: > Doxyfile

echo "running doxygen like this: doxygen Doxyfile 2>&1 | tee doxy.log"
doxygen Doxyfile 2>&1 | tee doxy.log
