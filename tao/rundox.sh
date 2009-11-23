#!/bin/sh

echo "removing html directory and Doxyfile"
rm -rf html Doxyfile

TAO_REV=`svn info | awk '/^Revision:/{print $2}'`

echo "substituting revision r${TAO_REV}"
cat Doxyfile.in | sed -e s:@TAO_REV@:${TAO_REV}: > Doxyfile

echo "running doxygen like this: doxygen Doxyfile 2>&1 | tee doxy.log"
doxygen Doxyfile 2>&1 | tee doxy.log
