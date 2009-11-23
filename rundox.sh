#!/bin/sh

echo "removing html directory and Doxyfile"
rm -rf html Doxyfile

STANFORD_WBC_REV=`svn info | awk '/^Revision:/{print $2}'`

echo "substituting revisions stanford-wbc r${STANFORD_WBC_REV}"
cat Doxyfile.in | sed -e s:@STANFORD_WBC_REV@:${STANFORD_WBC_REV}: > Doxyfile

echo "running doxygen like this: doxygen Doxyfile 2>&1 | tee doxy.log"
doxygen Doxyfile 2>&1 | tee doxy.log
