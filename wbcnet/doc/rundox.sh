#!/bin/sh

echo "removing html directory and Doxyfile"
rm -rf html Doxyfile

REVISION=`git show --format="%H %aD" HEAD | head -n 1`

echo "substituting revision ${REVISION}"
cat Doxyfile.in | sed "s/@REVISION@/${REVISION}/" > Doxyfile

echo "running doxygen like this: doxygen Doxyfile 2>&1 | tee doxy.log"
doxygen Doxyfile 2>&1 | tee doxy.log
