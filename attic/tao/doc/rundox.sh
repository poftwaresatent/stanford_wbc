#!/bin/sh

echo "removing html directory and Doxyfile"
rm -rf html Doxyfile

TAO_REV=`git show --format="%H %aD" HEAD | head -n 1`

echo "substituting revision ${TAO_REV}"
cat Doxyfile.in | sed -e "s/@TAO_REV@/${TAO_REV}/" > Doxyfile

echo "running doxygen like this: doxygen Doxyfile 2>&1 | tee doxy.log"
doxygen Doxyfile 2>&1 | tee doxy.log
