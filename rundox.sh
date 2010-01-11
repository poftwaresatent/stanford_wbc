#!/bin/sh

echo "removing html directory and Doxyfile"
rm -rf html Doxyfile

STANFORD_WBC_REV=`date`
if [ -d .svn ]; then
 STANFORD_WBC_REV="revision `svn info | awk '/^Revision:/{print $2}'`"
fi
STANFORD_WBC_REV=`echo ${STANFORD_WBC_REV} | sed 's: :_:g'`

echo "substituting STANFORD_WBC_REV: ${STANFORD_WBC_REV}"
cat Doxyfile.in | sed -e s/@STANFORD_WBC_REV@/${STANFORD_WBC_REV}/ > Doxyfile

echo "running doxygen like this: doxygen Doxyfile 2>&1 | tee doxy.log"
doxygen Doxyfile 2>&1 | tee doxy.log
