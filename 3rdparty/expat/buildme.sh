#!/bin/bash

PREFIX=`pwd`
if [ ! -z $1 ]; then
    if [ foo`echo $1 | sed 's:\(.\).*:\1:'` = "foo/" ]; then
        PREFIX=$1
    else
        PREFIX=`pwd`/$1
    fi
fi

if [ -e expat-2.0.1 ]; then
    echo "looks like you already have expat-2.0.1"
    echo "move it out of the way before running this script"
    exit 42
fi

echo "extracting expat tarball"
tar xfz expat-2.0.1.tar.gz
if [ $? -ne 0 ]; then
    echo "ERROR tar xfz expat-2.0.1.tar.gz"
    exit 42
fi

echo "configuring expat sources"
cd expat-2.0.1
if [ $? -ne 0 ]; then
    echo "ERROR cd expat-2.0.1"
    exit 42
fi
./configure --prefix=$PREFIX
if [ $? -ne 0 ]; then
    echo "ERROR ./configure --prefix=$PREFIX"
    cd ..
    exit 42
fi

echo "building expat"
make
if [ $? -ne 0 ]; then
    echo "ERROR make"
    cd ..
    exit 42
fi

echo "installing expat"
make install
if [ $? -ne 0 ]; then
    echo "ERROR make install"
    cd ..
    exit 42
fi

echo "looks good, you should have expat now"
