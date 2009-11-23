#!/bin/bash

PREFIX=`pwd`
if [ ! -z $1 ]; then
    if [ foo`echo $1 | sed 's:\(.\).*:\1:'` = "foo/" ]; then
        PREFIX=$1
    else
        PREFIX=`pwd`/$1
    fi
fi

if [ -e gtest-1.3.0 ]; then
    echo "looks like you already have gtest-1.3.0"
    echo "move it out of the way before running this script"
    exit 42
fi

echo "extracting gtest tarball"
tar xfj gtest-1.3.0.tar.bz2
if [ $? -ne 0 ]; then
    echo "ERROR tar xfj gtest-1.3.0.tar.bz2"
    exit 42
fi

echo "configuring gtest sources"
cd gtest-1.3.0
if [ $? -ne 0 ]; then
    echo "ERROR cd gtest-1.3.0"
    exit 42
fi
./configure --prefix=$PREFIX
if [ $? -ne 0 ]; then
    echo "ERROR ./configure --prefix=$PREFIX"
    cd ..
    exit 42
fi

echo "building gtest"
make
if [ $? -ne 0 ]; then
    echo "ERROR make"
    cd ..
    exit 42
fi

echo "installing gtest"
make install
if [ $? -ne 0 ]; then
    echo "ERROR make install, trying with sudo..."
    sudo make install
    if [ $? -ne 0 ]; then
	echo 'nah, that did not work either'
    fi
    cd ..
    exit 42
fi

echo "looks good, you should have gtest now"
