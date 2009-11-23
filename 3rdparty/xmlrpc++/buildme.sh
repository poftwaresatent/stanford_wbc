#!/bin/bash

PREFIX=`pwd`
if [ ! -z $1 ]; then
    if [ foo`echo $1 | sed 's:\(.\).*:\1:'` = "foo/" ]; then
        PREFIX=$1
    else
        PREFIX=`pwd`/$1
    fi
fi

if [ -e xmlrpc++ ]; then
    echo "looks like you already have xmlrpc++"
    echo "move it out of the way before running this script"
    exit 42
fi

echo "extracting xmlrpc++ tarball"
tar xfz xmlrpc++.tgz
if [ $? -ne 0 ]; then
    echo "ERROR tar xfz xmlrpc++.tgz"
    exit 42
fi

echo "patching xmlrpc++"
cd xmlrpc++
patch -p0 < ../patch
if [ $? -ne 0 ]; then
    echo "ERROR patch"
    cd ..
    exit 42
fi

echo "building xmlrpc++"
make prefix=$PREFIX
if [ $? -ne 0 ]; then
    echo "ERROR make"
    cd ..
    exit 42
fi

echo "installing xmlrpc++"
make prefix=$PREFIX install
if [ $? -ne 0 ]; then
    echo "ERROR make install, trying with sudo..."
    sudo make prefix=$PREFIX install
    if [ $? -ne 0 ]; then
	echo 'nah, that did not work either'
    fi
    cd ..
    exit 42
fi

echo "looks good, you should have xmlrpc++ now"
