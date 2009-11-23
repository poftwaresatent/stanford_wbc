#!/bin/bash

VERSION="0.10.0"
PACKAGE="apache-log4cxx-${VERSION}"
DOWNLOAD_URL="http://www.apache.org/dyn/closer.cgi/logging/log4cxx/${VERSION}/${PACKAGE}.tar.gz"

PREFIX=`pwd`
if [ ! -z $1 ]; then
    if [ foo`echo $1 | sed 's:\(.\).*:\1:'` = "foo/" ]; then
        PREFIX=$1
    else
        PREFIX=`pwd`/$1
    fi
fi

if [ -e ${PACKAGE} ]; then
    echo "looks like you already have ${PACKAGE}"
    echo "move it out of the way before running this script"
    exit 42
fi

if [ ! -f ${PACKAGE}.tar.gz ]; then
    echo "There is no ${PACKAGE}.tar.gz, please go to"
    echo "  ${DOWNLOAD_URL}"
    echo "and download it from there.  Alternatively, choose a more recent"
    echo "version by visiting"
    echo "  http://logging.apache.org/log4cxx/download.html"
    echo "and then changing the VERSION variable in this build script."
    exit 42
fi

echo "extracting tarball"
tar xfz ${PACKAGE}.tar.gz
if [ $? -ne 0 ]; then
    echo "ERROR tar xfz ${PACKAGE}.tar.gz"
    exit 42
fi

echo "patching sources"
patch -p0 < patch
if [ $? -ne 0 ]; then
    echo "ERROR patch"
    exit 42
fi

echo "configuring sources"
cd ${PACKAGE}
if [ $? -ne 0 ]; then
    echo "ERROR cd ${PACKAGE}"
    exit 42
fi
./configure --prefix=$PREFIX
if [ $? -ne 0 ]; then
    echo "ERROR ./configure --prefix=$PREFIX"
    echo "If the configure script complained about missing APR or APR-util,"
    echo "install them either from source (see http://apr.apache.org/) or"
    echo "by using your package manager, e.g."
    echo " $ sudo apt-get install libapr1-dev libaprutil1-dev"
    cd ..
    exit 42
fi

echo "building"
make
if [ $? -ne 0 ]; then
    echo "ERROR make"
    cd ..
    exit 42
fi

echo "installing"
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

echo "looks good, you should have ${PACKAGE} now"
