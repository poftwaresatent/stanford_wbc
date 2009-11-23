#!/bin/sh

test -z "$1" && echo "$0: require source directory as argument"
test -z "$1" && exit 42

test -L tao && exit 0

echo "Creating symbolic link to source directory $1"
ln -s $1 tao
