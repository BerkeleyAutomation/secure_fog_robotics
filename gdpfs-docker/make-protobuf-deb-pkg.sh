#!/usr/bin/env sh

## Creates a debian package for protobuf. This compilation should be
## a one time task, and wouldn't be even necessary if protobuf folks
## distributed compiled binaries.

set -e

PACKAGES="fakeroot checkinstall wget autoconf automake libtool curl make g++ unzip"
VER=3.5.1

NUM_PROCS=`cat /proc/cpuinfo  | grep "^processor" | wc -l`
NUM_JOBS=$((NUM_PROCS*2))

## dependencies
sudo apt-get update
sudo apt-get install -y $PACKAGES

## download
wget https://github.com/google/protobuf/releases/download/v$VER/protobuf-cpp-$VER.tar.gz
tar xvzf protobuf-cpp-$VER.tar.gz

## compile and install
# ( cd protobuf-$VER && \
#   ./configure && \
#   make -j $NUM_JOBS && \
#   make -j $NUM_JOBS check && \
#   make install && \
#   ldconfig )

( cd protobuf-$VER && \
  ./configure --prefix=/usr && \
  make -j $NUM_JOBS && \
  make -j $NUM_JOBS check && \
  sudo checkinstall -D --install=no -y \
	    --pkgname=protobuf-dev \
	    --pkgversion=$VER \
	    --pkglicense="See /LICENSE" \
	    --pkggroup="libs" \
	    --maintainer="mor@eecs.berkeley.edu" \
	    make install )


## Cleanup
# rm -rf protobuf-*
# apt-get --purge -y remove $PACKAGES
# apt-get --purge -y autoremove
# apt-get clean 
# rm -rf /var/lib/apt/lists/*
