#!/usr/bin/env sh

set -e

PACKAGES="wget autoconf automake libtool curl make g++ unzip"

NUM_PROCS=`cat /proc/cpuinfo  | grep "^processor" | wc -l`
NUM_JOBS=$((NUM_PROCS*2))

## dependencies
apt-get update
apt-get install -y $PACKAGES

## download
wget https://github.com/google/protobuf/releases/download/v3.5.1/protobuf-cpp-3.5.1.tar.gz
tar xvzf protobuf-cpp-3.5.1.tar.gz

## compile and install
( cd protobuf-3.5.1 && \
  ./configure && \
  make -j $NUM_JOBS && \
  make -j $NUM_JOBS check && \
  make install && \
  ldconfig )

## Cleanup
rm -rf protobuf-*
# apt-get --purge -y remove $PACKAGES
# apt-get --purge -y autoremove
apt-get clean 
rm -rf /var/lib/apt/lists/*
