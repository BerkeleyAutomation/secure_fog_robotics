#!/usr/bin/env sh

set -e

PACKAGES="git" 

apt-get update
apt-get install -y $PACKAGES

## This assumes that GDP client side is already installed, and that
## other dependencies (notably protobuf) are installed as well.
git clone git://repo.eecs.berkeley.edu/projects/swarmlab/gdp-if.git
mv gdp-if/tensorflow gdpfs
rm -rf gdp-if
(cd gdpfs && make)

apt-get clean
rm -rf /var/lib/apt/lists/*
