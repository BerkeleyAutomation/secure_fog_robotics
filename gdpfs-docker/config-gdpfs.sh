#!/usr/bin/env sh

set -e

## These are our configuration parameters
echo "swarm.gdpfs.logprefix = edu.berkerley.tensorflow." >> /tmp/gdpfs
echo "swarm.gdp.debug = gdp.*=5" >> /tmp/gdpfs
echo "swarm.gdpfs.debug = gdpfs.*=19" >> /tmp/gdpfs

mkdir -p /etc/ep_adm_params
mv /tmp/gdpfs /etc/ep_adm_params
