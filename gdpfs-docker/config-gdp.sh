#!/usr/bin/env sh

set -e

## These are our configuration parameters
echo "swarm.gdp.zeroconf.enable=false" >> /tmp/gdp
echo "swarm.gdp.routers=$GDP_ROUTER" >> /tmp/gdp
echo "swarm.gdp.gdp-create.server=$GDP_CREATE_SERVER " >> /tmp/gdp

mkdir -p /etc/ep_adm_params
mv /tmp/gdp /etc/ep_adm_params



