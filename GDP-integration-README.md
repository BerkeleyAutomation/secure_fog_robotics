# Usage instructions for various GDP components

This document describes the current state of various GDP components. Note that
this document is in flux.

## Creating new logs

Please use `gdp-create` for creating new logs. This is distributed as part of
the GDP client side installation.

## Capturing images using ROS

Please use `rosgdp/capture_img.py`. This script connects to the ROS network,
subscribes to a default topic for either `color` or `depth` images, and
dumps those images to either a GDP log (with `-g` option) or a local directory.

Note that this requires an appropriate ROS setup. Also note that this requires
the log to be already created. An example invocation:

`python rosgdp/capture_img.py -g color <gdplogname>`

See `python rosgdp/capture_img.py -h` for a full list of options.

## Recovering images from GDP

The images stored in a GDP log using `capture_img.py` can be extracted and
stored to a local directory using `rosgdp/read_image.py`. Please see `python
rosgdp/read_image.py -h` for a full list of options.

## Storing data in a GDP file-system

Please see `gdp-if/tensorflow/README.md`

## Running GDP file-system with Tensorflow

Please see `gdp-if/tensorflow/README.md`
