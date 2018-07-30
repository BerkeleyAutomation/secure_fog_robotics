#!/usr/bin/env python

"""
Reads JPG images from a GDP log and stores them in a local directory
"""

import gdp
import sys
import time
import argparse


def _parse_args():
    """returns arguments passed to this node"""

    parser = argparse.ArgumentParser()
    parser.add_argument("logname", type=str,
                        help="GDP log from where to read the images")
    parser.add_argument("path", type=str,
                        help="Path where the images ought to be stored, "
                             "the images are named as path-timestamp.png")
    parser.add_argument("-n", type=int, default=10,
                        help="store last N images")

    return parser.parse_args()



if __name__=="__main__":

    args = _parse_args()
    lh = gdp.GDP_GIN(gdp.GDP_NAME(args.logname), gdp.GDP_MODE_RO)
    lh.read_by_recno_async(-1*args.n, args.n)

    while True:

        ev = lh.get_next_event(None)
        if ev is None or ev["type"] == gdp.GDP_EVENT_DONE:
            break
        datum = ev["datum"]
        ts = datum["ts"]
        fname = "%s/image-%d-%d.jpg" % (args.path, ts["tv_sec"], ts["tv_nsec"])
        with open(fname, "w") as fh:
            fh.write(datum["buf"].peek())

