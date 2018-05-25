#!/usr/bin/env python

import rospy
import rostopic
import gdp
import sys
import pickle
import zlib
import argparse
from threading import Lock
from rospy.msg import AnyMsg

## Let's start with a single-writer mode, logging to a single log
## that has been created in advance.

## We log a hardcoded list of topics to a single log. We note the name
## of the topic along with the message. For large messages, we need to
## split the message into multiple records.

MAX_RECSIZE = 60000

def callback(msg, (topic, lh, lck)):
    """Callback that serializes the message and logs to GDP"""

    with lck: # Locking is necessary to ensure no interleaving.

        print "Received message"
        buf = zlib.compress(pickle.dumps(msg))

        # Since we will soon have an upper limit on the size of
        # individual GDP records, we split large messages into
        # a sequence of records. The actual data that gets written
        # to a GDP log is a JSON dictionary that looks roughly like:
        # {topic: <>, msg_size: <>, offset: <>. data: <>}

        num_chunks = 1 + len(buf)/MAX_RECSIZE
        d = {}
        d["topic"] = topic
        _full_topic = rospy.names.resolve_name(topic)
        d["topic_type"] = rostopic.get_topic_type(_full_topic)[0]
        d["msg_size"] = len(buf)
    
        for i in xrange(0, len(buf), MAX_RECSIZE):
            d["offset"] = i
            d["data"] = buf[i:i+MAX_RECSIZE]
            serialized = pickle.dumps(d)
            lh.append({'data': serialized})

def _parse_args():
    """returns arguments passed to this node"""

    # setup argument parsing. These could be either passed on
    # command line (when invoking directly), or via a .launch file
    # when using roslaunch
    parser = argparse.ArgumentParser()
    parser.add_argument("logname", type=str, help="GDP log")
    parser.add_argument("-t", "--topic", action="append",
                            help="ROS topics to listen to (one or more).",
                            required=True)

    # cleaned up arguments 
    _args = rospy.myargv(argv=sys.argv)[1:]
    return parser.parse_args(args=_args)

def gdp_sink():

    rospy.init_node("gdp_sink")
    gdp.gdp_init()
    lock = Lock()
    args = _parse_args()

    gcl_name = gdp.GDP_NAME(args.logname)
    lh = gdp.GDP_GCL(gcl_name, gdp.GDP_MODE_RA)

    for t in args.topic:
        rospy.Subscriber(t, AnyMsg, callback, callback_args=(t, lh, lock))

    rospy.spin()

if __name__=="__main__":
    gdp_sink()
