#!/usr/bin/env python

import rospy
import roslib
import rostopic
import gdp
import pickle
import zlib
import sys
import argparse
from threading import Lock
from std_msgs.msg import String, Float64
from rospy.msg import AnyMsg

def _parse_args():
    """returns arguments passed to this node"""

    # setup argument parsing. These could be either passed on
    # command line (when invoking directly), or via a .launch file
    # when using roslaunch
    parser = argparse.ArgumentParser()
    parser.add_argument("--preserve", action="store_true", help="Preserve "
                            "the topic names as is. Default False")
    parser.add_argument("logname", type=str, help="GDP log")

    # cleaned up arguments 
    _args = rospy.myargv(argv=sys.argv)[1:]
    return parser.parse_args(args=_args)


def gdp_source():

    rospy.init_node("gdp_source")
    gdp.gdp_init()
    lock = Lock()
    args = _parse_args()

    topic_dict = {}

    gcl_name = gdp.GDP_NAME(args.logname)
    loghandle = gdp.GDP_GCL(gcl_name, gdp.GDP_MODE_RO)
    loghandle.subscribe(0, 0, None)

    try:
        buf = ""
        while not rospy.is_shutdown():

            event = loghandle.get_next_event(None)
            data = event["datum"]["data"]
            d = pickle.loads(data)
            if args.preserve:
                topic = d["topic"]
            else:
                topic = "/gdp" + rospy.names.resolve_name(d["topic"])
            topic_type = d["topic_type"]

            try:
                assert len(buf) == d["offset"]
            except AssertionError:
                ## This is when we start at the wrong time, and some
                ## chunks of a message already have been missed.
                continue

            buf = buf + d["data"]

            if len(buf) == d["msg_size"]:
                with lock: ## get publisher, create if doesn't exist
                    pub = topic_dict.get(topic, None)
                    if pub is None:
                        msg_class = roslib.message.get_message_class(topic_type)
                        pub = rospy.Publisher(topic, msg_class, queue_size=10)
                        topic_dict[topic] = pub 

                print "Publishing message"
                pub.publish(pickle.loads(zlib.decompress(buf)))
                buf = ""

    except rospy.ROSInterruptException:
        pass

    del loghandle

if __name__=="__main__":
    gdp_source()
