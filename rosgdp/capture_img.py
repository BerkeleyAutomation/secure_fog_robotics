#!/usr/bin/env python

import cv2
import rospy
import sys
import time
import argparse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def _parse_args():
    """returns arguments passed to this node"""

    parser = argparse.ArgumentParser()
    parser.add_argument("type", choices=["color", "depth"],
                        help="Type of image to capture")
    parser.add_argument("path", type=str,
                        help="Path where the images ought to be stored, "
                             "the images are named as path-timestamp.png")
    parser.add_argument("-t", "--topic", type=str,
                        help="override default topics for image source")
    parser.add_argument("-p", "--prefix", type=str, default="",
                        help="simply add a prefix to the topic name. "
                             "Don't use it together with --topic")

    # cleaned up arguments 
    _args = rospy.myargv(argv=sys.argv)[1:]
    return parser.parse_args(args=_args)


def cb(data, (pre, enc)):
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, desired_encoding=enc)
        path = "%s-%s.png" % (pre, str(time.time()))
        print "Writing image to %s" % path
        cv2.imwrite(path, img)

    except CvBridgeError as e:
        print e


def init():

    rospy.init_node("capture_image", anonymous=True)
    args = _parse_args()

    if args.type == "color":
        topic = args.prefix + "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        encoding = "bgr8"
    else:
        topic = args.prefix + "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw"
        encoding = "passthrough"

    if args.topic is not None:
        topic = args.topic
    
    subscriber = rospy.Subscriber(topic, Image, cb,
                            callback_args=(args.path, encoding))
    rospy.spin()

if __name__=="__main__":
    init()
