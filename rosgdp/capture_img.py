#!/usr/bin/env python

import cv2
import gdp
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
    parser.add_argument("-g", "--gdp", action="store_true",
                        help="Path is the name of a GDP DataCapsule/Log "
                            "where images should be written to. "
                            "It writes 1 image/record. Note "
                            "that there is a size limitation on individual "
                            "record")

    # cleaned up arguments 
    _args = rospy.myargv(argv=sys.argv)[1:]
    return parser.parse_args(args=_args)


def localdir_cb(data, (pre, enc)):
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, desired_encoding=enc)
        path = "%s-%s.png" % (pre, str(time.time()))
        print "Writing image to %s" % path
        cv2.imwrite(path, img)

    except CvBridgeError as e:
        print e

def gdp_cb(data, (lh, enc)):

    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, desired_encoding=enc)
        img_str = cv2.imencode('.jpg', img)[1].tostring()
        print "Appending image of size: %d" % len(img_str)
        datum = gdp.GDP_DATUM()
        datum["buf"].reset()
        datum["buf"].write(img_str)
        lh.append(datum)

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
   
    if args.gdp:
        lh = gdp.GDP_GCL(gdp.GDP_NAME(args.path), gdp.GDP_MODE_RA)
        subscriber = rospy.Subscriber(topic, Image, gdp_cb,
                        callback_args=(lh, encoding))

    else: 
        subscriber = rospy.Subscriber(topic, Image, localdir_cb,
                            callback_args=(args.path, encoding))
    rospy.spin()

if __name__=="__main__":
    init()
