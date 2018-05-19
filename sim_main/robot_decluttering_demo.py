import cv2
import IPython
import time
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy
import sys
import importlib

from core.hsr_robot_interface import Robot_Interface

"""
This class is for use with the robot
Pipeline it tests is labeling all objects in an image using web interface,
then choosing between and predicting a singulation or a grasp,
then the robot executing the predicted motion
"""

class SecureFogDemo():

    def __init__(self):
        """
        Class that runs decluttering routine with HSR

        """
        self.robot = Robot_Interface()       
        print "Finished init"


    def run_demo(self):
        c_img, d_img = self.robot.get_img_data()

        while not (c_img is None or d_img is None):
            c_img, d_img = self.robot.get_img_data()
	    cv2.imwrite("/home/ajay/debug.png", c_img)
	    time.sleep(2)
	
	if (c_img is None or d_img is None):
	    print("Images are not being retrieved.")
	    exit(1)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        DEBUG = True
    else:
        DEBUG = False

    task = SecureFogDemo()
    task.run_demo()
