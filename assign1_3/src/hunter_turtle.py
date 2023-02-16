#!/usr/bin/env python

import rospy
import time
import math
from turtlesim.srv import *

class Hunter:
    def __init__(self):
        rospy.wait_for_service('/spawn')
        spawn_hunter = rospy.ServiceProxy('/spawn', Spawn)
        spawn_hunter(1, 1, 0, "hunter_turtle")
        print("spawned the turtle")
        
if __name__ == "__main__":
    rospy.init_node('hunt')
    hunter = Hunter()
    hunter.start_hunt()
