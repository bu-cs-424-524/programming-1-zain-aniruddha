#!/usr/bin/env python

import rospy
import time
import math
from turtlesim.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtlesim.msg import Pose

class Hunter:
    def __init__(self):
        rospy.wait_for_service('/spawn')
        spawn_hunter = rospy.ServiceProxy('/spawn', Spawn)
        spawn_hunter(1, 1, 0, "hunter_turtle")

        rospy.wait_for_service('/kill')
        self.kill_hunter = rospy.ServiceProxy('/kill', Kill)

        self.linear_velocity = 2
        self.velocity_pub = rospy.Publisher('/hunter_turtle/cmd_vel', Twist, queue_size=10)
        self.kill_pub = rospy.Publisher('/hunter_turtle/kill_runner', Bool, queue_size=1)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = self.linear_velocity

        self.hunter_pos_subscriber = rospy.Subscriber('/hunter_turtle/pose', Pose, self.update_hunter_pose)
        self.runner_pose_subscriber = rospy.Subscriber('/runner_turtle/pose', Pose, self.update_runner_pose)

        self.hunter_pose = Pose(1, 1, 0, 0, 0)
        self.runner_pose = Pose(1, 1, 0, 0, 0)

        self.max_ang_vel = 2
        
    def start_hunt(self):
        while True:
            cur_ang = self.hunter_pose.theta

            x = self.hunter_pose.x - self.runner_pose.x
            y = self.hunter_pose.y - self.runner_pose.y
            req_ang = math.atan2(y, x)

            direction = 0
            if req_ang - cur_ang != 0:
                direction = -1 * (req_ang - cur_ang)/abs(req_ang - cur_ang)

            ang_vel = abs(req_ang - cur_ang)/math.pi
            if ang_vel > 1:
                ang_vel -= 2

            self.vel_msg.angular.z = direction * ang_vel * self.max_ang_vel
            self.velocity_pub.publish(self.vel_msg)

    def update_hunter_pose(self, pos):
        self.hunter_pose = pos

    def update_runner_pose(self, pos):
        self.runner_pose = pos
        
if __name__ == "__main__":
    rospy.init_node('hunt')
    hunter = Hunter()
    hunter.start_hunt()
    rospy.spin()
