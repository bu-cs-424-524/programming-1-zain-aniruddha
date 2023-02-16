#!/usr/bin/env python

import rospy
from turtlesim.srv import *
from geometry_msgs.msg import Twist
import time
import math

class DrawName:
    def __init__(self):
        rospy.wait_for_service('/kill')
        killer = rospy.ServiceProxy('/kill', Kill)
        killer('turtle1')

        rospy.wait_for_service('/spawn')
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(2, 7, 0, 'turtle')

        # Get publisher to move turtle
        self.velocity_publisher = rospy.Publisher('/turtle/cmd_vel', Twist, queue_size=10)

        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0

    def reset_vel_msg(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

        self.velocity_publisher.publish(self.vel_msg)
        
    def move_straight(self, x_speed, y_speed, x_distance, y_distance):
        self.reset_vel_msg()
        
        self.vel_msg.linear.x = abs(x_speed)
        self.vel_msg.linear.y = abs(y_speed)
        t0 = rospy.Time.now().to_sec()
        current_x_distance = 0
        current_y_distance = 0

        while (current_x_distance < x_distance or current_y_distance < y_distance):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_x_distance = x_speed * (t1 - t0)
            current_y_distance = y_speed * (t1 - t0)

        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.velocity_publisher.publish(self.vel_msg)

    def rotate(self, speed, angle, rotation):
        self.reset_vel_msg()
        
        angular_speed = speed * 2 * math.pi/360
        relative_angle = angle * 2 * math.pi/360

        if (rotation == 'clockwise'):
            self.vel_msg.angular.z = -abs(angular_speed)
        else:
            self.vel_msg.angular.z = abs(angular_speed)

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while (current_angle < relative_angle):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)

        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
            
if __name__ == '__main__':
    rospy.init_node('draw_name')
    
    draw_name = DrawName()

    travel_speed = 2
    rotation_speed = 100
    
    draw_name.move_straight(x_speed=travel_speed, y_speed=0, x_distance=3, y_distance=0)

    draw_name.rotate(speed=rotation_speed, angle=135, rotation='clockwise')
    draw_name.move_straight(x_speed=travel_speed, y_speed=0, x_distance=3, y_distance=0)

    draw_name.rotate(speed=rotation_speed, angle=135, rotation='anti-clockwise')
    draw_name.move_straight(x_speed=travel_speed, y_speed=0, x_distance=3, y_distance=0)

    draw_name.rotate(speed=rotation_speed, angle=70, rotation='anti-clockwise')
    draw_name.move_straight(x_speed=travel_speed, y_speed=0, x_distance=2.5, y_distance=0)

    draw_name.rotate(speed=rotation_speed, angle=140, rotation='clockwise')
    draw_name.move_straight(x_speed=travel_speed, y_speed=0, x_distance=2.5, y_distance=0)

    draw_name.rotate(speed=rotation_speed, angle=180, rotation='clockwise')
    draw_name.move_straight(x_speed=travel_speed, y_speed=0, x_distance=1.25, y_distance=0)

    draw_name.rotate(speed=rotation_speed, angle=70, rotation='anti-clockwise')
    draw_name.move_straight(x_speed=travel_speed, y_speed=0, x_distance=1.5, y_distance=0)

    rospy.spin()
