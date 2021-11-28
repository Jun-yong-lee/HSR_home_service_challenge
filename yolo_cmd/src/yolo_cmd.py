#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist

box_data = []
person = None
motor_pub = None
cmd_msg = Twist()

def init_node():
    global motor_pub
    rospy.init_node('human_follow')
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
    motor_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def exit_node():
    print('finished')

def drive(angle, speed):
    global motor_pub
    cmd_msg.linear.x = speed
    cmd_msg.angular.z = angle
    motor_pub.publish(cmd_msg)

def callback(msg):
    global box_data
    box_data = msg

if __name__ == '__main__':
    init_node()
    time.sleep(15)
    rate = rospy.Rate(10)

    while box_data is None:
        rate.sleep()

    while not rospy.is_shutdown():
        nobady = True
        boxes = box_data
        for i in range(len(boxes.bounding_boxes)):
            if boxes.bounding_boxes[i].Class == "person":
                nobady = False
                center = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin)/2
                angle = float(2.84*((center - 320.0)/320.0))
                drive(angle, 0.17)
        if nobady:
            drive(0, 0)
        rospy.on_shutdown(exit_node)
