#!/usr/bin/env python3.7

import rospy
from std_msgs.msg import String
from bci_messages.msg import object_state


class obj_class:
    def __init__(self, frame_id, queue=1):
        # frame_id=str, queue=int
        self.obj_msg = object_state()
        self.obj_msg.Header.stamp = rospy.get_rostime()
        self.obj_msg.Header.seq = None
        self.obj_msg.Header.frame_id = frame_id
        self.obj_msg.obj_type = None
        self.obj_msg.Pose.orientation.x = None
        self.obj_msg.Pose.orientation.y = None
        self.obj_msg.Pose.orientation.z = None
        self.obj_msg.Pose.orientation.w = None
        self.obj_msg.Pose.position.x = None
        self.obj_msg.Pose.position.y = None
        self.obj_msg.Pose.position.z = None

        self.publisher = rospy.Publisher('ObjectStates', object_state, queue_size=queue)

    def publish(self, pose, object_type):
        if self.obj_msg.Header.seq is None:
            self.obj_msg.Header.seq = 0
        else:
            self.obj_msg.Header.seq += 1
        
        self.obj_msg.obj_type = object_type
        self.obj_msg.Pose = pose
        # self.obj_msg.Pose.orientation.x = 
        # self.obj_msg.Pose.orientation.y = 
        # self.obj_msg.Pose.orientation.z = 
        # self.obj_msg.Pose.orientation.w = 
        # self.obj_msg.Pose.position.x = 
        # self.obj_msg.Pose.position.y = 
        # self.obj_msg.Pose.position.z = 
        self.obj_msg.Header.stamp = rospy.get_rostime()

        self.publisher.publish(self.obj_msg)


class move_class:
    def __init__(self, queue=10):
        self.publisher = rospy.Publisher('RobotMove', String, queue_size=queue)

    def publish(self, command):
        self.publisher.publish(command)
