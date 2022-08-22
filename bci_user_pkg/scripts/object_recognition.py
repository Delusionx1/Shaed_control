#!/usr/bin/env python3.7
# Must be run from scripts folder
# Import Statements
import pyrealsense2 as rs
import numpy as np
import traceback
import argparse
import sys, os
import time
import cv2
import matplotlib
matplotlib.use( 'tkagg' )
import matplotlib.pyplot as plt
from vision_recognition.rs_cam import rs_cam
from geometry_msgs.msg import PoseStamped
from math import sin, cos, radians
from pub_classes import obj_class
import rospy
from scipy.spatial.transform import Rotation
import tf
from tf import TransformListener


class object_detector:
    def __init__(self, frame_id, cameraInfo):
        # Setup object message publisher
        self.obj_obj = obj_class(frame_id=frame_id, queue=1)
        # Create pose transformer
        self.tf_listener_ = TransformListener()

        self.cameraInfo = cameraInfo

    def create_pose(self, x, y, angle, dist):
        # Transform point from pixels to meters
        result = rs.rs2_deproject_pixel_to_point(self.cameraInfo, [x, y], dist)

        # Create data in ROS Pose message
        p = PoseStamped()
        p.header.frame_id = "/camera_color_optical_frame"
        p.pose.position.x = result[0]
        p.pose.position.y = result[1]
        p.pose.position.z = result[2]
        # Make sure the quaternion is valid and normalized
        # angle = radians(angle+45)
        p.pose.orientation.x = sin(angle/2) * 0
        p.pose.orientation.y = sin(angle/2) * 0
        p.pose.orientation.z = sin(angle/2) * 1
        p.pose.orientation.w = cos(angle/2)
        # print(f"Pose in camera: {p}")
        return p

    def cam2rob_transform(self, pose, frame):
        ## Transform pose from camera frame to robot base frame
        p_in_base = None
        self.tf_listener_.waitForTransform("/base", frame, rospy.Time(), rospy.Duration(4.0))
        _ = self.tf_listener_.getLatestCommonTime("/base", frame)
        p_in_base = self.tf_listener_.transformPose("/base", pose)

        # Moveit inverse directions
        p_in_base.pose.position.x = -p_in_base.pose.position.x
        p_in_base.pose.position.y = -p_in_base.pose.position.y

        # Work with rotation in z axis only for ease. These lines might need adjusting.
        box_angle = Rotation.from_quat([p_in_base.pose.orientation.x, p_in_base.pose.orientation.y, p_in_base.pose.orientation.z, p_in_base.pose.orientation.w]).as_euler('xyz', degrees=True)
        p_in_base.pose.orientation.z = -(-box_angle[2]+90)

        # print(f"Pose in base: {p_in_base}")
        return p_in_base.pose

    def process_recognised_object(self, object_type, x, y, dist, angle):
        pose = self.create_pose(x, y, angle, dist)  # create object pose in camera frame
        pose = self.cam2rob_transform(pose, "/camera_color_optical_frame")  # transform pose to robot base frame
        self.obj_obj.publish(pose, object_type)  # publish ROS message

    def process_img(self, image, depth_image):
        # example recognitions [name, pixel x, pixel y, z rot angle]
        recognised_objects = [['blue_block', 210, 320, 34],
                              ['wood_block', 36, 470, 12],
                              ['red_tin', 340, 198, 48]]

        for [object_type, x, y, angle] in recognised_objects:
            dist = depth_image[y, x]
            self.process_recognised_object(object_type, x, y, dist, angle)


def object_recognition_run():
    # ROS node setup
    frame_id = 'camera_node'
    rospy.init_node(frame_id, anonymous=True)
    rate = rospy.Rate(10)  # Hz
    
    # Get initial frames from camera
    frames = cam.pipeline.wait_for_frames()
    color_image, depth_colormap, depth_image = cam.depth_frames(frames)
    cameraInfo = cam.depth_intrinsics
    
    # Create block detector
    detector = object_detector(frame_id, cameraInfo)

    while (not rospy.is_shutdown()):
        try:
            # Get frames from camera
            frames = cam.pipeline.wait_for_frames()
            color_image, depth_colormap, depth_image = cam.depth_frames(frames)
            detector.process_img(color_image, depth_image)

        except TypeError as e:
            print(e)
            time.sleep(1)
        except Exception as e:
            print("**Image Processing Error**")
            traceback.print_exc(file=sys.stdout)
            break

        if args.disp:
            disp_im = np.hstack((color_image, depth_colormap))
            
            cv2.namedWindow('Realsense viewer', cv2.WINDOW_NORMAL)
            cv2.imshow('Realsense viewer', disp_im)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run realsense vision recognition ROS node')
    parser.add_argument('--disp', '-V',
                        help='Enable displaying of camera image',
                        default=True,
                        action="store")

    args = parser.parse_known_args()[0]

    cam = rs_cam(args.disp)

    try:
        object_recognition_run()
    except rospy.ROSInterruptException:
        print("realsense_run ROS exception")
    except Exception as e:
        print("**Image Error**")
        traceback.print_exc(file=sys.stdout)
    finally:
        cam.pipeline.stop()
        cv2.destroyAllWindows()
        plt.close('all')
