#!/usr/bin/env python
import rospy
import mediapipe as mp
import numpy as np

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


class Pose_detect:

    global traget_real_world_pose
    global target_pose_stamped

    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(static_image_mode=True)
        self.bridge = CvBridge()

    def recognize_object_pose(self, image):
        """
        find the object pose in the image
        """
        cv_image = self.bridge.imgmsg_to_cv2(
            image, desired_encoding='passthrough')
        h, w = cv_image.shape[0], cv_image.shape[1]

        results = self.pose.process(cv_image)
        if results.pose_landmarks:
            # self.mp_drawing.draw_landmarks(cv_image, results.pose_landmarks, self.mp_pose.PoseLandmark.RIGHT_WRIST)
            cx = int(
                results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST].x * w)
            cy = int(
                results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST].y * h)
            cv2.circle(cv_image, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(cv_image, "right_wrist", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3)

            dx = int(
                results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].x * w)
            dy = int(
                results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].y * h)

            if (cy < dy):
                cv2.putText(cv_image, "customer call", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            else:
                cv2.putText(cv_image, "nothing", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)

        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        image_pub.publish(image_message)


if __name__ == '__main__':
    rospy.init_node('pose_detect')
    pose = Pose_detect()
    rospy.Subscriber('/xtion/rgb/image_raw', Image, pose.recognize_object_pose)
    image_pub = rospy.Publisher('/result_image', Image, queue_size=1)
    rospy.spin()
