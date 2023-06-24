#!/usr/bin/env python
import rospy
import ros_numpy
import mediapipe as mp
"""
$ pip install -q mediapipe==0.10.0
$ wget -q -O efficientdet.tflite -q https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/float32/latest/efficientdet_lite0.tflite
"""
from scipy.spatial.transform import Rotation as R


from sensor_msgs.msg import Image  
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped