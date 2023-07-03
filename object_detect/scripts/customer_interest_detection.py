import cv2
import math
import numpy as np
import itertools
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2

import rospy
from cv_bridge import CvBridge
import std_msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


class Customer_Interest:
    def __init__(self) -> None:
        # create a pose detector of mediapipe
        self.pose_detector = mp.tasks.vision.PoseLandmarker.create_from_options(
            mp.tasks.vision.PoseLandmarkerOptions(
                base_options=mp.tasks.BaseOptions(
                    model_asset_path="./src/RoboCup-Final/object_detect/model/pose_landmarker_heavy.task"),
                running_mode=mp.tasks.vision.RunningMode.IMAGE,
                num_poses=2))

        # create a face detector of mediapipe
        self.face_detector = mp.tasks.vision.FaceLandmarker.create_from_options(
            mp.tasks.vision.FaceLandmarkerOptions(
                base_options=mp.tasks.BaseOptions(
                    model_asset_path="./src/RoboCup-Final/object_detect/model/face_landmarker.task"),
                running_mode=mp.tasks.vision.RunningMode.IMAGE,
                num_faces=2))

        # create visualization tool of mediapipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/xtion/rgb/image_raw', Image,
                                    self.process)
        self.image_pub = rospy.Publisher('/result_image', Image, queue_size=1)
        self.pub = rospy.Publisher(
            '/customer_interest', std_msgs.msg.String, queue_size=10)

    def process(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(
            image_msg, desired_encoding='passthrough')
        self.img_height, self.img_width = image.shape[0], image.shape[1]
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        pose_result = self.pose_detector.detect(mp_image)
        face_result = self.face_detector.detect(mp_image)

        customer_interest = False

        for idx in range(len(pose_result.pose_landmarks)):
            pose_landmarks = pose_result.pose_landmarks[idx]
            if (idx < len(face_result.face_landmarks)):
                face_landmarks = face_result.face_landmarks[idx]
                if self.determine_interest(image, pose_landmarks, face_landmarks):
                    customer_interest = True
                    break

                # if self.determine_interest(image, pose_landmarks, face_landmarks):
                #     cv2.putText(image, "customer_call", (50, 50),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                # else:
                #     cv2.putText(image, "nothing", (50, 50),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)
        # image_message = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        # self.image_pub.publish(image_message)
        if customer_interest:
            self.pub.publish("True")
        else:
            self.pub.publish("False")

    def visualization(self, image, pose_landmarks, face_landmarks):
        # Draw the pose landmarks.
        pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        pose_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
        ])
        self.mp_drawing.draw_landmarks(
            image=image,
            landmark_list=pose_landmarks_proto,
            connections=mp.solutions.pose.POSE_CONNECTIONS,
            landmark_drawing_spec=self.mp_drawing_styles
            .get_default_pose_landmarks_style())

        # Draw the face landmarks.
        face_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        face_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in face_landmarks
        ])
        self.mp_drawing.draw_landmarks(
            image=image,
            landmark_list=face_landmarks_proto,
            connections=mp.solutions.face_mesh.FACEMESH_IRISES,
            landmark_drawing_spec=None,
            connection_drawing_spec=mp.solutions.drawing_styles
            .get_default_face_mesh_iris_connections_style())

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    def determine_interest(self, image, pose_landmarks, face_landmarks) -> bool:

        # if customer raise right hand
        _, right_wrist_y = self.get_landmark_position(
            pose_landmarks, mp.solutions.pose.PoseLandmark.RIGHT_WRIST)
        _, right_shoulder_y = self.get_landmark_position(
            pose_landmarks, mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER)
        right_hand_up = True if right_wrist_y < right_shoulder_y else False

        # if customer raise left hand
        _, left_wrist_y = self.get_landmark_position(
            pose_landmarks, mp.solutions.pose.PoseLandmark.LEFT_WRIST)
        _, left_shoulder_y = self.get_landmark_position(
            pose_landmarks, mp.solutions.pose.PoseLandmark.LEFT_SHOULDER)
        left_hand_up = True if left_wrist_y < left_shoulder_y else False

        face_me = self.if_face_me(image, face_landmarks)

        return face_me and (right_hand_up or left_hand_up)

    def if_face_me(self, image, face_landmarks):
        # if customer face to me
        cx = self.img_width / 2
        cy = self.img_height / 2
        fx = cx / np.tan(60 / 2 * np.pi / 180)
        fy = fx
        camera_matrix = np.float32([[fx, 0.0, cx],
                                    [0.0, fy, cy],
                                    [0.0, 0.0, 1.0]])

        dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

        # 3D model points.
        model_points = np.array([
            (0.0, 0.0, 0.0),            # Nose tip
            (0.0, -330.0, -65.0),       # Chin
            (-225.0, 170.0, -135.0),    # Left eye left corner
            (225.0, 170.0, -135.0),     # Right eye right corner
            (-150.0, -150.0, -125.0),   # Left Mouth corner
            (150.0, -150.0, -125.0)     # Right mouth corner
        ], dtype=np.float64)

        image_points = np.array([
            self.get_landmark_position(face_landmarks, 1),      # "nose"
            self.get_landmark_position(face_landmarks, 152),    # "chin"
            self.get_landmark_position(face_landmarks, 226),    # "left eye"
            self.get_landmark_position(face_landmarks, 446),    # "right eye"
            self.get_landmark_position(face_landmarks, 57),     # "left mouth"
            self.get_landmark_position(face_landmarks, 287)     # "right mouth"
        ], dtype=np.float64)

        (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix,
                                                                      dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        (R, j) = cv2.Rodrigues(rotation_vector)
        roll, pitch, yaw = self.rotationMatrixToEulerAngles(R)

        if pitch < 5.0 and yaw < 5.0:
            return True
        else:
            return False

        # (nose_end_point2D, jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector,
        #                                                  translation_vector, camera_matrix, dist_coeffs)

        # p1 = (int(image_points[0][0]), int(image_points[0][1]))
        # p2 = (int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))

        # cv2.line(image, p1, p2, (255, 0, 0), 2)

    def rotationMatrixToEulerAngles(self, R):

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.rad2deg(x), np.rad2deg(y), np.rad2deg(z)

    def get_landmark_position(self, landmarks, name):
        return int(landmarks[name].x * self.img_width), int(landmarks[name].y * self.img_height)


if __name__ == '__main__':
    rospy.init_node('customer_interest_detect')
    ci = Customer_Interest()
    rospy.spin()
