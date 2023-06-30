#!/usr/bin/env python
import rospy
import mediapipe as mp

from cv_bridge import CvBridge

from sensor_msgs.msg import Image  
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

class Object_detect:

    global traget_real_world_pose
    global target_pose_stamped

    def __init__(self):
        self.objectron = mp.solutions.objectron # 3D Object Detection https://github.com/google/mediapipe/blob/master/docs/solutions/objectron.md
        self.drawing = mp.solutions.drawing_utils
        self.bridge = CvBridge()

    
    def recognize_object_pose(self,image):
        """
        find the object pose in the image
        """
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        global traget_real_world_pose
        global target_pose_stamped

        with self.objectron.Objectron(static_image_mode=False,
                                      max_num_objects=10, # only grab one object
                                      model_name='Cup') as objectron:

            results = objectron.process(cv_image)

            if results.detected_objects: # only do when object is detected
                print("detected")
                for detected_object in results.detected_objects:
                    self.drawing.draw_landmarks(cv_image, detected_object.landmarks_2d, self.objectron.BOX_CONNECTIONS)
                    self.drawing.draw_axis(cv_image, detected_object.rotation,detected_object.translation)
                # traget_pose_msg = Pose()
                # three_dimention = results.detected_objects[0].translation # translated to [z,x,y]
                # quaternion = R.from_matrix(results.detected_objects[0].rotation).as_quat() # [x,y,z,w]

                # traget_pose_msg.position = Point(-three_dimention[2],-three_dimention[0],three_dimention[1]) # tiago's frame not the same as camera frame
                # traget_pose_msg.orientation = Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])

                # target_real_world_pose.publish(traget_pose_msg)
                
                # PoseStamped_msg = PoseStamped(pose=traget_pose_msg)
                # PoseStamped_msg.header.frame_id = "xtion_rgb_frame"

                # target_pose_stamped.publish(PoseStamped_msg)
        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        image_pub.publish(image_message)

if __name__=='__main__':
    rospy.init_node('object_detect')
    rospy.Subscriber('/xtion/rgb/image_raw', Image, Object_detect().recognize_object_pose)
    target_real_world_pose = rospy.Publisher('/object_pose/relative', Pose, queue_size=1)
    target_pose_stamped = rospy.Publisher('/object_pose/relative/stamped', PoseStamped, queue_size=1)
    image_pub = rospy.Publisher('/result_image', Image, queue_size=1)
    rospy.spin()






