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

class Object_detect:

    global traget_real_world_pose
    global target_pose_stamped

    def __init__(self):
        self.objectron = mp.solutions.objectron # 3D Object Detection https://github.com/google/mediapipe/blob/master/docs/solutions/objectron.md

    
    def recognize_object_pose(self,image):
        """
        find the object pose in the image
        """
        global traget_real_world_pose
        global target_pose_stamped

        with self.objectron.Objectron(static_image_mode=False,
                                      max_num_objects=1, # only grab one object
                                      model_name='Cup') as objectron:
            
            results = objectron.process(ros_numpy.numpify(image))

            if results.detected_objects: # only do when object is detected
                traget_pose_msg = Pose()
                three_dimention = results.detected_objects[0].translation # translated to [z,x,y]
                quaternion = R.from_matrix(results.detected_objects[0].rotation).as_quat() # [x,y,z,w]

                traget_pose_msg.position = Point(-three_dimention[2],-three_dimention[0],three_dimention[1]) # tiago's frame not the same as camera frame
                traget_pose_msg.orientation = Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])

                target_real_world_pose.publish(traget_pose_msg)
                
                PoseStamped_msg = PoseStamped(pose=traget_pose_msg)
                PoseStamped_msg.header.frame_id = "xtion_rgb_frame"

                target_pose_stamped.publish(PoseStamped_msg)

    if __name__=='__main__':
        rospy.init_node('object_detect')
        rospy.Subscriber('/xtion/rgb/image_raw', Image, recognize_object_pose)
        target_real_world_pose = rospy.Publisher('/object_pose/relative', Pose, queue_size=1)
        target_pose_stamped = rospy.Publisher('/object_pose/relative/stamped', PoseStamped, queue_size=1)
        rospy.spin()






