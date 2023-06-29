#!/usr/bin/env python
import rospy
import tf2_ros
from tf.transformations import *
from tf2_geometry_msgs import *
from geometry_msgs.msg import Pose, Quaternion, PointStamped
from .srv import Rel2Abs, RelToAbsoluteResponse

class GetAbs:
    """
    input: relative position of the object
    oupur: absolute position of the object
    """
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.absolute_pose = Pose()
    
    def getAbs(self, relative_pos):
        """
        Transform between xtion_rgb_frame & base_footprint
        """
        rospy.loginfo('start to get absolute position')
        trans_base_rel = self.tfBuffer.lookup_transform('base_footprint',
                                                  relative_pos.relative_pose.header.frame_id, rospy.Time())
        point_from_rel = PointStamped(point=relative_pos.relative_pose.pose.position)

        abs_pos = Pose()
        abs_pos.position = do_transform_point(point_from_rel, trans_base_rel).point #tf.transformations

        # calculates rotation between xtion_rgb_frame & base_footprint
        q0 = [trans_base_rel.transform.rotation.x, trans_base_rel.transform.rotation.y,
            trans_base_rel.transform.rotation.z, trans_base_rel.transform.rotation.w]

        # calculates orientation of the object in relative pose
        q1 = [relative_pos.relative_pose.pose.orientation.x, relative_pos.relative_pose.pose.orientation.y,
            relative_pos.relative_pose.pose.orientation.z, relative_pos.relative_pose.pose.orientation.w]
        
        # final orientation of the object in absolute pose
        q = quaternion_multiply(q1, q0)

        # Quaternion coordinate (x,y,z,w)[rad]
        abs_pos.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self.absolute_pose.pose=abs_pos
        self.absolute_pose.header.frame_id = 'base_footprint'

        return self.absolute_pose
    
if __name__ == '__main__':

    # ros node initialization --> Get Absolute Pose
    rospy.init_node('GetAbs')

    # service pass relative to abs position info on --> geometry_msgs/PoseStamped to communicate with other node
    absolute_service = rospy.Service("grasp_object/rel_to_absolute_pose",Rel2Abs,GetAbs().getAbs)

    rospy.loginfo("GetAbsolutePose")
    
    # start infinite loop until it receives a shutdown  signal
    rospy.spin()