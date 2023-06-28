import cv2
import rospy
import numpy as np
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg
from hand_detection.yolo import YOLO


class HandDetection:
    def __init__(self,
                 input_rgb_image_topic="/xtion/rgb/image_raw",
                 output_rgb_image_topic="/restaurant/hand_detection",
                 detections_topic="/restaurant/hand_exist"):
        
        self.input_rgb_image_topic = input_rgb_image_topic
        self.bridge = CvBridge()
        self.image = None

        # Parameters for YOLO
        self.yolo = None
        self.size = 416
        self.hands = -1
        self.confidence = 0.7
        self.is_hand_msg = Bool()
        self.is_hand = False

        if output_rgb_image_topic is not None:
            self.image_publisher = rospy.Publisher(output_rgb_image_topic, Image, queue_size=1)
        else:
            self.image_publisher = None

        if detections_topic is not None:
            self.hand_publisher = rospy.Publisher(detections_topic, Bool, queue_size=1)
        else:
            self.hand_publisher = None


    def listen(self):        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('hand_detection')
        model_cfg = package_path + '/models/cross-hands.cfg'
        model_weights = package_path + '/models/cross-hands.weights'
        print("loading yolo...")
        
        self.yolo = YOLO(model_cfg, model_weights, ["hand"])
        self.yolo.size = int(self.size)
        self.yolo.confidence = float(self.confidence)
        self.image_sub = rospy.Subscriber(self.input_rgb_image_topic, Image, self.callback)
        rospy.loginfo("Hand detection node started.")
        rospy.wait_for_message(self.input_rgb_image_topic, Image)



    def callback(self, msg):
        """
        Callback that processes the input data and publishes to the corresponding topics.
        :param data: Input image message
        :type data: sensor_msgs.msg.Image
        """
        # Convert sensor_msgs.msg.Image into OpenCV Image
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # rospy.loginfo(self.image.shape)
        except CvBridgeError as e:
            rospy.loginfo(e)

        if self.image is not None:
            width, height, inference_time, results = self.yolo.inference(self.image)

            # display fps
            cv2.putText(self.image, f'{round(1/inference_time,2)} FPS', (15,15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,255,255), 2)

            # sort by confidence
            results.sort(key=lambda x: x[2])

            # how many hands should be shown
            hand_count = len(results)
            if self.hands != -1:
                hand_count = int(self.hands)

            # display hands
            confidences = [0]
            for detection in results[:hand_count]:
                id, name, confidence, x, y, w, h = detection
                confidences.append(confidence)
                cx = x + (w / 2)
                cy = y + (h / 2)

                # draw a bounding box rectangle and label on the image
                color = (0, 255, 255)
                cv2.rectangle(self.image, (x, y), (x + w, y + h), color, 2)
                text = "%s (%s)" % (name, round(confidence, 2))
                cv2.putText(self.image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 2)  
                
            cv2.imshow('Hand Detection Window', self.image)
            cv2.waitKey(3)

            # while not rospy.is_shutdown():
            # print(np.max(confidences))
            if np.max(confidences)>=0.92:
                self.is_hand_msg.data = True
                self.is_hand = True
            else:
                self.is_hand_msg.data = False
            
            self.hand_publisher.publish(self.is_hand_msg)
                
            if self.image_publisher is not None:
                try:
                    self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.image, 'bgr8'))
                except CvBridgeError as e:
                    rospy.loginfo(e)
        else:
            print('Waiting for ROS connection...')
        
    def result(self):
        return self.is_hand
        
    
    
            
# def main():
#     # parser = argparse.ArgumentParser()
#     # parser.add_argument('-s', '--size', default=416, help='Size for yolo')
#     # parser.add_argument('-c', '--confidence', default=0.2, help='Confidence for yolo')
#     # parser.add_argument('-nh', '--hands', default=-1, help='Total number of hands to be detected per frame (-1 for all)')
#     # args = parser.parse_args()

#     """
#         Start the node and begin processing input data.
#         """     
#     hand_detection_node = HandDetection()
#     hand_detection_node.listen()
#     rospy.init_node('hand_detection_node', anonymous=True)
#     rospy.Rate(10)
#     # rospy.loginfo("Hand detection node started.")
    
#     # if args.network == "normal":
#     #     rospy.loginfo("loading yolo...")
#     #     yolo = YOLO("models/cross-hands.cfg", "models/cross-hands.weights", ["hand"])
#     # elif args.network == "prn":
#     #     rospy.loginfo("loading yolo-tiny-prn...")
#     #     yolo = YOLO("models/cross-hands-tiny-prn.cfg", "models/cross-hands-tiny-prn.weights", ["hand"])
#     # elif args.network == "v4-tiny":
#     #     rospy.loginfo("loading yolov4-tiny-prn...")
#     #     yolo = YOLO("models/cross-hands-yolov4-tiny.cfg", "models/cross-hands-yolov4-tiny.weights", ["hand"])
#     # else:
#     #     rospy.loginfo("loading yolo-tiny...")
#     #     yolo = YOLO("models/cross-hands-tiny.cfg", "models/cross-hands-tiny.weights", ["hand"])

#     # model_cfg = rospy.get_param('model_cfg')
#     # model_weights = rospy.get_param('model_weights')
#     # model_cfg = '/home/xlei/ros/tiago_public_ws/src/final/hand_detection/models/cross-hands.cfg'
#     # model_weights = '/home/xlei/ros/tiago_public_ws/src/final/hand_detection/models/cross-hands.weights'
#     # rospy.loginfo("loading yolo...")
#     # yolo = YOLO(model_cfg, model_weights, ["hand"])
#     # yolo.size = int(args.size)
#     # yolo.confidence = float(args.confidence)

#     rospy.spin()
#     cv2.destroyAllWindows()


if __name__ == '__main__':
    hand_detection_node = HandDetection()
    hand_detection_node.listen()
    rospy.init_node('hand_detection_node', anonymous=True)
    rospy.Rate(10)
    rospy.spin()
    cv2.destroyAllWindows()