import cv2
import rospy
import numpy as np
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg
from hand_detection.yolo import YOLO

# Camera2Global
import tf
from sensor_msgs.msg import CameraInfo



class ObjectDetection:
    def __init__(self,
                 input_rgb_image_topic="/xtion/rgb/image_raw",
                 output_rgb_image_topic="/restaurant/object_detection",
                 detections_topic="/restaurant/object"):
        self.input_rgb_image_topic = input_rgb_image_topic
        self.bridge = CvBridge()
        self.image = None

        # Parameters for YOLO
        self.yolo = None
        self.size = 416
        self.number = -1
        self.confidence = 0.5
        self.is_hand_msg = Bool()

        # Parameters for coordinate transformation
        self.camera_info_sub = rospy.Subscriber("/xtion/rgb/camera_info", CameraInfo, self.camera_info_callback)
        self.listener = tf.TransformListener()
        self.distoration = None
        self.camera_info = None
        self.camera_matrix = None

        if output_rgb_image_topic is not None:
            self.image_publisher = rospy.Publisher(output_rgb_image_topic, Image, queue_size=1)
        else:
            self.image_publisher = None

        if detections_topic is not None:
            self.hand_publisher = rospy.Publisher(detections_topic, String, queue_size=1)
        else:
            self.hand_publisher = None

def listen(self):
        self.image_sub = rospy.Subscriber(self.input_rgb_image_topic, Image, self.callback)
        rospy.loginfo("Object detection node started.")
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('object_detection')
        model_cfg = package_path + '/models/yolov4.cfg'
        model_weights = package_path + '/models/yolov4.weights'
        print("loading yolo...")
        
        self.yolo = YOLO(model_cfg, model_weights,) # might limit the detected classes
        self.yolo.size = int(self.size)
        self.yolo.confidence = float(self.confidence)

def camera_info_callback(self, msg):
        self.camera_info = msg
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.distortion = np.array(msg.D)

def image_callback(self, msg):
     if self.camera_info is None:
        return
     
    self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    world_coordinates = self.convert_image_to_world_coordinates(self.image, self.camera_info)

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

            # how many objects should be shown
            object_count = len(results)
            if self.number != -1:
                object_count = int(self.number)

            # display objects
            confidences = [0]
            object={}
            for detection in results[:object_count]:
                id, name, confidence, x, y, w, h = detection
                confidences.append(confidence)
                cx = x + (w / 2)
                cy = y + (h / 2)
                coordinates = (cx, cy)

                object[name] = coordinates
                # draw a bounding box rectangle and label on the image
                color = (0, 255, 255)
                cv2.rectangle(self.image, (x, y), (x + w, y + h), color, 2)
                text = "%s (%s)" % (name, round(confidence, 2))
                cv2.putText(self.image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 2)  
                
            cv2.imshow('Object Detection Window', self.image)
            cv2.waitKey(3)

            # while not rospy.is_shutdown():
            # print(np.max(confidences))
            return object
        else:
            print('Waiting for ROS connection...')
            

    #         if np.max(confidences)>=0.90:
    #             self.is_hand_msg.data = True
    #         else:
    #             self.is_hand_msg.data = False
            
    #         self.hand_publisher.publish(self.is_hand_msg)
                
    #         if self.image_publisher is not None:
    #             try:
    #                 self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.image, 'bgr8'))
    #             except CvBridgeError as e:
    #                 rospy.loginfo(e)
    #     else:
    #         print('Waiting for ROS connection...')
        
    # def result(self):
    #     if self.is_hand_msg.data:
    #         return True
    #     else: 
    #         return False