from unittest import result
from matplotlib import image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import os.path
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import torch

from PIL import Image as PILImage
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys


class ImageRecognizingNode(Node):

    def __init__(self, mode=0):
        super().__init__('image_recognizing_node')

        # VARIABLES
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.bridge = CvBridge()
        self.mode = int(mode)
        self.image_id = 0
        self.image_format = ".jpg"
        self.isSavingNeeded = False

        # SUBSCRIBER
        subscribed_image_topic = "/camera/image_raw"
        self.subscriber_ = self.create_subscription(
            Image,
            subscribed_image_topic,
            self.manage_new_image_callback,
            1)
        self.subscriber_  # prevent unused variable warning

        # PUBLISHER
        published_image_topic = "/camera/analyzed_image"
        self.publisher_ = self.create_publisher(
            Image, published_image_topic, 10)

        self.print_initialization_status()

    def manage_new_image_callback(self, image_raw_msg):

        self.analyze_image(image_raw_msg)

        match self.mode:

            case 0:  # OnDemand Saving Mode
                self.save_image()

            case 1:  # Continuous Publisher Mode
                self.publish_image()

    def analyze_image(self, image_raw_msg):
        cv2_img = self.bridge.imgmsg_to_cv2(image_raw_msg, "bgr8")
        results = self.model(cv2_img)
        results.render()
        cv2_img_recognized = results.imgs[0]
        self.cv2_img_recognized = cv2_img_recognized.astype(np.uint8) if isinstance(
            cv2_img_recognized, np.ndarray) else cv2_img_recognized

    def save_image(self):
        # Recognition nad saving
        if self.isSavingNeeded:
            cv2.imwrite(self.get_file_name(), self.cv2_img_recognized)

            print("INFO >> ImageRecognizingNode - image was saved")
            print(
                "INFO >> ImageRecognizingNode - if you want to recognize object and save image press 's' then enter")
            self.isSavingNeeded = False

        if 's' == input():
            self.isSavingNeeded = True
        else:
            print(
                "INFO >> ImageRecognizingNode - unrecognized input - TRY AGAIN : )")
            print(
                "INFO >> ImageRecognizingNode - if you want to recognize object and save image press 's' then enter")

    def publish_image(self):
        msg = Image()
        msg = self.bridge.cv2_to_imgmsg(self.cv2_img_recognized, "bgr8")
        self.publisher_.publish(msg)

# HELPERS

    def get_file_name(self):
        # increasing id
        while(os.path.exists(str(self.image_id) + self.image_format)):
            self.image_id = self.image_id + 1

        if self.image_id > 0:
            # decreasing id in case of folder change or photos deletion
            while():
                if self.image_id > 0:
                    smaller_id = self.image_id - 1
                else:
                    break

                if(not os.path.exists(str(smaller_id) + self.image_format)):
                    self.image_id = smaller_id

        return str(self.image_id) + self.image_format

    def print_initialization_status(self):
        print("INFO >> ImageRecognizingNode - constructed")
        match self.mode:
            case 0:  # OnDemandMode
                print("INFO >> ImageRecognizingNode - OnDemand Saving Mode")
                print(
                    "INFO >> ImageRecognizingNode - if you want to recognize object and save image press 's' then enter")
            case 1:  # ContinuousMode
                print("INFO >> ImageRecognizingNode - Continuous Publisher Mode")

            case _:
                print("INFO >> ImageRecognizingNode - UnknownMode = ", self.mode)


def main(args=None):

    rclpy.init(args=args)
    if len(sys.argv) == 1:
        image_subscriber = ImageRecognizingNode()
    else:
        image_subscriber = ImageRecognizingNode(sys.argv[1])

    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
