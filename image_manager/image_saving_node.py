from matplotlib import image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import os.path
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2


class ImageSavingNode(Node):

    def __init__(self):
        super().__init__('image_saving_node')

        image_topic = "/camera/image_raw"
        self.image_id = 0
        self.image_format = ".jpg"
        self.isSavingNeeded = False
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.save_image_callback,
            1)
        self.subscription  # prevent unused variable warning

        print("INFO >> ImageSavingNode - constructed")
        print("INFO >> ImageSavingNode - if you want to save image press 's' then enter")

    def save_image_callback(self, image_raw_msg):

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(image_raw_msg, "bgr8")
        except CvBridgeError as cv_bridge_error:
            print("ERROR >> ImageSavingNode - failed to save image - ",
                  cv_bridge_error)
        else:
            # Save your OpenCV2 image as a jpeg
            if self.isrecognizingNeeded:
                cv2.imwrite(self.get_file_name(), cv2_img)
                print("INFO >> ImageSavingNode - image was saved")
                print(
                    "INFO >> ImageSavingNode - if you want to save image press 's' then enter")
                self.isSavingNeeded = False

            if 's' == input():
                self.isSavingNeeded = True

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
            'testing_node_talker = image_manager.publisher_member_function:main',


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSavingNode()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
