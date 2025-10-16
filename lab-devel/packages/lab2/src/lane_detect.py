#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.Subscriber("/ee483mm01/camera_node/image/compressed", CompressedImage, self.image_callback, queue_size=1, buff_size=10000000)
        self.pub_image = rospy.Publisher("/sim/image", Image, queue_size=10)
        self.pub_cropped = rospy.Publisher("/sim/rqt_image_view/image_cropped", Image, queue_size=10)
        self.pub_white = rospy.Publisher("/sim/rqt_image_view/image_white", Image, queue_size=10)
        self.pub_yellow = rospy.Publisher("/sim/rqt_image_view/image_yellow", Image, queue_size=10)
        self.pub_combined = rospy.Publisher("/sim/rqt_image_view/image_combined", Image, queue_size=10)


    def image_callback(self, msg):
        try:
            # Convert ROS image to cv
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            


            # Crop a bit less than top half
            height = cv_image.shape[0]
            cropped_image = cv_image[int(height/2.2):height]


            # Publish cropped image
            cropped_msg = self.bridge.cv2_to_imgmsg(cropped_image,'bgr8')
            self.pub_cropped.publish(cropped_msg)


            # Convert to HSV
            hsv_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

            # bluish white color range in HSV
            lower_white = np.array([80, 0, 180])
            upper_white = np.array([140, 70, 255])


        # Create mask and apply it
            mask = cv2.inRange(hsv_image, lower_white, upper_white)
            white_filtered = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)


            # Publish white-filtered image
            white_msg = self.bridge.cv2_to_imgmsg(white_filtered, 'bgr8')
            self.pub_white.publish(white_msg)

            lower_yellow = np.array([15, 100, 100])
            upper_yellow = np.array([35, 255, 255])


        # Create mask and apply it
            mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
            yellow_filtered = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)


            # Publish white-filtered image
            yellow_msg = self.bridge.cv2_to_imgmsg(yellow_filtered, 'bgr8')
            self.pub_yellow.publish(yellow_msg)

            #combined images
            yellow_cv = self.bridge.imgmsg_to_cv2(yellow_msg, 'bgr8')
            white_cv = self.bridge.imgmsg_to_cv2(white_msg, 'bgr8')

            combined = cv2.bitwise_or(yellow_cv, white_cv)

            cv_img_msg = self.bridge.cv2_to_imgmsg(combined, 'bgr8')
            self.pub_combined.publish(cv_img_msg)



        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")



if __name__ == '__main__':

    try:
        rospy.init_node('image_processor', anonymous=True)
        ImageProcessor()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass