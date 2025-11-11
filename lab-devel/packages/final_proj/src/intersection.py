#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Intersector:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.Subscriber("/ee483mm01/camera_node/image/compressed", CompressedImage, self.image_callback, queue_size=1, buff_size=10000000)
        # self.pub_image = rospy.Publisher("/sim/image", Image, queue_size=10)
        # self.pub_cropped = rospy.Publisher("/sim/rqt_image_view/image_cropped", Image, queue_size=10)
        # self.pub_white = rospy.Publisher("/sim/rqt_image_view/image_white", Image, queue_size=10)
        # self.pub_yellow = rospy.Publisher("/sim/rqt_image_view/image_yellow", Image, queue_size=10)
        self.pub_red = rospy.Publisher("/sim/rqt_image_view/image_red", Image, queue_size=10)
        # self.pub_combined = rospy.Publisher("/sim/rqt_image_view/image_combined", Image, queue_size=10)
        # self.pub_edges_white = rospy.Publisher("/sim/rqt_image_view/edges_white", Image, queue_size=10)
        self.pub_edges_red = rospy.Publisher("/sim/rqt_image_view/edges_red", Image, queue_size=10)



    def output_lines(self, original_image, lines, line_color):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), line_color, 5, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 5, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 5, (0,0,255))
        return output



    def image_callback(self, msg):
        try:
            # Convert ROS image to cv
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            


            # Crop a bit less than top half
            height = cv_image.shape[0]
            cropped_image = cv_image[int(height/2.2):height]


            # Publish cropped image
            # cropped_msg = self.bridge.cv2_to_imgmsg(cropped_image,'bgr8')
            # self.pub_cropped.publish(cropped_msg)


            # Convert to HSV
            hsv_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

            # red color range in HSV
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([35, 255, 255])

            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([255, 255, 255])


        # Create mask and apply it
            red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
            red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(red_mask, red_mask2)
            red_filtered = cv2.bitwise_and(cropped_image, cropped_image, mask=red_mask)


            # Publish white-filtered image
            red_msg = self.bridge.cv2_to_imgmsg(red_filtered, 'bgr8')
            self.pub_red.publish(red_msg)

            #combined images
            
            red_cv = self.bridge.imgmsg_to_cv2(red_msg, 'bgr8')

        

            # get edges
            edges = cv2.Canny(cropped_image, 100, 255)
            

            #combine white and yellow mask with edges
            combined_red = cv2.bitwise_and(edges, red_mask)
            

            # hough transform
            lines_red = cv2.HoughLinesP(combined_red, 1, np.pi/180, 10, minLineLength=10, maxLineGap=50)
            output_red = self.output_lines(cropped_image, lines_red, (0,0,0))
           


            # Publish the image with lines
            output_msg = self.bridge.cv2_to_imgmsg(output_red, encoding='bgr8')
            self.pub_edges_red.publish(output_msg)



        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")



if __name__ == '__main__':

    try:
        rospy.init_node('intersector', anonymous=True)
        Intersector()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass