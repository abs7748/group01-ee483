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
        # self.pub_image = rospy.Publisher("/sim/image", Image, queue_size=10)
        self.pub_cropped = rospy.Publisher("/sim/rqt_image_view/image_cropped", Image, queue_size=10)
        self.pub_white = rospy.Publisher("/sim/rqt_image_view/image_white", Image, queue_size=10)
        self.pub_yellow = rospy.Publisher("/sim/rqt_image_view/image_yellow", Image, queue_size=10)
        self.pub_combined = rospy.Publisher("/sim/rqt_image_view/image_combined", Image, queue_size=10)
        self.pub_edges_white = rospy.Publisher("/sim/rqt_image_view/edges_white", Image, queue_size=10)
        self.pub_edges_yellow = rospy.Publisher("/sim/rqt_image_view/edges_yellow", Image, queue_size=10)



    def output_lines(self, original_image, lines, line_color):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), line_color, 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output



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
            lower_white = np.array([80, 0, 150])
            upper_white = np.array([140, 70, 255])


        # Create mask and apply it
            white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
            white_filtered = cv2.bitwise_and(cropped_image, cropped_image, mask=white_mask)


            # Publish white-filtered image
            white_msg = self.bridge.cv2_to_imgmsg(white_filtered, 'bgr8')
            self.pub_white.publish(white_msg)

            lower_yellow = np.array([15, 50, 100])
            upper_yellow = np.array([50, 255, 255])


        # Create mask and apply it
            yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
            yellow_filtered = cv2.bitwise_and(cropped_image, cropped_image, mask=yellow_mask)


            # Publish white-filtered image
            yellow_msg = self.bridge.cv2_to_imgmsg(yellow_filtered, 'bgr8')
            self.pub_yellow.publish(yellow_msg)

            #combined images
            yellow_cv = self.bridge.imgmsg_to_cv2(yellow_msg, 'bgr8')
            white_cv = self.bridge.imgmsg_to_cv2(white_msg, 'bgr8')

            combined = cv2.bitwise_or(yellow_cv, white_cv)

            cv_img_msg = self.bridge.cv2_to_imgmsg(combined, 'bgr8')
            self.pub_combined.publish(cv_img_msg)

            # get edges
            edges = cv2.Canny(cropped_image, 100, 255)
            

            #combine white and yellow mask with edges
            combined_white = cv2.bitwise_and(edges, white_mask)
            combined_yellow = cv2.bitwise_and(edges, yellow_mask)
            
            

            # hough transform
            lines_white = cv2.HoughLinesP(combined_white, 1, np.pi/180, 10, minLineLength=10, maxLineGap=50)
            output_white = self.output_lines(cropped_image, lines_white, (255,0,0))

            lines_yellow = cv2.HoughLinesP(combined_yellow, 1, np.pi/180, 10, minLineLength=10, maxLineGap=50)
            output_yellow = self.output_lines(cropped_image, lines_yellow, (0,0,255))



            combined_output = cv2.bitwise_and(output_white, output_yellow)
            


            # Publish the image with lines
            output_msg = self.bridge.cv2_to_imgmsg(combined_output, encoding='bgr8')
            self.pub_combined.publish(output_msg)



        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")



if __name__ == '__main__':

    try:
        rospy.init_node('image_processor', anonymous=True)
        ImageProcessor()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass