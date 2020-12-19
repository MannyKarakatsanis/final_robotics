#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np 
from duckietown_msgs.msg import Twist2DStamped, LanePose, FSMState, BoolStamped
import math

class Maze:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("/bib/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=2**24)        

        # rospy.Subscriber("image", Image, self.callback)
        
        self.hough_white = None
        self.hough_yellow = None

        self.pub_img = rospy.Publisher('~output/image_raw/compressed', CompressedImage, queue_size=1)

        self.pub_error = rospy.Publisher('~output/error', LanePose , queue_size=1)


    def callback(self, msg):


        # get both the horizontal white lines and vertical white lines using Hough Transform
        horizontal_white_lines, vertical_white_lines, cv_cropped = self.get_white_hough_lines(msg)


        #draw hough lines to output image
        cv_cropped = self.draw_hough_lines(cv_cropped, vertical_white_lines)
        cv_cropped = self.draw_hough_lines(cv_cropped, horizontal_white_lines)

        mode = "straight"

        if vertical_white_lines is not None:
            # draw the vertical lines as single lines on the left and right (if they exist)
            line_filter_length = 150
            cv_cropped, no_left_line, no_right_line, min_x_left, max_y_left, max_x_left, min_y_left, max_x_right, max_y_right, min_x_right, min_y_right = self.draw_lines(vertical_white_lines, cv_cropped, line_filter_length)
            

            # based on the lines, dictate which direction to turn (LEFT hand rule currently)
            cv_cropped, mode = self.decide_direction(cv_cropped, horizontal_white_lines, vertical_white_lines, no_left_line, no_right_line)

            
            # find point on the left and right lines at a certain height
            # cv_cropped, x3_left = self.find_points_error(cv_cropped, min_x_left, max_y_left, max_x_left, min_y_left)
            # cv_cropped, x3_right = self.find_points_error(cv_cropped, max_x_right, max_y_right, min_x_right, min_y_right)

            # cv_cropped = cv2.putText(cv_cropped, f'Left distance = {x3_left}', (100, 60) , cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA) 
            # cv_cropped = cv2.putText(cv_cropped, f'Right distance = {x3_right}', (100, 120) , cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA) 


            # cv_cropped = cv2.arrowedLine(cv_cropped,(x3_left,75),(319, 75),(255,0,255),5)
            # cv_cropped = cv2.arrowedLine(cv_cropped,(x3_right, 75),(319,75),(0,255,255),5)


            # calculate the error
            error = self.calculate_error(x3_left, x3_right, no_left_line, no_right_line)

        else:
            error = 0.0

        yellow_hough = cv2.HoughLinesP(yellow_and, 1, np.pi / 180, 5, None, 4, 4)




        # controlling the output
        control_duckiebot = LanePose()
        control_duckiebot.header.frame_id = mode
        control_duckiebot.d = error
        control_duckiebot.phi = 0.0
        self.pub_error.publish(control_duckiebot)

        self.pub_img.publish(self.bridge.cv2_to_compressed_imgmsg(cv_cropped))



        # publish the error out
        # self.pub_error.publish(control_duckiebot)

    def calculate_error(self, x3_left, x3_right, no_left_line, no_right_line):
        # normalize values
        a = -1
        b = 1
        # both left and right lines exist, use both to calculate the error
        if no_left_line is False and no_right_line is False:
            difference = x3_right - x3_left
            error = ((b-a)*((difference-0)/(648-0))) - 1
            error = 0.0
        # only the left line exists
        if no_left_line is False and no_right_line is True:
            # way too close to the right
            if x3_right < 610 and x3_left > 50:
                error = -0.1

            if x3_right > 638 and x3_left < 0:
                error = 0.1

        if no_left_line is True and no_right_line is False:
            if x3_right > 638 and x3_left < 0:
                error = 0.1
            else:
                error = 0.0
        if no_left_line is True and no_right_line is True:
            error = 0.0

        return error

    def area(self, x1, y1, x2, y2, x3, y3): 
  
        return abs((x1 * (y2 - y3) + x2 * (y3 - y1)  
                + x3 * (y1 - y2)) / 2.0) 
  
    def isInside(self, x1, y1, x2, y2, x3, y3, x, y): 
    
        # Calculate area of triangle ABC 
        A = self.area (x1, y1, x2, y2, x3, y3) 
    
        # Calculate area of triangle PBC  
        A1 = self.area (x, y, x2, y2, x3, y3) 
        
        # Calculate area of triangle PAC  
        A2 = self.area (x1, y1, x, y, x3, y3) 
        
        # Calculate area of triangle PAB  
        A3 = self.area (x1, y1, x2, y2, x, y) 
        
        # Check if sum of A1, A2 and A3  
        # is same as A 
        if(A == A1 + A2 + A3): 
            return True
        else: 
            return False

    def decide_direction(self, cv_cropped, horizontal_white_lines, vertical_white_lines, no_left_line, no_right_line):
        cv2.line(cv_cropped,(500,140),(500, 198),(255,255,0),5)
        cv2.line(cv_cropped,(500, 198),(138, 198),(255,255,0),5)
        cv2.line(cv_cropped,(138, 198),(138, 140),(255,255,0),5)
        cv2.line(cv_cropped,(138, 140),(500,140),(255,255,0),5)


        cv2.line(cv_cropped,(430,10),(430, 100),(255,0,255),5)
        cv2.line(cv_cropped,(430, 100),(228, 100),(255,0,255),5)
        cv2.line(cv_cropped,(228, 100),(228, 10),(255,0,255),5)
        cv2.line(cv_cropped,(228, 10),(430,10),(255,0,255),5)



        # RIGHT 
        cv2.line(cv_cropped,(638,130),(638, 198),(0,0,0),4)
        cv2.line(cv_cropped,(638, 198),(578, 198),(0,0,0),4)
        cv2.line(cv_cropped,(578, 198),(638, 130),(0,0,0),4)


        # LEFT 

        cv2.line(cv_cropped,(0,130),(0, 198),(0,0,0),4)
        cv2.line(cv_cropped,(0, 198),(60, 198),(0,0,0),4)
        cv2.line(cv_cropped,(60, 198),(0, 130),(0,0,0),4)
        # cv2.line(cv_cropped,(0, 65),(38,65),(255,255,255),5)


        mode = "straight"

        left_line = False
        right_line = False

        forward_line = False
        some_forward_line = False
        if horizontal_white_lines is not None:
            for i in range(len(horizontal_white_lines)):
                if horizontal_white_lines[i][0][0] < 500 and horizontal_white_lines[i][0][1] > 140 and horizontal_white_lines[i][0][0] > 138 and horizontal_white_lines[i][0][1] < 198: 
                    forward_line = True
                if horizontal_white_lines[i][0][0] < 430 and horizontal_white_lines[i][0][1] > 10 and horizontal_white_lines[i][0][0] > 228 and horizontal_white_lines[i][0][1] < 100: 
                    some_forward_line = True
 
        for l in range(len(vertical_white_lines)):
            if (self.isInside(0,130,0,198,60,198, vertical_white_lines[l][0][0], vertical_white_lines[l][0][1])): 
                left_line = True
            if (self.isInside(638,130,638,198,578,198, vertical_white_lines[l][0][0], vertical_white_lines[l][0][1])): 
                right_line = True

        if some_forward_line is True and left_line is True and right_line is False:
            mode = "right"


        if some_forward_line is True and left_line is False and right_line is True:
            mode = "left"


        if some_forward_line is True and left_line is False and right_line is False:
            mode = "left"

        # LS turn decision
        if forward_line is False and left_line is False and right_line is True and some_forward_line is False:
            mode = "left"
            # rospy.logwarn(f"some_forward_line: {some_forward_line}")

        # RS turn decision
        if forward_line is True and left_line is True and right_line is False and some_forward_line is False:
            mode = "right"
        
        # RLS turn decision
        if forward_line is False and left_line is False and right_line is False and some_forward_line is False:
            mode = "Left turn - RLS turn decision"

        # T-Shaped decision
        if forward_line is True and left_line is False and right_line is False:
            mode = "left"
            

        cv_cropped = cv2.putText(cv_cropped, mode, (150, 120) , cv2.FONT_HERSHEY_SIMPLEX, .8, (0,255,0), 2, cv2.LINE_AA) 
        return cv_cropped, mode


    def find_points_error(self,cv_cropped, min_x_left, max_y_left, max_x_left, min_y_left):
        slope = (max_x_left - min_x_left)/(min_y_left - max_y_left)
        y_int = min_x_left - (slope*max_y_left)

        x3 = 140*slope  + y_int
        return cv2.circle(cv_cropped, (int(x3),140), 5, (255,0,255), 2), int(x3)


    def filter_hsv(self, image_hsv, low, high):
        filtered = cv2.inRange(image_hsv, low, high)
        return filtered

    def bitwise(self, canny_white, white_filtered, canny_yellow, yellow_filtered):
        white = cv2.bitwise_and(canny_white, white_filtered)
        yellow = cv2.bitwise_and(canny_yellow, yellow_filtered)
        return white, yellow

    def hough(self, white_and, yellow_and):
        white_hough = cv2.HoughLinesP(white_and, 1, np.pi / 180, 5, None, 3, 1)
        yellow_hough = cv2.HoughLinesP(yellow_and, 1, np.pi / 180, 5, None, 4, 4)
        return white_hough, yellow_hough

    def draw_hough_lines(self, cv_cropped, hough_lines):
        hough = np.copy(cv_cropped)
        if hough_lines is not None:
            for i in range(len(hough_lines)):

                l = hough_lines[i][0]
                cv2.line(hough, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(hough, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(hough, (l[2],l[3]), 2, (0,0,255))
        return hough

    def find_left_line(self, white_hough_lines):
        min_x_left = 1000
        max_x_left = -10
        max_y_left = -10
        min_y_left = 1000
        if white_hough_lines is not None:
            for i in range(len(white_hough_lines)):
                if white_hough_lines[i][0][0] < min_x_left: 
                    min_x_left = white_hough_lines[i][0][0]
                if white_hough_lines[i][0][2] < 320:
                    if white_hough_lines[i][0][1] < min_y_left: 
                        min_y_left = white_hough_lines[i][0][1]
                    if white_hough_lines[i][0][3] < min_y_left:
                        min_y_left = white_hough_lines[i][0][3]
                    if white_hough_lines[i][0][1] > max_y_left: 
                        max_y_left = white_hough_lines[i][0][1]
                    if white_hough_lines[i][0][3] > max_y_left:
                        max_y_left = white_hough_lines[i][0][3]
                    if white_hough_lines[i][0][2] > max_x_left:
                        max_x_left = white_hough_lines[i][0][2]
            return min_x_left, max_y_left, max_x_left, min_y_left


    def find_right_line(self, white_hough_lines):
        min_x_right = 1000
        max_x_right = -10
        max_y_right = -10
        min_y_right = 1000
        for i in range(len(white_hough_lines)):
            if white_hough_lines[i][0][2] > 320:
                if white_hough_lines[i][0][1] < min_y_right: 
                    min_y_right = white_hough_lines[i][0][1]
                if white_hough_lines[i][0][3] < min_y_right:
                    min_y_right = white_hough_lines[i][0][3]
                if white_hough_lines[i][0][1] > max_y_right: 
                    max_y_right = white_hough_lines[i][0][1]
                if white_hough_lines[i][0][3] > max_y_right:
                    max_y_right = white_hough_lines[i][0][3]
                if white_hough_lines[i][0][2] > max_x_right:
                    max_x_right = white_hough_lines[i][0][2]
                if white_hough_lines[i][0][0] < min_x_right: 
                    min_x_right = white_hough_lines[i][0][0]
        return max_x_right, max_y_right, min_x_right, min_y_right

    def get_white_hough_lines(self, msg):
        # get the input image
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        

        # crop input image
        height, width, channels = cv_img.shape
        # cv_cropped = cv_img[int(height/1.35):height, 0:(width-2)]

        cv_cropped = cv_img[int(height/1.70):height, 0:(width-2)]

        # convert to hsv
        image_hsv = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)
                
        # filter the image
        white_filtered = self.filter_hsv(image_hsv, (0,0,100), (180,60,255))  
        yellow_filtered = self.filter_hsv(image_hsv, (23,95,110),(180,255,255))

        # perform canny edge detection
        canny_white = cv2.Canny(cv_cropped, 280, 400)
        canny_yellow = cv2.Canny(cv_cropped, 280, 400)

        # perform white and yellow bitwise
        white_and, yellow_and = self.bitwise(canny_white, white_filtered, canny_yellow, yellow_filtered)
        
        # perform hough transform for white and yellow
        white_hough_lines, yellow_hough_lines = self.hough(white_and, yellow_and)

        horizontal_white_lines = cv2.HoughLinesP(canny_white, 1, math.pi/2, 5, None, 3, 1)
        vertical_white_lines = cv2.HoughLinesP(canny_white, 1, math.pi/180, 3, None, 3, 2)

        return horizontal_white_lines, vertical_white_lines, cv_cropped

    def draw_lines(self, vertical_white_lines, cv_cropped, line_filter_length):
        min_x_left, max_y_left, max_x_left, min_y_left = self.find_left_line(vertical_white_lines)
        if (min_x_left < 600 and min_y_left < 600) and (max_y_left > 10 and max_x_left > 10):
            # cv2.line(cv_cropped,(min_x_left,max_y_left),(max_x_left, min_y_left),(255,0,0),5)
            distance_vertical_line = math.sqrt((max_x_left- min_x_left)**2 + (min_y_left-max_y_left)**2)
            # FILTER SHORTER LINES
            if distance_vertical_line < line_filter_length:
                no_left_line = True
            else:
                no_left_line = False
         
            # if not no_left_line:
            #     cv2.line(cv_cropped,(min_x_left,max_y_left),(max_x_left, min_y_left),color,5)
        else: 
            no_left_line = True

        max_x_right, max_y_right, min_x_right, min_y_right = self.find_right_line(vertical_white_lines)
        if (min_x_right < 600 and min_y_right < 600) and (max_y_right > 10 and max_x_right > 10):
            distance_vertical_line = math.sqrt((max_x_right- min_x_right)**2 + (min_y_right-max_y_right)**2)
            # FILTER SHORTER LINES
            if distance_vertical_line < line_filter_length:
                no_right_line = True
            else:
                no_right_line = False
         
            # if not no_right_line:
            #     cv2.line(cv_cropped,(max_x_right,max_y_right),(min_x_right, min_y_right),color,5)
        else: 
            no_right_line = True

        return cv_cropped, no_left_line, no_right_line, min_x_left, max_y_left, max_x_left, min_y_left, max_x_right, max_y_right, min_x_right, min_y_right
           
            # pass





if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("maze", anonymous=True)
    maze = Maze()
    rospy.spin()

