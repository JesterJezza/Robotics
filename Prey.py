#!/usr/bin/env python
# -*- coding: utf-8 -*-
#PREY PROGRAM
#DO A FINITE STATE MACHINE
#Behaviour for when green isn't present

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import namedWindow
import numpy
from random import choice

class Prey():
    """A class to enable the Turtlebot to operate as the Prey"""
    
    def __init__(self, name):
        """Function to intialise the class. Called when creating a new  instance
        :param name: The name of the ros node """
        
        rospy.loginfo("Starting Prey node %s" % name)   
        namedWindow("left image window",1)
        namedWindow("right image window", 1)
        namedWindow("left grey image", 1)
        namedWindow("right grey image", 1)
        namedWindow("hsv",1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        print "Window thread started"
        
        self.laser_sub = rospy.Subscriber("/turtlebot_1/scan",LaserScan,callback=self.laser_callback,queue_size=1)
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",Image,callback=self.image_callback,queue_size=1) #image_raw for sim, image_color for bot
        
        self.cmd_vel_pub = rospy.Publisher("/turtlebot_1/cmd_vel",Twist,queue_size=1)
        
    def laser_callback(self,msg):
        print "***************laser*******************"
        self.ranges = msg.ranges
        min_distance = numpy.nanmin(self.ranges)
        rospy.loginfo("Minimum distance: %f" % min_distance)
        twist_msg = Twist()
        
        if min_distance <= 0.8 and min_distance > 0.4:
            #rospy.loginfo.Rate(10)
            rospy.Rate(10)
            now = rospy.Time.now().to_sec()
            end_time = now + 1
            angle_velocity = choice([1.0,-1.0])
            
            while end_time > rospy.Time.now().to_sec():
                twist_msg.linear.x = 0
                twist_msg.angular.z = angle_velocity
                self.cmd_vel_pub.publish(twist_msg)
        
        if min_distance < 0.4:
            twist_msg.linear.x = 0;
            twist_msg.angular.z = 0;
            self.cmd_vel_pub.publish(twist_msg)

    def image_callback(self,img):
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError, e:
            print e
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #change to hsv - looking for green block, use cv2.inRange (takes image as input, lower and upper bounds of each colour channel)
        hsv_threshold = cv2.inRange(hsv_image, 
                                    numpy.array((64,29,80)),
                                    numpy.array((90,255,255)))        
        print"==========image callback============"
        print numpy.mean(hsv_image[:,:,0])
        print numpy.mean(hsv_image[:,:,1])
        print numpy.mean(hsv_image[:,:,2])
        
        hsv_contours, hierachy = cv2.findContours(hsv_threshold.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
                                                  
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image,c, -1, (255,0,0))
        print "====================="
        cv2.imshow("hsv",hsv_threshold)
#        left_image = gray_image[:,0:320]
#        right_image = gray_image[:,320:640]
#        left_mean_intensity = numpy.mean(left_image)
#        right_mean_intensity = numpy.mean(right_image)
        left_hsv = hsv_threshold[:,0:320]
        right_hsv = hsv_threshold[:,320:640]
        left_mean = numpy.mean(left_hsv)
        right_mean = numpy.mean(right_hsv)
#        lmi_norm = left_mean/255
#        rmi_norm = right_mean/255
        print "Left Mean intensity: ", left_mean
        print "Right Mean intensity: ", right_mean
#        print "Left Normalised Mean intensity: ", lmi_norm
#        print "Right Normalised Mean intensity: ",rmi_norm

        if left_mean <= 0.02 and right_mean <= 0.02:
            print "********roam called**********"
            self.roam(img)
            return
        
        left_wheel_power = right_mean
        right_wheel_power = left_mean
    
        if left_wheel_power > right_wheel_power:
            right_wheel_power = left_wheel_power
            #right_wheel_power=0
        
        if right_wheel_power > left_wheel_power:
            left_wheel_power = right_wheel_power
           # left_wheel_power=0
        
        twist_msg = self.wheel_motor_power_to_twist_msg(left_wheel_power,right_wheel_power, left_hsv, right_hsv)
        print "Left wheel power: ", left_wheel_power
        print "Right wheel power: ", right_wheel_power
    
        cv2.imshow("left image window", left_hsv)
        cv2.imshow("right image window", right_hsv)
        print "publish msg"
        print twist_msg
        self.cmd_vel_pub.publish(twist_msg)
    
        
    def wheel_motor_power_to_twist_msg(self, left_wheel_power, right_wheel_power=None, left_hsv=None, right_hsv=None):
        if right_wheel_power == None:
            right_wheel_power = left_wheel_power
            
        if left_wheel_power > 1.0 or left_wheel_power < -1.0:
            rospy.logfatal("Power for wheels has to be between -1.0 and 1.0")
            return
        if right_wheel_power > 1.0 or right_wheel_power < -1.0:
            rospy.logfatal("Power for wheels has to be between -1.0 and 1.0")
            return
        
        left_wheel = [0.5*left_wheel_power, 0.25*left_wheel_power]
        right_wheel = [0.5*right_wheel_power, 0.15*right_wheel_power]
        
        twist = Twist()
        twist.linear.x = left_wheel[0] + right_wheel[0]
        twist.angular.z = left_wheel[1] + right_wheel[1]
        return twist
        
#    def ranges(self):
#        ranges = self.ranges
#        min_dist = numpy.nanmin(ranges)
#        print "ranges"
#        print min_dist
#        return min_dist
        
    def roam(self, img):
        rospy.loginfo("********roam***********")
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))
        min_distance = numpy.nanmin(self.ranges)
        print "*******roam range ******"
        print min_distance
        if min_distance < 0.4:
            self.stop()
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError, e:
            print e
        
        gray_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        left_image = gray_image[:,0:320]
        right_image = gray_image[:,320:640]
        left_mean_intensity = numpy.mean(left_image)           # Getting the mean intensity of the whole image
        right_mean_intensity = numpy.mean(right_image)
        left_normalised_mean_intensity = left_mean_intensity / 255  # Normalising the intensity
        right_normalised_mean_intensity = right_mean_intensity / 255
        print "Left Mean intensity: ", left_mean_intensity
        print "Right Mean intensity: ", right_mean_intensity
        print "Left Normalised mean intensity: ", left_normalised_mean_intensity
        print "Right Normalised mean intensity: ", right_normalised_mean_intensity
        left_wheel_power = left_normalised_mean_intensity 
        right_wheel_power = right_normalised_mean_intensity
        twist_msg = self.wheel_motor_power_to_twist_msg(left_wheel_power,right_wheel_power)
       
        print "Left wheel power: ", left_wheel_power
        print "Right wheel power: ", right_wheel_power        
        
        cv2.imshow("left grey image", left_image)
        cv2.imshow("right grey image", right_image)
        print "publish msg"
        print twist_msg
        self.cmd_vel_pub.publish(twist_msg)
        
    def stop(self):
        print "============stop==========="
        min_range = numpy.nanmin(self.ranges)
        print "*******stop range***********"
        print min_range
        if min_range < 0.4 or min_range == 'nan':
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            print twist
            self.cmd_vel_pub.publish(twist)
            print "Turtlebot has found a suitable place to hide!"
        
if __name__ == '__main__':
    rospy.init_node("prey")
    p = Prey(rospy.get_name())
    rospy.spin()
        
