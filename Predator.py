#!/usr/bin/env python
# -*- coding: utf-8 -*-
#PREDATOR PROGRAM


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import namedWindow
import numpy
from random import choice

class Predator():
    """A class to enable the Turtlebot to operate as the Predator"""

    def __init__(self, name):
        """Function to initialise the class. Called when creating a new instace
        :param name: The name of the ros node"""
        
        rospy.loginfo("Starting Predator node %s" % name)
        namedWindow("object", 1)
        namedWindow("hsv",2)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        print "Window thread started"
        
        self.laser_sub = rospy.Subscriber("/turtlebot_1/scan",LaserScan,callback=self.laser_callback,queue_size=1)
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",Image,callback=self.image_callback,queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/turtlebot_1/cmd_vel",Twist,queue_size=1)
    
    def laser_callback(self,msg):

        self.obstacle = False        
        
        print "**********"
        self.ranges = msg.ranges
        min_distance = numpy.nanmin(self.ranges)
        rospy.loginfo("Minimum distance: %f" % min_distance)
        twist_msg = Twist()
        
        if min_distance <= 0.4:
            rospy.Rate(10)
            now = rospy.Time.Now().to_sec()
            end_time = now + 1
            angle_velocity = choice([0.4,-0.4])
            self.obstacle = True
            
            while end_time > rospy.Time.now().to_sec():
                twist_msg.linear.x = 0
                twist_msg.angular.z = angle_velocity
                self.cmd_vel_pub.publish(twist_msg)
                
    def image_callback(self,img):
        rospy.loginfo("Recieved image of size: %i x %i" % (img.width,img.height))
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError, e:
            print e
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        self.sim = True        
        #change for irl
        if(self.sim):
            hsv_threshold = cv2.inRange(hsv_image, numpy.array((60,0,0)),numpy.array((60,255,255)))
        else:
            hsv_threshold = cv2.inRange(hsv_image, numpy.array((64,29,80)),numpy.array((90,255,255)))
            
        print "==================="
        print numpy.mean(hsv_image[:,:,0])
        print numpy.mean(hsv_image[:,:,1])
        print numpy.mean(hsv_image[:,:,2])
        
        hsv_contours, hierachy = cv2.findContours(hsv_threshold.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        largestA = 0
        largestC = None                             
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 250.0 and a > largestA:#this is much bigger irl
                #cv2.drawContours(cv_image,c,-1,(255,0,0))
                largestA = a
                largestC = c
                
        x = -1
        if largestC != None:
            cv2.drawContours(cv_image,[largestC],0,(255,0,0),3)
            x,y,w,h = cv2.boundingRect(largestC)
            x = x + (w/2)
            x = x/640.0
            print "X: " + str(x)
        else:
            self.roam(img)
            return
            
        print "======================="
        cv2.imshow("hsv",cv_image)
        kernel = numpy.ones((3,3), numpy.uint8)
        hsv_out = cv2.erode(hsv_threshold,kernel,iterations = 1)
        cv2.imshow("object", hsv_out)
        
        wheel_power = 0
        if x < 0.5:
            # turn right
            wheel_power = -(x  -0.5)
            
        else:
            # turn left
            wheel_power = 0.5 - x
        
        
        twist = Twist()
        twist.angular.z = wheel_power
        twist.linear.x = 0.5
        
        print "publish msg"
        print twist
        self.cmd_vel_pub.publish(twist)
        
    def roam(self, img):
        twist = Twist()
        min_distance = numpy.nanmin(self.ranges)
        print "roam range: ", min_distance
        if min_distance <= 0.9:
            rospy.Rate(10)
            now = rospy.Time.now().to_sec()
            end_time = now + 1
            angle_velocity = choice([0.6,-0.6])
            
            while end_time > rospy.Time.now().to_sec():
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = angle_velocity
                self.cmd_vel_pub.publish(twist)

        vel = choice([0.2,-0.2,0])                    
        twist.linear.x= 0.5
        twist.angular.z = vel
        self.cmd_vel_pub.publish(twist)
        
if __name__ == '__main__':
    rospy.init_node("Predator")
    p = Predator(rospy.get_name())
    rospy.spin()
