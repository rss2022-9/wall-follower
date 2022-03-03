#!/usr/bin/env python2

from cmath import pi
from distutils.log import debug
from multiprocessing import pool
from operator import truediv
# import queue
from turtle import end_fill
import numpy as np
# from scipy import linalg

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Float32

from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    WALL_TOPIC = "/wall"

    laser_angle = 5*pi/8 # 5*pi/8 # the range we use to fit a line
    laser_offset = pi/6

    vel_max = 4
    steering_max = 0.34
    wheel_base = 0.325
    L1 = 1.6 # 1.6 seperation distance
    VLratio = VELOCITY/L1 # 1.6 & 3

    err_y_last = 0
    err_theta_last = 0
    last_ack_angle = 0
    Pgain = 1
    Dgain = 5
    thresh = 2
    def __init__(self):
        # TODO:
        # Initialize your publishers and subscribers here
        pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.debug_pub = rospy.Publisher("debug", String, queue_size=10)
        self.error_pub = rospy.Publisher("error", Float32, queue_size=10)
        line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback, (pub, line_pub))

    # Write your callback functions here.
    def callback(self, msg, pub_list):
        turn_msg = AckermannDriveStamped()

        
        pub = pub_list[0]
        line_pub = pub_list[1]
        
        wall = self.wall_in_front(msg)
       
        if wall[0]==True:
            if self.SIDE == -1: # right wall
                ack_angle = 0.3 # turn left
            else:
                ack_angle = -0.3
            print("wall dectected")
        else:   
            k, b = self.find_line_aggr(msg, line_pub)
            # ack_angle = self.LQRcontroller(k, b)
            ack_angle = self.PDController(k, b)


        # steer angle: positive left, negative right
        turn_msg.header.stamp = rospy.Time.now()
        turn_msg.drive.steering_angle = ack_angle
        #turn_msg.drive.speed = self.VELOCITY
        turn_msg.drive.acceleration = 0.1
        # for the safety controller
        if wall[1]==True:
            turn_msg.drive.speed = 0 # safety
            turn_msg.drive.steering_angle = 0
        else:
            turn_msg.drive.speed = self.VELOCITY
        pub.publish(turn_msg)       

    def PDController(self, k, b):
        # if self.SIDE == 1: # left with P controller
            
        theta = np.arctan(k)
        err_y_now = np.abs(b*np.cos(theta)) - self.DESIRED_DISTANCE
        err_theta_now = theta
        err_dy = err_y_now - self.err_y_last
        a_cmd = 2*self.VLratio*(self.Dgain*err_dy + self.Pgain*self.VLratio*err_y_now) # assume theta_d is small
        ackman_ang = self.L1*a_cmd/self.VELOCITY**2

        self.err_y_last = err_y_now
        self.err_theta_last = err_theta_now

        if self.SIDE == -1: # right
            return -ackman_ang

        return ackman_ang

    # big problem: the system seems not controllable
    # def LQRcontroller(self, k, b):
    #     # x is current state for y and theta

    #     theta = np.arctan(k)
    #     y_err = np.abs(b*np.cos(theta)) - self.DESIRED_DISTANCE

    #     A = np.array([[0, np.cos(theta)*self.VELOCITY], [0,0]])
    #     B = np.array([[0, self.VELOCITY/(np.cos(self.last_ack_angle)**2)]]).T
    #     Q = np.array([[2,0], [0,50]])
    #     R = np.array([[1000]])
    #     # print(scipy.)
    #     S = linalg.solve_continuous_are(A,B,Q,R)
    #     K = np.dot(np.linalg.inv(R), np.dot(B.T, S))

    #     ack_angle = -np.dot(K, np.array([[y_err, theta]]).T)
    #     self.last_ack_angle = ack_angle
    #     print("ack_angle", ack_angle)
    #     return ack_angle

    # detect whether there is a wall in front of the car
    def wall_in_front(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        laserpt_num = len(msg.ranges)
        angle_list = np.arange(angle_min, angle_max, angle_increment)

        split = int(laserpt_num/10) # pi/3 in the front to dectect wall
        middle= int(laserpt_num/2)
        angle_front = angle_list[middle-split:middle+split]
        dist_all  = np.cos(angle_front)*msg.ranges[middle-split:middle+split]
        
        
        if min(dist_all) < self.wheel_base*self.VELOCITY: # a super simple safety controller
            print(min(dist_all))
            return [True,True]
        if min(dist_all) < self.DESIRED_DISTANCE+self.wheel_base*self.VELOCITY:
            return [True,False]
        else:
            return [False,False]

    # segment the laser data and fit it into a line and visualize
    # currently only start from the start or the end
    """
    def find_line(self, msg, line_pub):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        laserpt_num = len(msg.ranges)
        angle_list = np.arange(angle_min, angle_max, angle_increment)            

        # get how many number of the points we need
        if self.laser_angle > (angle_max-angle_min):
            rospy.loginfo("laser scan range out of bound")
            num = laserpt_num # TODO: may cause issue, fix later
        else:
            num = np.round(self.laser_angle/(angle_max-angle_min)*laserpt_num)
            num = int(num)
        
        # use min distance except when min_ind on wrong side
        min_dist = min(msg.ranges)
        min_ind  = msg.ranges.index(min_dist)
        # if (self.SIDE == -1 and min_ind<int(laserpt_num/2)) or (self.SIDE == 1 and min_ind>int(laserpt_num/2)):
        offset = np.round(self.laser_offset/(angle_max-angle_min)*laserpt_num)
        offset = int(offset)
        if self.SIDE == -1: # right wall
            laser_line_fit = msg.ranges[offset:num+offset] # TODO: correspondance check 
            angle = angle_list[offset:num+offset]
            min_dist = min(laser_line_fit)

            x_min = laser_line_fit[0]*np.cos(angle[0])
            x_max = laser_line_fit[-1]*np.cos(angle[-1])
            # polar_coordinate = zip(angle, laser_line_fit)
            polar_coordinate = []
            for i in range(len(laser_line_fit)):
                if laser_lin_fit[i] > 0.2 and  laser_lin_fit[i] - min_dist < self.thresh:
                    polar_coordinate.append(angle[i],laser_lin_fit[i])
            #polar_coordinate = np.vstack((angle, laser_line_fit))
            #polar_coordinate_f1 = polar_coordinate[1,:] - min_dist < self.thresh
            #polar_coordinate_f2 = polar_coordinate_f1[1,:]  > 0.2
            #print(polar_coordinate_f2.shape)
        elif self.SIDE == 1:
            laser_line_fit = msg.ranges[-num-offset:-offset]
            angle = angle_list[-num-offset:-offset]
            min_dist = min(laser_line_fit)
            x_max = laser_line_fit[0]*np.cos(angle[0])
            x_min = laser_line_fit[-1]*np.cos(angle[-1])
            # polar_coordinate = zip(angle, laser_line_fit)
            for i in range(len(laser_line_fit)):
                if laser_lin_fit[i] > 0.2 and  laser_lin_fit[i] - min_dist < self.thresh:
                    polar_coordinate.append(angle[i],laser_lin_fit[i])
            #polar_coordinate = np.vstack((angle, laser_line_fit))
            #polar_coordinate_f1 = polar_coordinate[1,:] - min_dist < self.thresh
            #polar_coordinate_f2 = polar_coordinate_f1[1,:]  > 0.2
            #print(polar_coordinate_f2.shape)
        else:
            rospy.loginfo("error: input correct side")
        # strmsg = String()
        # strmsg.data = "now in normal mode............."
        # self.debug_pub.publish(strmsg)
        # else:
        #     num_half = int(num/2.0)  # min pt in the middle, get the pts around it
        #     if num_half>min_ind+1:
        #         num_half = min_ind;
        #     elif num_half>laserpt_num-1-min_ind:
        #         num_half = laserpt_num-1-min_ind;
        #     start_ind= min_ind-num_half
        #     end_ind  = min_ind+num_half
        #     x_max = msg.ranges[start_ind]*np.cos(angle_list[start_ind])
        #     x_min = msg.ranges[end_ind]*np.cos(angle_list[end_ind])
        #     polar_coordinate = zip(angle_list[start_ind:end_ind], msg.ranges[start_ind:end_ind])
        #     strmsg = String()
        #     strmsg.data = "now in min mode"
        #     self.debug_pub.publish(strmsg)

        k, b = self.least_square(polar_coordinate, num)

        x_marker = np.linspace(x_min, x_max, num=20)
        y_marker = k*x_marker + b
        VisualizationTools.plot_line(x_marker, y_marker, line_pub, frame="/laser")

        return k, b
"""
    def find_line_aggr(self, msg, line_pub):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        laserpt_num = len(msg.ranges)
        angle_list = np.arange(angle_min, angle_max, angle_increment)            

        # get how many number of the points we need
        rho = []
        phi = []
        # use min distance except when min_ind on wrong side
        half_ind = int((laserpt_num*0.6))
        if self.SIDE == -1: # right wall
            laser_line_fit = msg.ranges[:half_ind] # TODO: correspondance check 
            angle = angle_list[:half_ind]
            min_dist = min(laser_line_fit)
            for i in range(len(laser_line_fit)):
                if laser_line_fit[i] > 0.2 and  laser_line_fit[i] - min_dist < self.thresh:
                    rho.append(angle[i])
                    phi.append(laser_line_fit[i])
            x_min = rho[0]*np.cos(phi[0])
            x_max = rho[-1]*np.cos(phi[-1])
            polar_coordinate = zip(rho,phi)
        elif self.SIDE == 1:
            laser_line_fit = msg.ranges[-half_ind:]
            angle = angle_list[-half_ind:]
            min_dist = min(laser_line_fit)
            for i in range(len(laser_line_fit)):
                if laser_line_fit[i] > 0.2 and  laser_line_fit[i] - min_dist < self.thresh:
                    rho.append(angle[i])
                    phi.append(laser_line_fit[i])
            polar_coordinate = zip(rho, phi)
            x_min = rho[0]*np.cos(phi[0])
            x_max = rho[-1]*np.cos(phi[-1])
        else:
            rospy.loginfo("error: input correct side")

        # strmsg = String()
        # strmsg.data = "now in normal mode............."
        # self.debug_pub.publish(strmsg)
        # else:
        #     num_half = int(num/2.0)  # min pt in the middle, get the pts around it
        #     if num_half>min_ind+1:
        #         num_half = min_ind;
        #     elif num_half>laserpt_num-1-min_ind:
        #         num_half = laserpt_num-1-min_ind;
        #     start_ind= min_ind-num_half
        #     end_ind  = min_ind+num_half
        #     x_max = msg.ranges[start_ind]*np.cos(angle_list[start_ind])
        #     x_min = msg.ranges[end_ind]*np.cos(angle_list[end_ind])
        #     polar_coordinate = zip(angle_list[start_ind:end_ind], msg.ranges[start_ind:end_ind])
        #     strmsg = String()
        #     strmsg.data = "now in min mode"
        #     self.debug_pub.publish(strmsg)
        num = len(rho)
        k, b = self.least_square(polar_coordinate, num)

        x_marker = np.linspace(x_min, x_max, num=20)
        y_marker = k*x_marker + b
        VisualizationTools.plot_line(x_marker, y_marker, line_pub, frame="/laser")
        dist = np.abs(b)/np.sqrt(1+k**2)
        #err_msg = Float32()
        #err_msg
        self.error_pub.publish(dist-self.DESIRED_DISTANCE)
        return k, b



    # input polar coordinate wrt laser frame, and fit the line in cartesian space
    def least_square(self, polar_coordinate, count):
        H = np.zeros([count, 2])
        y_col = np.zeros([count, 1])
        counter = 0
        # polar_coordinate_new = zip(polar_coordinate[0,:], polar_coordinate[1,:])
        for ang, dist in polar_coordinate:
            x = np.cos(ang)*dist
            y = np.sin(ang)*dist
            H[counter,0] = x
            H[counter,1] = 1
            y_col[counter] = y
            counter = counter + 1

        LHS = np.dot(H.T, H)
        RHS = np.dot(H.T, y_col)
        param = np.dot(np.linalg.pinv(LHS), RHS)
        return param[0], param[1]  # k, b
        

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
