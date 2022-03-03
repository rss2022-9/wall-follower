#!/usr/bin/env python2

from cmath import pi
from distutils.log import debug
from multiprocessing import pool
from operator import truediv
# import queue
from turtle import end_fill
import numpy as np
# from scipy import linalg
import math
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
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

    vel_max = 4
    steering_max = 0.34
    max_turn_rad = 0.9
    wheel_base = 0.325
    L1 = 1.6 # 1.6 seperation distance
    VLratio = VELOCITY/L1 # 1.6 & 3
    err_y_last = 0
    err_theta_last = 0
    last_ack_angle = 0
    Pgain = 6
    Dgain = 2
    max_thresh = 2
    min_thresh = 0.1
    front = (0.5,0.1)
    right = (0.2,0.75)
    left = (0.35,0.8)
    sweaty = 1.5
    safety_rad = 2

    def __init__(self):
        # TODO:
        # Initialize your publishers and subscribers here
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=10)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback, (self.pub, self.line_pub))

    # Write your callback functions here.
    def callback(self, msg, pub_list):
        turn_msg = AckermannDriveStamped()
        pub = pub_list[0]
        line_pub = pub_list[1]

        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        angles = np.arange(angle_min, angle_max, angle_increment)
        turn_msg.header.stamp = rospy.Time.now()

        stop = self.safety(ranges, angles)
        if stop:
            turn_msg.drive.speed = 0 # safety
        else:
            turn_msg.drive.speed = self.VELOCITY
            k, b = self.find_line_pub(ranges, angles)
            ack_angle = self.PPController(k, b)
            # steer angle: positive left, negative right
            turn_msg.drive.steering_angle = ack_angle
            
        pub.publish(turn_msg)       

    def safety(self, ranges, angles):
        """
        r_ranges, r_angles = self.select_data(ranges,angles,self.right,0)
        l_ranges, l_angles = self.select_data(ranges,angles,self.left,0)
        rk, rb = self.find_line(r_ranges, r_angles)
        lk, lb = self.find_line(l_ranges, l_angles)
        ld = abs(lb)/math.sqrt(1+(lk**2))
        rd = abs(rb)/math.sqrt(1+(rk**2))
        L = self.wheel_base
        n = self.steering_max
        rd1 = (L/math.tan(n))*(1+(self.SIDE*rk)/math.sqrt(rk**2+1)) + L*math.tan(0.5*n)
        ld1 = (L/math.tan(n))*(1+(-self.SIDE*lk)/math.sqrt(lk**2+1)) + L*math.tan(0.5*n)
        if (rd1 - rd >= self.sweaty):
            print("stopped")
            return True
        else:
            return False
        """
        rho = []
        phi = []
        s_ranges, s_angles = self.select_data(ranges,angles,self.front,1)
        for i in range(len(s_ranges)):
                if (s_ranges[i] < self.safety_rad):
                    rho.append(s_ranges[i])
                    phi.append(s_angles[i])
        L = self.wheel_base
        check = len(rho)
        if check:
            x = rho*np.cos(phi)
            y = rho*np.sin(phi)
            coords = zip(x,y)
            for x,y in coords:
                out_circle = ((x+0.9)**2) + y**2 < ((0.9+L/2)**2)
                inn_circle = ((x+0.9)**2) + y**2 > ((0.9-L/2)**2)
                infront = y > 0
                if out_circle and inn_circle and infront:
                    return True
            return False
        else:
            return False

    def PPController(self, k, b):
        theta = np.arctan(k)
        err_y_now = np.abs(b*np.cos(theta)) - self.DESIRED_DISTANCE
        err_theta_now = theta
        err_dy = err_y_now - self.err_y_last
        a_cmd = 2*self.VLratio*(self.Dgain*err_dy + self.Pgain*self.VLratio*err_y_now) # assume theta_d is small
        ackman_ang = a_cmd#self.L1*a_cmd/self.VELOCITY**2
        self.err_y_last = err_y_now
        self.err_theta_last = err_theta_now
        output = self.SIDE*ackman_ang
        return output

    def select_data(self, ranges, angles, (left,right), sel_type):
        (offset,split) = (left,right) 
        length = len(ranges)
        high_ind = length - 1

        if sel_type == 0:
        	start_ind = int(min(length*(left), high_ind))
        	end_ind = int(min(length*(right),high_ind))
        elif sel_type == 1:
            start_ind = int(min(length*(offset-split), high_ind))
            end_ind = int(min(length*(offset+split),high_ind))

        cut_ranges = ranges[start_ind:end_ind]
        cut_angles = angles[start_ind:end_ind]
        return cut_ranges, cut_angles

    """
    def wall_in_front(self, ranges, angles):
        ranges_front, angle_front = self.select_data(ranges,angles,self.front)
        dist_sum  = np.cos(angle_front)*ranges_front
        stop = sum(dist_sum)/(2*split+1) < self.wheel_base*1
        turn = sum(dist_sum)/(2*split+1) < self.DESIRED_DISTANCE+self.wheel_base*1.5
        return turn, stop
    """

    def find_line(self, ranges, angles):
        # use min distance except when min_ind on wrong side
        if self.SIDE == -1: # right wall
        	line_ranges, line_angles = self.select_data(ranges,angles,self.right,0)
        else:
        	line_ranges, line_angles = self.select_data(ranges,angles,self.left,0)


        min_dist = min(line_ranges)
        rho = [] # distances
        phi = [] # angles
        for i in range(len(line_ranges)):
                if (line_ranges[i] > self.min_thresh) and  (line_ranges[i] - min_dist < self.max_thresh):
                    rho.append(line_ranges[i])
                    phi.append(line_angles[i])
        polar_coordinate = zip(rho,phi)
        num = len(rho)
        k, b = self.least_square(polar_coordinate, num)
        return k, b

    def find_line_pub(self, ranges, angles):
        # use min distance except when min_ind on wrong side
        if self.SIDE == -1: # right wall
            line_ranges, line_angles = self.select_data(ranges,angles,self.right,0)
        else:
            line_ranges, line_angles = self.select_data(ranges,angles,self.left,0)


        min_dist = min(line_ranges)
        rho = [] # distances
        phi = [] # angles
        for i in range(len(line_ranges)):
                if (line_ranges[i] > self.min_thresh) and  (line_ranges[i] - min_dist < self.max_thresh):
                    rho.append(line_ranges[i])
                    phi.append(line_angles[i])
        x = rho*np.cos(phi)
        y = rho*np.sin(phi)
        #print(x,y)  
        x_min = rho[0]*np.cos(phi[0])
        x_max = rho[-1]*np.cos(phi[-1])
        polar_coordinate = zip(rho,phi)
        num = len(rho)
        k, b = self.least_square(polar_coordinate, num)
        #k,b = np.polyfit(x,y,1)
        x_marker = np.linspace(x_min, x_max, num=20)
        y_marker = k*x_marker + b
        VisualizationTools.plot_line(x_marker, y_marker, self.line_pub, frame="/laser")
        return k, b



    # input polar coordinate wrt laser frame, and fit the line in cartesian space
    def least_square(self, polar_coordinate, count):
        H = np.zeros([count, 2])
        y_col = np.zeros([count, 1])
        counter = 0
        for dist, ang  in polar_coordinate:
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