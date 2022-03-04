#!/usr/bin/env python2
import numpy as np
import math
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

# Team 9 RSS Wall Follower Lab
class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    WALL_TOPIC = "/wall"
    ERRO_TOPIC = "/error"

    t_a = 0.34 # max turning angle
    t_r = 0.75 # max turn radius of center of gravity of car 
    w_b = 0.325 # distance between front and rear axles
    L1 = 1.3 # Look ahead distance for pure pursuit controller 1.3 works
    max_thresh = 2.6 # remove distant wall noise from lidar 2.6 works
    min_thresh = 0.1 # remove car noise from lidar
    front = (0.5,0.1) #offset and split of front of car detection for safety controller
    right = (0.3,0.55) # start and end fraction for selection of lidar data for right side of car (0.3,0.55) works
    left = (1.0-right[1],1.0-right[0]) # start and end fraction for selection of lidar data for left side of car mirrior rights selection
    s_w = 0.5 # width of safety space 0.5 default wider than that is safer

    def __init__(self):
        # Initialize your publishers and subscribers here
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=10)
        self.err_pub = rospy.Publisher('Cur_Error', Float32, queue_size=10)
        self.count = 0
        self.toterr = 0
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback, (self.pub, self.line_pub))

    # Callback function
    def callback(self, msg, pub_list):
        turn_msg = AckermannDriveStamped()
        pub = pub_list[0]
        line_pub = pub_list[1]

        # rad and angle values for polar coordinates of laser data
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        angles = np.arange(angle_min, angle_max, angle_increment)
        
        # Find line then Pure Pursuit Controller 
        #k, b = self.find_line(ranges, angles)
        k, b = self.find_line(ranges, angles,1) # uncomment to publish line of wall
        ack_angle = self.PPController(k, b)

        #Safety Controller
        stop = self.safety(ranges, angles) 

        #turn_msg publishing
        turn_msg.header.stamp = rospy.Time.now()
        turn_msg.drive.speed = 0 if stop else self.VELOCITY
        turn_msg.drive.steering_angle = ack_angle
        pub.publish(turn_msg)      

    # error recording
    def rec_error(self, error):
        cur_error = abs(error)
        a = 0.6
        self.count += 1.0
        score = 1/(1+(a*cur_error)**2)
        self.toterr += score
        error = self.toterr/self.count
        self.err_pub.publish(error)
        return error

    # Select laser data with left fraction and right fraction or
    # Set selection type to 1 to select with offset and split range
    def select_data(self, ranges, angles, (left,right), sel_type=0):
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

    # Input polar coordinate wrt laser frame, and fit the line in cartesian space
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

    # Safety controller
    def safety(self, ranges, angles):
        tr = self.t_r 
        sw = self.s_w/2.0
        s = -self.SIDE
        s_ranges, s_angles = self.select_data(ranges,angles,self.front,1)

        polar_coordinate = zip(s_ranges,s_angles)
        for rho,phi in polar_coordinate:
            if rho < tr:
                x = rho*np.cos(phi)
                y = rho*np.sin(phi)
                out_circle = ((x+(s*tr))**2) + y**2 < ((tr+sw)**2)
                inn_circle = ((x+(s*tr))**2) + y**2 > ((tr-sw)**2)
                if out_circle and inn_circle:
                    return True
        return False

    # Pure pursuit controller
    def PPController(self, k, b):
        d = abs(b/(math.sqrt(1+(k**2)))) - self.DESIRED_DISTANCE
        ang = max(min(1.0,d/self.L1),-1.0)
        n1 = np.arcsin(ang)
        n2 = self.SIDE*np.arctan(k)
        n = n1 + n2
        #self.rec_error(d)
        print(self.rec_error(d))
        ackman_ang = np.arctan((2*self.w_b*np.sin(n))/self.L1)
        return self.SIDE*ackman_ang

    # Find line of data using right and left tuples at the top
    # Change max_thresh to change how far ahead the car looks for more points to fit
    def find_line(self, ranges, angles,show_line=0):

        if self.SIDE == -1: # right wall
            line_ranges, line_angles = self.select_data(ranges,angles,self.right)
        else:
            line_ranges, line_angles = self.select_data(ranges,angles,self.left)

        min_dist = min(line_ranges)
        rho = [] # distances
        phi = [] # angles

        for i in range(len(line_ranges)):
                if (line_ranges[i] > self.min_thresh) and  (line_ranges[i] - min_dist < self.max_thresh):
                    rho.append(line_ranges[i])
                    phi.append(line_angles[i])

        num = len(rho)
        polar_coordinate = zip(rho,phi)
        k, b = self.least_square(polar_coordinate, num)

        if show_line:
            x = rho*np.cos(phi)
            y = rho*np.sin(phi)
            x_min = rho[0]*np.cos(phi[0])
            x_max = rho[-1]*np.cos(phi[-1])
            x_marker = np.linspace(x_min, x_max, num=20)
            y_marker = k*x_marker + b
            VisualizationTools.plot_line(x_marker, y_marker, self.line_pub, frame="/laser")

        return k, b

     
        
if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()