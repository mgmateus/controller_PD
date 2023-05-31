#! /usr/bin/env python3
import rospy 
import math
import time
import numpy as np

from std_srvs.srv import Empty
#from trajectory_msgs.msg import Transform
from geometry_msgs.msg import Twist, Transform
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Position:
    def __init__(self, x, y, yaw):
        self.__x = x
        self.__y = y
        self.__yaw = yaw
       
    @property
    def x(self) -> float:
        return self.__x
    
    @property
    def y(self) -> float:
        return self.__y
    
    @property
    def yaw(self) -> float:
        return self.__yaw

    @x.setter
    def x(self, px):
        self.__x = px

    @y.setter
    def y(self, py):
        self.__y = py

    @yaw.setter
    def yaw(self, angle):
        self.__yaw = angle
        
    def quaternion_to_euler(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        self.__yaw = math.atan2(t3, t4)

    
        
class Diferencial_PID:

    def __init__(self, topic_odom, topic_lidar, topic_vel, px, py, tx, ty, kp, kd):
        rospy.Subscriber(topic_odom, Odometry, self.__callback_odom)
        rospy.Subscriber(topic_lidar, LaserScan, self.__callback_lidar)
        self.__pub_vel= rospy.Publisher(topic_vel, Twist, queue_size=10)

        self.__topic_odom = topic_odom
        self.__topic_lidar = topic_lidar
        self.__position = Position(px, py, 0.0)
        self.__target = Position(tx, ty, 0.0)
        self.__lidar = []
        self.__vel_cmd = Twist()
        
        self.__prev_error = Position(0.0, 0.0, 0.0)
        self.__kp = kp
        self.__kd = kd     

    @property
    def position(self):
        return self.__position
    
    @property
    def target(self):
        return self.__target

    @target.setter
    def target(self, p):
        self.__target.x = p.x
        self.__target.y = p.y

    def __callback_odom(self, msg):
        self.__position.x = msg.pose.pose.position.x
        self.__position.y = msg.pose.pose.position.y

        trans = Transform()
        trans.translation.x = msg.pose.pose.position.x
        trans.translation.y = msg.pose.pose.position.y
        trans.translation.z = msg.pose.pose.position.z
        trans.rotation.x = msg.pose.pose.orientation.x
        trans.rotation.y = msg.pose.pose.orientation.y
        trans.rotation.z = msg.pose.pose.orientation.z
        trans.rotation.w = msg.pose.pose.orientation.w

        self.__position.quaternion_to_euler(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w)

    def __callback_lidar(self, scan):
        scan_range = []

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(30)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0.1)
            else:
                scan_range.append(scan.ranges[i])

        self.__lidar = scan_range[270:811]


    def __linear_dist(self) -> float:
        return math.sqrt((self.__target.x - self.__position.x)**2 + (self.__target.y - self.__position.y)**2)

    def __angular_dist(self) -> float:
        _from = self.__position.yaw
        _to = np.arctan2(self.__target.y - self.__position.y, self.__target.x - self.__position.x)

        heading = _to - _from
        if heading > math.pi:
            heading -= 2 * math.pi

        elif heading < -math.pi:
            heading += 2 * math.pi
        
        return heading
    
    def compute_control(self):
        rospy.wait_for_message(self.__topic_odom, Odometry, timeout=5)

        current_error = Position(0.0, 0.0, 0.0)

        current_error.x = self.__linear_dist()
        current_error.yaw = self.__angular_dist()

        p_term_x = self.__kp * current_error.x
        p_term_yaw = self.__kp * current_error.yaw

        d_term_x = self.__kd * self.__prev_error.x
        d_term_yaw = self.__kd * self.__prev_error.yaw

        self.__prev_error = current_error
        
        if current_error.yaw <= np.radians(60):
            self.__vel_cmd.linear.x = p_term_x + d_term_x
        self.__vel_cmd.angular.z = p_term_yaw + d_term_yaw

        self.__pub_vel.publish(self.__vel_cmd)

    def obstacle_avoidance(self):
        rospy.wait_for_message(self.__topic_lidar, LaserScan, timeout=5)
        left = self.__lidar[round(len(self.__lidar)/2):]
        right = self.__lidar[:round(len(self.__lidar)/2)]

        self.__vel_cmd.linear.x = 0.0
        self.__vel_cmd.angular.z = 0.0
        self.__pub_vel.publish(self.__vel_cmd)

        
        
        if min(right) < 0.2:
            self.__vel_cmd.linear.x = 0.0
            self.__vel_cmd.angular.z = 0.0
            self.__pub_vel.publish(self.__vel_cmd)
            self.__vel_cmd.angular.z = 0.25 
            self.__pub_vel.publish(self.__vel_cmd)
            
            return True

        if min(left) < 0.25:
            self.__vel_cmd.linear.x = 0.0
            self.__vel_cmd.angular.z = 0.0
            self.__pub_vel.publish(self.__vel_cmd)
            self.__vel_cmd.linear.x = 0.0
            self.__vel_cmd.angular.z = -0.25 
            self.__pub_vel.publish(self.__vel_cmd)
    
            return True
        
        return False
    
    def reached_goal(self) -> bool:
        diff_lin = self.__linear_dist()
        return True if diff_lin <= 0.1 else False    
    
    def __str__(self) -> str:
        position = "Position: ({:.2f}, {:.2f}) - Distance to target: {:.2f} - Target: ({}, {})".format(\
            self.__position.x, self.__position.y, self.__linear_dist(), self.__target.x, self.__target.y)
        return position

if __name__ == '__main__':
    rospy.init_node('controller_PD')
    topic_odom = rospy.get_param('~odom')
    topic_lidar = rospy.get_param('~lidar') 
    topic_vel = rospy.get_param('~vel')
    px = rospy.get_param('~position_x')
    py = rospy.get_param('~position_y')
    tx = rospy.get_param('~target_x')
    ty = rospy.get_param('~target_y')
    kp = rospy.get_param('~kp')
    kd = rospy.get_param('~kd')

    time.sleep(20)

    robot = Diferencial_PID(topic_odom, topic_lidar, topic_vel, px, py, tx, ty, kp, kd)
    
    while not robot.reached_goal():
        if not robot.obstacle_avoidance():
            robot.compute_control()
        print(robot)

    
        



    



    