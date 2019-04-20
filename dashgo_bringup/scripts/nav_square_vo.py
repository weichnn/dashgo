#!/usr/bin/env python

""" nav_square.py - Version 1.1 2013-12-20

    A basic demo of the using odometry data to move the robot
    along a square trajectory.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, atan
import PyKDL


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[1]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


class NavSquare():
    def __init__(self):
        # Give the node a name
        rospy.init_node('nav_square', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values?
        rate = 20
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the parameters for the target square
        goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
        goal_angle = radians(rospy.get_param("~goal_angle", 90))    # degrees converted to radians
        linear_max_speed = rospy.get_param("~linear_max_speed", 0.2)        # meters per second
        angular_max_speed = rospy.get_param("~angular_max_speed", 0.7)      # radians per second
        linear_min_speed = rospy.get_param("~linear_min_speed", 0.02)        # meters per second
        angular_min_speed = rospy.get_param("~angular_min_speed", 0.07)      # radians per second
        angular_tolerance = radians(rospy.get_param("~angular_tolerance", 2)) # degrees to radians
        distance_tolerance = radians(rospy.get_param("~distance_tolerance", 0.02)) # degrees to radians
        linear_Kp = radians(rospy.get_param("~linear_Kp", 1)) # degrees to radians
        angular_Kp = radians(rospy.get_param("~angular_Kp", 0.02)) # degrees to radians
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
         
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/map'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
                
        # Create a list to hold the waypoint poses
        waypoints = list()
        
        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        waypoints.append(Point(goal_distance, 0.0, 0.0))
        waypoints.append(Point(goal_distance, goal_distance, 0.0))
        waypoints.append(Point(0.0, goal_distance, 0.0))
        waypoints.append(Point(0.0, 0.0, 0.0))
        
        # Initialize the position variable as a Point type
        position = Point()

        # Cycle through the four sides of the square
        for i in range(4):
            # Initialize the movement command
            move_cmd = Twist()

            goal = waypoints[i];

            # Get the starting position values     
            (position, rotation) = self.get_odom()
            
            now_position = Point(position.z, position.x, 0.0)
            vector_error = goal - now_position
            distance_error = sqrt(pow((vector_error.x), 2) + pow((vector_error.y), 2))
            angle_error = atan2(vector_error.y, vector_error.x)/pi*180.0 - rotation
            
            if abs(angle_error) > 30.0:
                # Set the movement command to a rotation
                if angle_error > 0:
                    move_cmd.angular.z = angular_speed
                else:
                    move_cmd.angular.z = -angular_speed
                
                # Begin the rotation
                while abs(angle_error) > abs(angular_tolerance) and not rospy.is_shutdown():
                    # Publish the Twist message and sleep 1 cycle         
                    self.cmd_vel.publish(move_cmd)
                    
                    r.sleep()
                    
                    # Get the current rotation
                    (position, rotation) = self.get_odom()
                    
                    now_position = Point(position.z, position.x, 0.0)
                    vector_error = goal - now_position
                    distance_error = sqrt(pow((vector_error.x), 2) + pow((vector_error.y), 2))
                    angle_error = atan2(vector_error.y, vector_error.x)/pi*180.0 - rotation

            while abs(angle_error) > abs(angular_tolerance) and distance_error > abs(distance_tolerance) and not rospy.is_shutdown():
                linear_speed = linear_Kp*distance_error
                if abs(linear_speed) > abs(linear_max_speed):
                    linear_speed = linear_max_speed;
                if abs(linear_speed) < abs(linear_min_speed):
                    linear_speed = linear_min_speed

                move_cmd.linear.x = linear_speed;


                angular_speed = angle_error*angular_Kp
                if abs(angular_speed) > abs(angular_max_speed):
                    linear_speed = angular_max_speed;
                if abs(angular_speed) < abs(angular_min_speed):
                    linear_speed = angular_min_speed
                if angle_error > 0:
                    move_cmd.angular.z = angular_speed
                else:
                    move_cmd.angular.z = -angular_speed

                # Publish the Twist message and sleep 1 cycle         
                self.cmd_vel.publish(move_cmd)
                
                r.sleep()
                
                # Get the current rotation
                (position, rotation) = self.get_odom()
                
                now_position = Point(position.z, position.x, 0.0)
                vector_error = goal - now_position
                distance_error = sqrt(pow((vector_error.x), 2) + pow((vector_error.y), 2))
                angle_error = atan2(vector_error.y, vector_error.x)/pi*180.0 - rotation



            # # Set the movement command to forward motion
            # move_cmd.linear.x = linear_speed

            # x_start = position.z
            # y_start = position.x

            # # Keep track of the distance traveled
            # distance = 0

            # # Enter the loop to move along a side
            # while distance < goal_distance and not rospy.is_shutdown():
            #     # Publish the Twist message and sleep 1 cycle         
            #     self.cmd_vel.publish(move_cmd)
                
            #     r.sleep()
        
            #     # Get the current position
            #     (position, rotation) = self.get_odom()
                
            #     # Compute the Euclidean distance from the start
            #     distance = sqrt(pow((position.z - x_start), 2) + 
            #                     pow((position.x - y_start), 2))
            #     # rospy.loginfo("x: "+position.z+"y: "+position.x+"distance: "+ distance)
                
                
            # # Stop the robot before rotating
            # move_cmd = Twist()
            # self.cmd_vel.publish(move_cmd)
            # rospy.sleep(1.0)
            
            # # Set the movement command to a rotation
            # move_cmd.angular.z = angular_speed
            
            # # Track the last angle measured
            # last_angle = rotation
            
            # # Track how far we have turned
            # turn_angle = 0
            
            # # Begin the rotation
            # while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            #     # Publish the Twist message and sleep 1 cycle         
            #     self.cmd_vel.publish(move_cmd)
                
            #     r.sleep()
                
            #     # Get the current rotation
            #     (position, rotation) = self.get_odom()
                
            #     # Compute the amount of rotation since the last lopp
            #     delta_angle = normalize_angle(rotation - last_angle)
                
            #     turn_angle += delta_angle
            #     last_angle = rotation

            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            
        # Stop the robot when we are done
        self.cmd_vel.publish(Twist())
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        rospy.loginfo("x: "+str(Point(*trans).z)+"y: "+str(Point(*trans).x))
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
            
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        NavSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
