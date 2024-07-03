#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry

class WaypointMux:
    def __init__(self):
        # Initialize the node
        rospy.init_node('waypoint_mux', anonymous=True)
        self.node_start_time = rospy.get_rostime().to_sec()
        self.first_time_flag = True
        # print(self.node_start_time)
        self.time_elapsed_since_start = 0
        # Create a publisher for the /way_points topic
        self.waypoint_pub = rospy.Publisher('/way_point', PointStamped, queue_size=10)
        self.home_position : Point = None
        
        # Subscribers for the exploration and far planner waypoints
        self.exploration_sub = rospy.Subscriber('/exploration/way_point', PointStamped, self.exploration_callback)
        self.far_planner_sub = rospy.Subscriber('/far_planner/way_point', PointStamped, self.far_planner_callback)
        self.far_planner_goal_pub = rospy.Publisher('/goal_point', PointStamped, queue_size=10)
        self.state_estimation_sub = rospy.Subscriber('/state_estimation_at_scan', Odometry, self.state_estimation_callback)
        # Internal storage for the waypoints
        self.exploration_waypoint = None
        self.far_planner_waypoint = None
        
        # Flag to decide which waypoints to forward
        self.use_exploration = True
        self.exploration_timeout_triggered = False
        self.exploration_timeout = 20 # in seconds
        
        # Timer to check the conditions periodically
        self.exploration_timeout_timer = rospy.Timer(rospy.Duration(nsecs=int(0.1*1e9)), self.exploration_timeout_timer_callback) # 10 Hz Timer for condition checks
    
    def exploration_callback(self, msg):
        self.exploration_waypoint = msg
        self.publish_waypoint()
    
    def far_planner_callback(self, msg):
        self.far_planner_waypoint = msg
        self.publish_waypoint()
    
    def state_estimation_callback(self, msg):
        if self.home_position is None:
            self.home_position = Point()
            self.home_position.x = msg.pose.pose.position.x
            self.home_position.y = msg.pose.pose.position.y
            self.home_position.z = msg.pose.pose.position.z
    
    def exploration_timeout_timer_callback(self, event):
        if not self.exploration_timeout_triggered:
            if self.first_time_flag:
                self.node_start_time = rospy.get_rostime().to_sec()
                self.first_time_flag = False
            self.time_elapsed_since_start = rospy.get_rostime().to_sec() - self.node_start_time
            # print(self.time_elapsed_since_start)
            if self.time_elapsed_since_start > self.exploration_timeout:
                self.exploration_timeout_triggered = True
                self.use_exploration = False
                rospy.logwarn("Exploration timed out, using far planner to force return back to start.")
        else:
            goal_point = PointStamped()
            goal_point.header.stamp = rospy.Time.now()
            goal_point.header.frame_id = "world_msf_graph"
            goal_point.point = self.home_position
            self.far_planner_goal_pub.publish(goal_point)
        
    
    def publish_waypoint(self):
        if self.use_exploration and self.exploration_waypoint is not None:
            self.waypoint_pub.publish(self.exploration_waypoint)
        elif not self.use_exploration and self.far_planner_waypoint is not None:
            self.waypoint_pub.publish(self.far_planner_waypoint)
    
if __name__ == '__main__':
    try:
        mux = WaypointMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
