#! /usr/bin/env python3

import rospy
import math
import tf

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

# Global variables
pub = None
goal_position = None
current_position = None
goal_reached = False

def odom_callback(msg):
    global current_position
    current_position = msg.pose.pose

def goal_callback(msg):
    global goal_position, goal_reached
    goal_position = msg.pose
    goal_reached = False
    rospy.loginfo("New goal received: (%.2f, %.2f)", 
                  goal_position.position.x, 
                  goal_position.position.y)

def calculate_distance(p1, p2):
    return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

def calculate_angle(p1, p2):
    return math.atan2(p2.y - p1.y, p2.x - p1.x)

def callback_laser(msg):
    global pub, goal_position, current_position, goal_reached
    
    # Skip processing if we don't have position or goal data
    if current_position is None or goal_position is None or goal_reached:
        return
        
    # Calculate distance and angle to goal
    distance_to_goal = calculate_distance(current_position.position, 
                                         goal_position.position)
    angle_to_goal = calculate_angle(current_position.position, 
                                   goal_position.position)
    
    # Check if goal is reached
    if distance_to_goal < 0.3:
        goal_reached = True
        twist = Twist()
        pub.publish(twist)
        rospy.loginfo("Goal reached!")
        return
    
    # Process laser data (120 degrees into 3 regions)
    regions = {
        'right':  min(min(msg.ranges[0:2]), 10),
        'front':  min(min(msg.ranges[3:5]), 10),
        'left':   min(min(msg.ranges[6:9]), 10),
    }
    
    # Combine obstacle avoidance with path planning
    take_action(regions, angle_to_goal, distance_to_goal)

def take_action(regions, angle_to_goal, distance_to_goal):
    # Parameters
    threshold_dist = 1.5
    linear_speed = 0.6
    angular_speed = 1
    goal_gain = 0.8  # Weight for goal direction vs obstacle avoidance

    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''
    
    # Calculate base goal direction behavior
    goal_angular_z = angular_speed * angle_to_goal * goal_gain
    
    # Obstacle avoidance with goal seeking integration
    if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
        # No obstacles - head directly to goal
        state_description = 'Clear path to goal'
        linear_x = linear_speed * min(distance_to_goal, 1.0)  # Slow down near goal
        angular_z = goal_angular_z
        
    elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
        state_description = 'Front, left and right blocked'
        linear_x = -linear_speed * 0.7
        angular_z = angular_speed  # Prioritize turning
        
    elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
        state_description = 'Front blocked'
        linear_x = 0
        # Turn in the direction that points toward the goal
        angular_z = angular_speed if angle_to_goal > 0 else -angular_speed
        
    elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
        state_description = 'Right blocked'
        linear_x = linear_speed * 0.7
        angular_z = goal_angular_z + angular_speed * 0.5  # Bias away from obstacle
        
    elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
        state_description = 'Left blocked'
        linear_x = linear_speed * 0.7
        angular_z = goal_angular_z - angular_speed * 0.5  # Bias away from obstacle
        
    elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
        state_description = 'Front and right blocked'
        linear_x = 0
        angular_z = angular_speed  # Turn left
        
    elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
        state_description = 'Front and left blocked'
        linear_x = 0
        angular_z = -angular_speed  # Turn right
        
    elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
        state_description = 'Left and right blocked'
        linear_x = linear_speed
        angular_z = goal_angular_z  # Continue toward goal if front is clear
        
    else:
        state_description = 'Unknown situation'
        rospy.loginfo(regions)

    rospy.loginfo(state_description + " | Goal angle: %.2f rad", angle_to_goal)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main():
    global pub
    
    rospy.init_node('path_planning_node')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Subscribe to necessary topics
    rospy.Subscriber('/robot/laser/scan', LaserScan, callback_laser)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    
    rospy.loginfo("Path planning node started. Waiting for goals...")
    rospy.spin()

if __name__ == '__main__':
    main()