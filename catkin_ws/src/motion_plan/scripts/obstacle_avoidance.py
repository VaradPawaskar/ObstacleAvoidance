 #! /usr/bin/env python
 
 import rospy
 import math
 
 from sensor_msgs.msg import LaserScan
 from nav_msgs.msg import Odometry
 from geometry_msgs.msg import Twist
 from tf.transformations import euler_from_quaternion
 
 pub = None
 current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
 x_goal = 0.0
 y_goal = 0.0
 goal_reached = False

 def odom_callback(msg):
   global current_pose
   current_pose['x'] = msg.pose.pose.position.x
   current_pose['y'] = msg.pose.pose.position.y
   quat = msg.pose.pose.orientation
   orientation_list = [quat.x, quat.y, quat.z, quat.w]
   (_, _, yaw) = euler_from_quaternion(orientation_list)
   current_pose['yaw'] = yaw
 
 
 def compute_goal_cmd():
   global goal_reached
   dx = x_goal - current_pose['x']
   dy = y_goal - current_pose['y']
   dist = math.hypot(dx, dy)
   if dist < 0.2:
     goal_reached = True
     return 0.0, 0.0
   desired_yaw = math.atan2(dy, dx)
   yaw_error = desired_yaw - current_pose['yaw']
   yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
   angular_z = yaw_error
   linear_x = 0.5
   return linear_x, angular_z
 
 
 def callback_laser(msg):
   global goal_reached
   # 120 degrees into 3 regions
   regions = {
     'right':  min(min(msg.ranges[0:2]), 10),
     'front':  min(min(msg.ranges[3:5]), 10),
     'left':   min(min(msg.ranges[6:9]), 10),
   }


  goal_linear, goal_angular = compute_goal_cmd()
  linear_x, angular_z = take_action(regions, goal_linear, goal_angular)
  twist = Twist()
  twist.linear.x = linear_x
  twist.angular.z = angular_z
  pub.publish(twist)
  if goal_reached:
    rospy.signal_shutdown('Goal reached')

def take_action(regions, goal_linear, goal_angular):
   threshold_dist = 1.5
   linear_speed = 0.6
   angular_speed = 1
 
  linear_x = goal_linear
  angular_z = goal_angular

   state_description = ''
 

   if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
     state_description = 'case 1 - no obstacle'

    linear_x = goal_linear
    angular_z = goal_angular
   elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
     state_description = 'case 7 - front and left and right'
     linear_x = -linear_speed
     angular_z = angular_speed # Increase this angular speed for avoiding obstacle faster
   elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
     state_description = 'case 2 - front'
     linear_x = 0
     angular_z = angular_speed
   elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
     state_description = 'case 3 - right'
     linear_x = 0
     angular_z = -angular_speed
   elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
     state_description = 'case 4 - left'
     linear_x = 0
     angular_z = angular_speed
   elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
     state_description = 'case 5 - front and right'
     linear_x = 0
     angular_z = -angular_speed
   elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
     state_description = 'case 6 - front and left'
     linear_x = 0
     angular_z = angular_speed
   elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
     state_description = 'case 8 - left and right'
     linear_x = linear_speed
     angular_z = 0
   else:
     state_description = 'unknown case'
     rospy.loginfo(regions)
 
   rospy.loginfo(state_description)
  return linear_x, angular_z
 
 def main(): 
  global pub, x_goal, y_goal

  rospy.init_node('obstacle_avoidance')

  x_goal = rospy.get_param('~x_goal', 0.0)
  y_goal = rospy.get_param('~y_goal', 0.0)

   pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

  rospy.Subscriber('/odom', Odometry, odom_callback)
  rospy.Subscriber('/robot/laser/scan', LaserScan, callback_laser)

   rospy.spin()
 
 if __name__ == '__main__':
   main()

# This script implements a simple obstacle avoidance algorithm using laser scan data.
# It calculates the robot's current position and orientation from odometry data,
# computes the goal command based on the desired goal position, and takes actions
# based on the laser scan data to avoid obstacles. The robot will stop and turn
# when it detects obstacles in its path, and it will move towards the goal when
# the path is clear.