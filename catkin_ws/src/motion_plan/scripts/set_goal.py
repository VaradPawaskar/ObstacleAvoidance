#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def set_goal(x, y):
    rospy.init_node('goal_publisher', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.sleep(1)  # Wait for publisher to register

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"  
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0  # Neutral orientation (facing forward)

    pub.publish(goal)
    rospy.loginfo(f"Goal set to ({x}, {y})")

if __name__ == '__main__':
    # Set your desired coordinates here
    set_goal(5.0, 3.0)  # Example coordinates