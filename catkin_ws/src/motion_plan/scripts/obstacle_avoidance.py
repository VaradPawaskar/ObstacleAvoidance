import rospy
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion


class ObstacleAvoider(object):
    """Navigate towards a goal while avoiding obstacles."""

    def __init__(self):
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.goal_reached = False

        self.x_goal = rospy.get_param('~x_goal', 0.0)
        self.y_goal = rospy.get_param('~y_goal', 0.0)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/robot/laser/scan', LaserScan, self.laser_callback)

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        """Update the stored robot pose."""
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        (_, _, yaw) = euler_from_quaternion(q)
        self.current_pose['yaw'] = yaw

    # ------------------------------------------------------------------
    def compute_goal_cmd(self):
        """Compute velocity commands towards the goal."""
        dx = self.x_goal - self.current_pose['x']
        dy = self.y_goal - self.current_pose['y']
        dist = math.hypot(dx, dy)
        if dist < 0.2:
            self.goal_reached = True
            return 0.0, 0.0
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.current_pose['yaw']
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        angular_z = yaw_error
        linear_x = min(0.5, dist)
        return linear_x, angular_z

    # ------------------------------------------------------------------
    def laser_callback(self, msg):
        """Handle laser scans and publish a Twist command."""
        regions = {
            'right': min(min(msg.ranges[0:2]), 10),
            'front': min(min(msg.ranges[3:5]), 10),
            'left': min(min(msg.ranges[6:9]), 10),
        }

        goal_linear, goal_angular = self.compute_goal_cmd()
        linear_x, angular_z = self.take_action(regions, goal_linear, goal_angular)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub.publish(twist)

        if self.goal_reached:
            rospy.signal_shutdown('Goal reached')

    # ------------------------------------------------------------------
    def take_action(self, regions, goal_linear, goal_angular):
        """Modify goal commands if an obstacle is detected."""
        threshold_dist = 1.5
        linear_speed = 0.6
        angular_speed = 1.0

        linear_x = goal_linear
        angular_z = goal_angular

        if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
            pass  # keep goal commands
        elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
            linear_x = -linear_speed
            angular_z = angular_speed
        elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
            linear_x = 0
            angular_z = angular_speed
        elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
            linear_x = 0
            angular_z = -angular_speed
        elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
            linear_x = 0
            angular_z = angular_speed
        elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
            linear_x = 0
            angular_z = -angular_speed
        elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
            linear_x = 0
            angular_z = angular_speed
        elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
            linear_x = linear_speed
            angular_z = 0
        else:
            rospy.loginfo('Unknown laser case: %s', regions)

        return linear_x, angular_z

def main():
    rospy.init_node('obstacle_avoidance')
    ObstacleAvoider()
    rospy.spin()

if __name__ == '__main__':
    main()
