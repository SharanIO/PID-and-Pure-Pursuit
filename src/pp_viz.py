#!/usr/bin/env python3

import rospy
import numpy as np
import rospkg
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from ackermann_msgs.msg import AckermannDrive
from tf.transformations import euler_from_quaternion

class Pure_Pursuit:
    """
    Pure Pursuit Controller for path following using waypoints.

    This class implements the Pure Pursuit algorithm to calculate the steering 
    angle for a car-like robot to follow a given path. It subscribes to odometry 
    data, publishes control commands using Ackermann steering, and visualizes 
    the path and waypoints in RViz.
    """

    def __init__(self, x_path, y_path):
        """
        Initialize the Pure Pursuit controller.

        Args:
            x_path (list): List of x-coordinates of waypoints.
            y_path (list): List of y-coordinates of waypoints.
        """
        self.ref_sub = rospy.Publisher("/car_1/ref", MarkerArray, queue_size=1)
        self.ack_Publisher = rospy.Publisher("/car_1/command", AckermannDrive, queue_size=1)

        self.x_path = x_path
        self.y_path = y_path

        self.l_d = rospy.get_param("pure_pursuit/lookahead_distance", 1.0)  # Lookahead distance
        self.kdd = 1.2  # Gain factor for lookahead distance
        self.roll, self.pitch, self.yaw = 0, 0, 0  # Orientation variables

        self.nearest_x = 0
        self.nearest_y = 0
        self.look_ahead_x = 0
        self.look_ahead_y = 0

        self.ack = AckermannDrive()
        self.ack.steering_angle = 0
        self.ack.steering_angle_velocity = 0
        self.ack.speed = rospy.get_param("Vmax")

        self.ind = 0  # Current waypoint index
        self.ind_m = 0  # Index of the matched waypoint

        self.rate = rospy.Rate(10)  # ROS rate (10 Hz)
        self.viz = MarkerArray()  # Marker array for visualization

    def publish_viz(self):
        """Publish the visualization markers to RViz."""
        self.ref_sub.publish(self.viz)

    def path_viz(self):
        """Create and populate markers for visualizing the path."""
        self.viz = MarkerArray()  # Clear previous markers
        for i in range(len(self.x_path)):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = marker.scale.y = marker.scale.z = 0.15

            if (self.x_path[i] == self.nearest_x) and (self.y_path[i] == self.nearest_y):
                marker.color.a = 1.0
                marker.color.r, marker.color.g, marker.color.b = 0.627, 0.125, 0.941  # Purple
            elif (self.x_path[i] == self.look_ahead_x) and (self.y_path[i] == self.look_ahead_y):
                marker.color.a = 1.0
                marker.color.r, marker.color.g, marker.color.b = 1, 1, 0  # Yellow
            else:
                marker.color.a = 1.0
                marker.color.r, marker.color.g, marker.color.b = 0, 0.392, 0.392  # Dark green

            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.x_path[i]
            marker.pose.position.y = self.y_path[i]
            self.viz.markers.append(marker)

    def closest_point(self, x_pos, y_pos):
        """
        Find the closest waypoint to the current position.

        Args:
            x_pos (float): Current x-coordinate of the robot.
            y_pos (float): Current y-coordinate of the robot.
        """
        f_path_x = self.x_path[self.ind:]
        f_path_y = self.y_path[self.ind:]
        distances = [((x_pos - fx) ** 2 + (y_pos - fy) ** 2) ** 0.5 for fx, fy in zip(f_path_x, f_path_y)]

        self.ind = distances.index(min(distances))
        value = f_path_x[self.ind]
        self.ind_m = np.where(self.x_path == value)

        self.nearest_x = f_path_x[self.ind]
        self.nearest_y = f_path_y[self.ind]

    def look_ahead_point(self, x_pos, y_pos):
        """
        Find the lookahead point based on the lookahead distance.

        Args:
            x_pos (float): Current x-coordinate of the robot.
            y_pos (float): Current y-coordinate of the robot.
        """
        f_path_x = self.x_path[self.ind_m[0][0]:]
        f_path_y = self.y_path[self.ind_m[0][0]:]

        self.l_d = self.kdd * self.ack.speed

        distances = [((x_pos - fx) ** 2 + (y_pos - fy) ** 2) ** 0.5 for fx, fy in zip(f_path_x, f_path_y)]
        difference_array = np.abs(np.asarray(distances) - self.l_d)

        index = difference_array.argmin()
        self.look_ahead_x = f_path_x[index]
        self.look_ahead_y = f_path_y[index]

    def steering_input(self, x_pos, y_pos):
        """
        Compute the required steering angle based on the lookahead point.

        Args:
            x_pos (float): Current x-coordinate of the robot.
            y_pos (float): Current y-coordinate of the robot.
        """
        x_tp = (self.look_ahead_x - x_pos) * np.cos(-self.yaw) - (self.look_ahead_y - y_pos) * np.sin(-self.yaw)
        y_tp = (self.look_ahead_x - x_pos) * np.sin(-self.yaw) + (self.look_ahead_y - y_pos) * np.cos(-self.yaw)

        alpha = np.arctan2(y_tp, x_tp)
        num = 2 * 0.384 * np.sin(alpha)  # 0.384 is the wheelbase
        self.angle = np.arctan2(num, self.l_d)

        max_angle = np.radians(25)
        min_angle = np.radians(-25)
        self.ack.steering_angle = np.clip(self.angle, min_angle, max_angle)

    def reset(self):
        """Reset the controller state when the end of the path is reached."""
        self.ind = self.ind_m = 0
        self.nearest_x = self.nearest_y = 0
        self.look_ahead_x = self.look_ahead_y = 0

    def ack_publish(self):
        """Publish the computed steering command."""
        self.ack_Publisher.publish(self.ack)

    def odom_callback(self, msgs):
        """
        Callback function to process odometry data and update the vehicle state.

        Args:
            msgs (Odometry): Odometry message containing the robot's position and orientation.
        """
        x = msgs.pose.pose.position.x
        y = msgs.pose.pose.position.y
        ori = msgs.pose.pose.orientation
        orientations = [ori.x, ori.y, ori.z, ori.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientations)

        if self.nearest_x == self.x_path[-1]:
            self.reset()

        self.path_viz()
        self.publish_viz()
        self.closest_point(x, y)
        self.look_ahead_point(x, y)
        self.steering_input(x, y)
        self.ack_publish()
        self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("viz_pp")  # Initialize the ROS node

    file = rospkg.RosPack()
    path = file.get_path("pid_and_pp")
    file_path = path + "/ground_track.csv"
    data = np.genfromtxt(file_path, delimiter=",")

    x = data[:, 1]
    y = data[:, 2]
    x_pos = np.delete(x, [0])
    y_pos = np.delete(y, [0])

    pp = Pure_Pursuit(x_pos, y_pos)

    while not rospy.is_shutdown():
        rospy.Subscriber("/car_1/base/odom", Odometry, pp.odom_callback, queue_size=2)
        rospy.spin()
