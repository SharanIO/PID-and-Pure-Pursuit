#!/usr/bin/env python3

# Import necessary ROS and Python modules
import rospy
import numpy as np
from nav_msgs.msg import Odometry  # For receiving odometry data
from ackermann_msgs.msg import AckermannDrive  # For sending drive commands

class PID:
    """
    A PID controller class for controlling a car using ROS.

    The controller calculates the appropriate steering angle to follow
    a target trajectory based on the car’s position and heading.
    """

    def __init__(self):
        """Initializes the PID controller and sets up variables."""
        # Publisher to send Ackermann drive commands
        self.ack_Publisher = rospy.Publisher("/car_1/command", AckermannDrive, queue_size=1)

        # Variables to track position and error
        self.x, self.y = 0, 0  # Current position
        self.prev_x = 0  # Previous x-coordinate
        self.y_err = 0  # Lateral error

        # Desired steering angle and related variables
        self.des_angle = 0  
        self.ack = AckermannDrive()  # Ackermann drive message
        self.ack.steering_angle = 0  # Initialize steering angle
        self.ack.speed = rospy.get_param("Vmax", 1.0)  # Speed with default value of 1.0

        # Time and error tracking
        self.current_time, self.prev_time = 0, 0
        self.prev_err = 0  # Previous error for derivative calculation

        # ROS rate control (10 Hz)
        self.rate = rospy.Rate(10)

    def ack_publish(self):
        """Publishes the current Ackermann drive message."""
        self.ack_Publisher.publish(self.ack)

    def calculate_y_error(self, x, y):
        """
        Calculates the lateral error (y_err) based on the car's position.
        The error increases for larger x-ranges to reflect the target trajectory.
        """
        # Define the error offset based on the x-range
        x_ranges = [(0, 15, 0), (15, 30, 2), (30, 45, 5), (45, 60, 10), (60, 200, 15)]

        # Find the appropriate offset for the current x-coordinate
        offset = next((o for (start, end, o) in x_ranges if start <= x <= end), 0)

        # Calculate and return the y-error based on trajectory side
        if y < 0:
            rospy.loginfo("Car is left of the target trajectory.")
            return offset - y  # Adjusting for left trajectory side
        else:
            rospy.loginfo("Car is right of the target trajectory.")
            return offset - y  # Adjusting for right trajectory side


    def odom_callback(self, msgs):
        """
        Callback function to process odometry data.
        Updates the car's position, computes the lateral error, and applies PID control.
        """
        # Store previous position and time
        self.prev_x = self.x
        self.prev_time = self.current_time

        # Get the current position from the odometry message
        self.x = msgs.pose.pose.position.x
        self.y = msgs.pose.pose.position.y

        # Log the current position
        rospy.loginfo(f"x: {self.x}, y: {self.y}")

        # Update the current time
        self.current_time = rospy.get_time()

        # Calculate the lateral error
        self.y_err = self.calculate_y_error(self.x, self.y)
        rospy.loginfo(f"Lateral Error: {self.y_err}")

        # Apply PID control
        self.pid()

        # Publish the drive command
        self.ack_publish()

        # Check for stopping condition
        self.stop()

    def stop(self):
        """
        Stops the car by setting speed to 0 if the x-coordinate reaches 200.
        """
        if self.x >= 200:
            rospy.loginfo("Stopping the car...")
            self.ack.speed = 0  # Stop the car

    def pid(self):
        """
        PID control logic to calculate the appropriate steering angle.
        """
        if self.current_time != 0:  # Ensure valid time
            # PID gains
            Kp = rospy.get_param("pid/kp", 0.09) # Proportional gain
            Kd = rospy.get_param("pid/kd", 6.0)  # Derivative gain
            Ki = rospy.get_param("pid/ki", 0.1)   # Integral gain (not used here)

            # Kp = 0.09  # Proportional gain
            # Kd = 6.0  # Derivative gain
            # Ki = 0.1  # Integral gain (not used here)

            # Calculate error components
            der_error = self.y_err - self.prev_err  # Derivative of error
            prop = Kp * self.y_err  # Proportional term
            der = Kd * der_error  # Derivative term

            # Combine PID terms
            pid_param = prop + der

            # Update the previous error
            self.prev_err = self.y_err

            # Constrain steering angle within [-25, 25] degrees
            max_angle = np.radians(25)
            min_angle = np.radians(-25)

            # Set the steering angle based on PID output
            if pid_param <= min_angle:
                self.ack.steering_angle = min_angle
            elif pid_param >= max_angle:
                self.ack.steering_angle = max_angle
            else:
                self.ack.steering_angle = pid_param

            # Log the calculated steering angle
            rospy.loginfo(f"Steering Angle: {np.degrees(self.ack.steering_angle)} degrees")

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("pid_node")

    # Create an instance of the PID controller
    pid = PID()

    # Subscribe to the odometry topic to receive position updates
    odom_Subscriber = rospy.Subscriber("/car_1/base/odom", Odometry, pid.odom_callback)

    # Keep the node running until shutdown
    rospy.spin()
