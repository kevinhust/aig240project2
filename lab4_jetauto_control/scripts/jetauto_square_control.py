#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
JetAuto Square Pattern Controller for Project 2 - AIG240
Calibrated Version to fix drifting in Gazebo.
"""

import rospy
from geometry_msgs.msg import Twist
import math
import sys

# Movement parameters - Calibrated for Gazebo physics
LINEAR_SPEED = 0.1        # m/s
STRAFE_SPEED = 0.1        # m/s
ANGULAR_SPEED = 0.4       # rad/s (approx 23 deg/s)
METERS_TO_MOVE = 1.0      
DEGREES_TO_TURN = 90.0    

# Calibration factors (adjust if robot still drifts)
# Increase if turn is < 90, decrease if > 90
TURN_CALIBRATION = 1.0  
# Interval between steps to allow physics to settle
SETTLE_TIME = 1.2

# Topic for JetAuto control
CMD_VEL_TOPIC = '/jetauto_controller/cmd_vel'


class JetAutoSquareController:
    def __init__(self):
        """Initialize the ROS node and publisher."""
        rospy.init_node('jetauto_square_controller', anonymous=True)

        self.pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)

        # Wait for publisher to connect
        rospy.sleep(1.0)

        rospy.loginfo("JetAuto Square Controller Initialized (Calibrated)")
        rospy.loginfo("Publishing to: {}".format(CMD_VEL_TOPIC))

    def publish_velocity(self, linear_x, linear_y, angular_z, duration):
        """Publish velocity commands for a specified duration."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z

        rate = rospy.Rate(20)  # Higher frequency for better precision
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.pub.publish(twist)
            rate.sleep()

        self.stop_robot()

    def stop_robot(self, duration=0.2):
        """Stop the robot completely."""
        twist = Twist()
        self.pub.publish(twist)
        rospy.sleep(duration)

    def move_forward(self, distance=1.0):
        """Move forward (Step 1)."""
        duration = distance / LINEAR_SPEED
        rospy.loginfo("Step 1: Moving forward {:.1f}m".format(distance))
        self.publish_velocity(LINEAR_SPEED, 0.0, 0.0, duration)

    def strafe_left(self, distance=1.0):
        """Strafe left (Step 2)."""
        duration = distance / STRAFE_SPEED
        rospy.loginfo("Step 2: Strafing left {:.1f}m".format(distance))
        self.publish_velocity(0.0, STRAFE_SPEED, 0.0, duration)

    def turn_clockwise(self, degrees=90.0):
        """Turn clockwise (Step 3)."""
        radians = (degrees * math.pi / 180.0) * TURN_CALIBRATION
        duration = radians / ANGULAR_SPEED
        rospy.loginfo("Step 3: Turning CW {:.1f} degrees".format(degrees))
        self.publish_velocity(0.0, 0.0, -ANGULAR_SPEED, duration)

    def strafe_right(self, distance=1.0):
        """Strafe right (Step 4)."""
        duration = distance / STRAFE_SPEED
        rospy.loginfo("Step 4: Strafing right {:.1f}m".format(distance))
        self.publish_velocity(0.0, -STRAFE_SPEED, 0.0, duration)

    def move_diagonal_with_rotation(self, distance=1.0, degrees=90.0):
        """
        Step 5: Move from (0, 1, -90 deg) back to (0, 0, 0 deg).
        To return 1m on Y axis while rotating 90 deg:
        v_x = v_y = 0.5 * distance * omega / (pi/2)
        """
        duration = 8.0 
        omega = (degrees * math.pi / 180.0) / duration
        v_corr = (distance * omega) / 2.0
        
        rospy.loginfo("Step 5: Diagonal rotation (0, 1, -90 deg) -> (0, 0, 0 deg)")
        self.publish_velocity(v_corr, -v_corr, omega, duration)

    def execute_square_pattern(self, repetitions=2):
        """Execute the square pattern with settling time between steps."""
        rospy.loginfo("="*50)
        rospy.loginfo("Starting Calibrated Square Pattern - {} reps".format(repetitions))
        rospy.loginfo("="*50)

        for rep in range(1, repetitions + 1):
            rospy.loginfo("\n--- Repetition {}/{} ---".format(rep, repetitions))

            self.move_forward(METERS_TO_MOVE)
            rospy.sleep(SETTLE_TIME)

            self.strafe_left(METERS_TO_MOVE)
            rospy.sleep(SETTLE_TIME)

            self.turn_clockwise(DEGREES_TO_TURN)
            rospy.sleep(SETTLE_TIME)

            self.strafe_right(METERS_TO_MOVE)
            rospy.sleep(SETTLE_TIME)

            self.move_diagonal_with_rotation(METERS_TO_MOVE, DEGREES_TO_TURN)
            rospy.sleep(SETTLE_TIME)

        rospy.loginfo("="*50)
        rospy.loginfo("Square Pattern Complete!")
        rospy.loginfo("="*50)


def wait_for_start():
    """Wait for user to press Enter."""
    rospy.loginfo("\n" + "="*50)
    rospy.loginfo("JetAuto Calibrated Controller Ready")
    rospy.loginfo("Press ENTER to start the 2-lap pattern...")
    rospy.loginfo("="*50)
    try:
        raw_input()
        return True
    except EOFError:
        return False


def main():
    try:
        controller = JetAutoSquareController()
        if wait_for_start():
            controller.execute_square_pattern(repetitions=2)
            rospy.loginfo("Task finished. Robot should be at (0,0).")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error: {}".format(e))


if __name__ == '__main__':
    main()
