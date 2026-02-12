#!/usr/bin/env python
"""
JetAuto Square Pattern Controller for Project 2 - AIG240

This script controls the JetAuto robot to move in a 1-meter square pattern:
1. Move forward from (0, 0, 0°) to (1, 0, 0°)
2. Move left sideways from (1, 0, 0°) to (1, 1, 0°)
3. Turn clockwise from (1, 1, 0°) to (1, 1, -90°)
4. Move right sideways from (1, 1, -90°) to (0, 1, -90°)
5. Move forward and turn from (0, 1, -90°) to (0, 0, 0°) - rotating while traveling

Repeat 2 times after keyboard input.
"""

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Movement parameters
LINEAR_SPEED = 0.1        # m/s - forward/backward speed
STRAFE_SPEED = 0.1        # m/s - sideways speed
ANGULAR_SPEED = 0.5       # rad/s - turning speed
METERS_TO_MOVE = 1.0      # 1 meter movement
DEGREES_TO_TURN = 90.0    # 90 degrees
RADIANS_TO_TURN = DEGREES_TO_TURN * 3.14159 / 180.0  # Convert to radians

# Topic for JetAuto control
CMD_VEL_TOPIC = '/jetauto_controller/cmd_vel'


class JetAutoSquareController:
    def __init__(self):
        """Initialize the ROS node and publisher."""
        rospy.init_node('jetauto_square_controller', anonymous=True)

        self.pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)

        # Wait for publisher to connect
        rospy.sleep(1.0)

        rospy.loginfo("JetAuto Square Controller Initialized")
        rospy.loginfo("Publishing to: {}".format(CMD_VEL_TOPIC))

    def publish_velocity(self, linear_x, linear_y, angular_z, duration):
        """
        Publish velocity commands for a specified duration.

        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right strafe velocity (m/s)
            angular_z: Rotational velocity (rad/s)
            duration: Time to publish in seconds
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z

        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.pub.publish(twist)
            rate.sleep()

        # Stop the robot after movement
        self.stop_robot()

    def stop_robot(self, duration=0.1):
        """Stop the robot by sending zero velocity commands."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.pub.publish(twist)
            rate.sleep()

    def move_forward(self, distance=1.0):
        """Move forward a specified distance in meters."""
        duration = distance / LINEAR_SPEED
        rospy.loginfo("Moving forward {:.1f} meters...".format(distance))
        self.publish_velocity(LINEAR_SPEED, 0.0, 0.0, duration)

    def strafe_left(self, distance=1.0):
        """Strafe left a specified distance in meters."""
        duration = distance / STRAFE_SPEED
        rospy.loginfo("Strafing left {:.1f} meters...".format(distance))
        self.publish_velocity(0.0, STRAFE_SPEED, 0.0, duration)

    def strafe_right(self, distance=1.0):
        """Strafe right a specified distance in meters."""
        duration = distance / STRAFE_SPEED
        rospy.loginfo("Strafing right {:.1f} meters...".format(distance))
        self.publish_velocity(0.0, -STRAFE_SPEED, 0.0, duration)

    def turn_clockwise(self, degrees=90.0):
        """Turn clockwise a specified number of degrees."""
        radians = degrees * 3.14159 / 180.0
        duration = radians / ANGULAR_SPEED
        rospy.loginfo("Turning clockwise {:.1f} degrees...".format(degrees))
        self.publish_velocity(0.0, 0.0, -ANGULAR_SPEED, duration)

    def move_diagonal_with_rotation(self, distance=1.0, rotation_degrees=90.0):
        """
        Move forward while simultaneously rotating.
        Used for the final segment: (0, 1, -90°) to (0, 0, 0°)
        """
        duration = distance / LINEAR_SPEED
        radians = rotation_degrees * 3.14159 / 180.0
        angular_speed = radians / duration

        rospy.loginfo("Moving diagonal {:.1f}m while rotating {:.1f} degrees...".format(
            distance, rotation_degrees))

        twist = Twist()
        twist.linear.x = LINEAR_SPEED
        twist.linear.y = -LINEAR_SPEED  # Moving diagonally backward-right
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed  # Counter-clockwise rotation

        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.pub.publish(twist)
            rate.sleep()

        self.stop_robot()

    def execute_square_pattern(self, repetitions=2):
        """
        Execute the square pattern for the specified number of repetitions.

        Pattern:
        1. Move forward from (0, 0, 0°) to (1, 0, 0°)
        2. Move left sideways from (1, 0, 0°) to (1, 1, 0°)
        3. Turn clockwise from (1, 1, 0°) to (1, 1, -90°)
        4. Move right sideways from (1, 1, -90°) to (0, 1, -90°)
        5. Move forward and turn from (0, 1, -90°) to (0, 0, 0°)
        """
        rospy.loginfo("="*50)
        rospy.loginfo("Starting Square Pattern - {} repetitions".format(repetitions))
        rospy.loginfo("="*50)

        for rep in range(1, repetitions + 1):
            rospy.loginfo("\n--- Repetition {}/{} ---".format(rep, repetitions))

            # Step 1: Move forward from (0, 0, 0°) to (1, 0, 0°)
            rospy.loginfo("Step 1: Forward (0, 0, 0°) -> (1, 0, 0°)")
            self.move_forward(METERS_TO_MOVE)
            rospy.sleep(0.5)

            # Step 2: Move left sideways from (1, 0, 0°) to (1, 1, 0°)
            rospy.loginfo("Step 2: Left strafe (1, 0, 0°) -> (1, 1, 0°)")
            self.strafe_left(METERS_TO_MOVE)
            rospy.sleep(0.5)

            # Step 3: Turn clockwise from (1, 1, 0°) to (1, 1, -90°)
            rospy.loginfo("Step 3: Turn clockwise (1, 1, 0°) -> (1, 1, -90°)")
            self.turn_clockwise(DEGREES_TO_TURN)
            rospy.sleep(0.5)

            # Step 4: Move right sideways from (1, 1, -90°) to (0, 1, -90°)
            rospy.loginfo("Step 4: Right strafe (1, 1, -90°) -> (0, 1, -90°)")
            self.strafe_right(METERS_TO_MOVE)
            rospy.sleep(0.5)

            # Step 5: Move forward and turn from (0, 1, -90°) to (0, 0, 0°)
            # This is a diagonal movement with simultaneous rotation
            rospy.loginfo("Step 5: Diagonal with rotation (0, 1, -90°) -> (0, 0, 0°)")
            self.move_diagonal_with_rotation(METERS_TO_MOVE, DEGREES_TO_TURN)
            rospy.sleep(0.5)

        rospy.loginfo("="*50)
        rospy.loginfo("Square Pattern Complete!")
        rospy.loginfo("="*50)


def get_key():
    """Get a single keypress from terminal (non-blocking with timeout)."""
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        # Check if there's a key ready to read
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
        else:
            key = None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def wait_for_start():
    """Wait for user to press Enter to start the movement."""
    rospy.loginfo("="*50)
    rospy.loginfo("Press ENTER to start the square pattern...")
    rospy.loginfo("Press Ctrl+C to exit")
    rospy.loginfo("="*50)

    # Wait for Enter key
    while not rospy.is_shutdown():
        key = get_key()
        if key == '\r' or key == '\n':  # Enter key
            return True
        rospy.sleep(0.1)


def main():
    """Main function to run the JetAuto square controller."""
    try:
        # Create controller
        controller = JetAutoSquareController()

        # Wait for user input
        wait_for_start()

        # Execute square pattern 2 times
        controller.execute_square_pattern(repetitions=2)

        rospy.loginfo("Pattern execution completed. Robot at starting position.")

    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user.")
    except Exception as e:
        rospy.logerr("Error occurred: {}".format(e))


if __name__ == '__main__':
    main()
