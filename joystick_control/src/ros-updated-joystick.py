#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped   
import pygame  # for joystick control

class JoystickTwistStamped:
    def __init__(self):
        rospy.init_node('joystick_twist_node')

        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.linear_scale = rospy.get_param("~linear_scale", 0.5)
        self.angular_scale = rospy.get_param("~angular_scale", 0.5)
        self.deadzone = rospy.get_param("~deadzone", 0.1)

        # Publisher will publish TwistStamped messages
        self.pub = rospy.Publisher('/twist_controller/zoned_command', TwistStamped, queue_size=1)
        self.rate = rospy.Rate(10)  

        # joystick detection
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            rospy.logerr("No joystick detected!")
            raise SystemExit
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        rospy.loginfo(f"Joystick connected: {self.joy.get_name()}")

    def read_axis(self, axis):
        """Read a joystick axis value with deadzone filtering."""
        val = self.joy.get_axis(axis)
        return 0.0 if abs(val) < self.deadzone else val

    def run(self):
        while not rospy.is_shutdown():
            # Process Pygame events to update joystick state
            pygame.event.pump()

            # Joystick mapping:
            # Left joystick - angular motion
            angular_z = self.read_axis(1) * self.angular_scale   # adjust index if needed for your controller
            # Right joystick - linear forward and backward motion
            linear_x  = -self.read_axis(4) * self.linear_scale   # invert if pushing forward gives negative

            # Create TwistStamped message and fill in data
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = rospy.Time.now()        # current time stamp
            twist_stamped.header.frame_id = "base_link"          # coordinate frame of the velocities
            twist_stamped.twist.linear.x = linear_x
            twist_stamped.twist.linear.y = 0.0
            twist_stamped.twist.linear.z = 0.0
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = angular_z

            # Publish the TwistStamped message
            self.pub.publish(twist_stamped)

            # Sleep to maintain loop rate
            # self.rate.sleep()

if __name__ == "__main__":
    node = JoystickTwistStamped()
    node.run()
