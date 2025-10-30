#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pygame #for joystick control

class JoystickTwist:
    def __init__(self):
        rospy.init_node('joystick_twist_node')

        # Parameters
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.linear_scale = rospy.get_param("~linear_scale", 0.5)
        self.angular_scale = rospy.get_param("~angular_scale", 0.5)
        self.deadzone = rospy.get_param("~deadzone", 0.1)

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.rate = rospy.Rate(10) 

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            rospy.logerr("No joystick detected!")
            raise SystemExit

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        rospy.loginfo(f"Joystick connected: {self.joy.get_name()}")

    def read_axis(self, axis):
        val = self.joy.get_axis(axis)
        return 0.0 if abs(val) < self.deadzone else val

    def run(self):
        while not rospy.is_shutdown():
            pygame.event.pump()

            # Your mapping:
            angular_z = self.read_axis(1) * self.angular_scale     # Left joystick X
            linear_x  = -self.read_axis(4) * self.linear_scale     # Right joystick Y (invert for natural forward)

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z

            self.pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    node = JoystickTwist()
    node.run()
