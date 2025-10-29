#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwist:
def __init__(self):
rospy.init_node('joy_to_twist')

# TODO: Replace with your actual topic name
self.cmd_pub = rospy.Publisher('___UR5E_TWIST_TOPIC___', Twist, queue_size=1)

self.linear_scale = 0.5 # tune as needed
self.angular_scale = 0.5 # tune as needed

rospy.Subscriber('/joy', Joy, self.joy_callback)
self.twist = Twist()
self.rate = rospy.Rate(20) # 20 Hz update

def joy_callback(self, msg):
# Right stick: linear (axes 3, 4 for example)
self.twist.linear.x = msg.axes[3] * self.linear_scale
self.twist.linear.y = msg.axes[4] * self.linear_scale
self.twist.linear.z = msg.axes[1] * self.linear_scale # optional (use triggers, etc.)

# Left stick: angular
self.twist.angular.x = msg.axes[0] * self.angular_scale
self.twist.angular.y = msg.axes[1] * self.angular_scale
self.twist.angular.z = msg.axes[2] * self.angular_scale

def run(self):
while not rospy.is_shutdown():
self.cmd_pub.publish(self.twist)
self.rate.sleep()

if __name__ == '__main__':
node = JoyToTwist()
node.run()
