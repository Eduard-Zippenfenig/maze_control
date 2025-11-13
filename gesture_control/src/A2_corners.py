#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped
import time

class MoveRobotA2:
    def __init__(self):
        rospy.init_node('move_robot_a2_corners')

        # Publisher for robot commands
        self.pub = rospy.Publisher('/twist_controlled/zoned_command', TwistStamped, queue_size=1)

        # A2 paper size in meters
        # A2 = 420 mm x 594 mm
        self.corners = {
            'top_left':     (-0.210,  0.297),
            'top_right':    ( 0.210,  0.297),
            'bottom_left':  (-0.210, -0.297),
            'bottom_right': ( 0.210, -0.297)
        }

        self.speed = rospy.get_param('~speed', 0.05)  # m/s
        self.duration = rospy.get_param('~duration', 2.0)  # seconds per move

    def send_velocity(self, vx, vy, dur):
        twist = TwistStamped()
        twist.header.frame_id = "arm_tool0"
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = 0

        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)

        while rospy.Time.now().to_sec() - start < dur and not rospy.is_shutdown():
            twist.header.stamp = rospy.Time.now()
            self.pub.publish(twist)
            rate.sleep()

        # stop afterwards
        twist.twist.linear.x = 0
        twist.twist.linear.y = 0
        self.pub.publish(twist)

    def go_to_corner(self, corner_name):
        if corner_name not in self.corners:
            rospy.logerr(f"Unknown corner: {corner_name}")
            return

        x, y = self.corners[corner_name]

        # Normalize velocity direction
        vx = self.speed if x > 0 else -self.speed
        vy = self.speed if y > 0 else -self.speed

        rospy.loginfo(f"Moving to {corner_name} at ({x}, {y})")
        self.send_velocity(vx, vy, self.duration)

    def run(self):
        rospy.sleep(1)

        for corner in ['top_left', 'top_right', 'bottom_right', 'bottom_left']:
            self.go_to_corner(corner)
            rospy.sleep(0.5)

        rospy.loginfo("Completed visiting A2 corners.")

if __name__ == '__main__':
    try:
        node = MoveRobotA2()
        node.run()
    except rospy.ROSInterruptException:
        pass
