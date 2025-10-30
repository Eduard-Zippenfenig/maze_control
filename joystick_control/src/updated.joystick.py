#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import pygame

class JoystickJointControl:
    def __init__(self):
        rospy.init_node('joystick_joint_control_node')

        # Parameters - reduced for safety with joint control
        self.linear_scale = rospy.get_param("~linear_scale", 0.05)
        self.angular_scale = rospy.get_param("~angular_scale", 0.05)
        self.deadzone = rospy.get_param("~deadzone", 0.1)

        # Joint velocity publisher
        self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
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

            # Initialize all 6 joint velocities to zero
            joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            # Map joystick axes to joint velocities
            # UR5e joint order: 
            # [0] shoulder_pan_joint (base rotation)
            # [1] shoulder_lift_joint (up/down) 
            # [2] elbow_joint
            # [3] wrist_1_joint
            # [4] wrist_2_joint
            # [5] wrist_3_joint
            
            # Control scheme:
            # Left stick X -> base rotation (joint 0)
            # Left stick Y -> shoulder up/down (joint 1) 
            # Right stick Y -> elbow control (joint 2)
            joint_velocities[0] = self.read_axis(0) * self.angular_scale    # Left stick X - base rotation
            joint_velocities[1] = -self.read_axis(1) * self.linear_scale    # Left stick Y - shoulder up/down
            joint_velocities[2] = -self.read_axis(3) * self.linear_scale    # Right stick Y - elbow

            # Create and publish Float64MultiArray message
            velocity_msg = Float64MultiArray()
            velocity_msg.data = joint_velocities
            self.pub.publish(velocity_msg)

            rospy.loginfo(f"Joystick - J0: {joint_velocities[0]:.3f}, J1: {joint_velocities[1]:.3f}, J2: {joint_velocities[2]:.3f}")
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = JoystickJointControl()
        node.run()
    except rospy.ROSInterruptException:
        pygame.quit()
