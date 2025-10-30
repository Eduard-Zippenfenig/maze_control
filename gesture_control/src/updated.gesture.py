#!/usr/bin/env python3
import rospy
import cv2
import mediapipe as mp
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge, CvBridgeError

class HandTracker:
    def __init__(self):
        rospy.init_node('hand_tracker')

        # Parameters
        self.camera_topic = rospy.get_param('~camera_topic', '/cam/image_raw')
        self.linear_speed = rospy.get_param('~linear_speed', 0.1)
        self.control_zone = rospy.get_param('~control_zone', 0.3)
        self.deadzone = rospy.get_param('~deadzone', 0.05)

        # ROS I/O - UPDATED FOR Float64MultiArray
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback, queue_size=1)

        # Joint velocity publisher for UR5e - TwistStamped
        self.cmd_pub = rospy.Publisher('hand_twist', TwistStamped, queue_size=1)

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils

        rospy.loginfo("Hand Tracker node started - Float64MultiArray Control")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        # Initialize all 6 joint velocities to zero
        joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if results.multi_hand_landmarks:
            for landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, landmarks, self.mp_hands.HAND_CONNECTIONS)

                wrist = landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                mid = landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

                cx = (wrist.x + mid.x) / 2
                cy = (wrist.y + mid.y) / 2

                dx = cx - 0.5  # Horizontal deviation from center
                dy = cy - 0.5  # Vertical deviation from center

                # Apply deadzone
                if abs(dx) < self.deadzone:
                    dx = 0
                if abs(dy) < self.deadzone:
                    dy = 0

                # Map hand position to joint velocities
                # UR5e joint order: 
                # [0] shoulder_pan_joint (base rotation)
                # [1] shoulder_lift_joint (up/down)
                # [2] elbow_joint
                # [3] wrist_1_joint
                # [4] wrist_2_joint  
                # [5] wrist_3_joint
                
                # Control scheme:
                # Horizontal hand movement -> base rotation (joint 0)
                # Vertical hand movement -> shoulder up/down (joint 1)
                joint_velocities[0] = (dx / self.control_zone) * self.linear_speed  # Base rotation
                joint_velocities[1] = -(dy / self.control_zone) * self.linear_speed # Shoulder up/down

                # Clip velocities for safety
                joint_velocities[0] = np.clip(joint_velocities[0], -self.linear_speed, self.linear_speed)
                joint_velocities[1] = np.clip(joint_velocities[1], -self.linear_speed, self.linear_speed)

                rospy.loginfo(f"Hand control - Base: {joint_velocities[0]:.3f}, Shoulder: {joint_velocities[1]:.3f}")
                break

        msg = TwistStamped()
        msg.header.frame_id = "arm_tool0"
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = 1
        self.cmd_pub.publish(msg)

        # Show debug window
        cv2.imshow("Hand Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User quit")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = HandTracker()
        node.run()
    except rospy.ROSInterruptException:
        pass
