#!/usr/bin/env python3
import rospy
import cv2
import mediapipe as mp
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class SimpleHandTracker:
    def __init__(self):
        rospy.init_node('simple_hand_tracker')

        # Parameters
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.linear_speed = rospy.get_param('~linear_speed', 0.05)
        self.control_zone = rospy.get_param('~control_zone', 0.3)
        self.deadzone = rospy.get_param('~deadzone', 0.05)

        # ROS I/O
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('/ur5e/twist_controller/command', Twist, queue_size=1)

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils

        rospy.loginfo("Simple Hand Tracker node started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        vel_x, vel_y = 0.0, 0.0
        h, w = frame.shape[:2]

        if results.multi_hand_landmarks:
            for landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, landmarks, self.mp_hands.HAND_CONNECTIONS)

                wrist = landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                mid = landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

                cx = (wrist.x + mid.x) / 2
                cy = (wrist.y + mid.y) / 2

                dx = cx - 0.5
                dy = cy - 0.5

                if abs(dx) < self.deadzone:
                    dx = 0
                if abs(dy) < self.deadzone:
                    dy = 0

                vel_x = -(dy / self.control_zone) * self.linear_speed
                vel_y = (dx / self.control_zone) * self.linear_speed

                vel_x = np.clip(vel_x, -self.linear_speed, self.linear_speed)
                vel_y = np.clip(vel_y, -self.linear_speed, self.linear_speed)

                break

        # Publish Twist
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        self.cmd_pub.publish(twist)

        # Show debug window
        cv2.imshow("Hand Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User quit")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = SimpleHandTracker()
        node.run()
    except rospy.ROSInterruptException:
        pass
