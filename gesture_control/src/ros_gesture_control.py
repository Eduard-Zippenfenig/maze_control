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
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.max_speed = rospy.get_param('~max_speed', 0.05)  # 5 cm/s
        self.deadzone = rospy.get_param('~deadzone', 0.05) # area where 0 is detected

        # ROS setup
        self.bridge = CvBridge() # for converting the image into a frame
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback, queue_size=1)
        self.twist_pub = rospy.Publisher('/hand_velocity', TwistStamped, queue_size=1)

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils

        rospy.loginfo("Hand Tracker node started - publishing TwistStamped velocities")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        vx, vy = 0.0, 0.0

        # Create the TwistStamped from the xy-frame
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

                vx = dx * (self.max_speed / 0.5)
                vy = -dy * (self.max_speed / 0.5)

                rospy.loginfo(f"Velocity (x={vx:.3f}, y={vy:.3f})")
                break

        # Publish TwistStamped
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.twist_pub.publish(twist_msg)

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
