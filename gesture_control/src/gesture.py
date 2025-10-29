#!/usr/bin/env python3
"""
Continuous Hand Tracking Node with MediaPipe
Tracks hand position in camera frame and outputs continuous velocity commands
Based on hand movement rather than discrete gestures
"""

import rospy
import cv2
import mediapipe as mp
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge, CvBridgeError
import threading


class HandTrackingNode:
    """
    ROS node that processes camera images and tracks hand position
    Publishes continuous velocity commands based on hand movement
    """

    def __init__(self):
        rospy.init_node('hand_tracking_node')

        self.bridge = CvBridge()

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            model_complexity=1
        )

        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.use_compressed = rospy.get_param('~use_compressed', False)
        self.linear_speed = rospy.get_param('~linear_speed', 0.05)
        self.control_zone_size = rospy.get_param('~control_zone_size', 0.3)
        self.deadzone = rospy.get_param('~deadzone', 0.05)

        self.hand_detected = False
        self.hand_center = None
        self.reference_position = None
        self.calibrated = False
        self.is_stopped = False

        self.current_frame = None
        self.frame_lock = threading.Lock()

        self.vel_pub = rospy.Publisher('/ur5e/twist_controller/command', Twist, queue_size=1)
        self.hand_pos_pub = rospy.Publisher('/hand_tracking/position', Point, queue_size=1)
        self.hand_detected_pub = rospy.Publisher('/hand_tracking/detected', Bool, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/hand_tracking/debug_image', Image, queue_size=1)

        if self.use_compressed:
            self.image_sub = rospy.Subscriber(
                self.camera_topic,
                CompressedImage,
                self.compressed_image_callback,
                queue_size=1,
                buff_size=2**24
            )
        else:
            self.image_sub = rospy.Subscriber(
                self.camera_topic,
                Image,
                self.image_callback,
                queue_size=1,
                buff_size=2**24
            )

        self.calibrate_sub = rospy.Subscriber(
            '/hand_tracking/calibrate',
            Bool,
            self.calibrate_callback
        )

        rospy.loginfo("Hand tracking node initialized")
        rospy.loginfo(f"Subscribing to: {self.camera_topic}")

    def compressed_image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self.frame_lock:
                self.current_frame = frame
        except Exception as e:
            rospy.logerr(f"Error decoding compressed image: {e}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.current_frame = frame
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

    def calibrate_callback(self, msg):
        if msg.data and self.hand_center is not None:
            self.reference_position = self.hand_center.copy()
            self.calibrated = True
            rospy.loginfo(f"Calibrated! Reference position: {self.reference_position}")
        elif msg.data:
            rospy.logwarn("Cannot calibrate - no hand detected!")

    def get_hand_center(self, hand_landmarks, image_shape):
        h, w = image_shape[:2]
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        middle_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

        center_x = (wrist.x + middle_mcp.x) / 2
        center_y = (wrist.y + middle_mcp.y) / 2

        px = int(center_x * w)
        py = int(center_y * h)

        return np.array([center_x, center_y]), np.array([px, py])

    def detect_stop_gesture(self, hand_landmarks):
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        fingertips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP
        ]

        extended_count = sum(
            1 for fingertip_idx in fingertips
            if hand_landmarks.landmark[fingertip_idx].y < wrist.y - 0.1
        )

        return extended_count >= 3

    def calculate_velocity(self, hand_center_norm):
        if not self.calibrated or self.reference_position is None:
            return 0.0, 0.0

        dx = hand_center_norm[0] - self.reference_position[0]
        dy = hand_center_norm[1] - self.reference_position[1]

        if abs(dx) < self.deadzone:
            dx = 0
        if abs(dy) < self.deadzone:
            dy = 0

        vel_x = -(dy / self.control_zone_size) * self.linear_speed
        vel_y = (dx / self.control_zone_size) * self.linear_speed

        vel_x = np.clip(vel_x, -self.linear_speed, self.linear_speed)
        vel_y = np.clip(vel_y, -self.linear_speed, self.linear_speed)

        return vel_x, vel_y

    def process_frame(self, frame):
        if frame is None:
            return

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        self.hand_detected = False
        vel_x, vel_y = 0.0, 0.0

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                hand_center_norm, hand_center_px = self.get_hand_center(hand_landmarks, frame.shape)
                self.hand_center = hand_center_norm
                self.hand_detected = True

                if self.detect_stop_gesture(hand_landmarks):
                    self.is_stopped = not self.is_stopped
                    rospy.loginfo(f"Stop gesture! Stopped: {self.is_stopped}")
                    rospy.sleep(0.5)

                if self.calibrated and not self.is_stopped:
                    vel_x, vel_y = self.calculate_velocity(hand_center_norm)

                point_msg = Point(x=hand_center_norm[0], y=hand_center_norm[1], z=0)
                self.hand_pos_pub.publish(point_msg)

                break

        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        self.vel_pub.publish(twist)

        self.hand_detected_pub.publish(Bool(data=self.hand_detected))

        try:
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except CvBridgeError:
            pass

        return frame

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            with self.frame_lock:
                frame = self.current_frame
            if frame is not None:
                processed = self.process_frame(frame)
                cv2.imshow('Hand Tracking', processed)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.signal_shutdown("User quit")
                    break
            rate.sleep()
        cv2.destroyAllWindows()


class HandTrackingCameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher')

        self.camera_id = rospy.get_param('~camera_id', 0)
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)
        self.compressed = rospy.get_param('~compressed', False)

        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.bridge = CvBridge()

        if self.compressed:
            self.image_pub = rospy.Publisher('/camera/image_raw/compressed', CompressedImage, queue_size=1)
        else:
            self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)

    def run(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                continue
            try:
                if self.compressed:
                    msg = CompressedImage()
                    msg.header.stamp = rospy.Time.now()
                    msg.format = "jpeg"
                    msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
                    self.image_pub.publish(msg)
                else:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = rospy.Time.now()
                    self.image_pub.publish(msg)
            except CvBridgeError:
                pass
            rate.sleep()
        self.cap.release()


def main():
    import sys

    if len(sys.argv) < 2:
        print("Usage:")
        print("  Camera publisher: rosrun hri_experiment hand_tracking.py camera")
        print("  Hand tracker:     rosrun hri_experiment hand_tracking.py tracker")
        print("  Standalone:       rosrun hri_experiment hand_tracking.py standalone")
        sys.exit(1)

    mode = sys.argv[1].lower()

    try:
        if mode == 'camera':
            node = HandTrackingCameraPublisher()
            node.run()
        elif mode == 'tracker':
            node = HandTrackingNode()
            node.run()
        elif mode == 'standalone':
            rospy.init_node('hand_tracking_standalone')

            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            node = HandTrackingNode()
            node.image_sub.unregister()

            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                ret, frame = cap.read()
                if ret:
                    with node.frame_lock:
                        node.current_frame = frame
                    processed = node.process_frame(frame)
                    cv2.imshow('Hand Tracking', processed)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                rate.sleep()

            cap.release()
            cv2.destroyAllWindows()

        else:
            print(f"Unknown mode: {mode}")
            sys.exit(1)

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
