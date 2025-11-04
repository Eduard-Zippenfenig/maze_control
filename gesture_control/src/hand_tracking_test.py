import cv2
import mediapipe as mp
import numpy as np
import time

class HandVelocityTester:
    def __init__(self, max_speed=0.05, deadzone=0.05):
        self.max_speed = max_speed
        self.deadzone = deadzone
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)

        # Simulation window setup
        self.sim_size = 500
        self.turtle_pos = np.array([self.sim_size // 2, self.sim_size // 2], dtype=float)
        self.last_time = time.time()

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb)

            vx, vy = 0.0, 0.0

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

                    cv2.putText(frame, f"vx={vx:.3f} m/s, vy={vy:.3f} m/s", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    break

            # --- Turtle simulation ---
            sim_frame = np.zeros((self.sim_size, self.sim_size, 3), dtype=np.uint8)

            # Compute elapsed time (delta t)
            now = time.time()
            dt = now - self.last_time
            self.last_time = now

            # Scale velocities to pixels/sec (arbitrary: 2000 px per meter)
            px_per_m = 2000
            self.turtle_pos[0] += vx * px_per_m * dt  # x direction
            self.turtle_pos[1] += vy * px_per_m * dt  # y direction

            # Boundaries
            self.turtle_pos[0] = np.clip(self.turtle_pos[0], 10, self.sim_size - 10)
            self.turtle_pos[1] = np.clip(self.turtle_pos[1], 10, self.sim_size - 10)

            # Draw turtle and center
            cv2.circle(sim_frame, (int(self.turtle_pos[0]), int(self.turtle_pos[1])), 10, (0, 255, 0), -1)
            cv2.circle(sim_frame, (self.sim_size // 2, self.sim_size // 2), 5, (0, 0, 255), -1)
            cv2.putText(sim_frame, "Turtle simulation", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            # Display both windows
            cv2.imshow("Hand Velocity Tracker", frame)
            cv2.imshow("Turtle Simulation", sim_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    tester = HandVelocityTester()
    tester.run()
