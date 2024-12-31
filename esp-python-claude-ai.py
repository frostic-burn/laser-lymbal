import cv2
import numpy as np
import serial
import time
from cvzone.HandTrackingModule import HandDetector
import atexit

class LaserTracker:
    def __init__(self):
        # Configuration
        self.CAMERA_INDEX = 0
        self.SERIAL_PORT = 'COM3'
        self.BAUD_RATE = 115200
        self.MIN_LASER_RADIUS = 3
        self.HAND_DETECTION_CONF = 0.7
        self.UPDATE_RATE = 0.05  # 20Hz
        self.MOVE_THRESHOLD = 10
        self.SMOOTHING_FACTOR = 0.1

        # Initialize states
        self.pan_angle = 90
        self.tilt_angle = 90
        self.last_command_time = 0
        self.transmitter = None
        self.cap = None
        self.detector = None

    def initialize(self):
        # Initialize serial connection
        try:
            self.transmitter = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for ESP32 to initialize
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            return False

        # Initialize camera
        self.cap = cv2.VideoCapture(self.CAMERA_INDEX)
        if not self.cap.isOpened():
            print("Failed to open camera")
            self.cleanup()
            return False

        # Initialize hand detector
        self.detector = HandDetector(detectionCon=self.HAND_DETECTION_CONF, maxHands=1)
        return True

    def send_servo_command(self, pan_or_tilt, angle):
        if time.time() - self.last_command_time < self.UPDATE_RATE:
            return

        try:
            servo_num = 1 if pan_or_tilt == 'pan' else 2
            angle = max(0, min(angle, 180))
            command = f"{servo_num},{angle}\n"
            self.transmitter.write(command.encode())
            self.last_command_time = time.time()
            print(f"Sent command: {pan_or_tilt} to {angle} degrees")
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")

    def detect_laser_point(self, frame):
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Define red color range in HSV
            mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([179, 255, 255]))
            mask = mask1 + mask2

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                center = (int(x), int(y))
                
                if radius > self.MIN_LASER_RADIUS:
                    cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
                    return center
            return None
        except Exception as e:
            print(f"Error in laser detection: {e}")
            return None

    def calculate_servo_adjustment(self, delta):
        if abs(delta) < self.MOVE_THRESHOLD:
            return 0
        return int(delta * self.SMOOTHING_FACTOR)

    def cleanup(self):
        if self.transmitter:
            self.transmitter.close()
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

    def run(self):
        if not self.initialize():
            return

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to capture frame")
                    break

                # Detect hand
                hands, img = self.detector.findHands(frame, flipType=False)
                hand_center = None
                if hands:
                    lmList = hands[0]['lmList']
                    x, y = lmList[9][:2]
                    hand_center = (x, y)
                    cv2.circle(img, hand_center, 10, (255, 0, 0), -1)

                # Detect laser
                laser_position = self.detect_laser_point(img)
                if laser_position:
                    cv2.circle(img, laser_position, 5, (0, 255, 0), -1)

                # Calculate movements
                if hand_center and laser_position:
                    delta_x = hand_center[0] - laser_position[0]
                    delta_y = hand_center[1] - laser_position[1]
                    
                    pan_adjustment = self.calculate_servo_adjustment(delta_x)
                    tilt_adjustment = self.calculate_servo_adjustment(delta_y)
                    
                    if pan_adjustment:
                        self.pan_angle = max(0, min(180, self.pan_angle - pan_adjustment))
                        self.send_servo_command('pan', self.pan_angle)
                    
                    if tilt_adjustment:
                        self.tilt_angle = max(0, min(180, self.tilt_angle + tilt_adjustment))
                        self.send_servo_command('tilt', self.tilt_angle)

                cv2.imshow('Laser and Hand Tracker', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            self.cleanup()

if __name__ == "__main__":
    tracker = LaserTracker()
    atexit.register(tracker.cleanup)
    tracker.run()
