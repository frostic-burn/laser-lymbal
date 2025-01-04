import cv2
import serial
import time
from cvzone.HandTrackingModule import HandDetector

# Initialize hand detector
detector = HandDetector(detectionCon=0.7, maxHands=1)

# Setup serial communication
transmitter = serial.Serial('COM14', 115200)
time.sleep(2)

def send_servo_command(pan_or_tilt, angle):
    """
    Sends a command to the servo motor to adjust its position.
    """
    servo_num = 1 if pan_or_tilt == 'pan' else 2
    angle = max(0, min(angle, 180))  # Ensure angle is within valid range
    coordinates = f"{servo_num},{angle}\r"
    transmitter.write(coordinates.encode())
    print(f"Sent command: {pan_or_tilt} to {angle} degrees")

def track_hand():
    """
    Tracks the hand and adjusts the servo angles based on its position.
    """
    pan_angle = 90
    tilt_angle = 90
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Camera could not be opened.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        hands, img = detector.findHands(frame, flipType=False)
        hand_center = None

        if hands:
            lmList = hands[0]['lmList']
            x, y = lmList[9][:2]  # Use the index finger base as reference
            hand_center = (x, y)
            cv2.circle(img, hand_center, 10, (255, 0, 0), -1)
            print(f"Hand Center: X={hand_center[0]}, Y={hand_center[1]}")

            # Adjust servo angles based on hand position
            delta_x = hand_center[0] - frame.shape[1] // 2
            delta_y = hand_center[1] - frame.shape[0] // 2

            if abs(delta_x) > 10:  # Horizontal movement
                pan_angle -= 1 if delta_x > 0 else -1
                pan_angle = max(0, min(pan_angle, 180))  # Clamp angle
                send_servo_command('pan', pan_angle)

            if abs(delta_y) > 10:  # Vertical movement
                tilt_angle += 1 if delta_y > 0 else -1
                tilt_angle = max(0, min(tilt_angle, 180))  # Clamp angle
                send_servo_command('tilt', tilt_angle)

        else:
            print("No hand detected.")

        cv2.imshow('Hand Tracker', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

track_hand()
