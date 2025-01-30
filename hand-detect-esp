import cv2
import serial
import time
from cvzone.HandTrackingModule import HandDetector

# Initialize hand detector
detector = HandDetector(detectionCon=0.7, maxHands=1)

# Setup serial communication
transmitter = serial.Serial('COM14', 115200)
time.sleep(2)

def map_range(x, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def send_servo_command(pan_or_tilt, angle):
    """Sends a command to the servo motor to adjust its position."""
    servo_num = 1 if pan_or_tilt == 'pan' else 2
    angle = max(0, min(angle, 180))  # Ensure angle is within valid range
    coordinates = f"{servo_num},{angle}\r"
    transmitter.write(coordinates.encode())
    print(f"Sent command: {pan_or_tilt} to {angle} degrees")

def track_hand():
    """Tracks the hand and adjusts the servo angles based on its position."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera could not be opened.")
        return

    # Get camera frame dimensions
    _, frame = cap.read()
    frame_height, frame_width = frame.shape[:2]

    # Initialize servo angles
    pan_angle = 90
    tilt_angle = 90

    # Exponential smoothing factor
    alpha = 0.2  # Lower values for smoother motion, adjust between 0.1 and 0.5

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        hands, img = detector.findHands(frame, flipType=False)
        
        if hands:
            lmList = hands[0]['lmList']
            x, y = lmList[9][:2]  # Use the index finger base as reference
            
            # Calculate target angles based on hand position
            target_pan = map_range(x, 
                                 frame_width * 0.1, frame_width * 0.9,  # Input range
                                 160, 20)  # Output range (reversed for natural movement)
            target_tilt = map_range(y,
                                  frame_height * 0.1, frame_height * 0.9,  # Input range
                                  20, 160)  # Output range (reversed for natural movement)
            
            # Apply exponential smoothing
            pan_angle = alpha * target_pan + (1 - alpha) * pan_angle
            tilt_angle = alpha * target_tilt + (1 - alpha) * tilt_angle
            
            # Send commands only if the change exceeds 1 degree
            if abs(target_pan - pan_angle) > 1:
                send_servo_command('pan', int(pan_angle))
            if abs(target_tilt - tilt_angle) > 1:
                send_servo_command('tilt', int(tilt_angle))
            
            # Visualization
            cv2.circle(img, (int(x), int(y)), 10, (0, 255, 0), -1)
            cv2.putText(img, f"Pan: {int(pan_angle)} Tilt: {int(tilt_angle)}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Hand Tracker', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    track_hand()
