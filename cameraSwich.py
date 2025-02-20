import cv2
import serial
import time
from cvzone.HandTrackingModule import HandDetector

# Initialize hand detector
detector = HandDetector(detectionCon=0.7, maxHands=1)

# Setup serial communication
try:
    transmitter = serial.Serial('COM14', 115200)
    time.sleep(2)
    serial_connected = True
    print("Serial connection established successfully")
except:
    serial_connected = False
    print("Failed to connect to serial port. Running in simulation mode.")

def map_range(x, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def send_servo_command(pan_or_tilt, angle):
    """Sends a command to the servo motor to adjust its position."""
    if not serial_connected:
        return False
    
    servo_num = 1 if pan_or_tilt == 'pan' else 2
    angle = max(0, min(angle, 180))  # Ensure angle is within valid range
    coordinates = f"{servo_num},{angle}\r"
    transmitter.write(coordinates.encode())
    print(f"Sent command: {pan_or_tilt} to {angle} degrees")
    return True

def get_available_cameras():
    """Get list of available camera indices."""
    indices = []
    max_to_check = 10  # Check first 10 indices
    
    for i in range(max_to_check):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)  # Using DirectShow API
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                indices.append(i)
            cap.release()
    
    return indices

def track_hand():
    """Tracks the hand and adjusts the servo angles based on its position."""
    # Get available cameras
    available_cameras = get_available_cameras()
    if not available_cameras:
        print("No cameras detected. Please check your camera connections.")
        return
    
    print(f"Available camera indices: {available_cameras}")
    
    # Start with first available camera
    camera_idx = 0
    current_camera = available_cameras[camera_idx]
    
    print(f"Using camera index: {current_camera}")
    cap = cv2.VideoCapture(current_camera, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print(f"Failed to open camera at index {current_camera}.")
        return
    
    # Get camera frame dimensions
    _, frame = cap.read()
    frame_height, frame_width = frame.shape[:2]
    print(f"Camera initialized: {frame_width}x{frame_height}")
    
    # Initialize servo angles
    pan_angle = 90
    tilt_angle = 90
    
    # Exponential smoothing factor
    alpha = 0.2  # Lower values for smoother motion, adjust between 0.1 and 0.5
    
    mode = "SIMULATION MODE" if not serial_connected else "CONNECTED TO SERVOS"
    color = (0, 0, 255) if not serial_connected else (0, 255, 0)
    
    print("\nPress 'c' to switch between cameras")
    print("Press 'q' to quit")
    
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
            
            # Send commands only if the change exceeds 1 degree and serial is connected
            if serial_connected:
                if abs(target_pan - pan_angle) > 1:
                    send_servo_command('pan', int(pan_angle))
                if abs(target_tilt - tilt_angle) > 1:
                    send_servo_command('tilt', int(tilt_angle))
            
            # Visualization
            cv2.circle(img, (int(x), int(y)), 10, (0, 255, 0), -1)
            cv2.putText(img, f"Pan: {int(pan_angle)} Tilt: {int(tilt_angle)}", 
                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Add a visual indicator for servo positions
            indicator_size = 100
            indicator_x = frame_width - indicator_size - 20
            indicator_y = 20
            
            # Draw a frame for the indicator
            cv2.rectangle(img, (indicator_x, indicator_y), 
                         (indicator_x + indicator_size, indicator_y + indicator_size), 
                         (0, 0, 0), 2)
            
            # Draw pan-tilt position (cross-hair)
            pos_x = int(indicator_x + (indicator_size * (180 - pan_angle) / 180))
            pos_y = int(indicator_y + (indicator_size * tilt_angle / 180))
            cv2.line(img, (pos_x - 5, pos_y), (pos_x + 5, pos_y), (0, 0, 255), 2)
            cv2.line(img, (pos_x, pos_y - 5), (pos_x, pos_y + 5), (0, 0, 255), 2)
        
        # Add camera info and mode indicator
        cv2.putText(img, f"Camera: {current_camera} | Press 'c' to switch", 
                  (10, frame_height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(img, mode, (10, frame_height - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        cv2.imshow('Hand Tracker', img)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c') and len(available_cameras) > 1:
            # Switch to next camera
            cap.release()
            camera_idx = (camera_idx + 1) % len(available_cameras)
            current_camera = available_cameras[camera_idx]
            print(f"Switching to camera index: {current_camera}")
            
            cap = cv2.VideoCapture(current_camera, cv2.CAP_DSHOW)
            if not cap.isOpened():
                print(f"Failed to open camera at index {current_camera}. Trying next...")
                continue
                
            # Get new camera dimensions
            _, frame = cap.read()
            if frame is not None:
                frame_height, frame_width = frame.shape[:2]
                print(f"New camera dimensions: {frame_width}x{frame_height}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    # Close serial connection if it was open
    if serial_connected:
        transmitter.close()
        print("Serial connection closed")

if __name__ == "__main__":
    track_hand()
