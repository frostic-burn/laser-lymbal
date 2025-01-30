import cv2
import serial
import time
import os
import numpy as np
import face_recognition
from cvzone.HandTrackingModule import HandDetector

# Initialize detectors and serial communication
detector = HandDetector(detectionCon=0.7, maxHands=1)  # Removed flipType from here
transmitter = serial.Serial('COM14', 115200)
time.sleep(2)

def map_range(x, in_min, in_max, out_min, out_max):
    """
    Maps a value from one range to another.
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def send_servo_command(pan_or_tilt, angle):
    """
    Sends a command to the servo motor to adjust its position.
    """
    servo_num = 1 if pan_or_tilt == 'pan' else 2
    angle = max(0, min(angle, 180))  # Ensure angle is within valid range
    coordinates = f"{servo_num},{angle}\r"
    transmitter.write(coordinates.encode())
    print(f"Sent command: {pan_or_tilt} to {angle} degrees")

def load_known_faces(directory):
    """
    Load known faces from a directory
    """
    known_face_encodings = []
    known_face_names = []
    
    for filename in os.listdir(directory):
        if filename.lower().endswith('.jpg'):
            image_path = os.path.join(directory, filename)
            try:
                image = face_recognition.load_image_file(image_path)
                encodings = face_recognition.face_encodings(image)
                
                if encodings:
                    known_face_encodings.append(encodings[0])
                    known_face_names.append("Known Person")
                    print(f"Loaded face from {filename}")
                else:
                    print(f"No face detected in {filename}")
            
            except Exception as e:
                print(f"Error processing {filename}: {e}")
    
    return known_face_encodings, known_face_names

def track_combined():
    """
    Combined tracking system that maintains fixed position for known faces
    and tracks hands only for unknown faces or no faces
    """
    # Load known faces
    known_face_encodings, known_face_names = load_known_faces(r"C:\Users\sukhi\OneDrive\Documents\face")
    
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
    
    # Send initial center position
    send_servo_command('pan', pan_angle)
    send_servo_command('tilt', tilt_angle)
    
    # Movement smoothing parameters
    smoothing_factor = 0.3
    
    # Previous tracking mode for state changes
    prev_tracking_mode = "None"

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Face detection
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
        
        known_face_detected = False
        tracking_mode = "None"

        # First priority: Check for known faces
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            name = "Unknown"

            if True in matches:
                name = "Known Person"
                known_face_detected = True
                tracking_mode = "Known Face"
                
                # Draw rectangle and name
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                cv2.putText(frame, name, (left, bottom + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # If we just switched to known face mode, center the servos
                if prev_tracking_mode != "Known Face":
                    pan_angle = 90
                    tilt_angle = 90
                    send_servo_command('pan', pan_angle)
                    send_servo_command('tilt', tilt_angle)
                
                break  # Stop checking other faces if we found a known face

        # Second priority: If no known face is detected, try hand tracking
        if not known_face_detected:
            # Changed flipType to True here
            hands, frame = detector.findHands(frame, flipType=True)
            
            if hands:
                tracking_mode = "Hand"
                lmList = hands[0]['lmList']
                x, y = lmList[9][:2]  # Use the index finger base as reference
                
                # Calculate target angles based on hand position
                target_pan = map_range(x, 
                                     frame_width * 0.1, frame_width * 0.9,
                                     160, 20)
                target_tilt = map_range(y,
                                      frame_height * 0.1, frame_height * 0.9,
                                      20, 160)
                
                # Smooth the movement
                pan_angle = pan_angle + (target_pan - pan_angle) * smoothing_factor
                tilt_angle = tilt_angle + (target_tilt - tilt_angle) * smoothing_factor
                
                # Send commands only if the change is significant
                if abs(target_pan - pan_angle) > 0.5:
                    send_servo_command('pan', int(pan_angle))
                if abs(target_tilt - tilt_angle) > 0.5:
                    send_servo_command('tilt', int(tilt_angle))
                
                # Visualization for hand tracking
                cv2.circle(frame, (int(x), int(y)), 10, (0, 0, 255), -1)

        # Display tracking mode and angles
        cv2.putText(frame, f"Mode: {tracking_mode}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Pan: {int(pan_angle)} Tilt: {int(tilt_angle)}", 
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Update previous tracking mode
        prev_tracking_mode = tracking_mode

        cv2.imshow('Combined Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    track_combined()
