mport cv2
import serial
import time
import numpy as np
from cvzone.HandTrackingModule import HandDetector

# Initialize hand detector
detector = HandDetector(detectionCon=0.7, maxHands=1)

# Setup serial communication
try:
    transmitter = serial.Serial('COM14', 115200)
    time.sleep(2)  # Allow time for serial connection to establish
    print("Serial connection established on COM14")
except serial.SerialException:
    print("Error: Could not open serial port COM14")
    print("Please check your connection and port number")
    transmitter = None

def map_range(x, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def send_servo_command(pan_or_tilt, angle):
    """Sends a command to the servo motor to adjust its position."""
    if transmitter is None or not transmitter.is_open:
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

def create_button(img, text, position, size, is_selected=False):
    """Create a button on the image."""
    x, y = position
    w, h = size
    
    # Button colors
    if is_selected:
        bg_color = (0, 170, 0)  # Green for selected camera
        text_color = (255, 255, 255)  # White text
    else:
        bg_color = (70, 70, 70)  # Dark gray for unselected
        text_color = (255, 255, 255)  # White text
    
    # Draw button background
    cv2.rectangle(img, (x, y), (x + w, y + h), bg_color, -1)
    
    # Draw button border
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), 2)
    
    # Add text
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, 0.5, 2)[0]
    text_x = x + (w - text_size[0]) // 2
    text_y = y + (h + text_size[1]) // 2
    cv2.putText(img, text, (text_x, text_y), font, 0.5, text_color, 2)
    
    return (x, y, w, h)  # Return button coordinates for hit testing

def is_point_in_rect(point, rect):
    """Check if a point is inside a rectangle."""
    x, y = point
    rect_x, rect_y, rect_w, rect_h = rect
    return rect_x <= x <= rect_x + rect_w and rect_y <= y <= rect_y + rect_h

def track_hand():
    """Tracks the hand and adjusts the servo angles based on its position."""
    # Get available cameras
    available_cameras = get_available_cameras()
    if not available_cameras:
        print("No cameras detected. Please check your camera connections.")
        return
    
    print(f"Available camera indices: {available_cameras}")
    
    # Start with first available camera
    current_camera_idx = 0
    
    # Open the selected camera
    cap = cv2.VideoCapture(available_cameras[current_camera_idx], cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"Failed to open camera at index {available_cameras[current_camera_idx]}.")
        return
    
    # Get camera frame dimensions
    _, frame = cap.read()
    frame_height, frame_width = frame.shape[:2]
    print(f"Camera initialized: {frame_width}x{frame_height}")
    
    # Initialize servo angles
    pan_angle = 90
    tilt_angle = 90
    last_sent_pan = 90
    last_sent_tilt = 90
    
    # Exponential smoothing factor
    alpha = 0.2  # Lower values for smoother motion, adjust between 0.1 and 0.5
    
    # Control rate limiting
    last_send_time = time.time()
    min_send_interval = 0.05  # 50ms minimum between commands
    
    # Mouse events
    mouse_x, mouse_y = 0, 0
    clicked = False
    
    def mouse_callback(event, x, y, flags, param):
        nonlocal mouse_x, mouse_y, clicked
        mouse_x, mouse_y = x, y
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked = True
    
    # Create a named window and set mouse callback
    window_name = 'Hand Tracker'
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)
    
    # Store button coordinates for hit testing
    camera_buttons = []
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        # Make a copy of the frame for drawing UI elements
        img = frame.copy()
        
        # Detect hands
        hands, img = detector.findHands(img, flipType=False)
        
        current_time = time.time()
        send_allowed = current_time - last_send_time >= min_send_interval
        
        if hands and send_allowed:
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
            commands_sent = False
            if abs(pan_angle - last_sent_pan) > 1:
                if send_servo_command('pan', int(pan_angle)):
                    last_sent_pan = pan_angle
                    commands_sent = True
            
            if abs(tilt_angle - last_sent_tilt) > 1:
                if send_servo_command('tilt', int(tilt_angle)):
                    last_sent_tilt = tilt_angle
                    commands_sent = True
            
            if commands_sent:
                last_send_time = current_time
            
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
        
        # Create camera selection buttons
        button_y = 10
        button_width = 120
        button_height = 40
        button_spacing = 10
        camera_buttons = []
        
        for i, cam_idx in enumerate(available_cameras):
            button_x = button_spacing + i * (button_width + button_spacing)
            button_text = f"Camera {cam_idx}"
            is_selected = (cam_idx == available_cameras[current_camera_idx])
            
            button_rect = create_button(img, button_text, (button_x, button_y), 
                                      (button_width, button_height), is_selected)
            camera_buttons.append((button_rect, cam_idx))
        
        # Display connection status
        status_color = (0, 255, 0) if transmitter and transmitter.is_open else (0, 0, 255)
        status_text = "Connected to ESP" if transmitter and transmitter.is_open else "Disconnected"
        cv2.putText(img, status_text, (10, frame_height - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # Show current camera info
        cv2.putText(img, f"Using camera: {available_cameras[current_camera_idx]}", 
                    (10, frame_height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Handle button clicks
        if clicked:
            for button_rect, cam_idx in camera_buttons:
                if is_point_in_rect((mouse_x, mouse_y), button_rect):
                    if cam_idx != available_cameras[current_camera_idx]:
                        print(f"Switching to camera {cam_idx}")
                        cap.release()
                        current_camera_idx = available_cameras.index(cam_idx)
                        cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
                        
                        if not cap.isOpened():
                            print(f"Failed to open camera {cam_idx}")
                            # Try to reopen the previous camera
                            current_camera_idx = 0
                            cap = cv2.VideoCapture(available_cameras[current_camera_idx], cv2.CAP_DSHOW)
                        else:
                            # Get new camera dimensions
                            _, frame = cap.read()
                            if frame is not None:
                                frame_height, frame_width = frame.shape[:2]
                                print(f"New camera dimensions: {frame_width}x{frame_height}")
                    break
            clicked = False
        
        cv2.imshow(window_name, img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Clean up
    if transmitter and transmitter.is_open:
        transmitter.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    track_hand()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    track_hand()
