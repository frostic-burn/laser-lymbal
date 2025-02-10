import cv2
import serial
import time
import numpy as np
from cvzone.HandTrackingModule import HandDetector
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk

# Initialize hand detector
detector = HandDetector(detectionCon=0.7, maxHands=2)  # Changed to track both hands

# Setup serial communication
transmitter = serial.Serial('COM14', 115200)
time.sleep(2)

# Buffer for storing hand positions
BUFFER_SIZE = 100
x_points_right = deque(maxlen=BUFFER_SIZE)
y_points_right = deque(maxlen=BUFFER_SIZE)
x_points_left = deque(maxlen=BUFFER_SIZE)
y_points_left = deque(maxlen=BUFFER_SIZE)

def map_range(x, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def send_servo_command(pan_or_tilt, angle):
    """Sends a command to the servo motor to adjust its position."""
    servo_num = 1 if pan_or_tilt == 'pan' else 2
    angle = max(0, min(angle, 180))
    coordinates = f"{servo_num},{angle}\r"
    transmitter.write(coordinates.encode())
    print(f"Sent command: {pan_or_tilt} to {angle} degrees")

class HandTracker:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Camera could not be opened.")
        
        _, frame = self.cap.read()
        self.frame_height, self.frame_width = frame.shape[:2]
        
        # Initialize servo angles
        self.pan_angle = 90
        self.tilt_angle = 90
        self.alpha = 0.2
        
        # Setup plotting
        self.setup_plot()
        
    def setup_plot(self):
        self.root = tk.Tk()
        self.root.title("Hand Movement Plot")
        
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack()
        
        self.ax.set_xlim(0, self.frame_width)
        self.ax.set_ylim(0, self.frame_height)
        self.ax.invert_yaxis()  # Invert Y axis to match camera coordinates
        self.ax.set_title("Hand Movement Tracking")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        
        # Initialize plot lines
        self.line_right, = self.ax.plot([], [], 'r-', label='Right Hand', alpha=0.7)
        self.line_left, = self.ax.plot([], [], 'b-', label='Left Hand', alpha=0.7)
        self.ax.legend()
        
    def update_plot(self):
        # Update right hand line
        if x_points_right:
            self.line_right.set_data(list(x_points_right), list(y_points_right))
        
        # Update left hand line
        if x_points_left:
            self.line_left.set_data(list(x_points_left), list(y_points_left))
        
        self.canvas.draw()
        
    def track_hand(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to capture frame")
                break
                
            hands, img = detector.findHands(frame, flipType=False)
            
            if hands:
                for hand in hands:
                    lmList = hand['lmList']
                    x, y = lmList[9][:2]  # Use the index finger base as reference
                    
                    # Determine if it's left or right hand
                    if hand['type'] == 'Right':
                        x_points_right.append(x)
                        y_points_right.append(y)
                        color = (0, 0, 255)  # Red for right hand
                    else:
                        x_points_left.append(x)
                        y_points_left.append(y)
                        color = (255, 0, 0)  # Blue for left hand
                    
                    # Calculate target angles for the primary tracked hand (using right hand if available)
                    if hand['type'] == 'Right':
                        target_pan = map_range(x, 
                                             self.frame_width * 0.1, self.frame_width * 0.9,
                                             160, 20)
                        target_tilt = map_range(y,
                                              self.frame_height * 0.1, self.frame_height * 0.9,
                                              20, 160)
                        
                        # Apply exponential smoothing
                        self.pan_angle = self.alpha * target_pan + (1 - self.alpha) * self.pan_angle
                        self.tilt_angle = self.alpha * target_tilt + (1 - self.alpha) * self.tilt_angle
                        
                        # Send servo commands
                        if abs(target_pan - self.pan_angle) > 1:
                            send_servo_command('pan', int(self.pan_angle))
                        if abs(target_tilt - self.tilt_angle) > 1:
                            send_servo_command('tilt', int(self.tilt_angle))
                    
                    # Visualization on camera feed
                    cv2.circle(img, (int(x), int(y)), 10, color, -1)
                    cv2.putText(img, f"Pan: {int(self.pan_angle)} Tilt: {int(self.tilt_angle)}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow('Hand Tracker', img)
            self.update_plot()
            self.root.update()
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()
        self.root.destroy()

if __name__ == "__main__":
    tracker = HandTracker()
    tracker.track_hand()
