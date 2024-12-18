# laser-lymbal

chatgpt description 
The **Laser-Lymbal** is a versatile and innovative device housed in a secure, PIN-locked enclosure, ensuring safety and controlled access. Upon unlocking, the system's laser gimbal mechanism either deploys automatically using motors and gears or can be manually lifted, offering flexibility in operation. At its core, an ESP32-CAM mounted on the gimbal enables advanced facial recognition and person tracking, seamlessly directing the laser's movements for precise targeting. Beyond functionality, the device also features a captivating aesthetic mode, allowing the laser to create mesmerizing long-exposure light painting effects, blending technology with art.

info about the project
i want to make a project which itself has several functions listed as below
a) be enclosed in a box which shall be unlocked when the correct pin is fed into it
b) as the box opens the laser gymbal thing either rolls up using motors and gears or it is lifted manually
c) a esp32 cam mounted laser gymbal, works with facial recognition and can track a person with the laser
d) for aesthetic purpose the laser can also function as a long exposure light painitng model

materials used 
1)arduino for unlock purpose and servo gymbal part
2)esp32 cam module for camera and detection functionm
3) CVZone library for hand detection in esp 32 combined with Laser Detection on Screen (Red Pixel Tracking) for improved accuaracy
4) 4x4 keypad for password enter

Here's how we can divide the responsibilities between the Arduino and the ESP32:

Arduino Responsibilities
PIN Verification and System Activation:

Read input from the 4x4 Keypad.
Unlock the solenoid lock if the correct PIN is entered.
Activate the system by turning on LEDs and allowing other operations.
Mode Management:

Manage mode switching between:
Laser Painting Mode (predefined servo patterns).
Hand Detection Mode (servo positions controlled by ESP32).
Indicate the active mode with LEDs.
Servo Control:

In Laser Painting Mode:
Move the servo motors in pre-programmed patterns.
In Hand Detection Mode:
Adjust servo positions based on coordinates received from the ESP32.
Communication with ESP32:

Receive hand position data via Serial or I2C in Hand Detection Mode.
ESP32 Responsibilities
Hand Detection:

Use the ESP32-CAM to process the video feed and detect hands using the CVZone or Mediapipe library.
Calculate the coordinates (x, y) of the detected hand relative to the screen.
Data Transmission:

Send the calculated hand coordinates to the Arduino via Serial or I2C.
Feedback on Laser Position (Optional):

Detect the red laser dot in the video feed and provide feedback to ensure it is aligned with the target.
Communication Flow
The Arduino activates the system when the correct PIN is entered.
The ESP32 continuously tracks the hand and sends its position to the Arduino in Hand Detection Mode.
The Arduino processes these coordinates to adjust the servos accordingly.
