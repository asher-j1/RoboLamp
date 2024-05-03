# Smart Lamp Project 
## Overview

This project is a smart lamp that tracks the user's gaze and adjusts its lighting direction accordingly. It uses a combination of cameras, servos, and algorithms to compute the gaze point and adjust the lamp's arms to match the user's focus area.

## GitHub Repository

The repository contains all the code and design files necessary to reconstruct the smart lamp robot.

## Hardware Requirements

- Raspberry Pi 
- OAK-D Camera 
- Three MG 996 Servos
- Wires and connectors
- Power supply
- Frame and mount for the lamp
- Adafruit Servocontroller
- Adafruit Neopixel for the light source

## Software Requirements

- ROS (Robot Operating System)
- Python 3.11
- Packages listed in requirements.txt

## Assembly Instructions

1. **Frame Assembly:**  
   Assemble the lamp frame and mount using the provided STEP files in the `design_files/` directory. Print the parts separately using gcode generated from the STL file.
   
2. **Hardware Setup:**
   - Mount the servos on the lamp arms according to the design files.
   - Mount the OAK-D Camera to the lamp head or frame.
   - Connect the servos to the Raspberry Pi as per the wiring diagram in the repository.
   - Connect the power supply to the Raspberry Pi and servos.

3. **Raspberry Pi Code Setup:**
   - Install the packages listed in the requirements.txt file
   - Install the needed Neural networks from the file networks.txt, 
        omz_downloader -list networks.txt

4. **ROS Package Setup:**
   - Install ROS and required dependencies.
   - Clone the `src/` folder into your ROS workspace.
   - Install the Python dependencies following by [ROS2 Documentation] (https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

## Running the Code

1. **Raspberry Pi Initialization:**
   - Power on the Raspberry Pi

2. **ROS Package:**
   - Source ROS and compile the project with colcon
   - Source the package's install/setup.bash file
   - Start ROS with the appropriate launch file in the `RoboLamp/launch/` folder.

3. **Testing:**
   - Test the gaze tracking by moving your gaze across the workspace to ensure the lamp adjusts correctly.

## Troubleshooting
  
- **Gaze Tracking:**  
  If the gaze tracking is not accurate, ensure proper lighting and camera calibration.

- **Multiple Users:**  
  The system may not work well in multi-user environments; it's best suited for single-user scenarios.

## Notes and Learnings

- **Gaze Tracking:**  
  The project uses several neural networks for gaze tracking, which requires accurate calibration and lighting conditions for best performance.

- **Servo Noise Reduction:**  
  The servo movement algorithm gradually adjusts angles to reduce noise.

This project demonstrates the capability of a smart lamp to adjust its lighting based on the user's gaze. While it requires careful calibration, it provides a unique and interactive experience. For further details, consult the provided code and documentation in the repository.
