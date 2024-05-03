# Smart Lamp Project 
## Overview

This project is a smart lamp that tracks the user's gaze and adjusts its lighting direction accordingly. It uses a combination of cameras, servos, and algorithms to compute the gaze point and adjust the lamp's arms to match the user's focus area.

## GitHub Repository


The repository contains all the code and design files necessary to reconstruct the smart lamp robot.

### Repository Structure

- `microcontroller_code/`: Contains code to run on the microcontroller that controls the servos and other hardware components.
- `ros_package/`: Contains the ROS package for gaze tracking and robotic arm control.
- `design_files/`: Contains .STEP files for the physical design of the robot.

## Hardware Requirements

- Microcontroller 
- OAK-D Camera 
- Three MG 996 Servos
- Wires and connectors
- Power supply
- Frame and mount for the lamp

## Software Requirements

- ROS (Robot Operating System)
- Python 3.11
- Microcontroller IDE 
- OpenCV (for gaze tracking)
- Additional Python libraries (listed in `ros_package/requirements.txt`)

## Assembly Instructions

1. **Frame Assembly:**  
   Assemble the lamp frame and mount using the provided STEP files in the `design_files/` directory. Print the parts separately using gcode generated from the STL file.
   
2. **Hardware Setup:**
   - Mount the servos on the lamp arms according to the design files.
   - Mount the OAK-D Camera to the lamp head or frame.
   - Connect the servos to the microcontroller as per the wiring diagram in the repository.
   - Connect the power supply to the microcontroller and servos.

3. **Microcontroller Code Setup:**
   - xx.

4. **ROS Package Setup:**
   - Install ROS and required dependencies.
   - Clone the `ros_package/` into your ROS workspace.
   - Install the Python dependencies following by [ROS2 Documentation] (https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

## Running the Code

1. **Microcontroller Initialization:**
   - Power on the microcontroller and ensure the servos initialize correctly.

2. **ROS Package:**
   - Start ROS with the appropriate launch file in the `ros_package/launch/` folder.
   - Follow the prompts to calibrate the camera and servos.

3. **Testing and Calibration:**
   - Use the calibration tools provided in the ROS package to fine-tune the system.
   - Test the gaze tracking by moving your gaze across the workspace to ensure the lamp adjusts correctly.

## Troubleshooting

- **Servo Calibration:**  
  Ensure the servos are correctly calibrated by following the calibration process.
  
- **Gaze Tracking:**  
  If the gaze tracking is not accurate, ensure proper lighting and camera calibration.

- **Multiple Users:**  
  The system may not work well in multi-user environments; it's best suited for single-user scenarios.

## Notes and Learnings

- **Communication Protocols:**  
  The microcontroller code uses a serial communication protocol to interact with the ROS package. Detailed documentation on this protocol can be found in the repository.

- **Gaze Tracking:**  
  The project uses OpenCV for gaze tracking, which requires accurate calibration and lighting conditions for best performance.

- **Servo Noise Reduction:**  
  The servo movement algorithm gradually adjusts angles to reduce noise.

This project demonstrates the capability of a smart lamp to adjust its lighting based on the user's gaze. While it requires careful calibration, it provides a unique and interactive experience. For further details, consult the provided code and documentation in the repository.
