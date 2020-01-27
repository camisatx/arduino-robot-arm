[//]: # (image reference)
[robot_arm_and_controller]: ./images/robot-arm-kit-finished-2.jpg
[robot_arm_electronics]: ./images/complete-electronics.jpg
[shield_picture]: ./images/SainSmart-Robot-Shield-Arduino-MEGA2560-R3.jpg
[shield_schematic]: ./images/SainSmart-Robot-Shield-Arduino-MEGA2560-Schematic.png

# Arduino Controlled 5 DoF Robot Arm

This repository contains the code needed to remotely control the 5 DoF robot arm that I built [here](https://joshschertz.com/2017/07/15/Robot-Arm-Part-1-Arm-Build/). If you want to duplicate the same development environment, I recommend you use [VSCode](https://code.visualstudio.com/) with [PlatformIO](https://platformio.org/). PlatformIO handles complex Arduiono projects must better than native Arduino IDE, relevant for this project due to multiple Arduinos being used.

![Robot arm and controller][robot_arm_and_controller]

## Hardware Components

The system comprises two arduino controlled components: the robot arm and the controller. The robot arm uses an Arduino Mega2560 and servo shield, where the controller uses a Arduino Uno and shield.

The controller uses two joysticks for collecting human inputs. A 16x2 LCD shows relevant information, including joystick inputs, connection status, and arm status. A NRF24 2.4 Ghz radio transceiver handles communication functions with the robot base.

The robot arm uses 6 servos to control arm movement. Arm joints include the shoulder rotation, shoulder elevation, elbow elevation, wrist elevation, wrist rotation, and end-effector open/close movement. A NRF24 2.4 Ghz radio transceiver performs communiction functions with the controller.

Do note that the robot arm servo shield has a silk-screen error, where the servo numbers are flipped: the pins marked 8, 9, and 10 are actually pins 11, 12, and 13.

![Robot arm shield][shield_picture]

View the actual servo shield schematic to understand what any unmarked pin is used for. The actual documentation is pretty sparse!

![Robot arm shield schematic][shield_schematic]

Also, make sure you provide dedicated power to the arm servo shield. My specific shield requires 12 volts, but double check what your board needs. 12 volts will instantly kill any Arduino!

![Robot arm electronics][robot_arm_electronics]

## Process

### Receive input

The controller detects changes in the joystick positions. Simple noise processing ensures only actual joystick movements are processed, and not potentiometer noise or joystick misalignment. The analog signals are converted into a byte, and packaged into an array for transmission.

### Transmit data

Utilizing the RadioHead library and UDP protocol approach, the control data is transmitted to the base. The controller only waits 1 millisecond before sending the next transmission. It is more important to stream current commands than to receive individual command confirmations. This provides a more fluid control environment due to shorter latency between human input and arm movement.

### Receive data

The robot arm continually seeks new data. When a packet is received and buffered, an error check ensures the value is between expected ranges, and converts the byte into a movement delta. The delta indicates how much a servo should move, with a higher number indicating more movement.

### Arm movement

For each movement received, an error check ensures the command keeps the servo within its operational range. Some servos (like the shoulder rotation) can rotate to their full range, whereas others have restricted movement (like the end-effector claw). If the movement is valid, the movement delta is added or subtracted from the current servo time. All movements are processed before the actual movement is given. The current code commands the servos using microseconds instead of angles, giving servos higher definition movements.

### Arm feedback

After processing arm movements, the arm transmits its status back to the controller. This command allows the controller to know whether it has a live connection with the arm. Currently, this is only used to inform the operator and does not impact system operation.