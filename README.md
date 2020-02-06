[//]: # (image reference)
[controller_complete]: ./images/controller-complete.jpg
[controller_lcd_active]: ./images/controller-lcd-active.jpg
[controller_lcd_failed]: ./images/controller-lcd-failed.jpg
[pca9685]: ./images/pca9685.jpg
[robot_arm_and_controller]: ./images/robot-arm-kit-finished-2.jpg
[robot_arm_electronics]: ./images/complete-electronics.jpg
[shield_picture]: ./images/SainSmart-Robot-Shield-Arduino-MEGA2560-R3.jpg
[shield_schematic]: ./images/SainSmart-Robot-Shield-Arduino-MEGA2560-Schematic.png

# Arduino Controlled 5 DoF Robot Arm

This repository contains the code needed to remotely control the 5 DoF robot arm that I built [here](https://joshschertz.com/2017/07/15/Robot-Arm-Part-1-Arm-Build/). You can read my post about the project [here](https://joshschertz.com/2020/02/05/Robot-Arm-Part-6-Advanced-Code/). If you want to duplicate the same development environment, I recommend you use [VSCode](https://code.visualstudio.com/) with [PlatformIO](https://platformio.org/). PlatformIO handles complex Arduino projects better than native Arduino IDE. This is important for this project because it is using two Arduinos.

![Robot arm and controller][robot_arm_and_controller]

## Hardware Components

The system comprises two arduino controlled components: the robot arm and the controller. The robot arm uses an Arduino Mega2560 and PCA9685 board, where the controller uses a Arduino Uno and shield.

The controller uses two joysticks for collecting human inputs. A 16x2 LCD shows relevant information, including joystick inputs, connection status, and arm status. A NRF24 2.4 Ghz radio transceiver handles communication functions with the robot base.

The robot arm uses 6 servos to control arm movement. Arm joints include the shoulder rotation, shoulder elevation, elbow elevation, wrist elevation, wrist rotation, and end-effector open/close movement. A nNRF24L01 2.4 Ghz radio transceiver performs communication functions with the controller.

When trying to use the Sainsmart exclusive servo shield, the servos became very jittery. The cause of the issue was related to inconsistent PWM signals being send to the servos due to the nRF24L01 interrupting the system. If I wanted to use the nRF24L01, I needed to find another solution. The PCA9685 I2C PWM driver provided that, allowing its dedicated controller to exclusively send PWM signals to the servos. So I replaced the SainSmart servo shield for the PCA9685.

![PCA 9685 servo board][pca9685]

The code in this repository assumes you are also using an I2C servo controller, hence the use of the [Adafruit PCA9685 PWM Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) included in the [platformio.ini](https://github.com/camisatx/arduino-robot-arm/blob/master/platformio.ini#L22). You will also need to supply a 6 volt power input to the PCA9685 with at least 2 amps of current.

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

## Controller LCD

![Controller complete][controller_complete]

The following image shows the LCD status during normal operation. The top row shows the direction of the joystick signals, and the state of the connection between the controller and robot arm. The bottom row shows the servo angles and the status of the claw (open or close).

![Controller LCD active][controller_lcd_active]

The following image shows the LCD status with a failed connection between the controller and robot arm. The top row will still show the joystick signals, however, the connection status will show an `X`. The bottom row will show `NA` indicating no data is available.

![Controller LCD failed][controller_lcd_failed]
