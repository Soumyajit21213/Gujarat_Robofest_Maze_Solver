# Gujarat Robofest Maze Solver

Welcome to the **Gujarat Robofest Maze Solver** project repository! This repository contains all the resources and files used in our maze-solving robot project for Gujarat Robofest 4.0. Below, you'll find instructions on how to navigate and utilize the files effectively.
We reached the Finals and solved the final maze in 1 min 40 seconds .
Our rank was 5th . 
---

## Repository Structure

### 1. [ARDUINO CODES/](./ARDUINO%20CODES)

This folder contains the Arduino source code used to program the maze-solving robot.

- **Requirement:**
  - To open the `.ino` files , you need [Arduino IDE](https://www.arduino.cc/en/software) (version 2.3.2 or later recommended).
  - Make sure your ESP32-S3 microcontroller is getting recognized by your system .
 
- **Folder Structure:**
- ARDUINO CODES/
  - Tremaux/
    - tremaux_8/
      - tremaux_8.ino: Implements the 8th version of the Tremaux maze-solving algorithm.
      - helpers_2.c: Helper functions used in Tremaux 8.
    - tremaux_6/
      - tremaux_6.ino: Implements the 6th version of the Tremaux maze-solving algorithm.
      - helpers_2.c: Helper functions used in Tremaux 6.
    - tremaux_5_wifi/
      - tremaux_5_wifi.ino: Adds Wi-Fi-based features to Tremaux 5.
      - helpers_2.c: Helper functions used in Tremaux 5 Wi-Fi.
    - tremaux_7/
      - tremaux_7.ino: Implements the 7th version of the Tremaux maze-solving algorithm.
      - helpers_2.c: Helper functions used in Tremaux 7.
    - tremaux_line/
      - tremaux_line.ino: Focuses on line-following adaptations of the Tremaux algorithm.
      - helpers_2.c: Helper functions for line-following logic.
    - tremaux_5/
      - tremaux_5.ino: Implements the 5th version of the Tremaux algorithm.
      - helpers_2.c: Helper functions used in Tremaux 5.
    - tremaux_4/
      - tremaux_4.ino: Implements the 4th version of the Tremaux algorithm.
      - helpers_2.c: Helper functions used in Tremaux 4.
    - tremaux_3/
      - tremaux_3.ino: Implements the 3rd version of the Tremaux algorithm.
      - helpers_2.c: Helper functions used in Tremaux 3.
  - Other Algorithms/
    - Motor_Encoder_PID_Synchronus/
      - Motor_Encoder_PID_Synchronus.ino: Synchronizes motor encoders using PID control.
    - block_5sensors/
      - block_5sensors.ino: Demonstrates blocking-based navigation with 5 sensors.
    - path_tracing/
      - path_tracing.ino: Implements path-tracing logic for the robot.
    - floodfilll_final/
      - floodfilll_final.ino: Implements the final version of the Floodfill algorithm.
    - COORDINATES_WIFI/
      - COORDINATES_WIFI.ino: Sends coordinate data over Wi-Fi.
    - OJASS_CODE_TEST/
      - OJASS_CODE_TEST.ino: Contains experimental code used during OJASS testing.
    - block_width_copy_20250110001923/
      - block_width_copy_20250110001923.ino: Tests block width adjustments for sensors.
    - matrix_code_final/
      - matrix_code_final.ino: Implements matrix-based navigation logic.
    - block_wifi/
      - block_wifi.ino: Demonstrates block-based navigation with Wi-Fi support.
  - Hardware Basic Codes/
    - mpu6050/
      - mpu6050.ino: Reads data from the MPU6050 sensor.
      - OSCData.h: Header file for handling OSC data.
      - OSCMessage.h: Header file for OSC messaging.
    - TBU6612/
      - TBU6612.ino: Controls motors using the TB6612FNG driver.
    - US100_ESP32/
      - US100_ESP32.ino: Reads distance data from the US100 ultrasonic sensor with ESP32.
    - VL53l0X/
      - VL53l0X.ino: Reads distance data from the VL53L0X ToF sensor.
    - MotorRotationCountEncoder/
      - MotorRotationCountEncoder.ino: Tracks motor rotation counts using encoders.
    - mpu_wifi/
      - mpu_wifi.ino: Sends MPU6050 data over Wi-Fi.
    - US-100 UART/
      - US-100.ino: Reads distance data from the US100 ultrasonic sensor using UART.
    - ESP32Wifi_Ultrasonic/
      - ESP32Wifi_Ultrasonic.ino: Combines ultrasonic sensing with Wi-Fi data transmission.
    - US-100_Manual/
      - US-100_Manual.ino: Provides manual control for the US100 ultrasonic sensor.


### 2. [PCB Design/](./PCB%20Design)

This folder contains the design files for the robot's custom PCB.


### 3. [Proposal/](./Proposal)

This folder contains the project proposal documents to understand the project's objectives, methodology, and outcomes.

---

## Additional Files

### [POC\_Video.mp4](./POC_Video.mp4)

This video demonstrates the robot successfully solving a maze. Use it as a reference for understanding the robot's capabilities and performance during the competition.


## Getting Started

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/Soumyajit21213/Gujarat_Robofest_Maze_Solver.git
   ```

2. **Navigate to the Folder:**

   ```bash
   cd Gujarat_Robofest_Maze_Solver
   ```

3. **Open Relevant Files:**

   - For Arduino code, navigate to the `ARDUINO CODES/` folder.
   - For PCB design, use the files in the `PCB Design/` folder.
   - For project documentation, review the `Proposal/` folder.

---

## Contributions

Contributions are welcome! If you have suggestions or improvements, feel free to fork the repository and create a pull request.

---

## Contact

If you have any questions or need assistance, please feel free to reach out:

- **Author:** Soumyajit Samanta
- **Email:** [Soumyajit2121@gmail.com](mailto\:soumyajit2121@gmail.com)



