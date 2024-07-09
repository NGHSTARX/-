# README

## Overview

This project involves the implementation of a robot cart that can follow predefined routes when specific buttons are pressed. The cart is equipped with motors, an OLED display, a buzzer, and a battery voltage monitoring system. It also includes UART communication for remote control and status updates.

## Features

1. **Motor Control**:
   - The robot has two wheels controlled by PWM signals.
   - Functions for moving the wheels forward, backward, and stopping.

2. **Button Handling**:
   - Buttons are used to select predefined routes for the robot to follow.
   - Interrupt-based button handling to detect button presses.

3. **Voltage Monitoring**:
   - The CW2015 module is used to monitor the battery voltage.
   - The voltage is read via I2C communication and displayed on the OLED screen.

4. **OLED Display**:
   - The OLED screen displays messages such as battery voltage and operational status.

5. **Buzzer**:
   - A buzzer is used to provide audio feedback when buttons are pressed and at certain points during the route.

6. **UART Communication**:
   - UART is used to send and receive data for remote control and monitoring.
   - Configured for communication with a baud rate of 115200.

7. **Predefined Routes**:
   - Route1: The robot follows a specific path, providing audio and visual feedback at various points.
   - Route2: Another predefined path similar to Route1.

## Code Description

### Initialization Functions

- **CW2015Init**: Initializes the I2C communication for the CW2015 voltage monitoring module.
- **Uart1GpioInit**: Initializes the GPIO pins for UART1 communication.
- **Uart1Config**: Configures UART1 with the required settings.
- **FuncKeyInit**: Initializes the function keys and registers interrupt handlers.
- **Hcsr04Init**: Initializes the GPIO pins and PWM channels for the motors.
- **BuzzerInit**: Initializes the GPIO pin and PWM channel for the buzzer.

### Utility Functions

- **Cw20_WriteRead**: Reads data from the CW2015 module.
- **Cw20_Write**: Writes data to the CW2015 module.
- **GetVoltage**: Calculates the battery voltage from the CW2015 module readings.
- **FeedFood**: Displays a welcome message and battery voltage on the OLED screen.
- **GetFunKeyState**: Continuously monitors the state of function keys and triggers appropriate actions.

### Motor Control Functions

- **LeftWheelForward, LeftWheelBackward, LeftWheelStop**: Control the left wheel's movement.
- **RightWheelForward, RightWheelBackward, RightWheelStop**: Control the right wheel's movement.

### Route Functions

- **Route1**: Defines the sequence of movements for Route 1, including turning, moving forward/backward, and providing feedback via buzzer and OLED.
- **Route2**: Similar to Route1 but with different movement sequences.

### Main Execution

- **TimerThread**: Starts a timer for periodic tasks and checks the function key states.
- **OnFuncKeyPressed**: Interrupt handler for function key presses.

## How to Use

1. **Setup**:
   - Ensure all hardware components (motors, buttons, OLED, buzzer, CW2015 module) are connected correctly.
   - Flash the code to the microcontroller.

2. **Operation**:
   - Press the function keys to select a route.
   - The robot will follow the predefined path and provide feedback via the OLED screen and buzzer.
   - Monitor the battery voltage on the OLED screen.

3. **Remote Control**:
   - Use UART communication to send commands and receive status updates.
   - Configure the UART settings as specified in the code.

## Dependencies

- **CMSIS-RTOS2**: For real-time operating system functionalities.
- **IoT libraries**: For GPIO, PWM, I2C, UART, and other peripheral control.
- **OLED and PCA9555 libraries**: For display and I/O expander functionalities.

## Notes

- Ensure the baud rate for UART communication matches the configuration in the code.
- Modify the route sequences as needed to fit your specific requirements.
- Debug and test the system in a controlled environment before deploying it in a real-world scenario.
