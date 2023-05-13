# Window Motor Control System

The Window Motor Control System is a project that implements a task-based system for controlling window motors in a vehicle. 
It utilizes FreeRTOS to manage concurrent execution of multiple tasks, synchronization between tasks using semaphores,and GPIO interactions for controlling the motors.

## Project Structure

The project consists of the following files:

- `main.c`: Contains the main function and task creation.
- `tasks.c`: Defines the task functions for window control, obstacle detection, and system control.
- `gpio.c`: Implements GPIO handling functions.
- `delay.c`: Provides a delay function for introducing delays in milliseconds.

## Tasks

The project includes the following tasks:

1. `initTask`: Initializes the system and creates necessary semaphores and mutexes.
2. `WindowUp_Driver`: Controls the upward movement of the driver-side window.
3. `WindowDown_Driver`: Controls the downward movement of the driver-side window.
4. `WindowUp_Passenger`: Controls the upward movement of the passenger-side window.
5. `WindowDown_Passenger`: Controls the downward movement of the passenger-side window.
6. `OnOff`: Monitors the On/Off state of the system for passenger control.
7. `Obstacle`: Handles obstacle detection and performs necessary actions.

## Semaphores and Mutexes

The project utilizes the following semaphores and mutexes for task synchronization:

- `WindowUp_driverS`: Semaphore for signaling the `WindowUp_Driver` task.
- `WindowDown_driverS`: Semaphore for signaling the `WindowDown_Driver` task.
- `WindowUp_passengerS`: Semaphore for signaling the `WindowUp_Passenger` task.
- `WindowDown_passengerS`: Semaphore for signaling the `WindowDown_Passenger` task.
- `ObstacleS`: Semaphore for signaling the `Obstacle` task.
- `OnOffS`: Semaphore for signaling the `OnOff` task.
- `MotorMutex`: Mutex for controlling access to the motor.

## GPIO Interaction

The project utilizes GPIO interactions for reading inputs and controlling the window motors. The following GPIO pins and ports are used:

- `WINDOW_UP_DRIVER_PIN`: GPIO pin for upward movement of the driver-side window.
- `WINDOW_DOWN_DRIVER_PIN`: GPIO pin for downward movement of the driver-side window.
- `WINDOW_UP_PASSENGER_PIN`: GPIO pin for upward movement of the passenger-side window.
- `WINDOW_DOWN_PASSENGER_PIN`: GPIO pin for downward movement of the passenger-side window.
- `OBSTACLE_PIN`: GPIO pin for obstacle detection.
- `ON_OFF_PIN`: GPIO pin for On/Off control.
- `WINDOW_MOTOR_PORT`: GPIO port for window motor control.
- `OBSTACLE_PORT`: GPIO port for obstacle detection.
- `ON_OFF_PORT`: GPIO port for On/Off control.
- `BUTTONS_PORT`: GPIO port for buttons.

Note: It is necessary to configure the hardware peripherals and pin assignments according to the specific requirements.

## Usage

1. Configure the necessary hardware peripherals and pin assignments.
2. Build and flash the project to the microcontroller.
3. The system will start and initialize all tasks, semaphores, and mutexes.
4. The tasks will handle window control, obstacle detection, and system control based on GPIO inputs.
5. The system will respond to button presses and perform the corresponding actions on the window motors.
6. The system will detect obstacles and take appropriate actions to prevent damage to the windows.
7. The system can be turned On/Off using the designated button.

## Contributors

This project was developed by Team 8.

## Date

The project was completed on 12/5/2023.

---
Note: This README assumes that the reader has a basic understanding of embedded systems, microcontrollers, and C programming. If you are new to these concepts, it is recommended to familiarize yourself with them before working with this project.

## Dependencies

The Window Motor Control System relies on the following dependencies:

- FreeRTOS: A real-time operating system that provides task scheduling and synchronization mechanisms.
- GPIO Library: A library for interacting with GPIO pins and ports on the microcontroller.

Ensure that these dependencies are properly installed and configured before building and running the project.

## Building and Flashing

To build and flash the project to the microcontroller, follow these steps:

1. Set up the development environment and toolchain for your microcontroller.
2. Clone the project repository to your local machine or download the source files.
3. Configure the necessary hardware peripherals and pin assignments in the project files.
4. Open the project in your preferred integrated development environment (IDE).
5. Build the project to generate the binary executable file.
6. Connect the microcontroller to your computer using a suitable programming interface.
7. Flash the generated binary executable file to the microcontroller.

Refer to your microcontroller's documentation for detailed instructions on building and flashing projects.

## Troubleshooting

If you encounter any issues or errors while working with the Window Motor Control System, consider the following:

- Check your hardware connections to ensure they are properly set up and functional.
- Verify that the GPIO pin assignments in the project match your hardware configuration.
- Review the dependencies and make sure they are correctly installed and configured.
- Double-check the build settings and compiler options in your IDE.
- Refer to the project's documentation and comments for additional guidance.

If problems persist, consult the community forums or resources related to your microcontroller and development environment for further assistance.

## License

The Window Motor Control System is designed to be a learning opportunity for students and is not intended for commercial purposes.


## Contact

For any questions, suggestions, or inquiries, please contact our team at mohamada14@gmail.com .

