# mobilelab_data_gatherer

Gathers actuation data and sends it to the main control system.

## Inputs
* A0 - Controller Steering (PPM) - The steering output from the radio controller
* A1 - Controller Acceleration (PPM) - The acceleration output from the radio controller
* A2 - Autonomous Steering - The steering from the autonomous controller
* A3 - Autonomous Acceleration - The acceleration from the autonomous controller
* B12 - Enable Left (Bit) - Whether the left steering is enabled
* B13 - Enable Right (Bit) - Whether the right steering is enabled
* B14 - Autonomous Mode (Bit) - Whether the car is autonomous

## Outputs
* A6 - Right Steering (PWM)
* A7 - Left Steering (PWM)
* B0 - Acceleration (PWM)
* B15 - Reverse Acceleration (Bit) - Whether the acceleration is backwards
* C13 - Indicator Led (Bit) - Used as a visual indicator