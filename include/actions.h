/**
 * Sends actuation data to the control system
 */

#ifndef ACTIONS_H_
#define ACTIONS_H_

void setSteeringLeft(float value);
void setSteeringRight(float value);
void resetSteeringLeft();
void resetSteeringRight();

void setAcceleration(float value);
void resetAcceleration();
void setAccelerationForward();
void setAccelerationBackward();

void setControlEnabled();
void setControlDisabled();

void setLedEnabled();
void setLedDisabled();

uint8_t isAutonomousMode();

uint8_t isSteeringLeftEnabled();
uint8_t isSteeringRightEnabled();

uint16_t getAutonomousSteering();
uint16_t getAutonomousAcceleration();

#endif
