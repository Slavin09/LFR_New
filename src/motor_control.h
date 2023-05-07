#pragma once

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#define LEFT_MOTOR_PIN_1 5
#define LEFT_MOTOR_PIN_2 6
#define RIGHT_MOTOR_PIN_1 9
#define RIGHT_MOTOR_PIN_2 10

void moveStraight(int leftMotorSpeed, int rightMotorSpeed);
void turnCCW(int leftMotorSpeed, int rightMotorSpeed);
void turnCW(int leftMotorSpeed, int rightMotorSpeed);
void shortBrake(int durationMillis);
void stop();

#endif