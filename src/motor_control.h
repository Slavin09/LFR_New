#pragma once

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#define STBY 6

#define LEFT_MOTOR_PIN_1 8
#define LEFT_MOTOR_PIN_2 7
#define RIGHT_MOTOR_PIN_1 4
#define RIGHT_MOTOR_PIN_2 5

#define LEFT_MOTOR_PWM_PIN 9
#define RIGHT_MOTOR_PWM_PIN 11

void motorInit();
void moveStraight(int leftMotorSpeed, int rightMotorSpeed);
void turnCCW(int leftMotorSpeed, int rightMotorSpeed);
void turnCW(int leftMotorSpeed, int rightMotorSpeed);
void shortBrake(int durationMillis);
void stop();

#endif