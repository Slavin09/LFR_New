#include <Arduino.h>
#include "motor_control.h"
#include "Config.h"

void moveStraight(int leftMotorSpeed, int rightMotorSpeed)
{
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

    
    if(leftMotorSpeed>=0){
        // Left motor front
    analogWrite(LEFT_MOTOR_PIN_1, leftMotorSpeed);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);
    }
    else{
        // Left motor back
        analogWrite(LEFT_MOTOR_PIN_2, abs(leftMotorSpeed));
        digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    }
    // Right motor front
    if(rightMotorSpeed>=0)
    {
        // Right motor front
        analogWrite(RIGHT_MOTOR_PIN_1, rightMotorSpeed);
        digitalWrite(RIGHT_MOTOR_PIN_2, LOW);
    }
    else
    {
        analogWrite(RIGHT_MOTOR_PIN_2, abs(rightMotorSpeed));
        digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    }
}

void turnCCW(int leftMotorSpeed, int rightMotorSpeed)
{
#if (TURN_SPEED_REDUCTION_ENABLED == 1)
leftMotorSpeed = (leftMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif

leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

Serial.print("Turning CCW with ");
Serial.print(leftMotorSpeed);
Serial.print(" and ");
Serial.println(rightMotorSpeed);

    // Left motor back
    digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    analogWrite(LEFT_MOTOR_PIN_2, leftMotorSpeed);

    // Right motor front
    analogWrite(RIGHT_MOTOR_PIN_1, rightMotorSpeed);
    digitalWrite(RIGHT_MOTOR_PIN_2, LOW);
}

void turnCW(int leftMotorSpeed, int rightMotorSpeed)
{
    #if (TURN_SPEED_REDUCTION_ENABLED == 1)
    leftMotorSpeed = (leftMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
    rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
    #endif

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    Serial.print("Turning CW with ");
    Serial.print(leftMotorSpeed);
    Serial.print(" and ");
    Serial.println(rightMotorSpeed);

    // Left motor front
    analogWrite(LEFT_MOTOR_PIN_1, leftMotorSpeed);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);

    // Right motor back
    digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    analogWrite(RIGHT_MOTOR_PIN_2, rightMotorSpeed);
}

void shortBrake(int durationMillis)
{
    analogWrite(LEFT_MOTOR_PIN_1, 0 );
    analogWrite(RIGHT_MOTOR_PIN_1, 0);

    // Left motor back
    analogWrite(LEFT_MOTOR_PIN_2, 255);

    // Right motor back
    analogWrite(RIGHT_MOTOR_PIN_2, 255);

    delay(durationMillis);
}

void stop()
{
    // Left motor stop
    digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    analogWrite(LEFT_MOTOR_PIN_2, 5);

    // Right motor stop
    digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    analogWrite(RIGHT_MOTOR_PIN_2, 5);
}




