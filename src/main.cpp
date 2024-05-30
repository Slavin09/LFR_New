#include <Arduino.h>
#include <stdint.h>
#include "motor_control.h"
#include "Config.h"

#define WHITE_LINE_BLACK_TRACK 1
#define BLACK_LINE_WHITE_TRACK 0
#define SENSOR_COUNT 8
#define POSITIONAL_WEIGHT_W 100 

#define ALL_SENSORS_DETECT_LINE_COLOR(R) (R == 0b11111111)
#define ALL_SENSORS_OUT_OF_LINE_COLOR(R) (R == 0b00000000)

#define MID_6_SENSORS_DETECT_LINE_COLOR(R) ((R & 0b01111110) == 0b01111110)

#define LED 3

String dataBL = "";

int Kp = DEFAULT_KP;
int Ki = DEFAULT_KI;
int Kd = DEFAULT_KD;

int P = 0;
int I = 0;
int D = 0;
int previousError = 0;
int PID_value = 0;

int TrackType= BLACK_LINE_WHITE_TRACK;

int S0=A7;
int S1=A6;
int S2=A5;
int S3=A4;
int S4=A3;
int S5=A2;
int S6=A1;
int S7=A0;

int loopDelay = DEFAULT_LOOP_DELAY;
int leftMotorOffset = 50;
int rightMotorOffset = 50;
int error=0;
int error_dir =0;
int baseMotorSpeed= DEFAULT_MOTOR_SPEED;
uint8_t getSensorReadings()
{
  uint8_t sensorData=0x00;

  int Sen1=analogRead(S0);
  int Sen2=analogRead(S1);
  int Sen3=digitalRead(S2);
  int Sen4=digitalRead(S3);
  int Sen5=digitalRead(S4);
  int Sen6=digitalRead(S5);
  int Sen7=digitalRead(S6);
  int Sen8=digitalRead(S7);

  if(Sen1>400)
    Sen1=1;
  else
    Sen1=0;
    
  if(Sen2>400)
    Sen2=1;
  else{
    Sen2=0;
}

    uint8_t s1=(uint8_t) Sen1;
    uint8_t s2=(uint8_t) Sen2;
    uint8_t s3=(uint8_t) Sen3;
    uint8_t s4=(uint8_t) Sen4;
    uint8_t s5=(uint8_t) Sen5;
    uint8_t s6=(uint8_t) Sen6;
    uint8_t s7=(uint8_t) Sen7;
    uint8_t s8=(uint8_t) Sen8;

    sensorData |= (s1 << 7) | (s2 << 6) | (s3 << 5) | (s4 << 4) | (s5 << 3) | (s6 << 2) | (s7 << 1) | s8;

    // Serial.print(s1);
    // Serial.print(s2);
    // Serial.print(s3);
    // Serial.print(s4);
    // Serial.print(s5);
    // Serial.print(s6);
    // Serial.print(s7);
    // Serial.println(s8);
    if (sensorData == 0b01100000)
        TrackType = WHITE_LINE_BLACK_TRACK;
    else if (sensorData == 0b01110000 || sensorData == 0b00100000)
        TrackType = WHITE_LINE_BLACK_TRACK;
    else if (sensorData == 0b00110000)
        TrackType = WHITE_LINE_BLACK_TRACK;
    else if (sensorData == 0b00111000 || sensorData == 0b00010000)
        TrackType = WHITE_LINE_BLACK_TRACK;
    else if (sensorData == 0b00011000)
        TrackType = WHITE_LINE_BLACK_TRACK;
    else if (sensorData == 0b00011100 || sensorData == 0b00001000)
        TrackType = WHITE_LINE_BLACK_TRACK;
    else if (sensorData == 0b00001100)
        TrackType = WHITE_LINE_BLACK_TRACK;
    else if (sensorData == 0b00001110 || sensorData == 0b00000100)
        TrackType = WHITE_LINE_BLACK_TRACK;
    else if (sensorData == 0b00000110)
        TrackType = WHITE_LINE_BLACK_TRACK;

    else if (sensorData == 0b10011111)
        TrackType = BLACK_LINE_WHITE_TRACK;
    else if (sensorData == 0b10001111 || sensorData == 0b11011111)
        TrackType = BLACK_LINE_WHITE_TRACK;
    else if (sensorData == 0b11001111)
        TrackType = BLACK_LINE_WHITE_TRACK;
    else if (sensorData == 0b11000111 || sensorData == 0b11101111)
        TrackType = BLACK_LINE_WHITE_TRACK;
    else if (sensorData == 0b11100111)
        TrackType = BLACK_LINE_WHITE_TRACK;
    else if (sensorData == 0b11100011 || sensorData == 0b11110111)
        TrackType = BLACK_LINE_WHITE_TRACK;
    else if (sensorData == 0b11110011)
        TrackType = BLACK_LINE_WHITE_TRACK;
    else if (sensorData == 0b11110001 || sensorData == 0b11111011)
        TrackType = BLACK_LINE_WHITE_TRACK;
    else if (sensorData == 0b11111001)
        TrackType = BLACK_LINE_WHITE_TRACK;


    if (TrackType == BLACK_LINE_WHITE_TRACK)
  sensorData ^=0b11111111;

  return sensorData;
}

int getCalculatedError(int fallbackError)
{
    uint8_t sensorReading = getSensorReadings();
    int numeratorSum = 0, denominatorSum = 0;

    // Assuming that the the MSB represents the index 0 of the array (left to right)
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        // Check the digital values at the ith bit
        uint8_t sensorValue = ((sensorReading & (1 << (SENSOR_COUNT - 1 - i))) >> (SENSOR_COUNT - 1 - i));
        numeratorSum += (i + 1) * POSITIONAL_WEIGHT_W * (int)sensorValue;
        denominatorSum += sensorValue;
    }

    int error = fallbackError;
    if (denominatorSum != 0)
        error = ((numeratorSum / (denominatorSum * (POSITIONAL_WEIGHT_W/2))) - (SENSOR_COUNT + 1));
    return error;
}

int isOutOfLine(uint8_t sensorReadings)
{
    uint8_t options[] = {
        0b01100000,
        0b01110000,
        0b00110000,
        0b00111000,
        0b00011000,
        0b00011100,
        0b00001100,
        0b00001110,
        0b00000110,
        0b10011111,
        0b10001111,
        0b11001111,
        0b11000111,
        0b11100111,
        0b11100011,
        0b11110011,
        0b11110001,
        0b11111001};

    int n = sizeof(options) / sizeof(uint8_t);

    for (int i = 0; i < n; i++)
    {
        if (sensorReadings == options[i])
            return 0;
    }

    return 1;
}

void indicateOn()
{
	digitalWrite(LED, HIGH);
}

void indicateOff()
{
	digitalWrite(LED, LOW);
}


void readSensors()
{

	uint8_t sensorData = getSensorReadings();
	error = getCalculatedError(0);

	// left most sensor value
	int s1 = (sensorData & (1 << 7)) >> 7;

	// right most sensor value
	int s8 = (sensorData & (1 << 0)) >> 0;

	if (s1 != s8)
		error_dir = s1 - s8;

	// This else block bypasses the fallback error value of 255
	if (ALL_SENSORS_OUT_OF_LINE_COLOR(sensorData))
	{
		// Moved out of the line
		if (error_dir < 0)
			error = OUT_OF_LINE_ERROR_VALUE;
		else if (error_dir > 0)
			error = -1 * OUT_OF_LINE_ERROR_VALUE;
	}
	else if (ALL_SENSORS_DETECT_LINE_COLOR(sensorData))
	{
		moveStraight(baseMotorSpeed, baseMotorSpeed);
		delay(STOP_CHECK_DELAY);
		uint8_t sensorDataAgain = getSensorReadings();
		if (ALL_SENSORS_DETECT_LINE_COLOR(sensorDataAgain))
		{
			shortBrake(100);
			stop();
			delay(10000);
		}
	}

	if (MID_6_SENSORS_DETECT_LINE_COLOR(sensorData))
		indicateOn();
	else
		indicateOff();

	// if (TrackType == BLACK_LINE_WHITE_TRACK)
	// 	digitalWrite(LED, HIGH);
	// else if (TrackType == WHITE_LINE_BLACK_TRACK)
	// 	digitalWrite(LED, LOW);
}

void calculatePID()
{
	P = error;
	I = (error = 0) ? 0 : I + error;
	I = constrain(I, -200, 200);
	D = error - previousError;
  if((error-previousError)!=0)
    delay(5);
	PID_value = (Kp * P) + (Ki * I) + (Kd * D);
	PID_value = constrain(PID_value, -150, 150);
	previousError = error;
}

void controlMotors()
{
	if (error == OUT_OF_LINE_ERROR_VALUE)
	{
#if BRAKING_ENABLED == 1
		shortBrake(BRAKE_DURATION_MILLIS);
#endif
		uint8_t sensorReadings = getSensorReadings();
		while (isOutOfLine(sensorReadings))
		{
			turnCCW(baseMotorSpeed - leftMotorOffset, baseMotorSpeed - rightMotorOffset);
			sensorReadings = getSensorReadings();
		}
#if GAPS_ENABLED == 1
		error_dir = 0;
#endif
	}
	else if (error == (-1 * OUT_OF_LINE_ERROR_VALUE))
	{
#if BRAKING_ENABLED == 1
		shortBrake(BRAKE_DURATION_MILLIS);
#endif
		uint8_t sensorReadings = getSensorReadings();
		while (isOutOfLine(sensorReadings))
		{
            turnCW(baseMotorSpeed - leftMotorOffset, baseMotorSpeed - rightMotorOffset);
			sensorReadings = getSensorReadings();
		}
#if GAPS_ENABLED == 1
		error_dir = 0;
#endif
	}
	else
	{
		int rightMotorSpeed = baseMotorSpeed + PID_value - leftMotorOffset;
		int leftMotorSpeed = baseMotorSpeed - PID_value - rightMotorOffset;
  // Serial.print(leftMotorSpeed);
  // Serial.print(" --- ");
  // Serial.println(rightMotorSpeed);
		moveStraight(leftMotorSpeed, rightMotorSpeed);

		if (D != 0)
			delay(loopDelay);
	}
}

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  pinMode(S7, INPUT);

  pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);

  pinMode(LED, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);

  Serial.begin(9600);

}

void loop() {
    
  // Establishing seral connection to HC-05:
  #if BLUETOOTH_TUNING_ENABLED == 1
    if(Serial.available()>0)
    {
      dataBL = Serial.readString();

      if(dataBL == "stand")
      {
        digitalWrite(STBY, LOW); //Standby Mode
        Serial.println("Mode: " +dataBL+"     "+ "\n");
      }

      else if(dataBL == "run")
      {
        digitalWrite(STBY, HIGH); //Power Mode
        Serial.println("Mode: " +dataBL+"     "+ "\n");
      }
        

      else if (dataBL.charAt(0) == 'p') //KP tuning
      {
        dataBL.remove(0, 1);
        Kp = dataBL.toInt();
        Serial.println("Proportion: "+dataBL+"    "+Kp+ "\n");
      }
      else if (dataBL.charAt(0) == 'i') //KI Tuning
      {
        dataBL.remove(0, 1);
        Ki = dataBL.toInt();
      }
      else if (dataBL.charAt(0) == 'd') //KD Tuning
      {
        dataBL.remove(0, 1);
        Kd = dataBL.toInt();
        Serial.println("Derivative: "+dataBL+"    "+Kd+ "\n");
      }
      else if(dataBL.charAt(0) == 'm')
      {
        while (dataBL.charAt(0) == 'm') //Base Motor Speed
      {
          dataBL.remove(0, 1);
          baseMotorSpeed = dataBL.toInt();
          Serial.println("Motor Speed: "+dataBL+"    "+baseMotorSpeed+ "\n");
        
      }
      }
      
    }
  #endif
 

  readSensors();
	calculatePID();
	controlMotors();
}