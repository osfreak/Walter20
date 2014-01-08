/*
  Program:      W.A.L.T.E.R. 2.0, Main navigation and reactive behaviors
  Date:         28-Dec-2013
  Version:      0.1.2 ALPHA

  Purpose:      Added new displaySoundDirection() routine. I've started to add some
                  error handling code, as well as code to handle Wire (I2C) slave
                  operation.

  Dependencies: Adafruit libraries:
                  LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307

                Hybotics libraries:
                  BMP180 (modified from Adafruit's BMP085 library)

  Comments:     Credit is given, where applicable, for code I did not originate.
                  This sketch started out as an Adafruit tutorial for the electret
                  microphones being used for sound detection. I've also pulled
                  code for the GP2D12 IR and PING sensors from the Arduino
                  Playground, which I have modified to suit my needs.

                Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/
#ifndef	__NAVIGATION_H__
#define	__NAVIGATION_H__

#define	NAV_I2C_ADDRESS			0x50

#define	SPEAKER_OUT				05
#define	HEARTBEAT_LED       	13

//  Sensor routine settings
#define	ANALOG_PIN_BASE			00
#define	DIGITAL_PIN_BASE		12

#define	MAX_PING				0
#define	MAX_GP2D12				3

//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN		08
#define	SOFT_I2C_SCL_PIN		09

//	The following settings apply to the SSC-32 servo controller
#define	SERVO_MAX_DEGREES		90
#define	SERVO_CENTER_MS			1500

#define	SERVO_PAN_PIN			00
#define	SERVO_PAN_ADJUST		0
#define	SERVO_PAN_LEFT_MAX		500
#define	SERVO_PAN_RIGHT_MAX		2400

#define	SERVO_TILT_PIN			01
#define	SERVO_TILT_ADJUST		-100
#define	SERVO_TILT_DOWN_MAX		500
#define	SERVO_TILT_UP_MAX		2000

#define	SERVO_MOTOR_LEFT_PIN	04
#define	SERVO_MOTOR_LEFT_ADJ	0
#define	SERVO_MOTOR_RIGHT_PIN	05
#define	SERVO_MOTOR_RIGHT_ADJ	0

//  Display routine constants
const int maxScale = 8;
const int redZone = 5;

//  Sample window width in mS (50 mS = 20Hz)
const int sampleWindow = 50;

struct Servo {
	byte pin;
	int offset;
	int msPulse;
	int angle;
	int minPulse;
	int maxPulse;
	int maxDegrees;

	int error;
};

#endif
