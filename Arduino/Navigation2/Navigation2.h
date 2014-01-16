/*
	Program:      	W.A.L.T.E.R. 2.0, Main navigation, and reactive behaviors, header file
	Date:         	15-Jan-2014
	Version:      	0.1.5 ALPHA

	Purpose:      	Added new displaySoundDirection() routine. I've started to add some
						error handling code, as well as code to handle Wire (I2C) slave
						operation.

	Dependencies:	Adafruit libraries:
                  		LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307

					Hybotics libraries:
						BMP180 (modified from Adafruit's BMP085 library)

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2D12 IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.

					Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/
#ifndef	__NAVIGATION2_H__
#define	__NAVIGATION2_H__

#define	NAV_I2C_ADDRESS					0x50

#define	SPEAKER_OUT						5
#define	HEARTBEAT_LED       	        13

//  Sensor routine settings
#define	ANALOG_PIN_BASE					0
#define	DIGITAL_PIN_BASE				6

#define	MAX_PING						3
#define	MAX_IR							1

//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN				2
#define	SOFT_I2C_SCL_PIN				3

/*
	The following settings apply to the SSC-32 servo controller
*/
#define	SERVO_MAX_DEGREES				90
#define	SERVO_CENTER_MS					1500

#define	SERVO_PAN_PIN					0
#define	SERVO_PAN_ADJUST				0
#define	SERVO_PAN_LEFT_MAX				500
#define	SERVO_PAN_RIGHT_MAX				2400

#define	SERVO_TILT_PIN					1
#define	SERVO_TILT_ADJUST				-100
#define	SERVO_TILT_DOWN_MAX				500
#define	SERVO_TILT_UP_MAX				2000

#define	SERVO_MOTOR_LEFT_PIN			4
#define	SERVO_MOTOR_LEFT_ADJ	        0
#define	SERVO_MOTOR_RIGHT_PIN	        5
#define	SERVO_MOTOR_RIGHT_ADJ	        0
/*
	End of SSC-32 definitions
*/

#define	NUMBER_DISPLAYS					1
#define	SEVEN_SEG_BASE_ADDR				0x70

#define	MATRIX_DISPLAY_ADDR				SEVEN_SEG_BASE_ADDR + NUMBER_DISPLAYS

#define	MAX_VOLTS						5.0
#define	MAX_STEPS						1024.0

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

enum SensorLocation {
	FRONT_LEFT,
	FRONT_CENTER,
	FRONT_RIGHT,

	BACK_LEFT,
	BACK_CENTER,
	BACK_RIGHT
};

enum MotorLocation {
	LEFT,
	RIGHT,
	FRONT,
	BACK
};

#endif
