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
#ifndef	__IMU_H__
#define	__IMU_H__

#define	SPEAKER_OUT				05
#define	HEARTBEAT_LED			13

#define	NUMBER_DISPLAYS			01
#define	DISPLAY_ADDR_1			0x70
#define	DISPLAY_ADDR_2			0x71

#define	MATRIX_DISPLAY_ADDR		0x71

//  Sensor routine settings
#define  ANALOG_PIN_BASE      	00
#define  DIGITAL_PIN_BASE     	12

//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN		08
#define	SOFT_I2C_SCL_PIN		09

#define	MAX_VOLTS				5.0
#define	MAX_STEPS				1024.0

#endif
