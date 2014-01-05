/*
  Program:      W.A.L.T.E.R. 2.0, Main navigation and reactive behaviors
  Date:         29-Dec-2013
  Version:      0.1.2 ALPHA

  Purpose:      Added new displaySoundDirection() routine. I've started to add some
                  error handling code, as well as code to handle Wire (I2C) slave
                  operation.
                  
                Connected BotBoarduino to SSC-32, and they communicate fine. Added a
                  nice little moveServoPw() and moveServoDegrees() routines that
                  include speed and time parameters. Added servo move routines,
                  sensor display routines, modified the servo structure to include
                  max degrees (90 or 180), so the appropriate value can ge stored
                  in servo.angle for each servo. The servo move routines support
                  combination moves.

  Dependencies: Adafruit libraries:
                  LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307
                
                Basic Micro libraries:
                  RoboClaw, BMSerial  

                Hybotics libraries:
                  BMP180 (modified from Adafruit's BMP085 library)
                  
  Comments:     Credit is given, where applicable, for code I did not originate.
                  This sketch started out as an Adafruit tutorial for the electret
                  microphones being used for sound detection. I've also pulled
                  code for the GP2D12 IR and PING sensors from the Arduino
                  Playground, which I have modified to suit my needs.
 
                Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/

#include <Wire.h>
#include <SoftI2CMaster.h>

#include <Adafruit_Sensor.h>

#include <Adafruit_BMP180_Unified.h>
#include <Adafruit_LSM303DLHC_Unified.h>
#include <Adafruit_L3GD20.h>

#include <Adafruit_TCS34725.h>
#include <Adafruit_TMP006.h>

#include <RTClib.h>

#include <RoboClaw.h>

#include <KalmanFilter.h>

#include "Walter.h"

/*
    Initialize our sensors
    
    We have:
      These are all on a single small board from Adafruit
        http://www.adafruit.com/products/1604
        A BMP180 temperature and pressure sensor
        An L3GD20 Gyro
        An LSM303 3-Axis accelerometer / magnetometer (compass)
      
      These are also from Adafruit:
        http://www.adafruit.com/products/1334 (TCS34725 RGB Color sensor)
        http://www.adafruit.com/products/1296 (TMP006 Heat sensor)
        http://www.adafruit.com/products/264 (DS1307 Realtime Clock)
      
      GP2D12 IR Ranging sensors (3)
      PING Ultrasonic Ranging sensors (3)
*/

Adafruit_BMP180_Unified temperature = Adafruit_BMP180_Unified(10001);
Adafruit_L3GD20 gyro;
Adafruit_LSM303_Accel_Unified accelerometer = Adafruit_LSM303_Accel_Unified(10002);
Adafruit_LSM303_Mag_Unified compass = Adafruit_LSM303_Mag_Unified(10003);

Adafruit_TCS34725 color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TMP006 heat;
RTC_DS1307 clock;

/*
    Initialize global variables
*/

/*
    Initialize servos
*/
Servo panS = {
  SERVO_PAN_PIN,
  SERVO_PAN_ADJUST,
  0,
  0,
  SERVO_PAN_LEFT_MAX,
  SERVO_PAN_RIGHT_MAX,
  SERVO_MAX_DEGREES,
  0
};

Servo tiltS = {
  SERVO_TILT_PIN,
  SERVO_TILT_ADJUST,
  0,
  0,
  SERVO_TILT_DOWN_MAX,
  SERVO_TILT_UP_MAX,
  SERVO_MAX_DEGREES,
  0
};

//  These are where the sensor readings are stored.
int ping[MAX_PING];
float gp2d12[MAX_GP2D12];

/****************************************************************
                          Code starts here
*****************************************************************/
/*
    Display accelerometer settings
*/
void displayAccelerometerSettings (void) {
  sensor_t accelSettings;
  
  accelerometer.getSensor(&accelSettings);
  
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(accelSettings.name);
  Serial.print  ("Driver Ver:   "); Serial.println(accelSettings.version);
  Serial.print  ("Unique ID:    "); Serial.println(accelSettings.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(accelSettings.max_value);
  Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(accelSettings.min_value);
  Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(accelSettings.resolution);
  Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println();

  delay(500);
}

/*
    Display the GP2D12 sensor readings (cm)
*/
void displayGP2D12 (void) {
  int sensorNr;
  
  for (sensorNr = 0; sensorNr < MAX_GP2D12; sensorNr++) { 
    Serial.print("gp2d12 #");
    Serial.print(sensorNr + 1);
    Serial.print(" range = ");
    Serial.print(gp2d12[sensorNr]);
    Serial.println(" cm");
  }  
  
  Serial.println("");  
}

/* 
    Function that reads a value from GP2D12 infrared distance sensor and returns a
      value in centimeters.

    This sensor should be used with a refresh rate of 36ms or greater.

    Javier Valencia 2008

    float read_gp2d12(byte pin)

    It can return -1 if something gone wrong.
    
    TODO: Make several readings over a time period, and average them
      for the final reading.
*/
float readGP2D12 (byte pin) {
  int tmp;

  tmp = analogRead(pin);

  if (tmp < 3)
    return -1;                                  // Invalid value
  else
    return (6787.0 /((float)tmp - 3.0)) - 4.0;  // Distance in cm
} 

/*
    Convert a pulse width in ms to inches
*/
long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

/*
    Convert a pulse width in ms to a distance in cm
*/
long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void displayPING (void) {
  int sensorNr;
  
  // Display PING sensor readings (cm)
  for (sensorNr = 0; sensorNr < MAX_PING; sensorNr++) {
    Serial.print("Ping #");
    Serial.print(sensorNr + 1);
    Serial.print(" range = ");
    Serial.print(ping[sensorNr]);
    Serial.println(" cm");
  }
 
  Serial.println("");
}

/*
    Ping))) Sensor 
    This sketch reads a PING))) ultrasonic rangefinder and returns the
    distance to the closest object in range. To do this, it sends a pulse
    to the sensor to initiate a reading, then listens for a pulse
    to return.  The length of the returning pulse is proportional to
    the distance of the object from the sensor.
     
    The circuit:
      * +V connection of the PING))) attached to +5V
      * GND connection of the PING))) attached to ground
      * SIG connection of the PING))) attached to digital pin 7

    http://www.arduino.cc/en/Tutorial/Ping
   
    Created 3 Nov 2008
      by David A. Mellis
 
    Modified 30-Aug-2011
      by Tom Igoe

    Modified 09-Aug-2013
      by Dale Weber

    Set units = true for inches, and false for cm
*/
int readPING (byte pingPin, boolean units) {
  long duration;
  int result;

  /*
    The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  */
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  /*
    The same pin is used to read the signal from the PING))): a HIGH
      pulse whose duration is the time (in microseconds) from the sending
      of the ping to the reception of its echo off of an object.

      pinMode(pingPin, INPUT);
      duration = pulseIn(pingPin, HIGH);
  */

  //  Convert the time into a distance
  if (units) {
    //  Return result in inches.
    result = microsecondsToInches(duration);
  } else {
    //  Return result in cm
    result = microsecondsToCentimeters(duration);
  }
 
  delay(100);
  
  return result;
}

/*
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
  digitalWrite(pin, HIGH);   // Turn the ON by making the voltage HIGH (5V)
  delay(duration);           // Wait for duration ms
  digitalWrite(pin, LOW);    // Turn the pin OFF by making the voltage LOW (0V)
  delay(duration);           // Wait for duration ms
}

/*
    Move a servo by pulse width in ms (500ms - 2500ms)
*/
void moveServoPw (Servo *servo, int servoPosition, int moveSpeed, int moveTime, boolean term) {
  servo->error = 0;
  
  if ((servoPosition >= servo->minPulse) && (servoPosition <= servo->maxPulse)) {
    Serial.print("#");
    Serial.print(servo->pin);
    Serial.print(" P");
    Serial.print(servoPosition + servo->offset);

    servo->msPulse = servoPosition;
    servo->angle = ((servoPosition - SERVO_CENTER_MS) / 10);
    
    if (servo->maxDegrees == 180) {
      servo->angle += 90;
    }
  } else if ((servoPosition < servo->minPulse) || (servoPosition > servo->maxPulse)) {
    servo->error = 200;
  }
 
  if (servo->error == 0) {
    //  Add servo move speed
    if (moveSpeed != 0) {
      Serial.print(" S");
      Serial.print(moveSpeed);
    }
    
    //  Terminate the command
    if (term) {
      if (moveTime != 0) {
        Serial.print(" T");
        Serial.print(moveTime);
      }
      
      Serial.println();
    }
  }
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180)
*/
void moveServoDegrees (Servo *servo, int servoDegrees, int moveSpeed, int moveTime, boolean term) {
  int servoPulse = SERVO_CENTER_MS + servo->offset;

  servo->error = 0;
  
  //  Convert degrees to ms for the servo move
  if (servo->maxDegrees == 90) {
    servoPulse = (SERVO_CENTER_MS + servo->offset) + (servoDegrees * 10);
  } else if (servo->maxDegrees == 180) {
    servoPulse = (SERVO_CENTER_MS + servo->offset) + ((servoDegrees - 90) * 10);
  }

  if ((servoPulse >= servo->minPulse) && (servoPulse <= servo->maxPulse)) {
    Serial.print("#");
    Serial.print(servo->pin);
    Serial.print(" P");
    Serial.print(servoPulse);
    servo->msPulse = (servoDegrees * 10) + SERVO_CENTER_MS;
    servo->angle = servoDegrees;
    
    if (servo->maxDegrees == 180) {
      servo->angle += 90;
    }
  } else if ((servoPulse < servo->minPulse) || (servoPulse > servo->maxPulse)) {
    servo->error = 200;
  }
  
  if (servo->error == 0) {
    //  Add servo move speed
    if (moveSpeed != 0) {
      Serial.print(" S");
      Serial.print(moveSpeed);
    }
    
    //  Terminate the command
    if (term) {
      if (moveTime != 0) {
        Serial.print(" T");
        Serial.print(moveTime);
      }
      
      Serial.println();
    }
  }
}

/*
    Process error conditions
*/
void processError (byte err) {
  Serial.print("Error: ");
  Serial.print(err);
  Serial.println("");
}

/*
    Called when a request from an I2C (Wire) Master comes in
*/
void wireRequestEvent (void) {
  
}

/*
    Called when the I2C (Wire) Slave receives data from an I2C (Wire) Master
*/
void wireReceiveData (int nrBytesRead) {

}

/*
    Setup routine - runs just one time
*/
void setup (void) {
  //  Initialize the hardware serial port  
  Serial.begin(115200);

  //  Start up the Wire library as a slave device at address 0xE0
  Wire.begin(NAV_I2C_ADDRESS);

  //  Register event handler
  Wire.onRequest(wireRequestEvent);
  Wire.onReceive(wireReceiveData);
  
  //  Initialize the LED pin as an output.
  pinMode(HEARTBEAT_LED, OUTPUT);
  
  /*  Initialize the accelerometer */
  if(!accelerometer.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  /*  Initialize the compass (magnetometer) sensor */
  if(!compass.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /*  Initialize and warn if we couldn't detect the chip */
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS)) {
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS)) {
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS)) {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  
  //  Put the front pan/tilt in home position
  moveServoPw(&panS, SERVO_CENTER_MS, 0, 0, false);
  moveServoPw(&tiltS, SERVO_CENTER_MS, 0, 0, true);
//  moveServoDegrees(&panS, moveDegrees, moveSpeed, moveTime, false);
//  moveServoDegrees(&tiltS, moveDegrees, moveSpeed, moveTime, true);
}

/*
    The loop routine runs forever
*/
void loop (void) {
  byte error = 0;

  int analogPin = 0;
  int digitalPin = 0;

  DateTime now = clock.now();
  sensors_event_t accelEvent, compassEvent, temperatureEvent;

  float accelX, accelY, accelZ;
  float compassX, compassY, compassZ;
  int gyroX, gyroY, gyroZ;

  float celsius, fahrenheit, altitude;
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  
  // Pulse the heartbeat LED
  pulseDigital(HEARTBEAT_LED, 500);

  if (MAX_GP2D12 > 0) {
    //  Get distance readings from the GP2D12 Analog IR sensors and store the readings  
    for (analogPin = 0; analogPin < MAX_GP2D12; analogPin++) { 
      gp2d12[analogPin] = readGP2D12(analogPin);
    }
  }

  if (MAX_PING > 0) {
    //  Get distance readings from PING Ultrasonic sensors in cm and store the readings
    for (digitalPin = 0; digitalPin < MAX_PING; digitalPin++) {
      ping[digitalPin] = readPING(digitalPin + DIGITAL_PIN_BASE, false);
    }
  }
  
  /*
      Temperature

      Get a new BMP180 sensor event 
  */
  temperature.getEvent(&temperatureEvent);
  
  //  Display the barometric pressure in hPa
  if (temperatureEvent.pressure) {
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */

    //  First we get the current temperature from the BMP180 in celsius and fahrenheit
    temperature.getTemperature(&celsius);
    fahrenheit = (celsius * 1.8) + 32;

    //   Convert the atmospheric pressure, SLP and temp to altitude in meters
    altitude = temperature.pressureToAltitude(seaLevelPressure, accelEvent.pressure, celsius); 
  }

  /*
      Get accelerometer readings
  */
  accelerometer.getEvent(&accelEvent);
 
  /* Display the results (acceleration is measured in m/s^2) */
  accelX = accelEvent.acceleration.x;
  accelY = accelEvent.acceleration.y;
  accelZ = accelEvent.acceleration.z;

/* 
  Serial.print("X: ");
  Serial.print(accelEvent.acceleration.x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(accelEvent.acceleration.y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(accelEvent.acceleration.z);
  Serial.print("  ");
  Serial.println("m/s^2 ");
*/

  /*
      Get compass readings
  */
  compass.getEvent(&compassEvent);

  compassX = compassEvent.magnetic.x;
  compassY = compassEvent.magnetic.y;
  compassZ = compassEvent.magnetic.z;
  
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */

/*
  Serial.print("X: ");
  Serial.print(compassEvent.magnetic.x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(compassEvent.magnetic.y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(compassEvent.magnetic.z);
  Serial.print("  ");
  Serial.println("uT");
*/  

  /*
      Get gyro readings
  */
  gyro.read();
  
  gyroX = (int)gyro.data.x;
  gyroY = (int)gyro.data.y;
  gyroZ = (int)gyro.data.z;

/*  
  Serial.print("X: ");
  Serial.print(gyroX);
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.print(gyroY);
  Serial.print(" ");
  Serial.print("Z: ");
  Serial.println(gyroZ);
*/

  /*
      Accelerometer and Gyro reactive behaviors
  */

  /*
      Distance related reactive behaviors
  */

  if (error != 0) {
    processError(error);
  }
}
