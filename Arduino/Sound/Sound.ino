/*
  Program:      W.A.L.T.E.R. 2.0, Sound Detection and reactive behaviors
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

#include <RTClib.h>

#include "Walter.h"

/*
    Initialize global variables
*/

//  Storage for sound detection sample data
Sample samples[MAX_SAMPLES];

/****************************************************************
                          Code starts here
*****************************************************************/

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
    Display the data for a given sound sample
*/
int displaySoundSample (byte channel, Sample samples[MAX_SAMPLES]) {
  String st;
  int error = 0;

  if (! error) {  
    Serial.print(st);
    Serial.print(" Sample = ");
    Serial.print(samples[channel].value);
    Serial.print(", Voltage = ");
    Serial.print(samples[channel].volts);
    Serial.println(" volts");
  }

  return error;
}

/*
    Display the direction of a sound detection
*/
int displaySoundDirection (byte dir) {
  byte error = 0;

  if ((dir != 0) && (dir != NO_SOUND_DETECTED)) {
    Serial.print("Sound detected from the ");
    
    switch (dir) {
      case FRONT_LEFT_SIDE:
        Serial.print("Front Left");
        break;
      
      case FRONT_RIGHT_SIDE:
        Serial.print("Front Right");
        break;
      
      case BACK_LEFT_SIDE:
        Serial.print("Back Left");
        break;
      
      case BACK_RIGHT_SIDE:
        Serial.print("Back Right");
        break;
      
      default:
        Serial.print("Invalid");
        error = 100;
        break;
    }
    
    Serial.println(".");
  } else if (dir == NO_SOUND_DETECTED) {
    Serial.println("No sound detected.");
  }
  
  return error;
}

/*
    Get a sound sample from the given channel
*/
Sample soundSampleOf (byte channel) {
  Sample sample;
 
  sample.value = analogRead(channel);
  sample.volts = (sample.value * MAX_VOLTS) / MAX_STEPS;

  //  Initialize the rest of the sample
  sample.signalMin = 1024;
  sample.signalMax = 0;
  sample.signalMinVolts = 0.0;
  sample.signalMaxVolts = 0.0;
  sample.peakToPeakVolts = 0.0;

  return sample;
}

/*
    Get samples of sound from four microphones
*/
void getSoundSamples (Sample samples[MAX_SAMPLES]) {
  byte channel;
  Sample ts;

  //  Start of sample window
  unsigned long startMillis= millis();

  while ((millis() - startMillis) < sampleWindow) {
    /*
        Get and process raw samples from each microphone
    */
    for (channel = 0; channel < MAX_CHAN; channel++) {
      samples[channel] = soundSampleOf(channel);
      ts = samples[channel];

      if (ts.value < 1024) {
        //  Toss out spurious readings
        if (ts.value > ts.signalMax) {
          //  Save just the max levels
          ts.signalMax = ts.value;
          ts.signalMaxVolts = ts.volts;
        } else if (ts.value < ts.signalMin) {
          //  Save just the min levels
          ts.signalMin = ts.value;
          ts.signalMinVolts = ts.volts;
        }
        
        samples[channel] = ts;
      }
    }  
  } //  End of sample collection loop
  
  //  Calculate the peak to peak voltages
  for (channel = 0; channel < MAX_CHAN; channel++) {
    ts = samples[channel];
    ts.peakToPeakVolts = abs(ts.signalMaxVolts - ts.signalMinVolts); 

    samples[channel] = ts;
  }
}

/*
    Try to detect the loudest sound from one of four directions
*/
byte detectSound (void) {
  unsigned int sampleValue;
  Sample *ts;

  /*
      Variables for sound detection
  */
  double detectionFrontVolts = 0.0, detectionBackVolts = 0.0;
  byte detectionFront = 0;
  byte detectionBack = 0;
  byte detectionResult = 0;

  int displayPeakFrontLeft, displayPeakFrontRight;
  int displayPeakBackLeft, displayPeakBackRight; 

  /*
      Turn all the sound detection LEDs off
  */
  digitalWrite(FRONT_LEFT_LED, LOW);
  digitalWrite(FRONT_RIGHT_LED, LOW);
  digitalWrite(BACK_LEFT_LED, LOW);
  digitalWrite(BACK_RIGHT_LED, LOW);

  getSoundSamples(samples);

  /*
      Calculate the FRONT detection value
  */
  detectionFrontVolts = abs(samples[FRONT_LEFT_SIDE].peakToPeakVolts - samples[FRONT_RIGHT_SIDE].peakToPeakVolts);
//  Serial.print("Front Detection value = ");
//  Serial.println(detectionFrontVolts);

  /*
      Calculate the BACK detection value
  */
  detectionBackVolts = abs(samples[BACK_LEFT_SIDE].peakToPeakVolts - samples[BACK_RIGHT_SIDE].peakToPeakVolts);
//  Serial.print("Back Detection value = ");
//  Serial.println(detectionBackVolts);

  /*
      Get our final detection result
  */
  if ((detectionFrontVolts > detectionBackVolts) && (detectionFrontVolts > DETECTION_THRESHOLD)) {
    //  Check for sound detection
    if (samples[FRONT_LEFT_SIDE].peakToPeakVolts > samples[FRONT_RIGHT_SIDE].peakToPeakVolts) {
      digitalWrite(FRONT_LEFT_LED, HIGH);
      detectionFront = FRONT_LEFT_SIDE;
    } else if (samples[FRONT_RIGHT_SIDE].peakToPeakVolts > samples[FRONT_LEFT_SIDE].peakToPeakVolts) {
      digitalWrite(FRONT_RIGHT_LED, HIGH);
      detectionFront = FRONT_RIGHT_SIDE;
    } else {
      detectionFront = NO_SOUND_DETECTED;
    }
    
    detectionResult = detectionFront;
  } else if ((detectionBackVolts > detectionFrontVolts) && (detectionBackVolts > DETECTION_THRESHOLD)) {
    //  Check for sound detection
    if (samples[BACK_LEFT_SIDE].peakToPeakVolts > samples[BACK_RIGHT_SIDE].peakToPeakVolts) {
      digitalWrite(BACK_LEFT_LED, HIGH);
      detectionBack = BACK_LEFT_SIDE;
    } else if (samples[BACK_RIGHT_SIDE].peakToPeakVolts > samples[BACK_LEFT_SIDE].peakToPeakVolts) {
      digitalWrite(BACK_RIGHT_LED, HIGH);
      detectionBack = BACK_RIGHT_SIDE;
    } else {
      detectionBack = NO_SOUND_DETECTED;
    }

    detectionResult = detectionBack;
  }
  
  return detectionResult;
}

/*
    Move a servo by pulse width in ms (500ms - 2500ms)
*/
Servo moveServoPw (Servo servo, int servoPosition, int moveSpeed, int moveTime, boolean term) {
  Servo tServo = servo;
  
  tServo.error = 0;
  
  if ((servoPosition >= tServo.minPulse) && (servoPosition <= tServo.maxPulse)) {
    Serial.print("#");
    Serial.print(tServo.pin);
    Serial.print(" P");
    Serial.print(servoPosition + tServo.offset);

    tServo.msPulse = servoPosition;
    tServo.angle = ((servoPosition - SERVO_CENTER_MS) / 10);
    
    if (tServo.maxDegrees == 180) {
      tServo.angle += 90;
    }
  } else if ((servoPosition < tServo.minPulse) || (servoPosition > tServo.maxPulse)) {
    tServo.error = 200;
  }
 
  if (tServo.error == 0) {
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
  
  return tServo;
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180)
*/
Servo moveServoDegrees (Servo servo, int servoDegrees, int moveSpeed, int moveTime, boolean term) {
  Servo tServo = servo;
  int servoPulse = SERVO_CENTER_MS + servo.offset;

  tServo.error = 0;
  
  //  Convert degrees to ms for the servo move
  if (tServo.maxDegrees == 90) {
    servoPulse = (SERVO_CENTER_MS + tServo.offset) + (servoDegrees * 10);
  } else if (tServo.maxDegrees == 180) {
    servoPulse = (SERVO_CENTER_MS + tServo.offset) + ((servoDegrees - 90) * 10);
  }

  if ((servoPulse >= tServo.minPulse) && (servoPulse <= tServo.maxPulse)) {
    Serial.print("#");
    Serial.print(tServo.pin);
    Serial.print(" P");
    Serial.print(servoPulse);
    tServo.msPulse = (servoDegrees * 10) + SERVO_CENTER_MS;
    tServo.angle = servoDegrees;
    
    if (tServo.maxDegrees == 180) {
      tServo.angle += 90;
    }
  } else if ((servoPulse < tServo.minPulse) || (servoPulse > tServo.maxPulse)) {
    tServo.error = 200;
  }
  
  if (tServo.error == 0) {
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
  
  return tServo;
}

/*
    Called when a request from an I2C (Wire) Master comes in
*/
void wireRequestEvent (void) {
  
}

//  Called when the I2C (Wire) Slave receives data from an I2C (Wire) Master
void wireReceiveData (int nrBytesRead) {

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
    Setup routine - runs just one time
*/
void setup (void) {
  //  Start up the Wire library as a slave device at address 0xE0
  Wire.begin(NAV_I2C_ADDRESS);

  //  Register event handler
  Wire.onRequest(wireRequestEvent);
  Wire.onReceive(wireReceiveData);

  //  Initialize the hardware serial port  
  Serial.begin(115200);
  
  //  Initialize the LED pin as an output.
  pinMode(HEARTBEAT_LED, OUTPUT);
  
  //  Set the LED pins to be outputs  
  pinMode(COLOR_SENSOR_LED, OUTPUT);
  pinMode(FRONT_LEFT_LED, OUTPUT);
  pinMode(FRONT_RIGHT_LED, OUTPUT);
  pinMode(BACK_LEFT_LED, OUTPUT);
  pinMode(BACK_RIGHT_LED, OUTPUT);
 
  //  Test the LEDs
  digitalWrite(FRONT_LEFT_LED, HIGH);
  digitalWrite(FRONT_RIGHT_LED, HIGH);
  digitalWrite(BACK_LEFT_LED, HIGH);
  digitalWrite(BACK_RIGHT_LED, HIGH);

  delay(1000);

  digitalWrite(FRONT_LEFT_LED, LOW);
  digitalWrite(FRONT_RIGHT_LED, LOW);
  digitalWrite(BACK_LEFT_LED, LOW);
  digitalWrite(BACK_RIGHT_LED, LOW);
  digitalWrite(COLOR_SENSOR_LED, LOW);
}

/*
    The loop routine runs forever
*/
void loop (void) {
  byte error = 0;
  byte directionOfSound = 0;

  // Pulse the heartbeat LED
  pulseDigital(HEARTBEAT_LED, 500);

  //  Do sound detection
  directionOfSound = detectSound();
//  error = displaySoundDirection(directionOfSound);

  if (error != 0) {
    processError(error);
  }
}
