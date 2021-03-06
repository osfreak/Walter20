#include <BMSerial.h>
#include <RoboClaw.h>

#include "PacketSerialEncoderSpeedAccelDistance.h"

#define address 0x80

#define Kp 0x00010000
#define Ki 0x00008000
#define Kd 0x00004000
#define qpps 44000

#define  ONOFF_TIME_MS  750

BMSerial console(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN);

//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

RoboClaw roboClaw(SERIAL_ROBOCLAW_RX_PIN, SERIAL_ROBOCLAW_TX_PIN, 10000, false);

/*
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
	digitalWrite(pin, HIGH);			// Turn the ON by making the voltage HIGH (5V)
	delay(duration);					// Wait for duration ms
	digitalWrite(pin, LOW);				// Turn the pin OFF by making the voltage LOW (0V)
	delay(duration);					// Wait for duration ms
}

void displayspeed (void) {
  uint8_t rStatus;
  bool valid;
  uint32_t enc1, speed1, enc2, speed2;
  
  enc1 = roboClaw.ReadEncM1(address, &rStatus, &valid);

  if (valid) {
    console.print("Encoder1: ");
    console.print(enc1, DEC);
    console.print(", Status: ");
    console.print(rStatus, HEX);
    console.print(" ");
  }

  enc2 = roboClaw.ReadEncM2(address, &rStatus, &valid);

  if (valid) {
    console.print("Encoder2: ");
    console.print(enc2, DEC);
    console.print(", Status: ");
    console.print(rStatus, HEX);
    console.print(" ");
  }

  speed1 = roboClaw.ReadSpeedM1(address, &rStatus, &valid);

  if (valid) {
    console.print("Speed1: ");
    console.print(speed1, DEC);
    console.print(" ");
  }
  
  speed2 = roboClaw.ReadSpeedM2(address, &rStatus, &valid);

  if (valid) {
    console.print("Speed2: ");
    console.print(speed2, DEC);
    console.print(" ");
  }
  
  console.println();
}

void setup() {
  console.begin(115200);
  roboClaw.begin(38400);
  
  pinMode(HEARTBEAT_LED, OUTPUT);
  
  roboClaw.SetM1VelocityPID(address, Kd, Kp, Ki, qpps);
  roboClaw.SetM2VelocityPID(address, Kd, Kp, Ki, qpps);  
}

void loop() {
  uint8_t depth1,depth2;

  pulseDigital(HEARTBEAT_LED, ONOFF_TIME_MS);
  
  roboClaw.SpeedAccelDistanceM1(address, 12000, 12000, 48000);

  do {
    displayspeed();
    roboClaw.ReadBuffers(address ,depth1, depth2);
  } while (depth1);

  roboClaw.SpeedAccelDistanceM1(address, 12000, -12000, 48000);

  do {
    displayspeed();
    roboClaw.ReadBuffers(address, depth1, depth2);
  } while (depth1);
}
