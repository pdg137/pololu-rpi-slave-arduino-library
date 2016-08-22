#include <Servo.h>
#include <AStar32U4.h>
#include <PololuRPiSlave.h>
#include "Encoders.h"

/* This example program shows how to make the A-Star 32U4 Robot
 * Controller into a Raspberry Pi I2C slave.  The RPi and A-Star can
 * exchange data bidirectionally, allowing each device to do what it
 * does best: high-level programming can be handled in a language such
 * as Python on the RPi, while the A-Star takes charge of motor
 * control, analog inputs, and other low-level I/O.
 *
 * The example and libraries are available for download at:
 *
 * https://github.com/pololu/pololu-rpi-slave-arduino-library
 *
 * You will need the corresponding Raspberry Pi code, which is
 * available in that repository under the pi/ subfolder.  The Pi code
 * sets up a simple Python-based web application as a control panel
 * for your Raspberry Pi robot.
 */

// Custom data structure that we will use for interpreting the buffer.
// We recommend keeping this under 64 bytes total.  If you change the
// data format, make sure to update the corresponding code in
// a_star.py on the Raspberry Pi.

struct Data
{
  bool yellow, green, red; // bytes 0 1 2
  bool buttonA, buttonB, buttonC; // bytes 3 4 5

  int16_t leftMotor, rightMotor; // bytes 67 89
  uint16_t batteryMillivolts; // bytes 10 11
  uint16_t analog[6]; // bytes 12-23

  bool playNotes; // byte 24
  char notes[14]; // bytes 25-38
  int16_t leftEncoder, rightEncoder; // bytes 39-42
  int16_t leftEncoderErrors, rightEncoderErrors; // bytes 43-46
};

PololuRPiSlave<struct Data,0> slave;
PololuBuzzer buzzer;
AStar32U4Motors motors;
AStar32U4ButtonA buttonA;
AStar32U4ButtonB buttonB;
AStar32U4ButtonC buttonC;

int8_t last_encoder1;
int8_t last_encoder2;

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(20);

  // Play startup sound.
  buzzer.play("v10>>g16>>>c16");

  Encoders::init();
}

void loop()
{
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  // Write various values into the data structure.
  slave.buffer.buttonA = buttonA.isPressed();
  slave.buffer.buttonB = buttonB.isPressed();
  slave.buffer.buttonC = buttonC.isPressed();
  slave.buffer.leftEncoder += (int8_t)(Encoders::count1 - last_encoder1);
  slave.buffer.rightEncoder += (int8_t)(Encoders::count2 - last_encoder2);
  last_encoder1 = Encoders::count1;
  last_encoder2 = Encoders::count2;
  slave.buffer.leftEncoderErrors = Encoders::error1;
  slave.buffer.rightEncoderErrors = Encoders::error2;

  // Change this to readBatteryMillivoltsLV() for the LV model.
  slave.buffer.batteryMillivolts = readBatteryMillivoltsSV();

  for(uint8_t i=0; i<6; i++)
  {
    slave.buffer.analog[i] = analogRead(i);
  }

  // READING the buffer is allowed before or after finishWrites().
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  ledRed(slave.buffer.red);
  motors.setSpeeds(slave.buffer.leftMotor, slave.buffer.rightMotor);

  // Playing music involves both reading and writing, since we only
  // want to do it once.
  if(slave.buffer.playNotes)
  {
    buzzer.play(slave.buffer.notes);
    while(buzzer.isPlaying());
    slave.buffer.playNotes = false;
  }

  // When you are done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();
}
