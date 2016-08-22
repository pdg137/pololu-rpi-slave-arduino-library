// Encoder 1 is connected to Arduino pins 0 & 1 (RX/TX; INT2/3).
// Encoder 2 is connected to Arduino pins 15 & 16 (PB1/2; PCINT1/2).
// Using the PCINT interrupts for encoder2 and INT interrupts for encoder1
// keeps the two interrupt routines shorter and faster.

#include "Arduino.h"
#include "Encoders.h"
#include <AStar32U4.h>

int8_t Encoders::count1;
int8_t Encoders::count2;
uint8_t last11, last12, last21, last22;
uint32_t Encoders::error1 = 0;
uint32_t Encoders::error2 = 0;

void Encoders::init() {
  cli();
  PCMSK0 = (1<<1) | (1<<2); // enable pin change interrupts on PCINT1 and 2 which are Arduino pins 15, 16: PB1 and PB2
  PCICR = 0xff; // turns on pin change interrupts in general
  PCIFR = 0; // clear interrupt flags

  EICRA = (1<<4) | (1<<6); // set INT2 and INT3 to interrupt on all edges
  EIMSK = (1<<2) | (1<<3); // enable INT2 and INT3
  EIFR = 0; // clear interrupt flags
  sei();
}

// Encoder 1 uses INT2 and INT3
ISR(INT2_vect,ISR_ALIASOF(INT3_vect));
ISR(INT3_vect)
{
  ledRed(1);
  uint8_t new11 = ((PIND & (1<<2)) != 0);
  uint8_t new12 = ((PIND & (1<<3)) != 0);

  Encoders::count1 += (last11 ^ new12) - (int)(new11 ^ last12);

  if((last11 ^ new11) & (last12 ^ new12))
    Encoders::error1 ++;

  last11 = new11;
  last12 = new12;
  ledRed(0);
}

// Encoder two uses PCINT1 and PCINT2 (which trigger the PCINT0 interrupt)
ISR(PCINT0_vect)
{
  ledGreen(1);
  uint8_t new21 = ((PINB & (1<<1)) != 0);
  uint8_t new22 = ((PINB & (1<<2)) != 0);

  Encoders::count2 += (last21 ^ new22) - (int)(new21 ^ last22);

  if((last21 ^ new21) & (last22 ^ new22))
    Encoders::error2 ++;

  last21 = new21;
  last22 = new22;
  ledGreen(0);
}
