#include <AStar32U4Prime.h>

  // Here we start experimenting with Timer-Counter 1, which is
  // the one whose values can be captured when the analog
  // comparator triggers.

static AStar32U4PrimeLCD lcd;

void setup() {
  // See table 14-5 on page 123 in the datasheet.  Normally WGM1 is initialized to 0001 by the library.  Here we
  // initialize it to 0000 to get the full 16 bit count, instead of just 8.
  bitWrite(TCCR1B,WGM13,0);
  bitWrite(TCCR1B,WGM12,0);
  bitWrite(TCCR1A,WGM11,0);
  bitWrite(TCCR1A,WGM10,0);

  // See table 14-6.  Here we set the timer source to come from the I/O clock with prescaling by division by 8, which
  // makes it tick at 1/8 of the CPU clock rate of 16 Mhz, i.e., at 2 Mhz.  It would overflow after 2^15 microseconds,
  // which is about 32 milliseconds, or 30 times per second.
#define DIVIDER 8
  bitWrite(TCCR1B,CS12,0);
  bitWrite(TCCR1B,CS11,1);
  bitWrite(TCCR1B,CS10,0);

}

void loop() {
  while (1) {
    char buf[20];
    int level = 1024-analogRead(A0);
    static unsigned tc1;
    tc1 = TCNT1;
    static unsigned long ticks;
    unsigned diff = tc1 - (int)ticks;
    ticks += diff;
    static unsigned long hsec;
    unsigned long last_hsec = hsec;
    hsec = ticks/(F_CPU/DIVIDER/10); // elapsed tenths of seconds
    if (hsec/2 != last_hsec/2) {    // display 5 times per second
      lcd.gotoXY(0,0), sprintf(buf,"%08lu",hsec), lcd.print(buf);
      lcd.gotoXY(0,1), sprintf(buf,"%03d %4u",level,diff/2), lcd.print(buf);
    }
  }
}
