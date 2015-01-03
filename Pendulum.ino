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
  // which is about 32 milliseconds.  
  bitWrite(TCCR1B,CS12,0);
  bitWrite(TCCR1B,CS11,1);
  bitWrite(TCCR1B,CS10,0);

}

void loop() {
  int level = 1024-analogRead(A0);
  static unsigned int c, oldc;
  oldc = c;
  cli(); c = TCNT1; sei();
  static unsigned long oldtime, time;
  oldtime = time;
  time = millis();
  lcd.clear();
  lcd.print(level);
  // if (Serial) lcd.print(" S");
  lcd.gotoXY(0, 1);
  lcd.print(c);
  // if (Serial)			// this conversion has a 10msec delay!
    // This printing is surprisingly slow if the serial port
    // isn't connected; that's why we test first.
    Serial.print(level), Serial.print(' '), Serial.print(c), Serial.print(' '), Serial.print(1000 * (double) (c-oldc) / (double)  (time-oldtime)), Serial.print('\r'), Serial.print('\n');
  delay(27);			// if the delay is too long, the counter may overflow; here we set it not quite so large
}
