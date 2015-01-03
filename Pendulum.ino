#include <AStar32U4Prime.h>

  // Here we start experimenting with Timer-Counter 1, which is
  // the one whose values can be captured when the analog
  // comparator triggers.

static AStar32U4PrimeLCD lcd;

void setup() {
  // See table 14-5 on page 123 in the datasheet.  Normally WGM1
  // is initialized to 0001 by the library.  Here we initialize
  // it to 0000 to get the full 16 bit count, instead of just 8.
  bitClear(TCCR1B,WGM13);
  bitClear(TCCR1B,WGM12);
  bitClear(TCCR1A,WGM11);
  bitClear(TCCR1A,WGM10);
}

void loop() {
  int level = 1024-analogRead(A0);
  static unsigned int c, oldc;
  oldc = c;
  cli(); c = TCNT1; sei();
  lcd.clear();
  lcd.print(level);
  lcd.gotoXY(0, 1);
  lcd.print(c);
  Serial.print(level), Serial.print('@'), Serial.print(c), Serial.print('\r'), Serial.print('\n');
  delay(100);
}
