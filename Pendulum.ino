#include <AStar32U4Prime.h>

void setup() {
}

void loop() {
  int level = 1024-analogRead(A0);
  static AStar32U4PrimeLCD lcd;
  lcd.clear();
  lcd.print(level);
  Serial.print(level), Serial.print(' ');
  delay(100);
}
