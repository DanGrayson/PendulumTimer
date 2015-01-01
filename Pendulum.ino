#include <AStar32U4Prime.h>

void setup() {
}

void loop() {
  int sensorValue = analogRead(A0);
  static AStar32U4PrimeLCD lcd;
  lcd.clear();
  lcd.print(1024-sensorValue);
  delay(100);
}
