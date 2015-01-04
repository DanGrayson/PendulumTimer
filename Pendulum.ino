#include <AStar32U4Prime.h>

  // Here we start experimenting with Timer-Counter 1, which is
  // the one whose values can be captured when the analog
  // comparator triggers.

static AStar32U4PrimeLCD lcd;

#define DISPLAY_FREQ 5		// per second
#define DISPLAY_RESOLUTION 100	// per second
#define TRIGGER_LEVEL 500	// 0..1023, 0 is white, 1023 is black.

void setup() {
  // See table 14-5 on page 123 in the datasheet.  Normally WGM1 is initialized to 0001 by the
  // library.  Here we initialize it to 0000 to get the full 16 bit count, instead of just 8.
  bitWrite(TCCR1B,WGM13,0);
  bitWrite(TCCR1B,WGM12,0);
  bitWrite(TCCR1A,WGM11,0);
  bitWrite(TCCR1A,WGM10,0);

  // See table 14-6.  Here we set the timer source to come from the I/O clock with prescaling by
  // division by 8, which makes it tick at 1/8 of the CPU clock rate of 16 Mhz, i.e., at 2 Mhz.  It
  // would overflow after 2^15 microseconds, which is about 32 milliseconds, or 30 times per second.
# define TICK_FREQ (F_CPU/8)
  bitWrite(TCCR1B,CS12,0);
  bitWrite(TCCR1B,CS11,1);
  bitWrite(TCCR1B,CS10,0);

  // Set up ADC MUX.  This will break analogRead().
  ADMUX = 0,		// ADC0 from the ADC MUX.
    ADCSRA = (1<<ADATE),	// ADC Auto Trigger Enable
    ADCSRB = (1<<ACME)		// Analog Comparator Multiplexer Enable, free running mode
    ;

  // Set up analog comparator:
  ACSR = (1<<ACIC)		// Input Capture Enable
    | (1<<ACBG)			// Analog Comparator Bandgap Select (pin AIN0 is already in use on
				// the A*')
    | (1<<ACI)			// clear the Analog Comparator Interrupt Flag
    | ((1<<ACIS0)|(1<<ACIS1))	// Comparator Interrupt on Rising Output Edge
    ;

}

void loop() {
  while (1) {
    static unsigned long tick;
    unsigned diff = TCNT1 - (unsigned)tick;
    tick += diff;
    static unsigned long last_display_tick;
    static unsigned aci_counter;
    static unsigned long last_aci_tick;
    if (ACSR & (1<<ACI)) {
      aci_counter++;
      bitSet(ACSR,ACI);
      unsigned int input_capture = ICR1;
      last_aci_tick = input_capture <= (unsigned) tick
	? (((tick >> 16)  ) << 16) | input_capture
	: (((tick >> 16)-1) << 16) | input_capture;
    }
    if (tick - last_display_tick > TICK_FREQ/DISPLAY_FREQ) {
      last_display_tick = tick;
      char buf[20];
      lcd.gotoXY(0,0),
	sprintf(buf,"%8lu",tick/(TICK_FREQ/DISPLAY_RESOLUTION)),
	lcd.print(buf);
      lcd.gotoXY(0,1),
	sprintf(buf,"%2u %5lu",aci_counter,last_aci_tick/(TICK_FREQ/DISPLAY_RESOLUTION)),
	lcd.print(buf);
    }
  }
}
