#include <AStar32U4Prime.h>

#define bitCopy(word,from,to) (bitRead(word,from) << to)
#define bitCopy2(word,from1,to1,from2,to2) ((bitRead(word,from1) << to1) | \
					    (bitRead(word,from2) << to2))
#define bitCopy3(word,from1,to1,from2,to2,from3,to3) ((bitRead(word,from1) << to1) | \
						      (bitRead(word,from2) << to2) | \
						      (bitRead(word,from3) << to3))

#define DISPLAY_FREQ 5		// per second
#define DISPLAY_RESOLUTION 100	// per second
#define TRIGGER_LEVEL 500	// 0..1023, 0 is white, 1023 is black.
#define REFRACTORY_PERIOD 700	// milliseconds
#define RESTART_PERIOD 1300 	// milliseconds
#define DIVISOR 8
  // Here we set the timer source to come from the I/O clock with prescaling by
  // division by 8, which makes it tick at 1/8 of the CPU clock rate of 16 Mhz,
  // i.e., at 2 Mhz.  It would overflow after 2^15 microseconds, which is about
  // 33 milliseconds, or 30 times per second.

#define TIMER_FREQ (F_CPU/DIVISOR)

// see Datasheet, table 13-9 or table 14-6, Clock Select Bit Description
#if   DIVISOR == 1
#  define CLOCK_SELECT 1
#elif DIVISOR == 8
#  define CLOCK_SELECT 2
#elif DIVISOR == 64
#  define CLOCK_SELECT 3
#elif DIVISOR == 256
#  define CLOCK_SELECT 4
#elif DIVISOR == 1024
#  define CLOCK_SELECT 5
#endif

// Waveform Generation Mode;
  // See table 14-5 on page 123 in the datasheet.  Normally WGM1 is initialized to 0001 by the
  // library.  Here we initialize it to 0000 to get the full 16 bit count, instead of just 8.
#define WGM1 0b0000

#define TRUE 1
#define FALSE 0

void setup() {
  TCCR1A =			// Timer/Counter1 Control Register A
    bitCopy2(WGM1,1,WGM11,0,WGM10);
  TCCR1B =			// Timer/Counter1 Control Register B
    bitCopy2(WGM1,3,WGM13,2,WGM12) |
    bitCopy3(CLOCK_SELECT,2,CS12,1,CS11,0,CS10) ;
  ADMUX =			// ADC Multiplexer Selection Register
    0;				//   ADC0 from the ADC MUX.  This will break analogRead().
  ADCSRA =			// ADC Control and Status Register A
    bit(ADATE);			//   ADC Auto Trigger Enable
  ADCSRB =			// ADC Control and Status Register B
    bit(ACME);	                //   Analog Comparator Multiplexer Enable, free running mode
  ACSR =			// Analog Comparator Control and Status Register
    bit(ACIC) |			//   Input Capture Enable
    bit(ACBG) |			//   Analog Comparator Bandgap Select (pin AIN0 is already in use on the A*')
    bit(ACI) |			//   clear the Analog Comparator Interrupt Flag
    bit(ACIS1) | bit(ACIS0);	//   Comparator Interrupt on Rising Output Edge
}

#define TICKS_PER_MS (TIMER_FREQ/1000)
#define TICKS_PER_HOUR (60L*60L*TIMER_FREQ)

static unsigned long display_timer(unsigned long timer) {
  return timer/(TIMER_FREQ/DISPLAY_RESOLUTION);
}

void loop() {
  while (1) {
    static unsigned long timer;
    unsigned diff = TCNT1 - (unsigned)timer;
    timer += diff;
    static unsigned int tick_counter;
    static unsigned long first_tick_time, second_tick_time, previous_tick_time, previous_odd_tick_time, previous_even_tick_time;
    if (ACSR & bit(ACI)) {
      bitSet(ACSR,ACI);
      unsigned int input_capture = ICR1;
      unsigned long this_tick_time = input_capture <= (unsigned int) timer
	? (((timer >> 16)  ) << 16) | input_capture
	: (((timer >> 16)-1) << 16) | input_capture;
      unsigned long diff_timer = this_tick_time - previous_tick_time;
      if (diff_timer > RESTART_PERIOD * TICKS_PER_MS) {
	// reinitialize if we miss a tick
	tick_counter = 0;
	first_tick_time = previous_tick_time = previous_odd_tick_time = previous_even_tick_time = 0;
      }
      if (diff_timer > REFRACTORY_PERIOD * TICKS_PER_MS) {
	tick_counter++;
	previous_tick_time = this_tick_time;
	if (tick_counter == 1) first_tick_time = this_tick_time;
	else if (tick_counter == 2) second_tick_time = this_tick_time;
	if (tick_counter & 2) previous_odd_tick_time = this_tick_time;
	else previous_even_tick_time = this_tick_time;
      }
    }
    static unsigned long previous_display_time;
    if (timer - previous_display_time > TIMER_FREQ/DISPLAY_FREQ) {
      static AStar32U4PrimeLCD lcd;
      previous_display_time = timer;
      unsigned long tick_rate = // in ticks per hundred hours
	tick_counter >= 4
	? 60.			// minutes per hour
	* 60.			// seconds per minute
	* TIMER_FREQ		// timer counts per second
	* 100.			// number of hours
	/ (
	   // The pendulum swings back and forth, so we expect the even ticks to be evenly spaced and we expect the odd
	   // ticks to be evenly spaced.  Since the sensor is not in the exact center, we don't expect all the ticks to
	   // be evenly spaced.  Here we average the average spacing between even ticks with the average spacing
	   // between odd ticks.  This formula starts working with the 4th tick.
	   (float)((previous_odd_tick_time-first_tick_time)/((tick_counter-1)/2) +
		   (previous_even_tick_time-second_tick_time)/((tick_counter-2)/2))/2
	   / 2			// the spacing between odd ticks (or even ticks) is twice the tick rate, so divide by 2
	   )
	: 0.;
      char buf[20];
      lcd.gotoXY(0,0),
	sprintf(buf,"%8u",tick_counter),
	lcd.print(buf);
      lcd.gotoXY(0,1),
	sprintf(buf,"%5lu.%02lu",tick_rate/100,tick_rate%100), // print the rate in the form of ticks per hour, as a decimal fraction
	lcd.print(buf);
    }
  }
}
