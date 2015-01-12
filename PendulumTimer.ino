#include <AStar32U4Prime.h>

#define TRUE 1
#define FALSE 0

// start user configuration section
#define REMAP_RS_TO_PIN 22		// LCD:RS is usually on pin 7
#define TICK_PERIOD 385			// my Ansonia mantle clock has 384.615 ms between ticks
#define REFRACTORY_PERIOD (TICK_PERIOD-100)	// milliseconds after each tick to ignore new events
#define RESTART_PERIOD    (TICK_PERIOD+100)	// milliseconds after which to declare a missed tick, so restart the count
#define USE_BANDGAP_REF FALSE		// use the bandgap reference (1.2V) for the comparator, instead of pin AIN0 (which requires RS to be remapped)
// end user configuration section

// To remap LCD:RS to another pin requires cutting a surface mount jumper on the board and soldering a wire into the RS hole near the LCD connector.

// Improvements needed:
//   determine REFRACTORY_PERIOD and RESTART_PERIOD dynamically

#define MAX_DISPLAY_FREQ 5		// per second
#define DISPLAY_RESOLUTION 100		// per second

class AStar32U4PrimeLCD_remapped : public PololuHD44780Base
{
  static const uint8_t rs =
#ifdef REMAP_RS_TO_PIN
    REMAP_RS_TO_PIN
#else
    7
#endif    
    , e = 8, db4 = 14, db5 = 17, db6 = 13, db7 = IO_D5;
  void sendNibble(uint8_t data) {
    FastGPIO::Pin<db4>::setOutput(data >> 0 & 1);
    FastGPIO::Pin<db5>::setOutput(data >> 1 & 1);
    FastGPIO::Pin<db6>::setOutput(data >> 2 & 1);
    FastGPIO::Pin<db7>::setOutput(data >> 3 & 1);
    FastGPIO::Pin<e>::setOutputHigh();
    _delay_us(1);   
    FastGPIO::Pin<e>::setOutputLow();
    _delay_us(1);   
  }
public:
  virtual void initPins() {
    FastGPIO::Pin<e>::setOutputLow();
  }
  virtual void send(uint8_t data, bool rsValue, bool only4bits) {
    SPIPause spiPause;
    USBPause usbPause;
    FastGPIO::PinLoan<rs> loanRS;
    FastGPIO::PinLoan<db4> loanDB4;
    FastGPIO::PinLoan<db5> loanDB5;
    FastGPIO::PinLoan<db6> loanDB6;
    FastGPIO::PinLoan<db7> loanDB7;
    FastGPIO::Pin<rs>::setOutput(rsValue);
    if (!only4bits) { sendNibble(data >> 4); }
    sendNibble(data & 0x0F);
  }
};
static AStar32U4PrimeLCD_remapped lcd;

#define bitCopy(word,from,to) (bitRead(word,from) << to)
#define bitCopy2(word,from1,to1,from2,to2) ((bitRead(word,from1) << to1) | \
					    (bitRead(word,from2) << to2))
#define bitCopy3(word,from1,to1,from2,to2,from3,to3) ((bitRead(word,from1) << to1) | \
						      (bitRead(word,from2) << to2) | \
						      (bitRead(word,from3) << to3))

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

static void timer1_setup() {
  TCCR1A =			// Timer/Counter1 Control Register A
    bitCopy2(WGM1,1,WGM11,0,WGM10);
  TCCR1B =			// Timer/Counter1 Control Register B
    bitCopy2(WGM1,3,WGM13,2,WGM12) |
    bitCopy3(CLOCK_SELECT,2,CS12,1,CS11,0,CS10) ;
}

static void adc_setup() {
  ADMUX =			// ADC Multiplexer Selection Register
    0;				//   ADC0 from the ADC MUX.
  ADCSRA =			// ADC Control and Status Register A
				//   no ADEN - ADC Enable
    0;
  ADCSRB =			// ADC Control and Status Register B
    bit(ACME);	                //   Analog Comparator Multiplexer Enable, free running mode
}

static void ac_setup() {
  ACSR =			// Analog Comparator Control and Status Register
    bit(ACIC) |			//   Input Capture Enable
#if USE_BANDGAP_REF
    bit(ACBG) |			//   Analog Comparator Bandgap Select (pin 7 is in use for AIN0 to the CPU and for RS to the LCD, but RS can be remapped)
#endif
    bit(ACI) |			//   clear the Analog Comparator Interrupt Flag
    bit(ACIS1) | bit(ACIS0);	//   Comparator Interrupt on Rising Output Edge
}

void setup() {
  // init() in Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring.c
  // is called before setup() is called, to initialize all the ports
  timer1_setup();
  adc_setup();
  ac_setup();
}

int analogueRead(uint8_t pin) {
  ADCSRA =				   // Analog Comparator Control and Status Register
    bit(ADEN)				   //   ADC Enable
    | bit(ADPS2)|bit(ADPS1)|bit(ADPS0);	   //   ADC Prescaler Select Bits (divide clock by 128)
  int n = analogRead(pin);
  adc_setup();
  return n;
}

#define TICKS_PER_MS (TIMER_FREQ/1000)
#define TICKS_PER_HOUR (60L*60L*TIMER_FREQ)

typedef uint64_t counter_t;

void loop() {
  static bool display_needed = TRUE;
  static uint16_t reset_counter;
  static uint32_t tick_counter;
  static counter_t timer = TCNT1, // initialize the timer so the refractory period is in effect at the start
    first_tick_time, second_tick_time,
    previous_tick_time, previous_odd_tick_time, previous_even_tick_time;
  while (1) {
    timer += (uint16_t)TCNT1 - (uint16_t)timer;
    if (ACSR & bit(ACI)) {
      bitSet(ACSR,ACI);
      uint16_t input_capture = ICR1;
      counter_t this_tick_time = input_capture <= (uint16_t) timer
	? (((timer >> 16)  ) << 16) | input_capture
	: (((timer >> 16)-1) << 16) | input_capture;
      counter_t diff_timer = this_tick_time - previous_tick_time;
      if (diff_timer > RESTART_PERIOD * TICKS_PER_MS) {
	reset_counter++;
	display_needed = TRUE;
	tick_counter =
	  first_tick_time = previous_tick_time =
	  previous_odd_tick_time = previous_even_tick_time = 0;
      }
      if (diff_timer > REFRACTORY_PERIOD * TICKS_PER_MS) {
	tick_counter++;
	display_needed = TRUE;
	previous_tick_time = this_tick_time;
	if (tick_counter == 1) first_tick_time = this_tick_time;
	else if (tick_counter == 2) second_tick_time = this_tick_time;
	if (tick_counter & 1) previous_odd_tick_time = this_tick_time;
	else previous_even_tick_time = this_tick_time;
      }
    }
    static counter_t previous_display_time;
    if (display_needed && timer - previous_display_time > TIMER_FREQ/MAX_DISPLAY_FREQ) {
      display_needed = FALSE;
      previous_display_time = timer;
      uint32_t tick_rate = // in ticks per hundred hours
	tick_counter >= 4
	? 60.			// minutes per hour
	* 60.			// seconds per minute
	* TIMER_FREQ		// timer counts per second
	* 100.			// number of hours
	/ (
	   // The pendulum swings back and forth, and we assume the sensor detects it twice per
	   // cycle, so we expect the even ticks to be evenly spaced and we expect the odd ticks to
	   // be evenly spaced.  Since the sensor is not in the exact center, we don't expect all
	   // the ticks to be evenly spaced.  Here we average the average spacing between even ticks
	   // with the average spacing between odd ticks.  This formula starts working with the 4th
	   // tick.
	   ((float)(previous_odd_tick_time-first_tick_time)/((tick_counter-1)/2) +
	    (float)(previous_even_tick_time-second_tick_time)/((tick_counter-2)/2))/2
	   / 2			// the spacing between odd ticks (or even ticks) is twice the tick rate, so divide by 2
	   )
	: 0.;
      char buf[20];
      lcd.gotoXY(0,0),
	sprintf(buf,"%2u:%5lu",reset_counter,tick_counter),
	lcd.print(buf);
      lcd.gotoXY(0,1),
	// sprintf(buf,"%4u",analogueRead(A5));
	sprintf(buf,"%5lu.%02lu",tick_rate/100,tick_rate%100), // print the rate in the form of ticks per hour, as a decimal fraction
	lcd.print(buf);
    }
  }
}
