// Plug the sensor into pin A5

// The pendulum swings back and forth, and we assume the sensor detects it twice per
// cycle, so we expect the even ticks to be evenly spaced and we expect the odd ticks to
// be evenly spaced.  Since the sensor is not in the exact center, we don't expect all
// the ticks to be evenly spaced.

// We display several parameters on the LCD, scroll to them with the B and C buttons.

#include <AStar32U4Prime.h>

#define TRUE 1
#define FALSE 0

#define DIVISOR 8
// Here we set the timer source to come from the I/O clock with prescaling by
// division by 8, which makes it tick at 1/8 of the CPU clock rate of 16 Mhz,
// i.e., at 2 Mhz.  It would overflow after 2^15 microseconds, which is about
// 33 milliseconds, or 30 times per second.
#define TIMER_FREQ (F_CPU/DIVISOR)
#define SECOND TIMER_FREQ
#define MILLISECOND (SECOND/1000)
#define MINUTES_PER_HOUR 60
#define SECONDS_PER_MINUTE 60
#define MINUTE (SECONDS_PER_MINUTE*SECOND)
#define HOUR (MINUTES_PER_HOUR*(uint64_t)MINUTE)

static uint64_t quot(uint64_t x,uint64_t y) {
  return (x+y/2)/y;	// rounded integer quotient
}

// start user configuration section
#define USE_BANDGAP_REF FALSE // use the bandgap reference (1.2V) for the comparator, instead of pin AIN0 (which requires RS to be remapped)
#define LCD_RS_PIN 22		// remap LCD:RS to pin 22
#define TICKS_PER_MINUTE 156    // my Ansonia mantle clock has 156 ticks per minute, but my tall case clock has 64
// end user configuration section

#ifdef TICKS_PER_MINUTE
#define TICK_PERIOD quot(MINUTE,TICKS_PER_MINUTE)
#else
#define TICK_PERIOD quot(HOUR,TICKS_PER_HOUR)
#endif
#define TICKS_PER_CYCLE 2
#define CYCLE (TICKS_PER_CYCLE*TICK_PERIOD)
#define TOLERANCE (50*MILLISECOND) // milliseconds; restart the count if the tolerance is not met

// To remap LCD:RS to another pin requires cutting a surface mount jumper on the board and soldering a wire into the RS hole near the LCD connector.
#ifndef LCD_RS_PIN
#define LCD_RS_PIN 7
#endif

// Improvements needed:
//   determine REFRACTORY_PERIOD and RESTART_PERIOD dynamically

#define DISPLAY_RESOLUTION 100		// per second

class AStar32U4PrimeLCD_remapped : public PololuHD44780Base
{
  static const uint8_t rs = LCD_RS_PIN, e = 8, db4 = 14, db5 = 17, db6 = 13, db7 = IO_D5;
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

static bool display_needed = TRUE;

typedef uint64_t counter_t;
static counter_t timer;		// the timer, continuously updated
static uint16_t clock_hour;
static uint8_t clock_minute, clock_second;
static uint16_t clock_millisecond;
static uint32_t clock_counter;

static void reset_clock() {
  clock_hour =
    clock_minute =
    clock_second =
    clock_millisecond =
    clock_counter = 0;
}

static uint16_t max_timer_diff;

static void update_timer() {
  uint16_t diff = (uint16_t)TCNT1 - (uint16_t)timer;
  if (diff > max_timer_diff) max_timer_diff = diff;
  timer += diff;		// the lowest 16 bits of timer always agree with the recently read value of
				// TCNT1, and the higher bits keep track of overflows
  clock_counter += diff;
  while (clock_counter >= MILLISECOND) {
    clock_counter -= MILLISECOND, clock_millisecond++;
    if (clock_millisecond % 500 == 0) display_needed = TRUE; // display something every 500ms
    while (clock_millisecond >= 1000) {
      clock_millisecond -= 1000, clock_second++;
      while (clock_second >= SECONDS_PER_MINUTE) {
	clock_second -= SECONDS_PER_MINUTE, clock_minute++;
	while (clock_minute >= MINUTES_PER_HOUR) {
	  clock_minute -= MINUTES_PER_HOUR, clock_hour++; } } } } }

static void timer1_setup() {
  TCCR1A =			// Timer/Counter1 Control Register A
    bitCopy2(WGM1,1,WGM11,0,WGM10);
  TCCR1B =			// Timer/Counter1 Control Register B
    bitCopy2(WGM1,3,WGM13,2,WGM12) |
    bitCopy3(CLOCK_SELECT,2,CS12,1,CS11,0,CS10) ;
  TCNT1 = 0;
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
    bit(ACIS1);			//   Comparator Interrupt on Falling Output Edge (could be important: this way we get
				//   the leading edge of the pendulum rod, for as the brightness increases,
				//   the voltage decreases.  If we try to detect the trailing edge we may
				//   occasionally be deceived by noise as the leading edge passes by.)
}

void setup() {
  // init() in Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/wiring.c
  // is called before setup() is called, to initialize all the ports
  timer1_setup();
  adc_setup();
  ac_setup();			// it's important to start the comparator after timer1 is started
}

static int analogReadMUX(uint8_t mux) // the library code handles reading only from ADC0,...,ADC7
{
  // this can interfere with the detect of the pendulum, perhaps restart?
  uint8_t saveA = ADCSRA, saveB = ADCSRB, saveMUX = ADMUX;
  uint8_t low, high;
  ADMUX =
    bit(REFS0) |		// AV_CC with external capacitor on AREF pin
    bitClear(mux,5);
  ADCSRB =
    bitRead(mux,5) << MUX5;
  ADCSRA =				   // Analog Comparator Control and Status Register
    bit(ADEN) |				   //   ADC Enable
    bit(ADSC) |				   //   ADC Start Conversion
    bit(ADPS2)|bit(ADPS1)|bit(ADPS0);	   //   ADC Prescaler Select Bits (divide clock by 128), see init()
  while (bitRead(ADCSRA, ADSC));
  low  = ADCL;
  high = ADCH;
  ADMUX = saveMUX, ADCSRB = saveB, ADCSRA = saveA;
  return (high << 8) | low;
}

// static int analogRead2(uint8_t pin) {
//   ADCSRA =				   // Analog Comparator Control and Status Register
//     bit(ADEN)				   //   ADC Enable
//     | bit(ADPS2)|bit(ADPS1)|bit(ADPS0);	   //   ADC Prescaler Select Bits (divide clock by 128), see init()
//   int n = analogRead(pin);
//   adc_setup();
//   return n;
// }

#define PREC 10		    // should be even
#define SKIP_TICKS 2	    // Ignoring the first event is a good idea, since it may not correspond to the
			    // leading edge of the pendulum rod.  Ignoring one other event seems also to help.

class AStar32U4PrimeButtonA buttonA;
class AStar32U4PrimeButtonB buttonB;
class AStar32U4PrimeButtonC buttonC;

static uint64_t square_prec(uint64_t n) {
  n >>= PREC/2;
  return n*n;
}

static uint32_t sqrt64(uint64_t x) {
  // adapted from http://freaknet.org/martin/tape/gos/misc/personal/msc/sqrt/sqrt.c
  // Square root by abacus algorithm, Martin Guy @ UKC, June 1985.
  // From a book on programming abaci by Mr C. Woo.
  uint64_t p = 1LL << (64-2);
  uint32_t res = 0;
  while (p > x) p >>= 2;
  while (p) {
    if (x >= res + p) x -= res + p, res += 2 * p;
    res >>= 1;
    p >>= 2;
  }
  return res;
}

// static int32_t abs32(int32_t x) {
//   return x<0 ? -x : x;
// }

static void row0(const char *p) {
  char buf[20];
  sprintf(buf,"%-8s",p);	// the LCD displays 8 characters per row
  lcd.gotoXY(0,0), lcd.print(buf);
}

static void blank_row1() {
  lcd.print("      --");
}

static void display_millis_time(uint64_t x, uint64_t y) {
  lcd.gotoXY(0,1);
  if (y == 0) blank_row1();
  else {
    char buf[20];
    uint32_t r = quot(x*100, y*MILLISECOND);
    sprintf(buf,"%3lu.%02lums",r/100,r%100);
    lcd.print(buf); } }

void loop() {
  static uint16_t reset_counter;
  while (TRUE) {
    update_timer();
    max_timer_diff = 0;
    reset_clock();
    uint32_t tick_counter = 0, cycle_counter = 0;
    static int32_t deviation;	// deviation of cycle length from expected cycle length
    int32_t max_deviation = 0;	// maximum deviation
    int32_t min_deviation = 0;	// minimum deviation
    counter_t
      cycle_sum = 0,		    // sum of the cycle timings (a cycle is two ticks, or one complete pendulum cycle)
      cycle_square_sum = 0,	    // sum of the squares of the cycle timings, shifted right by PREC bits
      last_tick_time[2] = {0,0};    // last_tick_time[0] is the previous tick time, and
				    // last_tick_time[1] is the one before that
    char buf[20];
    bitSet(ACSR,ACI);		// clear comparator event flag initially, to ignore some possible noise
    while (TRUE) {
      update_timer();
      if (ACSR & bit(ACI)) {
	bitSet(ACSR,ACI);		// clear comparator event flag
	uint16_t input_capture = ICR1;
	update_timer();		// ensure the timer has been updated after the capture of the time of the comparator event
	counter_t this_tick_time = ((input_capture > (uint16_t)timer // whether the timer overflowed after the capture
				     ? ((timer >> 16) - 1)
				     : (timer >> 16)
				     ) << 16) | input_capture;
	// now we know that this_tick_time <= timer, as it must be
	if (tick_counter == 0) {
	  max_timer_diff = 0;	// for some reason I always get a constant big timer difference, such as 16993, so ignore it
	  tick_counter++;
	  last_tick_time[0] = this_tick_time;
	  display_needed = TRUE; }
	else {
	  counter_t this_tick_length = this_tick_time - last_tick_time[0];
	  if (this_tick_length < 250 * MILLISECOND) continue; // allow entire pendulum rod to pass by, and ignore noisy nearby events
	  if (tick_counter == 1) {
	    tick_counter++;
	    last_tick_time[1] = last_tick_time[0];
	    last_tick_time[0] = this_tick_time;
	    reset_clock();
	    display_needed = TRUE; }
	  else {
	    counter_t this_cycle_length = this_tick_time - last_tick_time[1];
	    if (this_cycle_length < CYCLE - TOLERANCE) continue; // ignore spurious events
	    if (this_cycle_length > CYCLE + TOLERANCE) break; // restart if no tick occurs during the predicted period
	    tick_counter++;				   // count the tick
	    last_tick_time[1] = last_tick_time[0];
	    last_tick_time[0] = this_tick_time;
	    display_needed = TRUE;
	    deviation = this_cycle_length - CYCLE;
	    if (deviation > max_deviation) max_deviation = deviation;
	    if (deviation < min_deviation) min_deviation = deviation;
	    if (tick_counter > 2 + SKIP_TICKS) { // count the cycle
	      cycle_counter++;
	      cycle_sum += this_cycle_length;
	      cycle_square_sum += square_prec(this_cycle_length); } } } }
      static uint8_t looper;
      looper++;
      const int num_screens = 13;
      static uint8_t current_screen = 0;
      if (looper == 11 && buttonA.getSingleDebouncedPress()) break;
      if (looper == 33 && buttonB.getSingleDebouncedPress()) current_screen = (current_screen+num_screens-1)%num_screens, display_needed = TRUE;
      if (looper == 99 && buttonC.getSingleDebouncedPress()) current_screen = (current_screen            +1)%num_screens, display_needed = TRUE;
      if (display_needed) {
	display_needed = FALSE;
	switch (current_screen) {
	  case 0: {
	    row0("tick/hr");
	    uint32_t tick_rate = cycle_counter > 0 ? quot(100*HOUR*TICKS_PER_CYCLE*cycle_counter, cycle_sum) : 0;
	    lcd.gotoXY(0,1), sprintf(buf,"%5lu.%02lu",tick_rate/100,tick_rate%100), lcd.print(buf);
	    break;
	  }
	  case 1: {
	    row0("tick/min");
	    uint32_t tick_rate = cycle_counter > 0 ? quot(10000*(uint64_t)MINUTE*TICKS_PER_CYCLE*cycle_counter, cycle_sum) : 0;
	    lcd.gotoXY(0,1), sprintf(buf,"%3lu.%04lu",tick_rate/10000,tick_rate%10000), lcd.print(buf);
	    break; }
	  case 2: {
	    row0("tick len");
	    display_millis_time(cycle_sum, cycle_counter*TICKS_PER_CYCLE);
	    break; }
	  case 3: {
	    row0("std dev");
	    if (cycle_counter > 0) {
	      uint64_t cycle_mean = quot(cycle_sum,cycle_counter*(uint64_t)MILLISECOND);
	      uint64_t cycle_mean_square = square_prec(cycle_mean);
	      uint64_t cycle_square_mean = quot(cycle_square_sum,cycle_counter);
	      uint64_t diff = cycle_square_mean - cycle_mean_square;
	      display_millis_time((uint64_t)sqrt64(diff) << (PREC/2), cycle_counter*TICKS_PER_CYCLE); }
	    else blank_row1();
	    break; }
	  case 4: {
	    row0("max dev");
	    lcd.gotoXY(0,1), sprintf(buf,"%+8ld",max_deviation), lcd.print(buf);
	    break; }
	  case 5: {
	    row0("min dev");
	    lcd.gotoXY(0,1), sprintf(buf,"%+8ld",min_deviation), lcd.print(buf);
	    break; }
	  case 6: {
	    row0("prev dev");
	    lcd.gotoXY(0,1), sprintf(buf,"%+8ld",deviation), lcd.print(buf);
	    break; }
	  case 7: {
	    row0("tick #");
	    lcd.gotoXY(0,1), sprintf(buf,"%8lu",tick_counter), lcd.print(buf);
	    break; }
	  case 8: {
	    row0("batch #");
	    lcd.gotoXY(0,1), sprintf(buf,"%8u",reset_counter), lcd.print(buf);
	    break; }
	  case 9: {
	    row0("time");
	    lcd.gotoXY(0,1), sprintf(buf,"%02u:%02u:%02u",clock_hour,clock_minute,clock_second), lcd.print(buf);
	    break; }
	  case 10: {
	    row0("max diff");
	    lcd.gotoXY(0,1), sprintf(buf,"%8u",max_timer_diff), lcd.print(buf);
	    // max_timer_diff = 0;
	    break; }
	  case 11: {
	    row0("light");
	    lcd.gotoXY(0,1), sprintf(buf,"%8u",1024-analogReadMUX(0 /* ADC0, A5 */ )), lcd.print(buf);
	    break; }
	  case 12: {
	    static uint16_t vmin = 1023, vmax = 0;
	    static uint8_t n;
	    if (++n >= 2) n=0, vmin = 1023, vmax = 0;
	    uint16_t v;
	    uint32_t vsum = 0;
	    uint8_t k;
	    for (k=0;k<40;k++) {
	      v = analogReadMUX(0b011110 /* V band gap */);
	      vsum += v;
	      if (v < vmin) vmin = v;
	      if (v > vmax) vmax = v;
	    }
	    lcd.gotoXY(0,0), sprintf(buf,"BG  %4u",(int)quot(vsum,k)), lcd.print(buf);
	    lcd.gotoXY(0,1), sprintf(buf,"%4u%4u",vmin,vmax), lcd.print(buf);
	    break; }
	} } }
    reset_counter++; } }
