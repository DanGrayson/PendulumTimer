// Plug the sensor into pin A5

// The pendulum swings back and forth, and we assume the sensor detects it twice per
// cycle, so we expect the even ticks to be evenly spaced and we expect the odd ticks to
// be evenly spaced.  Since the sensor is not in the exact center, we don't expect all
// the ticks to be evenly spaced.

// We display several parameters on the LCD, scroll to them with the B and C buttons.

// My A-Star's clock seems to be slow by about 2 seconds in 20 hours.  We
// should add code to compensate for such calibration.

#include <AStar32U4Prime.h>
#include <SPI.h>
#include <SD.h>

#define TRUE 1
#define FALSE 0

#define USE_BANDGAP_REF FALSE    // use the bandgap reference (1.2V) for the comparator, instead of pin AIN0 (which requires RS to be remapped)
#define LCD_RS_PIN 22		// remap LCD:RS to pin 22

#define DIVISOR 8

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

typedef uint64_t counter_t;
static counter_t timer;		// the timer, continuously updated

static void update_timer() {
  uint16_t diff = (uint16_t)TCNT1 - (uint16_t)timer;
  timer += diff;		// the lowest 16 bits of timer always agree with the recently read value of
				// TCNT1, and the higher bits keep track of overflows
}

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

static void row(int r,const char *p) {
  char buf[20];
  sprintf(buf,"%-8s",p);	// the LCD displays 8 characters per row
  lcd.gotoXY(0,r), lcd.print(buf);
}
  
static void row0(const char *p) {
  row(0,p);
}
  
static void row1(const char *p) {
  row(1,p);
}

class AStar32U4PrimeButtonA buttonA; // won't work with SD card when the jumper is in use
class AStar32U4PrimeButtonB buttonB;
class AStar32U4PrimeButtonC buttonC;

#define chipSelect 4

void loop() {
  row0("ver 4");
  delay(600);
  update_timer();
  uint32_t tick_counter = 0, loop_counter = 0;
  char buf[30];
  bitSet(ACSR,ACI);		// clear comparator event flag initially, to ignore some possible noise
  if (!SD.begin(chipSelect))
  {
    static char m[] = "SD card error";
    row0(m), row1(m+8);
    while(TRUE);
  }
  File out = SD.open("events.txt", FILE_WRITE);
  if (!out) {
    static char m[] = "file open failed";
    row0(m), row1(m+8);
    while(TRUE);
  }
  while (TRUE) {
    loop_counter++;
    update_timer();
    if ((loop_counter & 0xff) == 0 && buttonC.getSingleDebouncedPress()) break;
    if (ACSR & bit(ACI)) {
      tick_counter++;
      bitSet(ACSR,ACI);		// clear comparator event flag
      uint16_t input_capture = ICR1;
      update_timer();		// ensure the timer has been updated after the capture of the time of the comparator event
      counter_t this_tick_time = ((input_capture > (uint16_t)timer // whether the timer overflowed after the capture
				   ? ((timer >> 16) - 1)
				   : (timer >> 16)
				   ) << 16) | input_capture;
      sprintf(buf,"%4lu%12lu",tick_counter,(long unsigned)this_tick_time);
      out.println(buf);
      lcd.gotoXY(0,0), lcd.print(buf);
      lcd.gotoXY(0,1), lcd.print(buf+8);
    }}
  out.close();
  lcd.clear();
  row0("done");
  while (TRUE); }
