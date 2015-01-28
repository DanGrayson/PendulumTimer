// Plug the sensor into pin A5

// constantly read the sensor level and write to SD, once every 2.5 ms

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

static void exit(const char *p, const char *q) {
  row0(p);
  row1(q);
  while (TRUE);
}

class AStar32U4PrimeButtonA buttonA; // won't work with SD card when the jumper is in use
class AStar32U4PrimeButtonB buttonB;
class AStar32U4PrimeButtonC buttonC;

#define chipSelect 4

void setup() { }

void loop() {
  uint32_t loop_counter = 0;
  char buf[30];
  row0("ver 6"), delay(600);
  timer1_setup();
  update_timer();
  if (!SD.begin(chipSelect)) exit("SD card","error");
  File out = SD.open("SCOPE.TXT", FILE_WRITE);
  if (!out) exit("openfile","failed");
  while (TRUE) {
    loop_counter++;
    if ((loop_counter & 0x0f) == 0 && buttonC.getSingleDebouncedPress()) break;
    uint16_t level = analogRead(A5);
    update_timer();
    sprintf(buf,"%lu,%u,%lu\n",loop_counter,level,(long unsigned)timer);
    out.print(buf);
    row0(buf);
    row1(buf+8);
  }
  out.close();
  lcd.clear();
  exit("done",""); }
