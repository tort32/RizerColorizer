#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stddef.h>

// Helper macros
#define setbits(port,mask)	(port)|=(mask)
#define clrbits(port,mask)	(port)&=~(mask)
#define tglbits(port,mask)	(port)^=(mask)
#define wrtbits(port,bits,mask) (port)=((port)&(~(mask)))|((bits)&(mask))

#define getbits(port,mask) ((port)&(mask))
#define is_bits(port,mask) (((port)&(mask))!=0)
#define no_bits(port,mask) (((port)&(mask))==0)

#define HI(wd) ((BYTE)((0xff00&wd)>>8))
#define LO(wd) ((BYTE)(0x00ff&wd))

#define CHK_BUT(pin,mask) ((~(pin))&(mask))

#define LED_PORT PORTC
#define LED_DDR DDRC
#define LED1 _BV(PC5)
#define LED2 _BV(PC4)
#define LED3 _BV(PC3)
#define LED4 _BV(PC2)
#define LED5 _BV(PC1)
#define LED6 _BV(PC0)
#define LEDS_MASK LED1|LED2|LED3|LED4|LED5|LED6

#define SIG_PORT PORTD
#define SIG_PIN PIND
#define PWM_SIG _BV(PD2)
#define BUT_SIG _BV(PD3)
#define LED_SIG _BV(PD4)

//#define TCCR0_PRESCL _BV(CS01) // Timer0 prescaler 1/8 ~3,8kHz
#define TCCR0_PRESCL _BV(CS01)|_BV(CS00) // Timer0 prescaler 1/64 ~500Hz
#define TCCR0_PRESCL_MASK _BV(CS02)|_BV(CS01)|_BV(CS00) // Timer0 prescaler mask

typedef uint8_t byte;

#define F_SRC_PWM 1000 /* Source PWM frequency = 1kHz */
#define PWM_LEVEL_MAX 127 /* Maximal Timer0 value T0_MAX = F_CPU * T0_PRESCL / F_SRC_PWM */
static volatile uint16_t srcPwmLevel[2] = {10, 0};

#define BUTTON_WAIT _BV(7)
#define BUTTON_PRESSED _BV(6)
#define BUTTON_CNT_MASK 0x0F
#define BUTTON_CNT_MAX 15
static volatile byte buttonStatus;
static volatile uint16_t buttonLongCnt;
static const uint16_t BUTTON_LONG_CNT_MAX = 2000; // ~5sec

static byte color[3];
static byte colorIdx;
#define COLOR_MAX 17
byte colorTable[COLOR_MAX][3] PROGMEM = { // higher bits
  /*  0 */ {8, 8, 8}, // White
  /*  1 */ {8, 0, 0}, // Red     ==========
  /*  2 */ {8, 6, 0}, // Orange
  /*  3 */ {8, 7, 0}, // Orange Yellow
  /*  4 */ {8, 8, 0}, // Lemon Yellow ==========
  /*  5 */ {7, 8, 0}, // Lime Yellow
  /*  6 */ {6, 8, 0}, // Lime
  /*  7 */ {0, 8, 0}, // Green   ==========
  /*  8 */ {0, 8, 6}, // Green Teal
  /*  9 */ {0, 8, 7}, // Teal
  /* 10 */ {0, 8, 8}, // Cyan    ==========
  /* 11 */ {0, 7, 8}, // Sky
  /* 12 */ {0, 5, 8}, // Cobalt
  /* 13 */ {0, 0, 8}, // Blue    ==========
  /* 14 */ {6, 0, 8}, // Violet
  /* 15 */ {8, 0, 8}, // Purple  ==========
  /* 16 */ {8, 0, 6}, // Pink
};

static void fillColor(byte index) {
  color[0] = pgm_read_byte(&(colorTable[index][0]));
  color[1] = pgm_read_byte(&(colorTable[index][1]));
  color[2] = pgm_read_byte(&(colorTable[index][2]));
}

byte eeColorIndex EEMEM;

static inline void saveColorIndex(byte index) {
  eeprom_update_byte(&eeColorIndex, index);
}

static inline byte loadColorIndex(void) {
  byte index = eeprom_read_byte(&eeColorIndex);
  if(index >= COLOR_MAX) {
    index = 0;
  }
  return index;
}

void init(void)
{
  colorIdx = loadColorIndex();
  fillColor(colorIdx);
  buttonStatus = BUTTON_WAIT;

  setbits(LED_DDR, LEDS_MASK);

  // TODO remove pullup for finals
  //setbits(SIG_PORT, PWM_SIG | BUT_SIG | LED_SIG); // set input pullup
  //setbits(SIG_PORT, BUT_SIG); // set input pullup

  setbits(MCUCR,_BV(ISC01) | _BV(ISC00)); // INT0 interrupt at rising edge
  setbits(GICR, _BV(INT0)); // Enable INT0 interrupt

  setbits(MCUCR, _BV(ISC11)); // INT1 interrupt at falling edge
  setbits(GICR, _BV(INT1)); // Enable INT1 interrupt

  setbits(TCCR0, TCCR0_PRESCL); // Timer0 prescaler
  setbits(TIMSK, _BV(TOIE0)); // Enable Timer0 Overflow interrupt

  sei();
}

ISR(TIMER0_OVF_vect)
{
  srcPwmLevel[0] = PWM_LEVEL_MAX; // Set to max
}

ISR(INT0_vect)
{
  cli();
  byte isRising = getbits(MCUCR, _BV(ISC00));
  if(isRising)
  {
    TCNT0 = 0; // Reset Timer0
    setbits(TCCR0, TCCR0_PRESCL); // Start Timer0
    clrbits(MCUCR, _BV(ISC00)); // toggle INT0 for falling edge 
  }
  else
  {
    clrbits(TCCR0, TCCR0_PRESCL_MASK); // Stop Timer0
    srcPwmLevel[0] = TCNT0; // Get ticks count
    setbits(MCUCR, _BV(ISC00)); // toggle INT0 for rising edge
  }
  sei();
}

ISR(INT1_vect)
{
  cli();
  if(is_bits(buttonStatus, BUTTON_WAIT)) {
    buttonStatus = 0; // clear WAIT and PRESS bits and counter value
  }
  sei();
}

static inline void onButPress(void) {
  // Empty
}

static inline void onButRelease(void) {
  // Next color
  colorIdx = (colorIdx + 1) % COLOR_MAX;
  fillColor(colorIdx);
}

static inline void onButLongPress(void) {
  // Save color and off LEDs
  saveColorIndex(colorIdx);
  color[0] = color[1] = color[2] = 0;
}

static inline void onButLongRelease(void) {
  // Restore color
  colorIdx = loadColorIndex();
  fillColor(colorIdx);
}

static inline void loop(void)
{
  // Process BUT signal if WAIT flag is down
  if(getbits(buttonStatus, BUTTON_WAIT) == 0) {
    byte butSigInv = getbits(SIG_PIN, BUT_SIG); // Inverted signal
    byte pressed = getbits(buttonStatus, BUTTON_PRESSED);
    // Increment press/release counter
    if((butSigInv == 0 && pressed == 0) || (butSigInv != 0 && pressed != 0)) {
      ++buttonStatus;
    }
    // Check for long press
    if(pressed && buttonLongCnt < BUTTON_LONG_CNT_MAX) {
      ++buttonLongCnt;
      if(buttonLongCnt == BUTTON_LONG_CNT_MAX) {
        onButLongPress();
      }
    }
    // Check for press/release
    byte cnt = getbits(buttonStatus, BUTTON_CNT_MASK);
    if(cnt == BUTTON_CNT_MAX) {
      // Counter bits will be cleared here
      if(pressed == 0) {
        buttonStatus = BUTTON_PRESSED;
        buttonLongCnt = 0;
        onButPress();
      } else {
        buttonStatus = BUTTON_WAIT;
        if(buttonLongCnt == BUTTON_LONG_CNT_MAX) {
          buttonLongCnt = 0;
          onButLongRelease();
        } else {
          onButRelease();
        }
      }
    }
  }

  // Read LED signal
  byte ledSig = getbits(SIG_PIN, LED_SIG);
  srcPwmLevel[1] = (ledSig != 0) ? PWM_LEVEL_MAX : 0;

  // Output PWM generation
  uint16_t pwm[2] = {
    PWM_LEVEL_MAX - srcPwmLevel[0],
    PWM_LEVEL_MAX - srcPwmLevel[1],
  };
  const byte shift = 2; // Number of additional bits for PWM accuracy (decreases PWM frequency)
  uint16_t max = PWM_LEVEL_MAX << shift;
  uint16_t level[6] = {
    max - ((pwm[0] << color[0]) >> (8 - shift)),
    max - ((pwm[0] << color[1]) >> (8 - shift)),
    max - ((pwm[0] << color[2]) >> (8 - shift)),
    max - ((pwm[1] << color[0]) >> (8 - shift)),
    max - ((pwm[1] << color[1]) >> (8 - shift)),
    max - ((pwm[1] << color[2]) >> (8 - shift)),
  };
  for(uint16_t i = 0; i < max; ++i) {
    if(i == level[0]) setbits(LED_PORT, LED1);
    if(i == level[1]) setbits(LED_PORT, LED2);
    if(i == level[2]) setbits(LED_PORT, LED3);
    if(i == level[3]) setbits(LED_PORT, LED4);
    if(i == level[4]) setbits(LED_PORT, LED5);
    if(i == level[5]) setbits(LED_PORT, LED6);
  }
  clrbits(LED_PORT, LEDS_MASK);
}

int main(void)
{
  init();
  for(;;) loop();
  return 0;
}