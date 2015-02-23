#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
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

#define HI(wd) ((BYTE)((0xff00&wd)>>8))
#define LO(wd) ((BYTE)(0x00ff&wd))

#define CHK_BUT(pin,mask) ((~(pin))&(mask))

#define LED_PORT PORTD
#define LED_DDR DDRD
#define LED1_PIN _BV(PD0)
#define LED2_PIN _BV(PD1)
#define LED3_PIN _BV(PD4)

#define LED4_PIN _BV(PD5)
#define LED5_PIN _BV(PD6)
#define LED6_PIN _BV(PD7)

#define PWM_SIG _BV(PD2)

#define BUT_SIG_PORT PORTD
#define BUT_SIG _BV(PD3)

#define LED_SIG_PIN PINB
#define LED_SIG _BV(PB0)

//#define TCCR0_PRESCL _BV(CS01) // Timer0 prescaler 1/8 ~3,8kHz
#define TCCR0_PRESCL _BV(CS01) | _BV(CS00) // Timer0 prescaler 1/64 ~500Hz
#define TCCR0_PRESCL_MASK _BV(CS02) | _BV(CS01) | _BV(CS00) // Timer0 prescaler mask

typedef uint8_t byte;

#define F_SRC_PWM 1000 // 1kHz
static const int PWM_LEVEL_MAX = 127; // T0_MAX = F_CPU * T0_PRESCL / F_SRC_PWM
static volatile int src_pwm_level = 10;
static volatile int src_pwm_level2 = 0;

static const byte COLOR_MAX = 4;
static volatile byte color_current = 0;
byte color_table[][3] = { // higher bits
  {8, 8, 8},
  {8, 0, 6},
  {8, 6, 0},
  {0, 6, 8}
};

void init(void)
{
  //PORTD = PWM_SIG; // set input pullup
  setbits(LED_DDR, LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN);

  // TODO remove pullup for finals
  setbits(BUT_SIG_PORT, BUT_SIG); // set input pullup

  MCUCR |= _BV(ISC01) | _BV(ISC00); // INT0 interrupt at rising edge
  GICR |= _BV(INT0); // Enable INT0 interrupt

  MCUCR |= _BV(ISC11); // INT1 interrupt at falling edge
  GICR |= _BV(INT1); // Enable INT1 interrupt

  TCCR0 |= TCCR0_PRESCL; // Timer0 prescaler
  TIMSK |= _BV(TOIE0); // Enable Timer0 Overflow interrupt

  sei();
}

ISR(TIMER0_OVF_vect)
{
  src_pwm_level = PWM_LEVEL_MAX; // Set to max
}

ISR(INT0_vect)
{
  cli();
  byte isRising = (MCUCR & _BV(ISC00));
  if(isRising)
  {
    TCNT0 = 0; // Reset Timer0
    setbits(TCCR0, TCCR0_PRESCL); // Start Timer0
    clrbits(MCUCR, _BV(ISC00)); // toggle INT0 for falling edge 
  }
  else
  {
    clrbits(TCCR0, TCCR0_PRESCL_MASK); // Stop Timer0
    src_pwm_level = TCNT0; // Get ticks count
    setbits(MCUCR, _BV(ISC00)); // toggle INT0 for rising edge
  }
  sei();
}

ISR(INT1_vect)
{
  cli();
  color_current = (color_current + 1) % COLOR_MAX;
  sei();
}

static inline void loop(void)
{
  // read LED signal
  byte ledSig = getbits(LED_SIG_PIN, LED_SIG);
  src_pwm_level2 = (ledSig != 0) ? PWM_LEVEL_MAX : 0;
  // output PWM generation
  const byte shift = 2;
  int pwm = PWM_LEVEL_MAX - src_pwm_level;
  int pwm2 = PWM_LEVEL_MAX - src_pwm_level2;
  int max = PWM_LEVEL_MAX << shift;
  byte* color = color_table[color_current];
  static int level[] = {0, 0, 0, 0, 0, 0};
  level[0] = max - ((pwm << color[0]) >> (8 - shift));
  level[1] = max - ((pwm << color[1]) >> (8 - shift));
  level[2] = max - ((pwm << color[2]) >> (8 - shift));
  level[3] = max - ((pwm2 << color[0]) >> (8 - shift));
  level[4] = max - ((pwm2 << color[1]) >> (8 - shift));
  level[5] = max - ((pwm2 << color[2]) >> (8 - shift));
  
  for(int i = 0; i < max; ++i) {
    if(i == level[0]) {
      setbits(LED_PORT, LED1_PIN);
    }
    if(i == level[1]) {
      setbits(LED_PORT, LED2_PIN);
    }
    if(i == level[2]) {
      setbits(LED_PORT, LED3_PIN);
    }
    if(i == level[3]) {
      setbits(LED_PORT, LED4_PIN);
    }
    if(i == level[4]) {
      setbits(LED_PORT, LED5_PIN);
    }
    if(i == level[5]) {
      setbits(LED_PORT, LED6_PIN);
    }
  }
  clrbits(LED_PORT, LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN);
}

int main(void)
{
  init();

  while (1)
  {
    loop();
  }

  return 0;
}