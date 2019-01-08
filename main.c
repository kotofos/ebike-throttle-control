/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <stdint.h>
#include <avr/io.h>

/**
* Milliseconds data type \n
* Data type				- Max time span			- Memory used \n
* unsigned char			- 255 milliseconds		- 1 byte \n
* unsigned int			- 65.54 seconds			- 2 bytes \n
* unsigned long			- 49.71 days			- 4 bytes \n
* unsigned long long	- 584.9 million years	- 8 bytes
*/
typedef unsigned long millis_t;

#define MILLIS_TIMER0 0 /**< Use timer0. */

/**
* Initialise, must be called before anything else!
*
* @return (none)
*/
void millis_init(void);

/**
* Get milliseconds.
*
* @return Milliseconds.
*/
millis_t millis_get(void);

/**
* Turn on timer and resume time keeping.
*
* @return (none)
*/
void millis_resume(void);

/**
* Pause time keeping and turn off timer to save power.
*
* @return (none)
*/
void millis_pause(void);

/**
* Reset milliseconds count to 0.
*
* @return (none)
*/
void millis_reset(void);

/**
* Add time.
*
* @param [ms] Milliseconds to add.
* @return (none)
*/
void millis_add(millis_t ms);

/**
* Subtract time.
*
* @param [ms] Milliseconds to subtract.
* @return (none)
*/
void millis_subtract(millis_t ms);

// I/O
#define In_Pin PB2 //pin where the potentiometer value is read
#define Out_Pin PB1  //pin of out signal
#define smoothingPlusPin PB3   //pin to control +smoothing
#define smoothingMinusPin PB4   //pin to control -smoothing
#define PORT PORTB
#define DDR DDRB
#define MUX MUX1 // PB 2. See datasheet
#define MILLIS_TIMER MILLIS_TIMER1

#define LOW 0
#define OUTPUT 1
#define INPUT 0

static void digitalWrite(int pin, int level);
static void pinMode(int pin, int mode);
static void setupAdc(int pin);
uint16_t analogRead(int pin);

int main(void)
{
  unsigned int sensorValueZero;  // Variable to store the value coming from the Sensor
  unsigned int sensorValue = 0;  // Variable to store the value coming from the Sensor
  unsigned int lastValue = 33;  // Variable to store the Last output Value
  unsigned int pastValue = 0; // Variable to store the past output value - ЕЁ НАДО ПРИВЛЕЧЬ ДЛЯ БЫСТРОГО ВОЗВРАТА К НЕДАВНЕМУ БОЛЬШЕМУ ЗНАЧЕНИЮ
  unsigned long int smoothing = 40 ;  // Задержка в мс шага на единицу
  unsigned int ledforce;  // Mapped output signal (ledforce)
  long previousMillis = 0;        // will store last time LED was updated
  long sensorMillis = 0;        // will store last time LED was updated

  digitalWrite(In_Pin, LOW);
  digitalWrite(Out_Pin, LOW);
  digitalWrite(smoothingPlusPin, LOW);
  digitalWrite(smoothingMinusPin, LOW);

  pinMode(In_Pin, INPUT); // set In_Pin as Input
  pinMode(Out_Pin, OUTPUT);  // set Out_Pin as Output
  pinMode(smoothingPlusPin, INPUT); // set smoothingPlusPin (7) as Input
  pinMode(smoothingMinusPin, INPUT); // set smoothingMinusPin (8) as Input
  smoothing = 1; //Read first EEPROM cell.
  setupAdc(In_Pin);

  while (1) {
    unsigned long currentMillis = millis_get();
    if (currentMillis - sensorMillis >= 53) {
      int val = 0;
      for (unsigned int i = 0; i < 15; ++i) {
        sensorValueZero = analogRead(In_Pin); // Считываем данные с датчика
        val = val + sensorValueZero;
      }
      sensorValue = (val) / 15;
      sensorMillis = currentMillis;
    }
    if (sensorValue <= lastValue) { //Если данные не увеличились..
      unsigned long currentMillis = millis_get();

      lastValue = sensorValue;  //Выходные данные не могут быть больше входных
      if (sensorValue < pastValue) {
        if (currentMillis - previousMillis >= (smoothing)) {  // Если данные уменьшились, ждём с момента предыдущего шага "(smoothing) мс"
          pastValue = pastValue - ((pastValue - lastValue) / 128);
          previousMillis = currentMillis;                 // обнуляем таймер цикла
        }
      }
    }

  };
    return 0;   /* never reached */
}

static void digitalWrite(int pin, int level) {
  if (level) {
    PORT |= (1 << pin);
  } else {
    PORT &= ~(1 << pin);
  }

}

static void pinMode(int pin, int mode) {
  if (mode) {
    DDR |= (1 << pin);
  } else {
    DDR &= ~(1 << pin);
  }
}

static void setupAdc(int pin) {
  // 10 bit setup
  ADMUX |= (1 << MUX);

  // Set the prescaler to clock/128 & enable ADC
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);

  //read first result
  while (ADCSRA & (1 << ADSC));
}

uint16_t analogRead(int pin){
  // Start the conversion
  ADCSRA |= (1 << ADSC);

  // Wait for it to finish - blocking
  while (ADCSRA & (1 << ADSC));

  return ADC;
}

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>

#ifndef F_CPU
	#error "F_CPU not defined!"
#endif

#if F_CPU < 256 || F_CPU >= 32640000
	#error "Bad F_CPU setting (<256 or >=32640000)"
#endif

#ifndef MILLIS_TIMER
	#error "Bad MILLIS_TIMER set"
#endif

// Decide what what prescaler and registers to use
// Timer0

#if F_CPU > 16320000 // 16.32MHz - 65.28MHz
	#define CLOCKSEL (_BV(CS20))
	#define PRESCALER 256
#elif F_CPU > 2040000 // 2.04MHz - 16.32MHz
	#define CLOCKSEL (_BV(CS01)|_BV(CS00))
	#define PRESCALER 64
#elif F_CPU > 255 // 256Hz - 2.04MHz
	#define CLOCKSEL (_BV(CS01))
	#define PRESCALER 8
#endif

#define REG_TCCRA		TCCR0A
#define REG_TCCRB		TCCR0B
#define REG_TIMSK		TIMSK0
#define REG_OCR			OCR0A
#define BIT_WGM			WGM01
#define BIT_OCIE		OCIE0A
#ifdef TIMER0_COMPA_vect
	#define ISR_VECT		TIMER0_COMPA_vect
#else
	#define ISR_VECT		TIM0_COMPA_vect
#endif
#define pwr_enable()	power_timer0_enable()
#define pwr_disable()	power_timer0_disable()

#define SET_TCCRA()	(REG_TCCRA = _BV(BIT_WGM))
#define SET_TCCRB()	(REG_TCCRB = CLOCKSEL)

static volatile millis_t milliseconds;

// Initialise library
void millis_init()
{
	// Timer settings
	SET_TCCRA();
	SET_TCCRB();
	REG_TIMSK = _BV(BIT_OCIE);
	REG_OCR = ((F_CPU / PRESCALER) / 1000);
}

// Get current milliseconds
millis_t millis_get()
{
	millis_t ms;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		ms = milliseconds;
	}
	return ms;
}


// Reset milliseconds count to 0
void millis_reset()
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		milliseconds = 0;
	}
}

// Add time
void millis_add(millis_t ms)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		milliseconds += ms;
	}
}

// Subtract time
void millis_subtract(millis_t ms)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		milliseconds -= ms;
	}
}

ISR(ISR_VECT)
{
	++milliseconds;
}
