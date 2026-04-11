#include <Arduino.h>
#include <avr/interrupt.h>

#define BITBANG_FREQUENCY 20000UL
#define TOP_FOR_FREQUENCY(freq) (BITBANG_FREQUENCY / (freq))
#define TOP_FOR_US(us) ((us) * BITBANG_FREQUENCY / 1000000UL)

#define PIN_SERVO1 3
#define PIN_SERVO2 5

// Set up timer1 to trigger interrupts with 100khz frequency
void setupTimer1()
{
  int prescaler = 1;
  int topValue = (F_CPU / (prescaler * BITBANG_FREQUENCY)) - 1;
  // Configure timer1 for CTC mode
  TCCR1A = 0;                          // Normal mode
  TCCR1B = (1 << WGM12) | (1 << CS10); // CTC mode, no prescaling
  OCR1A = topValue;                    // Set the top value for the timer
  TIMSK1 = (1 << OCIE1A);              // Enable timer1 compare interrupt
}

volatile unsigned long global_counter = 0;
volatile unsigned servo_counter = 0;
unsigned servo_top = TOP_FOR_FREQUENCY(50);

volatile unsigned pan_servo_ticks = TOP_FOR_US(1500);
volatile unsigned sampled_pan_servo_ticks = TOP_FOR_US(1500);

volatile unsigned tilt_servo_ticks = TOP_FOR_US(1500);
volatile unsigned sampled_tilt_servo_ticks = TOP_FOR_US(1500);

// Called with 100kHz frequency
ISR(TIMER1_COMPA_vect)
{
  global_counter++;
  servo_counter++;

  if (servo_counter >= servo_top)
  {
    servo_counter = 0;
    sampled_pan_servo_ticks = pan_servo_ticks;
    sampled_tilt_servo_ticks = tilt_servo_ticks;
    PORTD |= (1 << PD3); // HIGH
    PORTD |= (1 << PD5); // HIGH
  }

  if (servo_counter == sampled_pan_servo_ticks)
  {
    PORTD &= ~(1 << PD3); // LOW
  }

  if (servo_counter == sampled_tilt_servo_ticks)
  {
    PORTD &= ~(1 << PD5); // LOW
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_SERVO1, OUTPUT);
  pinMode(PIN_SERVO2, OUTPUT);

  Serial.begin(38400);

  setupTimer1();
  sei();
}

void loop()
{
  noInterrupts();
  unsigned long current_counter = global_counter;
  interrupts();

  unsigned long new_ticks_pan =
      TOP_FOR_US(1000 + sin(current_counter / (double)BITBANG_FREQUENCY) * 0.8 * 1000);
  unsigned long new_ticks_tilt =
      TOP_FOR_US(1320 + sin(current_counter / (double)BITBANG_FREQUENCY) * 0.3 * 1000);

  noInterrupts();
  pan_servo_ticks = new_ticks_pan;
  tilt_servo_ticks = new_ticks_tilt;
  interrupts();
}