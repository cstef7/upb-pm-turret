#include <Arduino.h>
#include <avr/interrupt.h>
#include "messages.h"
#include "slip.h"

constexpr uint16_t TOP_FOR_PRE_FREQ(unsigned prescaler, unsigned frequency)
{
  return (F_CPU / prescaler / frequency) - 1;
}

constexpr uint8_t PIN_PAN_SERVO = 9;
constexpr uint8_t PIN_TILT_SERVO = 10;

constexpr uint8_t PIN_LASER = 12;

constexpr uint16_t SERVO_FRAME_TOP = TOP_FOR_PRE_FREQ(8, 50);

constexpr uint16_t usToOcr(uint16_t us)
{
  return us * 2UL; // 0.5 µs ticks, non-inverting Fast PWM
}

void setupTimer1()
{
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Fast PWM, TOP = ICR1 (mode 14)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // prescaler = 8

  ICR1 = SERVO_FRAME_TOP;
  OCR1A = usToOcr(1500); // servo on pin 9
  OCR1B = usToOcr(1500); // servo on pin 10
}

void setupTimer2()
{
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // Timer interrupts every 1ms
  TCCR2A = (1 << WGM21);              // CTC mode
  TCCR2B = (1 << CS22) | (1 << CS21); // prescaler = 256
  OCR2A = (F_CPU / 256 / 1000) - 1;   // 1ms at 16MHz
  TIMSK2 = (1 << OCIE2A);             // Enable Timer2 compare interrupt
}

ISR(TIMER2_COMPA_vect)
{
  // Toggle laser pin every 500ms
  static uint16_t counter = 0;
  counter++;
  if (counter >= 500)
  {
    digitalWrite(PIN_LASER, !digitalRead(PIN_LASER));
    counter = 0;
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_PAN_SERVO, OUTPUT);
  pinMode(PIN_TILT_SERVO, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);

  Serial.begin(115200);

  setupTimer1();
  setupTimer2();
  sei();
}

// slip_send_fn
int serial_send_fn(uint8_t byte, void *ctx)
{
  Serial.write(byte);
  return 0; // success
}

// slip_recv_fn
int serial_recv_fn(uint8_t *byte, void *ctx)
{
  while (Serial.available() == 0)
  {
    // wait for the next byte
  }

  int c = Serial.read();
  if (c < 0)
    return -1;

  *byte = static_cast<uint8_t>(c);
  return 0;
}

void send_sound_command(float volume)
{
  sound_message_t sound_msg;
  sound_msg.command_id = 0x02; // Sound command
  sound_msg.volume = volume;

  uint8_t buffer[SOUND_MESSAGE_SIZE];
  message_write_sound(buffer, sizeof(buffer), &sound_msg);

  send_packet(buffer, sizeof(buffer), serial_send_fn, NULL);
}

typedef struct pan_tilt_t
{
  float pan;
  float tilt;
} pan_tilt_t;

pan_tilt_t coords_to_pan_tilt(float x, float y)
{
  pan_tilt_t result;
  // map x and y from 0-1 to 700-2300 for pan and 1400-2400 for tilt
  result.pan = 700 + (1.0f - x) * (2300 - 700);
  result.tilt = 1400 + (1.0f - y) * (2400 - 1400);
  return result;
}

void set_pan_tilt(pan_tilt_t pan_tilt)
{
  uint16_t pan = usToOcr(static_cast<uint16_t>(pan_tilt.pan));
  uint16_t tilt = usToOcr(static_cast<uint16_t>(pan_tilt.tilt));
  cli();
  OCR1A = pan;
  OCR1B = tilt;
  sei();
}

pan_tilt_t running_pan_tilt = {1500, 1900};

#define PAN_TILT_NEW_WEIGHT 0.5f

void loop()
{
  // Read position message from UART
  position_message_t pos_msg;
  uint8_t buffer[POSITION_MESSAGE_SIZE];
  size_t bytes_read;

  // Read into buffer, then read from buffer
  if (recv_packet(buffer, sizeof(buffer), &bytes_read, serial_recv_fn, NULL) == 0 && bytes_read == POSITION_MESSAGE_SIZE)
  {
    if (message_read_position(buffer, sizeof(buffer), &pos_msg) == POSITION_MESSAGE_SIZE)
    {
      // float volume = pos_msg.x + pos_msg.y;
      // send_sound_command(volume);
      // toggle led
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

      pan_tilt_t pan_tilt = coords_to_pan_tilt(pos_msg.x, pos_msg.y);
      running_pan_tilt.pan = PAN_TILT_NEW_WEIGHT * pan_tilt.pan + (1.0f - PAN_TILT_NEW_WEIGHT) * running_pan_tilt.pan;
      running_pan_tilt.tilt = PAN_TILT_NEW_WEIGHT * pan_tilt.tilt + (1.0f - PAN_TILT_NEW_WEIGHT) * running_pan_tilt.tilt;
      set_pan_tilt(running_pan_tilt);
    }
  }
}