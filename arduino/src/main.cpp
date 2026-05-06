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

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_PAN_SERVO, OUTPUT);
  pinMode(PIN_TILT_SERVO, OUTPUT);

  Serial.begin(115200);

  setupTimer1();
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

void loop()
{
  // Read position message from UART
  position_message_t pos_msg;
  uint8_t buffer[POSITION_MESSAGE_SIZE];
  size_t bytes_read;

  // Read into buffer, then read from buffer
  if (recv_packet(buffer, sizeof(buffer), &bytes_read, serial_recv_fn, NULL) == 0 && bytes_read == POSITION_MESSAGE_SIZE)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    if (message_read_position(buffer, sizeof(buffer), &pos_msg) == POSITION_MESSAGE_SIZE)
    {
      // just... sum the two coordinates and send the result as a sound command with volume = x + y
      sound_message_t sound_msg;
      sound_msg.command_id = 0x02; // Sound command
      sound_msg.volume = pos_msg.x + pos_msg.y;

      uint8_t sound_buffer[SOUND_MESSAGE_SIZE];
      message_write_sound(sound_buffer, sizeof(sound_buffer), &sound_msg);

      send_packet(sound_buffer, sizeof(sound_buffer), serial_send_fn, NULL);
    }
  }

#if 0
  static uint32_t current_counter = 0;
  current_counter++;

  static uint32_t last_update = 0;

  if (millis() - last_update >= 20)
  {
    last_update = millis();

    float t = current_counter * 0.00001f;

    uint16_t pan = usToOcr(1500 + sin(t) * 800);
    uint16_t tilt = usToOcr(1900 + sin(t) * 500);

    cli();
    OCR1A = pan;
    OCR1B = tilt;
    sei();
  }
#endif
}