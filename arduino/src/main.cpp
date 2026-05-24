#include <Arduino.h>
#include <avr/interrupt.h>
#include "messages.h"
#include "slip.h"

#define COMMAND_READ_BUFSIZE 32

constexpr uint16_t TOP_FOR_PRE_FREQ(unsigned prescaler, unsigned frequency)
{
  return (F_CPU / prescaler / frequency) - 1;
}

constexpr uint8_t PIN_PAN_SERVO = 9;
constexpr uint8_t PIN_TILT_SERVO = 10;

constexpr uint8_t PIN_LASER = 11;

constexpr uint8_t PIN_RED = 3;
constexpr uint8_t PIN_GREEN = 5;
constexpr uint8_t PIN_BLUE = 6;

constexpr uint16_t SERVO_FRAME_TOP = TOP_FOR_PRE_FREQ(8, 50);

constexpr uint8_t VOLUME_PIN = A0;

constexpr uint16_t usToOcr(uint16_t us)
{
  return us * 2UL; // 0.5 µs ticks, non-inverting Fast PWM
}

void setupTimer1()
{
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  ICR1 = SERVO_FRAME_TOP;
  OCR1A = usToOcr(750);  // servo on pin 9
  OCR1B = usToOcr(1550); // servo on pin 10

  // Fast PWM, TOP = ICR1 (mode 14)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // prescaler = 8
}

typedef struct rgb_t
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_t;

rgb_t shoot_sequence[] = {
    {255, 40, 8},
    {190, 20, 0},
    {60, 0, 0},
    {0, 0, 0},
};

void set_rgb(rgb_t color)
{
  analogWrite(PIN_RED, 255 - color.r);
  analogWrite(PIN_GREEN, 255 - color.g);
  analogWrite(PIN_BLUE, 255 - color.b);
}

void set_rgb_lerp(rgb_t color1, rgb_t color2, float t)
{
  uint8_t r = static_cast<uint8_t>(color1.r + t * (color2.r - color1.r));
  uint8_t g = static_cast<uint8_t>(color1.g + t * (color2.g - color1.g));
  uint8_t b = static_cast<uint8_t>(color1.b + t * (color2.b - color1.b));
  set_rgb((rgb_t){r, g, b});
}

constexpr unsigned long SHOOT_DURATION_MS = 90;
constexpr uint8_t SHOOT_REPEATS = 3;
constexpr uint8_t SHOOT_STEPS = 3; // three lerp segments between four shoot colors

struct shoot_state_t
{
  bool active;
  unsigned long start_ms;
  uint8_t repeats_left;
};

static shoot_state_t shoot_state = {false, 0, 0};

void trigger_shoot_sequence()
{
  shoot_state.active = true;
  shoot_state.start_ms = millis();
  shoot_state.repeats_left = SHOOT_REPEATS;
}

bool update_shoot_sequence()
{
  if (!shoot_state.active)
    return false;

  unsigned long now = millis();
  unsigned long elapsed = now - shoot_state.start_ms;

  while (elapsed >= SHOOT_DURATION_MS && shoot_state.repeats_left > 1)
  {
    elapsed -= SHOOT_DURATION_MS;
    shoot_state.repeats_left--;
    shoot_state.start_ms += SHOOT_DURATION_MS;
  }

  if (elapsed >= SHOOT_DURATION_MS)
  {
    set_rgb((rgb_t){0, 0, 0});
    shoot_state.active = false;
    return false;
  }

  float scaled = static_cast<float>(elapsed) * SHOOT_STEPS / SHOOT_DURATION_MS;
  int step = static_cast<int>(scaled);
  if (step >= SHOOT_STEPS)
    step = SHOOT_STEPS - 1;

  float t = scaled - step;
  set_rgb_lerp(shoot_sequence[step], shoot_sequence[step + 1], t);
  return true;
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
  if (Serial.available() <= 0)
    return -1; // no data available

  int c = Serial.read();
  if (c < 0)
    return -1;

  *byte = static_cast<uint8_t>(c);
  return 0;
}

void send_sound_command(float volume)
{
  sound_message_t sound_msg;
  sound_msg.command_id = VOLUME_COMMAND_ID;
  sound_msg.volume = volume;

  uint8_t buffer[SOUND_MESSAGE_SIZE];
  message_write_sound(buffer, sizeof(buffer), &sound_msg);

  send_packet(buffer, sizeof(buffer), serial_send_fn, NULL);
}

void send_time_measurement(float capture_ms, float processing_ms, float servo_ms)
{
  time_measurement_message_t time_msg;
  time_msg.command_id = TIME_MEASUREMENT_COMMAND_ID;
  time_msg.capture_ms = capture_ms;
  time_msg.processing_ms = processing_ms;
  time_msg.servo_ms = servo_ms;

  uint8_t buffer[TIME_MEASUREMENT_MESSAGE_SIZE];
  message_write_time_measurement(buffer, sizeof(buffer), &time_msg);

  send_packet(buffer, sizeof(buffer), serial_send_fn, NULL);
}

typedef struct pan_tilt_t
{
  float pan;
  float tilt;
} pan_tilt_t;

#define PICTURE_DISTANCE 0.3f

constexpr float cx = 0.5f;
constexpr float cy = 0.5f;
constexpr float x_radius = 0.1f;
constexpr float y_radius = 0.16f;

void clampToEllipse(float &x, float &y)
{
  float dx = x - cx;
  float dy = y - cy;

  // ellipse equation value
  float v = (dx * dx) / (x_radius * x_radius) + (dy * dy) / (y_radius * y_radius);

  // already inside
  if (v <= 1.0f)
    return;

  // scale factor to project onto ellipse boundary
  float scale = 1.0f / sqrtf(v);

  x = cx + dx * scale;
  y = cy + dy * scale;
}

pan_tilt_t coords_to_pan_tilt(float x, float y)
{
  clampToEllipse(x, y);

  float x_angle = atan2f(x - 0.5f, PICTURE_DISTANCE);
  float y_angle = atan2f(y - 0.5f, PICTURE_DISTANCE);

  pan_tilt_t result;
  result.pan = 950 - 400 * x_angle;
  result.tilt = 1800 - 500 * y_angle;
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

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_PAN_SERVO, OUTPUT);
  pinMode(PIN_TILT_SERVO, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  set_rgb((rgb_t){0, 0, 0});

  Serial.begin(115200);

  digitalWrite(PIN_PAN_SERVO, LOW);
  digitalWrite(PIN_TILT_SERVO, LOW);

  setupTimer1();
  set_pan_tilt(coords_to_pan_tilt(0.5f, 0.5f));

  sei();
}

void loop()
{
  static slip_recv_state_t slip_recv_state = {0, false};
  static float current_volume = 0.0f;

  static float laserIntensity = 0.0f;
  static float targetLaserIntensity = 0.0f;

  static bool has_time_message = false;
  static float capture_time_ms = 0.0f;
  static float processing_time_ms = 0.0f;
  static float time_measurement_local_ms = 0.0f;

  position_message_t pos_msg;
  light_message_t light_msg;
  uint8_t buffer[COMMAND_READ_BUFSIZE];
  size_t bytes_read;

  if (recv_packet_step(buffer, sizeof(buffer), &bytes_read, &slip_recv_state, serial_recv_fn, NULL) == 0)
  {
    if (bytes_read == POSITION_MESSAGE_SIZE && message_read_position(buffer, sizeof(buffer), &pos_msg) == POSITION_MESSAGE_SIZE)
    {
      pan_tilt_t pan_tilt = coords_to_pan_tilt(pos_msg.x, pos_msg.y);
      set_pan_tilt(pan_tilt);

      if (has_time_message)
      {
        unsigned long now = millis();
        float servo_time_ms = now - time_measurement_local_ms;
        send_time_measurement(capture_time_ms, processing_time_ms, servo_time_ms);
        has_time_message = false;
      }
    }

    if (bytes_read == LIGHT_MESSAGE_SIZE && message_read_light(buffer, sizeof(buffer), &light_msg) == LIGHT_MESSAGE_SIZE)
    {
      if (light_msg.command_id == LIGHT_COMMAND_ID)
      {
        trigger_shoot_sequence();
      }
      else if (light_msg.command_id == LASER_ON_COMMAND_ID)
      {
        targetLaserIntensity = 1.0f;
      }
      else if (light_msg.command_id == LASER_OFF_COMMAND_ID)
      {
        targetLaserIntensity = 0.0f;
      }
    }

    if (bytes_read == TIME_MEASUREMENT_MESSAGE_SIZE)
    {
      time_measurement_message_t time_msg;
      if (message_read_time_measurement(buffer, sizeof(buffer), &time_msg) == TIME_MEASUREMENT_MESSAGE_SIZE)
      {
        has_time_message = true;
        capture_time_ms = time_msg.capture_ms;
        processing_time_ms = time_msg.processing_ms;
        time_measurement_local_ms = millis();
      }
    }
  }

  float volume = analogRead(VOLUME_PIN) / 1023.0f;
  if (fabsf(volume - current_volume) > 0.01f)
  {
    current_volume = volume;
    send_sound_command(current_volume);
  }

  update_shoot_sequence();

  // Smoothly update laser intensity
  laserIntensity += (targetLaserIntensity - laserIntensity) * 0.002f;
  analogWrite(PIN_LASER, static_cast<uint8_t>(laserIntensity * 255));
}