#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include <stddef.h>

#define SYNC_BYTE 0xAA
#define SOUND_MESSAGE_SIZE 6
#define POSITION_MESSAGE_SIZE 10
#define LIGHT_MESSAGE_SIZE 2
#define TIME_MEASUREMENT_MESSAGE_SIZE 14

#define VOLUME_COMMAND_ID 0x01
#define POSITION_COMMAND_ID 0x02
#define LIGHT_COMMAND_ID 0x03
#define LASER_ON_COMMAND_ID 0x04
#define LASER_OFF_COMMAND_ID 0x05

#define TIME_MEASUREMENT_COMMAND_ID 0x06

typedef struct sound_message_t {
    uint8_t command_id;
    struct {
        float volume;
    };
} sound_message_t;

typedef struct position_message_t {
    uint8_t command_id;
    union {
        struct {
            float x;
            float y;
        };
    };
} position_message_t;

typedef struct light_message_t {
    uint8_t command_id;
} light_message_t;

typedef struct time_measurement_message_t {
    uint8_t command_id;
    float capture_ms;
    float processing_ms;
    float servo_ms;
} time_measurement_message_t;

#ifdef __cplusplus
extern "C" {
#endif

int message_write_sound(uint8_t *buffer, size_t max_len, const sound_message_t *msg);
int message_read_sound(const uint8_t *buffer, size_t len, sound_message_t *msg);
int message_write_position(uint8_t *buffer, size_t max_len, const position_message_t *msg);
int message_read_position(const uint8_t *buffer, size_t len, position_message_t *msg);
int message_write_light(uint8_t *buffer, size_t max_len, const light_message_t *msg);
int message_read_light(const uint8_t *buffer, size_t len, light_message_t *msg);
int message_write_time_measurement(uint8_t *buffer, size_t max_len, const time_measurement_message_t *msg);
int message_read_time_measurement(const uint8_t *buffer, size_t len, time_measurement_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif
