#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include <stddef.h>

#define SYNC_BYTE 0xAA
#define SOUND_MESSAGE_SIZE 6
#define POSITION_MESSAGE_SIZE 10

typedef struct sound_message_t {
    uint8_t command_id;
    union {
        struct {
            float volume;
        };
        struct {
            uint32_t sound_id;
        };
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

#ifdef __cplusplus
extern "C" {
#endif

int message_write_sound(uint8_t *buffer, size_t max_len, const sound_message_t *msg);
int message_read_sound(const uint8_t *buffer, size_t len, sound_message_t *msg);
int message_write_position(uint8_t *buffer, size_t max_len, const position_message_t *msg);
int message_read_position(const uint8_t *buffer, size_t len, position_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif
