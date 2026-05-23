#include <string.h>
#include <stdint.h>
#include "messages.h"

#define CRC8_POLY 0x07U

static uint8_t crc8_compute(const uint8_t *data, size_t len) {
    uint8_t crc = 0;

    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ CRC8_POLY) : (uint8_t)(crc << 1);
    }

    return crc;
}

int message_write_sound(uint8_t *buffer, size_t max_len, const sound_message_t *msg) {
    if (!buffer || !msg || max_len < SOUND_MESSAGE_SIZE)
        return -1;

    buffer[0] = msg->command_id;
    memcpy(&buffer[1], &msg->volume, sizeof(float));
    buffer[SOUND_MESSAGE_SIZE - 1] = crc8_compute(buffer, SOUND_MESSAGE_SIZE - 1);

    return SOUND_MESSAGE_SIZE;
}

int message_read_sound(const uint8_t *buffer, size_t len, sound_message_t *msg) {
    if (!buffer || !msg || len < SOUND_MESSAGE_SIZE)
        return -1;

    if (crc8_compute(buffer, SOUND_MESSAGE_SIZE - 1) != buffer[SOUND_MESSAGE_SIZE - 1])
        return -1;

    if (buffer[0] != VOLUME_COMMAND_ID)
        return -1;

    msg->command_id = buffer[0];
    memcpy(&msg->volume, &buffer[1], sizeof(float));

    return SOUND_MESSAGE_SIZE;
}

int message_write_position(uint8_t *buffer, size_t max_len, const position_message_t *msg) {
    if (!buffer || !msg || max_len < POSITION_MESSAGE_SIZE)
        return -1;

    buffer[0] = msg->command_id;
    memcpy(&buffer[1], &msg->x, sizeof(float));
    memcpy(&buffer[5], &msg->y, sizeof(float));
    buffer[POSITION_MESSAGE_SIZE - 1] = crc8_compute(buffer, POSITION_MESSAGE_SIZE - 1);

    return POSITION_MESSAGE_SIZE;
}

int message_read_position(const uint8_t *buffer, size_t len, position_message_t *msg) {
    if (!buffer || !msg || len < POSITION_MESSAGE_SIZE)
        return -1;

    if (crc8_compute(buffer, POSITION_MESSAGE_SIZE - 1) != buffer[POSITION_MESSAGE_SIZE - 1])
        return -1;
    
    if (buffer[0] != POSITION_COMMAND_ID)
        return -1;

    msg->command_id = buffer[0];
    memcpy(&msg->x, &buffer[1], sizeof(float));
    memcpy(&msg->y, &buffer[5], sizeof(float));

    return POSITION_MESSAGE_SIZE;
}

int message_write_light(uint8_t *buffer, size_t max_len, const light_message_t *msg) {
    if (!buffer || !msg || max_len < LIGHT_MESSAGE_SIZE)
        return -1;

    buffer[0] = msg->command_id;
    buffer[1] = crc8_compute(buffer, LIGHT_MESSAGE_SIZE - 1);

    return LIGHT_MESSAGE_SIZE;
}

int message_read_light(const uint8_t *buffer, size_t len, light_message_t *msg) {
    if (!buffer || !msg || len < LIGHT_MESSAGE_SIZE)
        return -1;

    if (crc8_compute(buffer, LIGHT_MESSAGE_SIZE - 1) != buffer[LIGHT_MESSAGE_SIZE - 1])
        return -1;

    if (!(buffer[0] == LIGHT_COMMAND_ID || buffer[0] == LASER_ON_COMMAND_ID || buffer[0] == LASER_OFF_COMMAND_ID))
        return -1;

    msg->command_id = buffer[0];

    return LIGHT_MESSAGE_SIZE;
}


int message_write_time_measurement(uint8_t *buffer, size_t max_len, const time_measurement_message_t *msg) {
    if (!buffer || !msg || max_len < TIME_MEASUREMENT_MESSAGE_SIZE)
        return -1;

    buffer[0] = msg->command_id;
    memcpy(&buffer[1], &msg->capture_ms, sizeof(float));
    memcpy(&buffer[5], &msg->processing_ms, sizeof(float));
    memcpy(&buffer[9], &msg->servo_ms, sizeof(float));
    buffer[TIME_MEASUREMENT_MESSAGE_SIZE - 1] = crc8_compute(buffer, TIME_MEASUREMENT_MESSAGE_SIZE - 1);

    return TIME_MEASUREMENT_MESSAGE_SIZE;
}

int message_read_time_measurement(const uint8_t *buffer, size_t len, time_measurement_message_t *msg) {
    if (!buffer || !msg || len < TIME_MEASUREMENT_MESSAGE_SIZE)
        return -1;

    if (crc8_compute(buffer, TIME_MEASUREMENT_MESSAGE_SIZE - 1) != buffer[TIME_MEASUREMENT_MESSAGE_SIZE - 1])
        return -1;

    if (buffer[0] != TIME_MEASUREMENT_COMMAND_ID)
        return -1;

    msg->command_id = buffer[0];
    memcpy(&msg->capture_ms, &buffer[1], sizeof(float));
    memcpy(&msg->processing_ms, &buffer[5], sizeof(float));
    memcpy(&msg->servo_ms, &buffer[9], sizeof(float));

    return TIME_MEASUREMENT_MESSAGE_SIZE;
}
