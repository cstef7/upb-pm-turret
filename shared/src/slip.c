#include "slip.h"

#include <stddef.h>
#include <stdint.h>

typedef enum SlipConstant {
    END = 0xC0U,
    ESC = 0xDBU,
    ESC_END = 0xDCU,
    ESC_ESC = 0xDDU,
} SlipConstant;

static int slip_send_escaped_byte(uint8_t byte,
                                  slip_send_fn send_fn,
                                  void *ctx) {
    if (byte == END) {
        if (send_fn(ESC, ctx) < 0 || send_fn(ESC_END, ctx) < 0)
            return -1;

        return 0;
    }

    if (byte == ESC) {
        if (send_fn(ESC, ctx) < 0 || send_fn(ESC_ESC, ctx) < 0)
            return -1;

        return 0;
    }

    return send_fn(byte, ctx) < 0 ? -1 : 0;
}

int send_packet(const uint8_t *data, size_t len, slip_send_fn send_fn, void *ctx) {
    if ((data == NULL && len > 0U) || send_fn == NULL)
        return -1;

    if (send_fn(END, ctx) < 0)
        return -1;

    for (size_t i = 0; i < len; ++i)
        if (slip_send_escaped_byte(data[i], send_fn, ctx) < 0)
            return -1;

    if (send_fn(END, ctx) < 0)
        return -1;

    return 0;
}

int recv_packet(uint8_t *buffer,
                size_t buffer_len,
                size_t *received_len,
                slip_recv_fn recv_fn,
                void *ctx) {
    size_t received = 0;

    if ((buffer == NULL && buffer_len > 0U) || received_len == NULL || recv_fn == NULL)
        return -1;

    for (;;) {
        uint8_t byte = 0;

        if (recv_fn(&byte, ctx) < 0)
            return -1;

        if (byte == END) {
            if (received > 0U) {
                *received_len = received;
                return 0;
            }

            continue;
        }

        if (byte == ESC) {
            if (recv_fn(&byte, ctx) < 0)
                return -1;

            if (byte == ESC_END)
                byte = END;
            else if (byte == ESC_ESC)
                byte = ESC;
        }

        if (received < buffer_len)
            buffer[received++] = byte;
    }
}