#ifndef SLIP_H
#define SLIP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*slip_send_fn)(uint8_t byte, void *ctx);
typedef int (*slip_recv_fn)(uint8_t *byte, void *ctx);

int send_packet(const uint8_t *data, size_t len, slip_send_fn send_fn, void *ctx);

int recv_packet(uint8_t *buffer, size_t buffer_len, size_t *received_len, slip_recv_fn recv_fn, void *ctx);

#ifdef __cplusplus
}
#endif

#endif
