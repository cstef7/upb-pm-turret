#ifndef DEBUG_H
#define DEBUG_H

#include <stddef.h>
#include "esp_err.h"

esp_err_t debug_network_init(void);
int debug_tcp_connect(void);
int debug_tcp_send(int sock, const void *data, size_t len);
int debug_tcp_printf(int sock, const char *format, ...);
void debug_tcp_close(int sock);
void debug_tcp_hexdump(int sock, const void *data, size_t len);

#endif
