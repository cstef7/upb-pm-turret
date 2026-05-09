#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "debug.h"
#include "messages.h"
#include "slip.h"
#include "esp_camera.h"
#include <string.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

int sock = -1;

#define QQVGA_WIDTH 160
#define QQVGA_HEIGHT 120
#define GRAYSCALE_BUFFER_SIZE (QQVGA_WIDTH * QQVGA_HEIGHT)

#define FRAME_WEIGHT 0.2f

static int uart_send_byte(uint8_t byte, void *ctx) {
    uart_port_t uart_num = (uart_port_t)(intptr_t)ctx;

    return uart_write_bytes(uart_num, &byte, 1) == 1 ? 0 : -1;
}

static int uart_recv_byte(uint8_t *byte, void *ctx) {
    uart_port_t uart_num = (uart_port_t)(intptr_t)ctx;

    return uart_read_bytes(uart_num, byte, 1, portMAX_DELAY) == 1 ? 0 : -1;
}

void commandReaderTask(void *pvParameters) {
    while (1) {
        uint8_t buffer[SOUND_MESSAGE_SIZE];
        sound_message_t sound_msg;

        size_t bytes_read;
        if (recv_packet(buffer,
                sizeof(buffer),
                &bytes_read,
                uart_recv_byte,
                (void *)(intptr_t)UART_NUM_1) < 0) {
            // recv packet failed, handle error via TCP
            debug_tcp_printf(sock, "Failed to receive packet over UART\n");
            continue;
        }
        
        if (bytes_read == SOUND_MESSAGE_SIZE) {
            if (message_read_sound(buffer, sizeof(buffer), &sound_msg) == SOUND_MESSAGE_SIZE) {
                // ESP_LOGI(TAG, "Received sound command: volume=%.2f", sound_msg.volume);
                if (sock >= 0) {
                    int bytes_sent = debug_tcp_printf(sock, "Received sum: %.2f\n", sound_msg.volume);
                    debug_tcp_hexdump(sock, buffer, bytes_read);
                }
            }
        } else {
            // ESP_LOGW(TAG, "Received packet with unexpected size: %d bytes", bytes_read);
            debug_tcp_printf(sock, "Received packet with unexpected size: %d bytes\n", bytes_read);
            debug_tcp_hexdump(sock, buffer, bytes_read);
        }
    }
}

void sendPositionCommand(float x, float y) {
    position_message_t pos_msg;
    uint8_t buffer[POSITION_MESSAGE_SIZE];

    pos_msg.command_id = 0x01;
    pos_msg.x = x;
    pos_msg.y = y;

    int serialized_len = message_write_position(buffer, sizeof(buffer), &pos_msg);
    if (serialized_len == POSITION_MESSAGE_SIZE) {
        if (send_packet(buffer, serialized_len, uart_send_byte, (void*)(intptr_t)UART_NUM_1) < 0)
            debug_tcp_printf(sock, "Failed to send position command over UART\n");
        else
            debug_tcp_printf(sock, "Sent position command: x=%.2f, y=%.2f\n", pos_msg.x, pos_msg.y);
    }
}

uint8_t background[QQVGA_HEIGHT][QQVGA_WIDTH];
int frames_captured = 0;

void commandWriterTask(void *pvParameters) {
    while (1) {
        // ESP_LOGI("MEM", "free heap: %d", esp_get_free_heap_size());
        // debug_tcp_printf(sock, "Capturing frame %d...\n", frames_captured + 1);

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            debug_tcp_printf(sock, "Failed to capture frame\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // debug_tcp_printf(sock, "Captured frame: %d bytes, format=%d\n", fb->len, fb->format);

        // Store grayscale frame data in allocated memory
        if (fb->len <= GRAYSCALE_BUFFER_SIZE) {
            // memcpy(current_frame_ptr, fb->buf, fb->len);
            // debug_tcp_printf(sock, "Grayscale frame stored: %d bytes\n", fb->len);

            if (frames_captured > 0) {
                float centroid_x = 0.0f;
                float centroid_y = 0.0f;

                float diff_count = 0;
                int index_x = 0;
                int index_y = 0;
                for (size_t i = 0; i < fb->len; i++) {
                    int diff = abs(fb->buf[i] - background[index_y][index_x]);
                    if (diff > 10) {
                        diff_count += diff;
                        centroid_x += index_x * diff;
                        centroid_y += index_y * diff;
                    }

                    background[index_y][index_x] = (uint8_t)(FRAME_WEIGHT * fb->buf[i]
                        + (1.0f - FRAME_WEIGHT) * background[index_y][index_x]);

                    index_x++;
                    if (index_x >= QQVGA_WIDTH) {
                        index_x = 0;
                        index_y++;
                    }
                }
                if (diff_count > 100) {
                    centroid_x /= diff_count * QQVGA_WIDTH;
                    centroid_y /= diff_count * QQVGA_HEIGHT;

                    sendPositionCommand(centroid_x, centroid_y);

                    debug_tcp_printf(sock, "Centroid of motion: (%.2f, %.2f)\n", centroid_x, centroid_y);
                }
                // debug_tcp_printf(sock, "Differing pixels: %d\n", diff_count);
            } else {
                memcpy(background, fb->buf, fb->len);
            }

            frames_captured++;
        } else {
            debug_tcp_printf(sock, "Frame size exceeds buffer capacity: %d bytes\n", fb->len);
        }

        esp_camera_fb_return(fb);
    }
}

void start_camera()
{
    camera_config_t config = {0};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;

    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0)
    {
        config.frame_size = FRAMESIZE_QQVGA;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
    }
    else
    {
        config.frame_size = FRAMESIZE_QQVGA;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_camera_init(&config);
}

void start_uart()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 16, 12, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0));
}

void app_main() {
    // ESP_ERROR_CHECK(debug_network_init());

    start_uart();
    start_camera();

    // while (sock < 0) {
    //     sock = debug_tcp_connect();
    //     if (sock < 0)
    //         vTaskDelay(pdMS_TO_TICKS(2000));
    // }

    // xTaskCreate(commandReaderTask, "command_reader", 4096, NULL, 5, NULL);
    xTaskCreate(commandWriterTask, "command_writer", 4096, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}