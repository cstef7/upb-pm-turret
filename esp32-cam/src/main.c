#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_camera.h"
#include "lwip/sockets.h"
#include "driver/i2s.h"

#include "debug.h"
#include "messages.h"
#include "slip.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

#define COMMAND_READ_BUFSIZE 32

#define I2S_SAMPLE_RATE 44100
#define I2S_BCLK 14
#define I2S_LRC 15
#define I2S_DOUT 13

#define I2S_PORT I2S_NUM_1

#define WAV_BUFFER_SIZE 256
#define WAV_HEADER_SIZE 44

#define QQVGA_WIDTH 160
#define QQVGA_HEIGHT 120
#define GRAYSCALE_BUFFER_SIZE (QQVGA_WIDTH * QQVGA_HEIGHT)

#define PIXEL_DIFF_THRESHOLD 15
#define MOTION_SCORE_THRESHOLD 120

#define MOTION_CHANGE_FRAMES_THRESHOLD 3

#define FRAME_WEIGHT 0.7f

#define ms_to_us(ms) ((ms) * 1000)

#define FOLLOW_TIMEOUT_US ms_to_us(3000)
#define SEARCH_TIMEOUT_US ms_to_us(3000)

#define MAX_VOLUME .2f

#ifdef ENABLE_DEBUG_LOGS
static int sock = -1;
#endif

typedef enum state_t {
    State_Idle,
    State_Follow,
    State_Search,
    State_Shoot
} state_t;

static state_t current_state = State_Idle;
static int64_t state_change_time = 0;

typedef enum sound_t {
    Sound_I_See_You,
    Sound_Searching,
    Sound_Target_Lost,
    Sound_Are_You_Still_There,
    Sound_Fire,

    Sound_Count
} sound_t;

static volatile float current_volume = 1.0f;

static const char *const sound_filenames[] = {
    [Sound_I_See_You] = "/spiffs/i-see-you.wav",
    [Sound_Searching] = "/spiffs/searching.wav",
    [Sound_Target_Lost] = "/spiffs/target-lost.wav",
    [Sound_Are_You_Still_There] = "/spiffs/are-you-still-there.wav",
    [Sound_Fire] = "/spiffs/fire.wav"
};

static const float sound_volumes[] = {
    [Sound_I_See_You] = 1.0f,
    [Sound_Searching] = 1.0f,
    [Sound_Target_Lost] = 1.0f,
    [Sound_Are_You_Still_There] = 1.0f,
    [Sound_Fire] = 1.7f
};

typedef struct play_sound_notif_t {
    sound_t sound;
    bool loop;
} play_sound_notif_t;

static QueueHandle_t sound_queue;

typedef struct send_light_notif_t {
    uint8_t dummy;
} send_light_notif_t;

static QueueHandle_t light_queue;

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
        /* Use the largest expected message size to avoid buffer overflow
         * when receiving different message types over SLIP. */
        uint8_t buffer[COMMAND_READ_BUFSIZE];
        sound_message_t sound_msg;

        size_t bytes_read;
        if (recv_packet(buffer,
                sizeof(buffer),
                &bytes_read,
                uart_recv_byte,
                (void *)(intptr_t)UART_NUM_1) < 0) {
            DBG_PRINTF(sock, "Failed to receive packet over UART\n");
            continue;
        }
        
        if (bytes_read == SOUND_MESSAGE_SIZE) {
            if (message_read_sound(buffer, sizeof(buffer), &sound_msg) == SOUND_MESSAGE_SIZE) {
                current_volume = sound_msg.volume;
            }
        }
        
        if (bytes_read == TIME_MEASUREMENT_MESSAGE_SIZE) {
            time_measurement_message_t time_msg;
            if (message_read_time_measurement(buffer, sizeof(buffer), &time_msg) == TIME_MEASUREMENT_MESSAGE_SIZE) {
                DBG_PRINTF(sock, "Received time measurement: %.2f,%.2f,%.2f\n", time_msg.capture_ms, time_msg.processing_ms, time_msg.servo_ms);
            }
        }
    }
}

static void sendPositionCommand(float x, float y) {
    position_message_t pos_msg;
    uint8_t buffer[POSITION_MESSAGE_SIZE];

    pos_msg.command_id = POSITION_COMMAND_ID;
    pos_msg.x = x;
    pos_msg.y = y;

    int serialized_len = message_write_position(buffer, sizeof(buffer), &pos_msg);
    if (serialized_len == POSITION_MESSAGE_SIZE) {
        if (send_packet(buffer, serialized_len, uart_send_byte, (void*)(intptr_t)UART_NUM_1) < 0) {
            DBG_PRINTF(sock, "Failed to send position command over UART\n");
        }
    }
}

static void sendLightCommand(uint8_t command_id) {
    light_message_t light_msg;
    uint8_t buffer[LIGHT_MESSAGE_SIZE];
    light_msg.command_id = command_id;
    int serialized_len = message_write_light(buffer, sizeof(buffer), &light_msg);
    if (serialized_len == LIGHT_MESSAGE_SIZE) {
        if (send_packet(buffer, serialized_len, uart_send_byte, (void*)(intptr_t)UART_NUM_1) < 0) {
            DBG_PRINTF(sock, "Failed to send light command over UART\n");
        }
    }
}

static void sendTimeMeasurementCommand(float capture_ms, float processing_ms, float servo_ms) {
    time_measurement_message_t time_msg;
    uint8_t buffer[TIME_MEASUREMENT_MESSAGE_SIZE];
    time_msg.command_id = TIME_MEASUREMENT_COMMAND_ID;
    time_msg.capture_ms = capture_ms;
    time_msg.processing_ms = processing_ms;
    time_msg.servo_ms = servo_ms;
    int serialized_len = message_write_time_measurement(buffer, sizeof(buffer), &time_msg);
    if (serialized_len == TIME_MEASUREMENT_MESSAGE_SIZE) {
        if (send_packet(buffer, serialized_len, uart_send_byte, (void*)(intptr_t)UART_NUM_1) < 0) {
            DBG_PRINTF(sock, "Failed to send time measurement command over UART\n");
        }
    }
}

static uint8_t background[QQVGA_HEIGHT][QQVGA_WIDTH];
static int frames_captured = 0;

typedef struct vec2_t {
    float x;
    float y;
} vec2_t;

static void analyze_frame(
    const uint8_t *frame_data,
    uint8_t background[QQVGA_HEIGHT][QQVGA_WIDTH],
    bool *motion,
    vec2_t *centroid
) {
    *motion = false;

    if (frames_captured > 0) {
        float centroid_x = 0.0f;
        float centroid_y = 0.0f;

        float diff_count = 0;
        int index_x = 0;
        int index_y = 0;
        for (size_t i = 0; i < GRAYSCALE_BUFFER_SIZE; i++) {
            int diff = abs(frame_data[i] - background[index_y][index_x]);
            if (diff > PIXEL_DIFF_THRESHOLD) {
                diff_count += diff;
                centroid_x += index_x * diff;
                centroid_y += index_y * diff;
            }

            background[index_y][index_x] = (uint8_t)(FRAME_WEIGHT * frame_data[i]
                + (1.0f - FRAME_WEIGHT) * background[index_y][index_x]);

            index_x++;
            if (index_x >= QQVGA_WIDTH) {
                index_x = 0;
                index_y++;
            }
        }
        if (diff_count > MOTION_SCORE_THRESHOLD) {
            centroid_x /= diff_count * QQVGA_WIDTH;
            centroid_y /= diff_count * QQVGA_HEIGHT;

            *motion = true;
            centroid->x = centroid_x;
            centroid->y = centroid_y;
        }
    } else {
        memcpy(background, frame_data, GRAYSCALE_BUFFER_SIZE);
    }
}

static void set_state(state_t new_state) {
    if (current_state != new_state) {
        current_state = new_state;
        state_change_time = esp_timer_get_time();
        DBG_PRINTF(sock, "State changed to: %d\n", current_state);
    }
}

static void play_sound(sound_t sound, bool loop) {
    play_sound_notif_t notif = {
        .sound = sound,
        .loop = loop
    };
    xQueueReset(sound_queue);
    xQueueSend(sound_queue, &notif, 0);
}

#define NEW_CENTROID_WEIGHT 0.5f

static void commandWriterTask(void *pvParameters) {
    int motion_frames = 0;
    int no_motion_frames = 0;

    static vec2_t running_centroid = {0.5f, 0.5f};

    while (1) {
        int64_t start_time = esp_timer_get_time();

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            DBG_PRINTF(sock, "Failed to capture frame\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

#ifdef ENABLE_DEBUG_LOGS
        float capture_time_ms = (esp_timer_get_time() - start_time) / 1000.0f;
#endif

        if (fb->len == GRAYSCALE_BUFFER_SIZE) {
            bool motion_detected = false;
            vec2_t centroid = {0.0f, 0.0f};
            analyze_frame(fb->buf, background, &motion_detected, &centroid);
            
            if (motion_detected) {
                motion_frames++;
                no_motion_frames = 0;

#ifdef ENABLE_DEBUG_LOGS
                float processing_time_ms = (esp_timer_get_time() - start_time) / 1000.0f - capture_time_ms;
                sendTimeMeasurementCommand(capture_time_ms, processing_time_ms, 0.0f);
#endif

                running_centroid.x = NEW_CENTROID_WEIGHT * centroid.x + (1.0f - NEW_CENTROID_WEIGHT) * running_centroid.x;
                running_centroid.y = NEW_CENTROID_WEIGHT * centroid.y + (1.0f - NEW_CENTROID_WEIGHT) * running_centroid.y;
                sendPositionCommand(running_centroid.x, running_centroid.y);

                if (motion_frames > MOTION_CHANGE_FRAMES_THRESHOLD
                    && (current_state == State_Idle || current_state == State_Search)
                ) {
                    if (current_state == State_Idle)
                        sendLightCommand(LASER_ON_COMMAND_ID);

                    set_state(State_Follow);
                    play_sound(Sound_I_See_You, false);
                }
            } else {
                motion_frames = 0;
                no_motion_frames++;

                if (no_motion_frames > MOTION_CHANGE_FRAMES_THRESHOLD
                    && (current_state == State_Follow || current_state == State_Shoot)
                ) {
                    set_state(State_Search);
                    play_sound(Sound_Searching, false);
                }
            }

            frames_captured++;
        } else {
            DBG_PRINTF(sock, "Frame size exceeds buffer capacity: %d bytes\n", fb->len);
        }

        esp_camera_fb_return(fb);

        int64_t now = esp_timer_get_time();
        if (current_state == State_Follow && now - state_change_time > FOLLOW_TIMEOUT_US) {
            set_state(State_Shoot);
            play_sound(Sound_Fire, true);
        } else if (current_state == State_Search && now - state_change_time > SEARCH_TIMEOUT_US) {
            set_state(State_Idle);
            play_sound(Sound_Target_Lost, false);

            running_centroid = (vec2_t){0.5f, 0.5f};
            sendPositionCommand(running_centroid.x, running_centroid.y);
            sendLightCommand(LASER_OFF_COMMAND_ID);
        }

        send_light_notif_t light_notif;
        if (xQueueReceive(light_queue, &light_notif, 0)) {
            sendLightCommand(LIGHT_COMMAND_ID);
        }
    }
}

static FILE *get_wav_handle(sound_t sound) {
    FILE *file = fopen(sound_filenames[sound], "rb");

    if (!file) {
        DBG_PRINTF(sock, "Failed to open sound file\n");
        return NULL;
    }

    if (fseek(file, WAV_HEADER_SIZE, SEEK_SET) != 0) {
        DBG_PRINTF(sock, "Failed to seek sound file\n");
        fclose(file);
        return NULL;
    }

    return file;
}

static void soundPlayerTask(void *pvParameters) {
    play_sound_notif_t notif;
    bool has_sound = false;
    FILE *file = NULL;
    uint8_t buffer[WAV_BUFFER_SIZE];

    while (1) {
        if (!has_sound) {
            xQueueReceive(sound_queue, &notif, portMAX_DELAY);
            file = get_wav_handle(notif.sound);
            has_sound = true;
            if (notif.sound == Sound_Fire) {
                send_light_notif_t light_notif;
                xQueueSend(light_queue, &light_notif, 0);
            }
        }

        size_t bytes_read = fread(buffer, 1, sizeof(buffer), file);

        for (size_t i = 0; i + 1 < bytes_read; i += 2)
        {
            uint16_t unscaled = buffer[i] | (buffer[i + 1] << 8U);
            int32_t sample = unscaled > 32767 ? (int32_t)unscaled - 65536 : unscaled;
            int32_t scaled = sample * current_volume * MAX_VOLUME * sound_volumes[notif.sound];

            // Clamp to int16_t range
            if (scaled > 32767)
                scaled = 32767;
            if (scaled < -32768)
                scaled = -32768;

            buffer[i] = scaled & 0xFF;
            buffer[i + 1] = (scaled >> 8) & 0xFF;
        }

        if (bytes_read == 0) {
            if (notif.loop) {
                fseek(file, WAV_HEADER_SIZE, SEEK_SET);
                if (notif.sound == Sound_Fire) {
                    send_light_notif_t light_notif;
                    xQueueSend(light_queue, &light_notif, 0);
                }
                continue;
            } else {
                fclose(file);
                has_sound = false;
                continue;
            }
        } else {
            size_t total_written = 0;

            while (total_written < bytes_read) {
                size_t written = 0;

                esp_err_t err = i2s_write(
                    I2S_PORT,
                    buffer + total_written,
                    bytes_read - total_written,
                    &written,
                    portMAX_DELAY
                );

                if (err != ESP_OK) {
                    DBG_PRINTF(sock, "Failed to write to I2S\n");
                    break;
                }

                total_written += written;
            }
        }

        play_sound_notif_t new_notif;
        if (xQueueReceive(sound_queue, &new_notif, 0)) {
            notif = new_notif;
            file = get_wav_handle(notif.sound);
            has_sound = true;
        }
    }
}

static void start_camera()
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

static void start_uart()
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

static void start_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            DBG_PRINTF(sock, "Failed to mount or format filesystem\n");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            DBG_PRINTF(sock, "Failed to find SPIFFS partition\n");
        } else {
            DBG_PRINTF(sock, "Failed to initialize SPIFFS (%s)\n", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0;
    size_t used = 0;

    ESP_ERROR_CHECK(
        esp_spiffs_info(NULL, &total, &used)
    );

    DBG_PRINTF(sock, "SPIFFS mounted");
    DBG_PRINTF(sock, "Partition size: total=%u used=%u",
             (unsigned)total,
             (unsigned)used);
}

static void start_i2s(uint32_t sampleRate)
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = sampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = true,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
        };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pin_config));
}

void app_main() {
#ifdef ENABLE_DEBUG_LOGS
    ESP_ERROR_CHECK(debug_network_init());
#endif

    start_uart();
    start_camera();
    start_spiffs();
    start_i2s(I2S_SAMPLE_RATE);

    sound_queue = xQueueCreate(4, sizeof(play_sound_notif_t));
    light_queue = xQueueCreate(4, sizeof(send_light_notif_t));

#ifdef ENABLE_DEBUG_LOGS
    while (sock < 0) {
        sock = debug_tcp_connect();
        if (sock < 0)
            vTaskDelay(pdMS_TO_TICKS(2000));
    }
#endif

    xTaskCreate(commandReaderTask, "command_reader", 4096, NULL, 5, NULL);
    xTaskCreate(commandWriterTask, "command_writer", 4096, NULL, 5, NULL);
    xTaskCreate(soundPlayerTask, "sound_player", 4096, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}