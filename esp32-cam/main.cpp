#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// Replace with your WiFi credentials
const char *ssid = "DIGI-sk9T_EXT";
const char *password = "ES4sqp2DeH";

WebServer server(80);

// AI Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Stream boundary
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char *STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void handleRoot()
{
  String html = R"rawliteral(
  <html>
  <body>
    <h2>ESP32-CAM Stream</h2>
    <img src="/stream">
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleStream()
{
  WiFiClient client = server.client();

  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: " + String(STREAM_CONTENT_TYPE) + "\r\n\r\n";
  server.sendContent(response);

  while (client.connected())
  {
    int64_t start = esp_timer_get_time();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
      continue;

    int64_t end = esp_timer_get_time();
    int64_t frameTime = end - start;
    Serial.printf("Frame captured in %lld ms\n", frameTime / 1000);

    server.sendContent(STREAM_BOUNDARY);

    char part[64];
    int hlen = snprintf(part, 64, STREAM_PART, fb->len);
    server.sendContent(part, hlen);
    server.sendContent((const char *)fb->buf, fb->len);

    Serial.printf("Frame sent in %lld ms\n", (esp_timer_get_time() - end) / 1000);

    esp_camera_fb_return(fb);
  }
}

void startCamera()
{
  camera_config_t config;
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

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }

  esp_camera_init(&config);
}

void setup()
{
  Serial.begin(115200);

  startCamera();

  WiFi.begin(ssid, password);
  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("Open: http://");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/stream", HTTP_GET, handleStream);

  server.begin();
}

void loop()
{
  server.handleClient();
}