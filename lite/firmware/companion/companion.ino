// NF-3D Lite v3 companion: Seeed XIAO ESP32-S3 Sense on the nose arch.
//
//   PMW3901 optical flow (SPI) + VL53L1X time of flight (I2C), both on the tail tab
//     -> SENS frames to the flight board over UART at 100 Hz (lite/link.hpp)
//   OV2640 camera -> MJPEG over Wi-Fi: http://192.168.4.1:81/stream
//   laptop commands (UDP 14560) -> CMD frames to the flight board; flight-board TELEM -> laptop
//
// Board: "XIAO_ESP32S3" (esp32 by Espressif, 3.x), Tools -> PSRAM: "OPI PSRAM" (the camera needs it).
// Libraries: "Bitcraze PMW3901" and "VL53L1X" by Pololu (Library Manager), plus lite_fc from
// lite/firmware/libraries (only its header-only link / companion parts are used here).
//
// Wi-Fi: the XIAO is an access point "NF3D-Lite", password "nf3d-lite" (change kApPass below).
// Join it from the laptop, then run  python -m lite.ground.station  (lite/ground/README.md).
//
// Serial monitor (115200): flow / range / link counters every second. Type "r" to zero the slide
// distance for the flow calibration (BUILD.md "flow calibration").
#include <Arduino.h>
#include <Bitcraze_PMW3901.h>
#include <SPI.h>
#include <VL53L1X.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include "esp_camera.h"
#include "esp_http_server.h"
#include <lite_fc.h>  // library discovery (see lite_fc.h)
#include "lite/companion.hpp"
#include "lite/link.hpp"

using namespace lite;

// ------------------------------------------------------------------ pins (XIAO ESP32-S3)
static const int kFlowCsPin = 2;          // D1 -> PMW3901 CS   (SCK D8, MISO D9, MOSI D10)
static const int kSdaPin = 5, kSclPin = 6;// D4 / D5 -> VL53L1X SDA / SCL
static const int kFcTxPin = 43;           // D6 -> flight board IO27 (its companion RX)
static const int kFcRxPin = 44;           // D7 <- flight board IO25 (its companion TX)
static const uint32_t kFcBaud = 115200;

// camera: XIAO ESP32S3 Sense (Seeed camera_pins.h, CAMERA_MODEL_XIAO_ESP32S3)
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 10
#define CAM_PIN_SIOD 40
#define CAM_PIN_SIOC 39
#define CAM_PIN_D7 48
#define CAM_PIN_D6 11
#define CAM_PIN_D5 12
#define CAM_PIN_D4 14
#define CAM_PIN_D3 16
#define CAM_PIN_D2 18
#define CAM_PIN_D1 17
#define CAM_PIN_D0 15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF 47
#define CAM_PIN_PCLK 13

static const char* kApSsid = "NF3D-Lite";
static const char* kApPass = "nf3d-lite";  // >= 8 characters
static const uint16_t kUdpPort = 14560;

// ------------------------------------------------------------------ state
static Bitcraze_PMW3901 g_flow(kFlowCsPin);
static VL53L1X g_tof;
static WiFiUDP g_udp;
static IPAddress g_laptop;
static uint16_t g_laptop_port = 0;
static link::Parser g_fc_parser, g_udp_parser;
static companion::FlowConfig g_flow_cfg;
static bool g_flow_ok = false, g_tof_ok = false, g_cam_ok = false;
static uint32_t g_sens_sent = 0, g_cmd_fwd = 0, g_telem_fwd = 0;
static float g_slide_x = 0, g_slide_y = 0;  // flow calibration: integrated distance (m)

// PMW3901 SQUAL (register 0x07): the Bitcraze library has no getter, so read it directly with
// the library's own SPI timing (mode 3, 2 MHz, 50 us gaps).
static uint8_t flow_squal() {
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  digitalWrite(kFlowCsPin, LOW);
  delayMicroseconds(50);
  SPI.transfer(0x07 & 0x7F);
  delayMicroseconds(50);
  const uint8_t v = SPI.transfer(0);
  delayMicroseconds(100);
  digitalWrite(kFlowCsPin, HIGH);
  SPI.endTransaction();
  return v;
}

// ------------------------------------------------------------------ camera stream
static bool camera_init() {
  camera_config_t c = {};
  c.pin_pwdn = CAM_PIN_PWDN; c.pin_reset = CAM_PIN_RESET; c.pin_xclk = CAM_PIN_XCLK;
  c.pin_sccb_sda = CAM_PIN_SIOD; c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_d7 = CAM_PIN_D7; c.pin_d6 = CAM_PIN_D6; c.pin_d5 = CAM_PIN_D5; c.pin_d4 = CAM_PIN_D4;
  c.pin_d3 = CAM_PIN_D3; c.pin_d2 = CAM_PIN_D2; c.pin_d1 = CAM_PIN_D1; c.pin_d0 = CAM_PIN_D0;
  c.pin_vsync = CAM_PIN_VSYNC; c.pin_href = CAM_PIN_HREF; c.pin_pclk = CAM_PIN_PCLK;
  c.xclk_freq_hz = 20000000;
  c.ledc_timer = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;       // 640 x 480: 15 cm AprilTags (A4 print) detected to ~4 m, ~15 fps
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;   // always the newest frame: latency over smoothness
  return esp_camera_init(&c) == ESP_OK;
}

static esp_err_t stream_handler(httpd_req_t* req) {
  static const char* kBoundary = "nf3dframe";
  char part[96];
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=nf3dframe");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  for (;;) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    const int n = snprintf(part, sizeof part, "\r\n--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                           kBoundary, (unsigned)fb->len);
    esp_err_t r = httpd_resp_send_chunk(req, part, n);
    if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (r != ESP_OK) return r;  // laptop closed the stream
  }
}

static void stream_start() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 81;
  cfg.ctrl_port = 32769;
  httpd_handle_t h = nullptr;
  if (httpd_start(&h, &cfg) != ESP_OK) return;
  httpd_uri_t uri = {};
  uri.uri = "/stream";
  uri.method = HTTP_GET;
  uri.handler = stream_handler;
  httpd_register_uri_handler(h, &uri);
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  Serial1.begin(kFcBaud, SERIAL_8N1, kFcRxPin, kFcTxPin);
  Wire.begin(kSdaPin, kSclPin, 400000);
  SPI.begin();
  g_flow_ok = g_flow.begin();
  g_tof.setTimeout(30);
  g_tof_ok = g_tof.init();
  if (g_tof_ok) {
    g_tof.setDistanceMode(VL53L1X::Short);         // best ambient-light immunity, to ~1.3 m
    g_tof.setMeasurementTimingBudget(20000);       // 20 ms
    g_tof.startContinuous(20);                     // 50 Hz
  }
  g_cam_ok = camera_init();
  WiFi.softAP(kApSsid, kApPass);
  WiFi.setSleep(false);                            // no power save: steady stream latency
  g_udp.begin(kUdpPort);
  if (g_cam_ok) stream_start();
  Serial.printf("companion: flow %d tof %d camera %d  AP %s  http://%s:81/stream  udp %u\n", g_flow_ok, g_tof_ok,
                g_cam_ok, kApSsid, WiFi.softAPIP().toString().c_str(), kUdpPort);
}

void loop() {
  static uint32_t last_flow_us = micros(), next_us = micros();
  static uint16_t seq = 0, range_mm = 0;
  static uint8_t range_status = 255;
  const uint32_t now_us = micros();

  // 1. sensors at 100 Hz -> SENS to the flight board
  if ((int32_t)(now_us - next_us) >= 0) {
    next_us += 10000;
    if ((int32_t)(now_us - next_us) > 50000) next_us = now_us + 10000;  // fell behind (Wi-Fi): resync
    int16_t dx = 0, dy = 0;
    uint8_t squal = 0;
    if (g_flow_ok) {
      g_flow.readMotionCount(&dx, &dy);
      squal = flow_squal();
    }
    const float dt = (now_us - last_flow_us) * 1e-6f;
    last_flow_us = now_us;
    bool range_new = false;
    if (g_tof_ok && g_tof.dataReady()) {
      range_mm = g_tof.read(false);
      range_status = (uint8_t)g_tof.ranging_data.range_status;
      range_new = true;
    }
    const link::Sens s = companion::make_sens(dx, dy, squal, dt, g_flow_ok, range_mm, range_new && g_tof_ok,
                                              range_status, seq++, g_flow_cfg);
    uint8_t buf[link::kSensLen + link::kOverhead];
    const size_t n = link::encode_sens(s, buf);
    Serial1.write(buf, n);
    ++g_sens_sent;
    if ((s.flags & link::FLOW_OK) && range_mm > 0) {  // slide test: integrated ground distance
      g_slide_x += s.flow_x_mrad * 0.001f * range_mm * 0.001f * dt;
      g_slide_y += s.flow_y_mrad * 0.001f * range_mm * 0.001f * dt;
    }
    if (g_laptop_port && (seq % 10) == 0) {  // 10 Hz copy to the laptop (sensor health display)
      g_udp.beginPacket(g_laptop, g_laptop_port);
      g_udp.write(buf, n);
      g_udp.endPacket();
    }
  }

  // 2. laptop -> flight board (valid CMD frames only)
  const int pkt = g_udp.parsePacket();
  if (pkt > 0) {
    uint8_t in[256], out[256];
    const int n = g_udp.read(in, sizeof in);
    g_laptop = g_udp.remoteIP();
    g_laptop_port = g_udp.remotePort();
    const size_t w = companion::bridge_from_laptop(in, n > 0 ? (size_t)n : 0, out, sizeof out, g_udp_parser);
    if (w) { Serial1.write(out, w); ++g_cmd_fwd; }
  }

  // 3. flight board -> laptop (TELEM)
  while (Serial1.available() > 0) {
    if (!g_fc_parser.push((uint8_t)Serial1.read()) || g_fc_parser.type() != link::TELEM || !g_laptop_port) continue;
    uint8_t buf[link::kMaxPayload + link::kOverhead];
    const size_t n = link::encode(link::TELEM, g_fc_parser.payload(), g_fc_parser.len(), buf);
    g_udp.beginPacket(g_laptop, g_laptop_port);
    g_udp.write(buf, n);
    g_udp.endPacket();
    ++g_telem_fwd;
  }

  // 4. serial monitor
  static uint32_t last_print = 0;
  if (millis() - last_print > 1000) {
    last_print = millis();
    Serial.printf("range %u mm (status %u)  slide x %.3f y %.3f m  sens %lu cmd %lu telem %lu  stations %d\n", range_mm,
                  range_status, g_slide_x, g_slide_y, (unsigned long)g_sens_sent, (unsigned long)g_cmd_fwd,
                  (unsigned long)g_telem_fwd, WiFi.softAPgetStationNum());
  }
  while (Serial.available()) {
    if (Serial.read() == 'r') { g_slide_x = g_slide_y = 0; Serial.println("slide distance zeroed"); }
  }
}
