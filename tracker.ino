#include "esp_camera.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// ------------------ Adjustable Settings -------------------
#define BUTTON_PIN       0          // GPIO0 (built-in flash button)
#define FRAME_WIDTH      160
#define FRAME_HEIGHT     120
#define TEMPLATE_W       20         // Template width
#define TEMPLATE_H       20         // Template height
#define SEARCH_WINDOW    25         // Search window radius around last position
#define MOTION_THRESHOLD 35         // Sensitivity for motion detection (0-255)
#define MIN_BLOB_SIZE    20         // Minimum moving pixels to be considered an object
#define MATCH_THRESHOLD  18000      // Maximum acceptable SAD value for a match
// -----------------------------------------------------------

// WiFi Access Point settings
const char *ap_ssid = "ESP32-CAM-Tracker";
const char *ap_password = "12345678";  // Minimum 8 characters, or leave empty for open network

WebServer server(80);

// Tracking variables
uint8_t template_img[TEMPLATE_H][TEMPLATE_W];
int last_cx = FRAME_WIDTH / 2;
int last_cy = FRAME_HEIGHT / 2;
bool tracking = false;

// Manually set sensor parameters for day/night consistency
void setup_camera_controls() {
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_gain_ctrl(s, 0);     // Auto gain off
    s->set_exposure_ctrl(s, 0); // Auto exposure off
    s->set_whitebal(s, 0);      // Auto white balance off
    s->set_gain(s, 20);         // Manual gain (0-30)
    s->set_aec2(s, 0);          // Additional AEC off
  }
}

// Setup WiFi Access Point + Web server for OTA updates
void setupOTA() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  // Upload page
  server.on("/update", HTTP_GET, []() {
    String html = "<html><body style='font-family:sans-serif;text-align:center;padding-top:50px;'>";
    html += "<h1>ESP32-CAM Tracker Update</h1>";
    html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='update' accept='.bin'><br><br>";
    html += "<input type='submit' value='Update Firmware'>";
    html += "</form></body></html>";
    server.send(200, "text/html", html);
  });

  // Handle firmware upload
  server.on("/update", HTTP_POST, []() {
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK. Rebooting...");
    delay(500);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.uploads();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %u bytes\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.begin();
  Serial.println("OTA update server ready. Connect to WiFi AP to update firmware.");
}

// Extract a grayscale template from a frame
void extract_template(uint8_t *frame, int cx, int cy) {
  int start_x = cx - TEMPLATE_W/2;
  int start_y = cy - TEMPLATE_H/2;
  for (int y = 0; y < TEMPLATE_H; y++) {
    for (int x = 0; x < TEMPLATE_W; x++) {
      int fx = constrain(start_x + x, 0, FRAME_WIDTH - 1);
      int fy = constrain(start_y + y, 0, FRAME_HEIGHT - 1);
      template_img[y][x] = frame[fy * FRAME_WIDTH + fx];
    }
  }
}

// Search for template using SAD in a limited window
bool find_template(uint8_t *frame, int &cx, int &cy) {
  int min_sad = 999999;
  int best_x = last_cx, best_y = last_cy;

  int sx = max(0, last_cx - SEARCH_WINDOW);
  int ex = min(FRAME_WIDTH - TEMPLATE_W, last_cx + SEARCH_WINDOW);
  int sy = max(0, last_cy - SEARCH_WINDOW);
  int ey = min(FRAME_HEIGHT - TEMPLATE_H, last_cy + SEARCH_WINDOW);

  for (int y = sy; y <= ey; y++) {
    for (int x = sx; x <= ex; x++) {
      int sad = 0;
      for (int ty = 0; ty < TEMPLATE_H; ty++) {
        for (int tx = 0; tx < TEMPLATE_W; tx++) {
          int pf = frame[(y + ty) * FRAME_WIDTH + (x + tx)];
          int pt = template_img[ty][tx];
          sad += abs(pf - pt);
        }
      }
      if (sad < min_sad) {
        min_sad = sad;
        best_x = x;
        best_y = y;
      }
    }
  }

  if (min_sad < MATCH_THRESHOLD) {
    last_cx = best_x + TEMPLATE_W/2;
    last_cy = best_y + TEMPLATE_H/2;
    cx = last_cx;
    cy = last_cy;
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Camera configuration
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
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (1);
  }

  setup_camera_controls();
  setupOTA(); // Start WiFi AP and OTA server

  Serial.println("Ready. Press the button to lock onto a moving object.");
}

void loop() {
  server.handleClient(); // Handle OTA web requests (minimal overhead)

  // --- Button press handling ---
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(30);
    if (digitalRead(BUTTON_PIN) == LOW) {
      camera_fb_t *fb1 = esp_camera_fb_get();
      if (!fb1) {
        Serial.println("Failed to capture frame 1");
        return;
      }
      delay(120);

      camera_fb_t *fb2 = esp_camera_fb_get();
      if (!fb2) {
        esp_camera_fb_return(fb1);
        Serial.println("Failed to capture frame 2");
        return;
      }

      uint8_t *buf1 = fb1->buf;
      uint8_t *buf2 = fb2->buf;

      long sum_x = 0, sum_y = 0;
      int count = 0;
      for (int y = 0; y < FRAME_HEIGHT; y += 2) {
        for (int x = 0; x < FRAME_WIDTH; x += 2) {
          int idx = y * FRAME_WIDTH + x;
          if (abs(buf1[idx] - buf2[idx]) > MOTION_THRESHOLD) {
            sum_x += x;
            sum_y += y;
            count++;
          }
        }
      }

      if (count >= MIN_BLOB_SIZE) {
        int cx = sum_x / count;
        int cy = sum_y / count;
        extract_template(buf1, cx, cy);
        last_cx = cx;
        last_cy = cy;
        tracking = true;
        Serial.println("Object locked! Tracking...");
      } else {
        Serial.println("Not enough motion detected.");
        tracking = false;
      }

      esp_camera_fb_return(fb1);
      esp_camera_fb_return(fb2);
      while (digitalRead(BUTTON_PIN) == LOW);
      delay(200);
    }
  }

  // --- Continuous tracking ---
  if (!tracking) {
    delay(10);
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Frame lost");
    return;
  }

  int cx, cy;
  if (find_template(fb->buf, cx, cy)) {
    Serial.print(cx);
    Serial.print(",");
    Serial.println(cy);
  } else {
    Serial.println("LOST");
    tracking = false;
  }

  esp_camera_fb_return(fb);
}
