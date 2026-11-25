// Mock of ESP-IDF esp_system.h: the reset-reason API the flight sketch reads at boot.
#pragma once
typedef enum { ESP_RST_UNKNOWN = 0, ESP_RST_POWERON = 1, ESP_RST_BROWNOUT = 9 } esp_reset_reason_t;
inline esp_reset_reason_t esp_reset_reason() { return ESP_RST_POWERON; }
