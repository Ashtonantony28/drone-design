// Minimal declarations of the arduino-esp32 API used by lite_fc.ino -- syntax check only.
#pragma once
#include <cstdint>
#include <cstdio>
#define ESP_ARDUINO_VERSION_MAJOR 3
#define HIGH 1
#define LOW 0
#define OUTPUT 1
enum adc_attenuation_t { ADC_0db, ADC_2_5db, ADC_6db, ADC_11db };
struct HardwareSerial {
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  void println(const char*) {}
  int printf(const char*, ...) { return 0; }
};
extern HardwareSerial Serial;
uint32_t millis();
void delay(uint32_t);
void pinMode(int, int);
void digitalWrite(int, int);
uint32_t analogReadMilliVolts(int);
void analogSetPinAttenuation(int, adc_attenuation_t);
bool ledcAttach(int pin, uint32_t freq, int bits);
bool ledcWrite(int pin, uint32_t duty);
// FreeRTOS
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define pdFALSE 0
#define configMAX_PRIORITIES 25
struct portMUX_TYPE { int x; };
#define portMUX_INITIALIZER_UNLOCKED {0}
void portENTER_CRITICAL(portMUX_TYPE*);
void portEXIT_CRITICAL(portMUX_TYPE*);
TickType_t xTaskGetTickCount();
BaseType_t xTaskDelayUntil(TickType_t*, TickType_t);
typedef void (*TaskFunction_t)(void*);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, int, void*, int);
