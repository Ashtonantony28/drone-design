// Times neuroflight-fc's learned controller (hover_ctbr_v3, 19-64-64-4 MLP) on the flight board's ESP32.
// Flash it to the bare ESP32 board (Arduino IDE: board "ESP32 Dev Module"), open the serial monitor at
// 115200, and it prints the inference time for the float32 and int8 models. The policy would run at
// 500 Hz, so anything under ~400 us leaves the 1 kHz rate loop plenty of room.
// nf_policy_gen.{hpp,cpp}: python3 neuroflight-fc/tools/codegen.py train/policies/hover_ctbr_v3.q.nfpol.json
#include <Arduino.h>
#include <lite_fc.h>  // pulls in nf_core (library discovery)
#include "nf/policy.hpp"
#include "nf_policy_gen.hpp"

static float obs[nf::kObsDim];
static float out[nf::kActDim];

static void run_f32() { nf::mlp_forward(nf_gen::kPolicyF32, obs, out); obs[0] += 1e-6f * out[0]; }
static void run_i8() { nf::mlp_forward_int8(nf_gen::kPolicyI8, obs, out); obs[0] += 1e-6f * out[0]; }

static float time_us(void (*f)(), int n) {
  const uint32_t t0 = micros();
  for (int i = 0; i < n; ++i) f();
  return float(micros() - t0) / n;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  for (int i = 0; i < nf::kObsDim; ++i) obs[i] = 0.1f * float(i % 7) - 0.3f;
  const bool f_ok = nf::validate(nf_gen::kPolicyF32) && nf::policy_crc32(nf_gen::kPolicyF32) == nf_gen::kPolicyF32Crc;
  const bool q_ok = nf::validate(nf_gen::kPolicyI8) && nf::policy_crc32(nf_gen::kPolicyI8) == nf_gen::kPolicyI8Crc;
  Serial.printf("policy %s  f32 crc %s  int8 crc %s\n", nf_gen::kPolicyName, f_ok ? "ok" : "BAD", q_ok ? "ok" : "BAD");
  const float tf = time_us(run_f32, 2000);
  const float tq = time_us(run_i8, 2000);
  Serial.printf("float32: %.1f us/inference (%.1f %% of a core at 500 Hz)\n", tf, tf / 20.0f);
  Serial.printf("int8:    %.1f us/inference (%.1f %% of a core at 500 Hz)\n", tq, tq / 20.0f);
}

void loop() { delay(1000); }
