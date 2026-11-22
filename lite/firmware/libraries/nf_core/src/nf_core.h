// Umbrella header so the Arduino IDE / arduino-cli finds this library (it only matches top-level
// headers, not "nf/..." paths).
#pragma once
// Arduino.h defines sq() as a macro, which breaks nf::sq(float). The sketch includes Arduino.h
// first, so drop the macro before the nf headers (sketches use nf::sq, not the macro).
#ifdef sq
#undef sq
#endif
#include "nf/math.hpp"
#include "nf/filters.hpp"
#include "nf/estimator.hpp"
#include "nf/control.hpp"
#include "nf/policy.hpp"
