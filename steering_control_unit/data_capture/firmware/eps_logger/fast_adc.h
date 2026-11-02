// ═══════════════════════════════════════════════════════════════════════
// EPS Data Logger — SAMD51 / SAME51 ADC, register level
//
// analogRead() takes ~10–20 µs per call and uses one ADC at a time. This
// sets both ADCs to 12-bit, VDDANA reference and hardware averaging, and
// lets VT3 (ADC1) and VT6 (ADC0) start converting within a few bus cycles
// of each other — simultaneous as far as steering signals are concerned.
//
// Timing at 48 MHz GCLK1 / 4 = 12 MHz ADC clock: 6 sample + 12 conversion
// cycles = 1.5 µs per conversion, ×4 averaging = 6 µs per result.
// ═══════════════════════════════════════════════════════════════════════
#pragma once
#include <Arduino.h>
#include "wiring_private.h"   // pinPeripheral()

namespace fastadc {

inline void syncWait(Adc* adc) {
  while (adc->SYNCBUSY.reg) {}
}

// log2Avg: 0..4 → 1..16 conversions averaged into one 12-bit result
inline void configure(Adc* adc, uint8_t log2Avg) {
  adc->CTRLA.bit.ENABLE = 0;
  syncWait(adc);
  adc->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV4;                 // GCLK1 (48 MHz, set by the core) / 4
  adc->CTRLB.reg = log2Avg ? ADC_CTRLB_RESSEL_16BIT          // averaging needs the 16-bit accumulator
                           : ADC_CTRLB_RESSEL_12BIT;
  syncWait(adc);
  adc->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;             // VDDANA = 3.3 V
  syncWait(adc);
  adc->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM(log2Avg) |        // 2^n samples...
                     ADC_AVGCTRL_ADJRES(log2Avg);            // ...divided back down to 12 bits
  syncWait(adc);
  adc->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(5);               // 6 cycles: every input is low impedance
  syncWait(adc);
  adc->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS(0);
  syncWait(adc);
  adc->CTRLA.bit.ENABLE = 1;
  syncWait(adc);
}

inline void select(Adc* adc, uint8_t ain) {
  adc->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS(ain);
  syncWait(adc);
}

inline void start(Adc* adc) {
  adc->SWTRIG.bit.START = 1;
}

// Reading RESULT clears RESRDY
inline uint16_t result(Adc* adc) {
  while (!adc->INTFLAG.bit.RESRDY) {}
  return adc->RESULT.reg;
}

inline uint16_t convert(Adc* adc, uint8_t ain) {
  select(adc, ain);
  start(adc);
  return result(adc);
}

// ain0 on ADC0 and ain1 on ADC1, started back to back
inline void convertPair(uint8_t ain0, uint8_t ain1, uint16_t& r0, uint16_t& r1) {
  select(ADC0, ain0);
  select(ADC1, ain1);
  ADC0->SWTRIG.bit.START = 1;
  ADC1->SWTRIG.bit.START = 1;
  r0 = result(ADC0);
  r1 = result(ADC1);
}

inline void begin(const uint8_t* pins, uint8_t nPins, uint8_t log2Avg) {
  for (uint8_t i = 0; i < nPins; i++) pinPeripheral(pins[i], PIO_ANALOG);
  // The core's init() already does this; repeated so the 48 MHz assumption is explicit
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1;
  GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  configure(ADC0, log2Avg);
  configure(ADC1, log2Avg);
  convert(ADC0, 0);                                          // first result after enable is discarded
  convert(ADC1, 0);
}

}  // namespace fastadc
