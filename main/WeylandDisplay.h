#ifndef WEYLAND_DISPLAY_H
#define WEYLAND_DISPLAY_H

#include <Arduino.h>

// ============================================================
// Weyland Display v0_28e
//
// Status + scope OLED module. It owns the SH1106 display and its
// low-priority Core 0 refresh task. It never touches I2S, MIDI,
// UART, SPI, or synth/control state directly.
// ============================================================

enum WeylandDisplayWaveform : uint8_t {
  WEYLAND_DISPLAY_WAVE_SINE = 0,
  WEYLAND_DISPLAY_WAVE_TRIANGLE,
  WEYLAND_DISPLAY_WAVE_SQUARE,
  WEYLAND_DISPLAY_WAVE_SAW
};

enum WeylandDisplayLfoTarget : uint8_t {
  WEYLAND_DISPLAY_LFO_OSC2 = 0,
  WEYLAND_DISPLAY_LFO_OFF,
  WEYLAND_DISPLAY_LFO_FILTER
};

enum WeylandDisplaySource : uint8_t {
  WEYLAND_DISPLAY_SOURCE_USB = 0,
  WEYLAND_DISPLAY_SOURCE_DEV,
  WEYLAND_DISPLAY_SOURCE_MID
};

struct WeylandDisplayStatus {
  uint8_t waveform;
  int8_t osc2RangeSemitones;
  uint8_t lfoTarget;
  uint8_t source;

  // Retained for compatibility with the status bridge. The v0_28e scope
  // uses one fixed 32 ms timebase, so this does not affect drawing or audio.
  float scopeFrequencyHz;

  float oscMix;
  float osc2DetuneCents;

  float lfoRateHz;
  float lfoDepth;

  float drive;
  float cutoffHz;
  float filterEnv;

  float attackMs;
  float releaseMs;
  float master;

  // Unsmoothened normalized panel readings. These are used only to
  // identify which physical control moved; displayed values above remain
  // the actual smoothed/derived synth values.
  float oscMixRaw;
  float osc2DetuneRaw;
  float lfoRateRaw;
  float lfoDepthRaw;
  float driveRaw;
  float cutoffRaw;
  float filterEnvRaw;
  float attackRaw;
  float releaseRaw;
  float masterRaw;
};

// Initializes the SH1106 on GPIO16/GPIO17. The synth continues
// normally if the OLED is missing; this function simply returns false.
bool weylandDisplayBegin();

// Submit panel/status state from the slow control-update path.
// It does not perform I2C traffic or draw in the caller.
void weylandDisplaySetStatus(const WeylandDisplayStatus& status);

// Submit the exact signed 16-bit PCM sample that will be sent to I2S.
// This makes the scope reflect OUTPUT_AMPLITUDE, master, envelope,
// velocity, and all final output scaling. It never performs I2C.
void weylandDisplayPushPcm(int16_t pcmSample);

bool weylandDisplayIsReady();

#endif
