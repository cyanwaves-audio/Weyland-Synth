// ============================================================
// Weyland_v0_28e_fixedTimebaseScope
// ESP32-S3 + PCM5102A + MCP3208
//
// Based on Weyland_v0_26_mgpPerformanceControls.
//
// Changes from v0_26:
// - Keeps raw-polled UART1 RX GPIO15; no UART driver / ISR / ringbuffer.
// - Keeps MGP Control Change (0x03) and Pitch Bend (0x04) packets.
// - Makes the physical HOST / DEVICE switch choose exactly one MIDI source.
// - HOST: Pico MGP bridge is accepted; ESP USB MIDI packets are discarded.
// - DEVICE: ESP USB MIDI is accepted; Pico MGP packets are parsed then ignored.
// - Uses GPIO1 INPUT_PULLUP: LOW = HOST, HIGH/open = DEVICE.
// - Debounces mode changes and silences/reset-controls when switching source.
//
// USB CDC On Boot must be Disabled.
// USB Mode: USB-OTG / TinyUSB.
//
// All pots inverted globally because I wired them reversed. Eh.
// Osc2 detune is flipped again because panel direction felt inverted.
// Switches not inverted.
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include "driver/i2s_std.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "soc/soc.h"
#include "soc/uart_reg.h"
#include <math.h>

#include "USB.h"
#include "USBMIDI.h"
#include "WeylandDisplay.h"

USBMIDI MIDI("Weyland DOS-1");

// -------------------- Types --------------------

enum Waveform {
  WAVE_SINE,
  WAVE_SQUARE,
  WAVE_SAW,
  WAVE_TRIANGLE
};

enum LfoTarget {
  LFO_TARGET_OSC2,
  LFO_TARGET_OFF,
  LFO_TARGET_FILTER
};

// -------------------- Audio --------------------

const int SAMPLE_RATE = 48000;
const int AUDIO_FRAMES = 64;

const int16_t OUTPUT_AMPLITUDE = 24000;
const float SYNTH_HEADROOM_GAIN = 0.55f;

// PCM5102A
const int I2S_WS_PIN   = 35;
const int I2S_DOUT_PIN = 36;
const int I2S_BCK_PIN  = 37;

// -------------------- Direct controls --------------------

const int POT_OSC_MIX_PIN = 4;
const int POT_OSC2_DETUNE_PIN = 5;

const int OSC2_RANGE_LEFT_PIN  = 6;
const int OSC2_RANGE_RIGHT_PIN = 7;

const int LFO_TARGET_LEFT_PIN  = 9;
const int LFO_TARGET_RIGHT_PIN = 10;

const int WAVE_SINE_PIN     = 11;
const int WAVE_TRIANGLE_PIN = 12;
const int WAVE_SQUARE_PIN   = 13;
const int WAVE_SAW_PIN      = 14;

// -------------------- MIDI input source switch --------------------
//
// Wiring on the spare pole of the HOST / DEVICE DPDT switch:
//   Common -> ESP32 GPIO1
//   HOST throw -> GND
//   DEVICE throw -> leave open
//
// INPUT_PULLUP makes an open contact read HIGH.
// Therefore:
//   GPIO1 LOW  = accept Pico USB-host MGP bridge
//   GPIO1 HIGH = accept ESP USB MIDI from the panel USB-C / DAW
const int MIDI_SOURCE_MODE_PIN = 1;
const unsigned long MIDI_SOURCE_DEBOUNCE_MS = 25;

// -------------------- Pico / MGP raw UART bridge --------------------

#define MGP_UART_NUM   UART_NUM_1
#define MGP_RX_PIN     15
#define MGP_BAUD       115200

#define MGP_SYNC_A       0xA5
#define MGP_SYNC_B       0x5A
#define MGP_VER          0x01
#define MGP_MAX_PAYLOAD  16

#define MGP_NOTE_ON         0x01
#define MGP_NOTE_OFF        0x02
#define MGP_CONTROL_CHANGE  0x03
#define MGP_PITCH_BEND      0x04
#define MGP_ALL_NOTES_OFF   0x10
#define MGP_ALL_SOUND_OFF   0x11
#define MGP_ALL_CHANNELS    0xFF

const bool MGP_DEBUG_PRINT = false; //noise cause?
const unsigned long MGP_DEBUG_INTERVAL_MS = 1000;

RTC_DATA_ATTR uint32_t rtcBootCount = 0;

uint32_t mgpRxBytes = 0;
uint32_t mgpPacketsOk = 0;
uint32_t mgpChkFail = 0;
uint32_t mgpBadVer = 0;
uint32_t mgpLenTooBig = 0;
uint32_t mgpSyncLoss = 0;
uint32_t mgpSeqJump = 0;
uint32_t mgpUnknownType = 0;
uint32_t mgpNoteOnCount = 0;
uint32_t mgpNoteOffCount = 0;
uint32_t mgpCcCount = 0;
uint32_t mgpPitchBendCount = 0;
uint32_t mgpAllNotesOffCount = 0;
uint32_t mgpAllSoundOffCount = 0;
uint32_t mgpPacketsIgnoredDeviceMode = 0;
uint32_t mgpMaxFifoSeen = 0;

uint32_t usbPacketsIgnoredHostMode = 0;
uint32_t midiSourceModeSwitchCount = 0;

uint8_t mgpLastSeq = 0;
bool mgpHaveSeq = false;

uint8_t mgpLastType = 0;
uint8_t mgpLastChannel = 0;
uint8_t mgpLastNote = 0;
uint8_t mgpLastVel = 0;

unsigned long lastMgpDebugMs = 0;

// -------------------- MCP3208 --------------------

const int MCP_CS_PIN   = 39;
const int MCP_MOSI_PIN = 40;
const int MCP_MISO_PIN = 41;
const int MCP_SCK_PIN  = 42;

const uint8_t MCP_CH_LFO_RATE   = 0;
const uint8_t MCP_CH_LFO_DEPTH  = 1;
const uint8_t MCP_CH_DRIVE      = 2;
const uint8_t MCP_CH_CUTOFF     = 3;
const uint8_t MCP_CH_FILTER_ENV = 4;
const uint8_t MCP_CH_ATTACK     = 5;
const uint8_t MCP_CH_RELEASE    = 6;
const uint8_t MCP_CH_MASTER     = 7;

SPISettings mcpSpiSettings(1000000, MSBFIRST, SPI_MODE0);

// -------------------- Control timing --------------------

const unsigned long CONTROL_UPDATE_INTERVAL_MS = 10;
unsigned long lastControlUpdateMs = 0;

// -------------------- ADC --------------------

const int ADC_MAX = 4095;
const int ADC_DEADZONE_LOW = 20;
const int ADC_DEADZONE_HIGH = 4075;

const bool INVERT_ALL_POTS = true;
const bool FLIP_OSC2_DETUNE_AFTER_GLOBAL_INVERT = true;

const float OSC_MIX_SMOOTH_AMOUNT = 0.05f;
const float OSC2_DETUNE_SMOOTH_AMOUNT = 0.05f;
const float LFO_DEPTH_SMOOTH_AMOUNT = 0.05f;
const float LFO_RATE_SMOOTH_AMOUNT = 0.05f;
const float DRIVE_SMOOTH_AMOUNT = 0.05f;
const float FILTER_CUTOFF_SMOOTH_AMOUNT = 0.05f;
const float FILTER_ENV_SMOOTH_AMOUNT = 0.05f;
const float ATTACK_SMOOTH_AMOUNT = 0.05f;
const float RELEASE_SMOOTH_AMOUNT = 0.05f;
const float MASTER_SMOOTH_AMOUNT = 0.05f;

// OSC mix and Master directly change audio amplitude/sample balance.
// They are smoothed at the audio rate below so fast knob moves cannot
// create 100 Hz control-step zipper noise.
const float OSC_MIX_AUDIO_SMOOTH_MS = 12.0f;
const float MASTER_AUDIO_SMOOTH_MS = 15.0f;

// -------------------- Synth ranges --------------------

const float MASTER_TUNE_CENTS = 0.0f;

const float OSC2_DETUNE_MIN_CENTS = 0.0f;
const float OSC2_DETUNE_MAX_CENTS = 50.0f;

const float LFO_MAX_DETUNE_DEPTH_CENTS = 40.0f;

// External performance controls.
// CC1 is a separate global vibrato depth; it does not alter panel LFO depth.
// The rate is fixed to keep the wheel predictable regardless of panel LFO settings.
const float MOD_WHEEL_VIBRATO_MAX_CENTS = 50.0f;
const float MOD_WHEEL_VIBRATO_RATE_HZ = 5.0f;

// Standard expressive keyboard default. Change later if a wider bend feels better.
const float PITCH_BEND_RANGE_SEMITONES = 2.0f;

// Extended. Exponential mapping keeps slow end useful.
const float LFO_RATE_MIN_HZ = 0.05f;
const float LFO_RATE_MAX_HZ = 50.0f;

const bool FILTER_ENABLED = true;
const float FILTER_CUTOFF_MIN_HZ = 120.0f;
const float FILTER_CUTOFF_MAX_HZ = 8000.0f;
const float FILTER_ENV_MAX_OCTAVES = 4.0f;
const float FILTER_LFO_MAX_OCTAVES = 3.0f;

// More aggressive than v0_21.
const float DRIVE_MIN_GAIN = 1.0f;
const float DRIVE_MAX_GAIN = 14.0f;
const float DRIVE_HARD_BLEND_MAX = 0.55f;
const float DRIVE_POST_GAIN_MAX = 1.35f;

const float ATTACK_MIN_MS = 1.0f;
const float ATTACK_MAX_MS = 1000.0f;

const float RELEASE_MIN_MS = 20.0f;
const float RELEASE_MAX_MS = 1500.0f;

const float FREQ_SMOOTH_MS = 5.0f;
const float VELOCITY_SMOOTH_MS = 5.0f;
const float ENVELOPE_SILENCE_THRESHOLD = 0.001f;

const uint8_t MIDI_LISTEN_CHANNEL = 255;
const int MAX_HELD_NOTES = 16;

enum MidiSourceMode {
  MIDI_SOURCE_DEVICE,
  MIDI_SOURCE_HOST
};

MidiSourceMode midiSourceMode = MIDI_SOURCE_DEVICE;
MidiSourceMode midiSourceCandidate = MIDI_SOURCE_DEVICE;
unsigned long midiSourceCandidateSinceMs = 0;

// -------------------- State --------------------

struct ControlState {
  float oscMixRaw;
  float oscMixSmoothed;

  float osc2DetuneRaw;
  float osc2DetuneSmoothed;
  float osc2DetuneCents;

  int osc2RangeSemitones;

  float lfoDepthRaw;
  float lfoDepthSmoothed;
  float lfoDepthCents;

  float lfoRateRaw;
  float lfoRateSmoothed;
  float lfoRateHz;

  LfoTarget lfoTarget;

  float driveRaw;
  float driveSmoothed;

  float filterCutoffRaw;
  float filterCutoffSmoothed;
  float filterCutoffHz;

  float filterEnvRaw;
  float filterEnvSmoothed;
  float filterEnvAmount;

  float attackRaw;
  float attackSmoothed;
  float attackMs;

  float releaseRaw;
  float releaseSmoothed;
  float releaseMs;

  float masterRaw;
  float masterSmoothed;
};

struct NoteState {
  bool gate;
  float targetFreqHz;
  float currentFreqHz;
  float finalFreqHz;
  int activeMidiNote;
  int velocity;
  float targetVelocityGain;
  float currentVelocityGain;
};

struct HeldNote {
  uint8_t note;
  uint8_t velocity;
  bool keyDown;
};

enum EnvelopeStage {
  ENV_IDLE,
  ENV_ATTACK,
  ENV_SUSTAIN,
  ENV_RELEASE
};

struct EnvelopeState {
  EnvelopeStage stage;
  float level;
  float attackStep;
  float releaseStep;
};

ControlState controlState = {
  0.5f, 0.5f,

  0.15f, 0.15f, 7.0f,

  0,

  0.0f, 0.0f, 0.0f,

  0.0f, 0.0f, LFO_RATE_MIN_HZ,

  LFO_TARGET_OFF,

  0.0f, 0.0f,

  1.0f, 1.0f, FILTER_CUTOFF_MAX_HZ,

  0.0f, 0.0f, 0.0f,

  0.0f, 0.0f, ATTACK_MIN_MS,

  0.1f, 0.1f, 150.0f,

  0.0f, 0.0f
};

NoteState noteState = {
  false,
  146.83f, 146.83f, 146.83f,
  -1, 0,
  0.0f, 0.0f
};

EnvelopeState envelope = {
  ENV_IDLE,
  0.0f,
  0.0f,
  0.0f
};

Waveform currentWaveform = WAVE_SAW;

HeldNote heldNotes[MAX_HELD_NOTES];
int heldNoteCount = 0;

float osc1Phase = 0.0f;
float osc2Phase = 0.0f;
float osc1PhaseInc = 0.0f;
float osc2PhaseInc = 0.0f;

float lfoPhase = 0.0f;
float lfoPhaseInc = 0.0f;
float currentLfoValue = 0.0f;

// Independent mod-wheel vibrato state.
float modVibratoPhase = 0.0f;
float currentModVibratoCents = 0.0f;
float modWheelVibratoDepthCents = 0.0f;

// Full 14-bit pitch bend, expressed as cents for cheap per-sample use.
float pitchBendCents = 0.0f;

// CC64 state.
bool sustainPedalDown = false;

float currentOsc2DetuneCents = 7.0f;
float currentOsc2Ratio = 1.0f;
float cachedOsc2TotalCents = 999999.0f;

float filterState = 0.0f;
float filterAlpha = 1.0f;

float freqSmoothAlpha = 1.0f;
float velocitySmoothAlpha = 1.0f;

// Audio-rate smoothing states for controls that otherwise cause audible
// zippering when the 10 ms panel scanner updates quickly.
float oscMixAudioSmoothed = 0.5f;
float masterAudioSmoothed = 0.0f;
float oscMixAudioSmoothAlpha = 1.0f;
float masterAudioSmoothAlpha = 1.0f;
float masterTuneRatio = 1.0f;

i2s_chan_handle_t tx_handle = nullptr;
int16_t audioBuffer[AUDIO_FRAMES * 2];

// -------------------- Forward declarations --------------------

void handleMidiNoteOff(uint8_t noteNumber);
void handleMidiControlChange(uint8_t controller, uint8_t value);
void handleMidiPitchBend(uint8_t lsb, uint8_t msb);
void updateEnvelopeSteps();
void allSoundOffImmediate();

// -------------------- Utility --------------------

float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

float clampFloat(float x, float low, float high) {
  if (x < low) return low;
  if (x > high) return high;
  return x;
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return outMin + ((x - inMin) * (outMax - outMin) / (inMax - inMin));
}

float mapExp(float x, float outMin, float outMax) {
  x = clamp01(x);
  return outMin * powf(outMax / outMin, x);
}

float centsToRatio(float cents) {
  return powf(2.0f, cents / 1200.0f);
}

float midiNoteToFrequency(uint8_t noteNumber) {
  return 440.0f * powf(2.0f, ((float)noteNumber - 69.0f) / 12.0f);
}

float smoothingAlphaFromMs(float ms) {
  if (ms <= 0.0f) return 1.0f;
  return 1.0f - expf(-1.0f / ((ms / 1000.0f) * (float)SAMPLE_RATE));
}

int applyAdcDeadzones(int adc) {
  if (adc < ADC_DEADZONE_LOW) return 0;
  if (adc > ADC_DEADZONE_HIGH) return ADC_MAX;
  return adc;
}

float normalizePotAdc(int adc) {
  adc = applyAdcDeadzones(adc);

  float value = clamp01((float)adc / (float)ADC_MAX);

  if (INVERT_ALL_POTS) {
    value = 1.0f - value;
  }

  return clamp01(value);
}

// -------------------- MCP3208 hardware SPI --------------------

uint16_t readMcp3208Channel(uint8_t channel) {
  if (channel > 7) return 0;

  uint8_t command1 = 0b00000110 | ((channel & 0b100) >> 2);
  uint8_t command2 = (channel & 0b011) << 6;

  SPI.beginTransaction(mcpSpiSettings);
  digitalWrite(MCP_CS_PIN, LOW);

  SPI.transfer(command1);
  uint8_t highByte = SPI.transfer(command2);
  uint8_t lowByte = SPI.transfer(0x00);

  digitalWrite(MCP_CS_PIN, HIGH);
  SPI.endTransaction();

  return (((uint16_t)(highByte & 0x0F) << 8) | lowByte) & 0x0FFF;
}

float readDirectPotNormalized(int pin) {
  return normalizePotAdc(analogRead(pin));
}

float readMcpPotNormalized(uint8_t channel) {
  return normalizePotAdc((int)readMcp3208Channel(channel));
}

// -------------------- Switches --------------------

int readOsc2RangeSemitones() {
  bool left  = digitalRead(OSC2_RANGE_LEFT_PIN) == LOW;
  bool right = digitalRead(OSC2_RANGE_RIGHT_PIN) == LOW;

  if (left && !right) return -12;
  if (!left && !right) return 0;
  if (!left && right) return 12;

  return 0;
}

LfoTarget readLfoTarget() {
  bool left  = digitalRead(LFO_TARGET_LEFT_PIN) == LOW;
  bool right = digitalRead(LFO_TARGET_RIGHT_PIN) == LOW;

  if (left && !right) return LFO_TARGET_FILTER;
  if (!left && !right) return LFO_TARGET_OFF;
  if (!left && right) return LFO_TARGET_OSC2;

  return LFO_TARGET_OFF;
}

void updateWaveSelector() {
  bool sine     = digitalRead(WAVE_SINE_PIN) == LOW;
  bool triangle = digitalRead(WAVE_TRIANGLE_PIN) == LOW;
  bool square   = digitalRead(WAVE_SQUARE_PIN) == LOW;
  bool saw      = digitalRead(WAVE_SAW_PIN) == LOW;

  int activeCount = 0;
  if (sine) activeCount++;
  if (triangle) activeCount++;
  if (square) activeCount++;
  if (saw) activeCount++;

  if (activeCount != 1) {
    return;
  }

  if (sine) {
    currentWaveform = WAVE_SINE;
  } else if (triangle) {
    currentWaveform = WAVE_TRIANGLE;
  } else if (square) {
    currentWaveform = WAVE_SQUARE;
  } else if (saw) {
    currentWaveform = WAVE_SAW;
  }
}

// -------------------- Waveforms --------------------

static inline float renderWave(float phase) {
  switch (currentWaveform) {
    case WAVE_SINE:
      return sinf(2.0f * PI * phase);

    case WAVE_SQUARE:
      return (phase < 0.5f) ? 1.0f : -1.0f;

    case WAVE_SAW:
      return (2.0f * phase) - 1.0f;

    case WAVE_TRIANGLE:
      if (phase < 0.25f) return phase * 4.0f;
      if (phase < 0.75f) return 2.0f - (phase * 4.0f);
      return (phase * 4.0f) - 4.0f;

    default:
      return 0.0f;
  }
}

float waveformGain(Waveform wave) {
  switch (wave) {
    case WAVE_SINE:
      return 1.5f;

    case WAVE_TRIANGLE:
      return 1.75f;

    case WAVE_SAW:
      return 0.35f;

    case WAVE_SQUARE:
      return 0.35f;

    default:
      return 1.0f;
  }
}

// -------------------- Controls --------------------

void setupControls() {
  pinMode(POT_OSC_MIX_PIN, INPUT);
  pinMode(POT_OSC2_DETUNE_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(POT_OSC_MIX_PIN, ADC_11db);
  analogSetPinAttenuation(POT_OSC2_DETUNE_PIN, ADC_11db);

  pinMode(OSC2_RANGE_LEFT_PIN, INPUT_PULLUP);
  pinMode(OSC2_RANGE_RIGHT_PIN, INPUT_PULLUP);

  pinMode(LFO_TARGET_LEFT_PIN, INPUT_PULLUP);
  pinMode(LFO_TARGET_RIGHT_PIN, INPUT_PULLUP);

  pinMode(WAVE_SINE_PIN, INPUT_PULLUP);
  pinMode(WAVE_TRIANGLE_PIN, INPUT_PULLUP);
  pinMode(WAVE_SQUARE_PIN, INPUT_PULLUP);
  pinMode(WAVE_SAW_PIN, INPUT_PULLUP);

  pinMode(MCP_CS_PIN, OUTPUT);
  digitalWrite(MCP_CS_PIN, HIGH);

  SPI.begin(MCP_SCK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN, MCP_CS_PIN);
}

void readControlInputs() {
  controlState.oscMixRaw = readDirectPotNormalized(POT_OSC_MIX_PIN);

  controlState.osc2DetuneRaw = readDirectPotNormalized(POT_OSC2_DETUNE_PIN);
  if (FLIP_OSC2_DETUNE_AFTER_GLOBAL_INVERT) {
    controlState.osc2DetuneRaw = 1.0f - controlState.osc2DetuneRaw;
  }

  controlState.lfoRateRaw = readMcpPotNormalized(MCP_CH_LFO_RATE);
  controlState.lfoDepthRaw = readMcpPotNormalized(MCP_CH_LFO_DEPTH);
  controlState.driveRaw = readMcpPotNormalized(MCP_CH_DRIVE);
  controlState.filterCutoffRaw = readMcpPotNormalized(MCP_CH_CUTOFF);
  controlState.filterEnvRaw = readMcpPotNormalized(MCP_CH_FILTER_ENV);
  controlState.attackRaw = readMcpPotNormalized(MCP_CH_ATTACK);
  controlState.releaseRaw = readMcpPotNormalized(MCP_CH_RELEASE);
  controlState.masterRaw = readMcpPotNormalized(MCP_CH_MASTER);

  controlState.osc2RangeSemitones = readOsc2RangeSemitones();
  controlState.lfoTarget = readLfoTarget();

  updateWaveSelector();
}

void smoothControls(bool immediate) {
  if (immediate) {
    controlState.oscMixSmoothed = controlState.oscMixRaw;
    controlState.osc2DetuneSmoothed = controlState.osc2DetuneRaw;
    controlState.lfoDepthSmoothed = controlState.lfoDepthRaw;
    controlState.lfoRateSmoothed = controlState.lfoRateRaw;
    controlState.driveSmoothed = controlState.driveRaw;
    controlState.filterCutoffSmoothed = controlState.filterCutoffRaw;
    controlState.filterEnvSmoothed = controlState.filterEnvRaw;
    controlState.attackSmoothed = controlState.attackRaw;
    controlState.releaseSmoothed = controlState.releaseRaw;
    controlState.masterSmoothed = controlState.masterRaw;
    return;
  }

  controlState.oscMixSmoothed +=
    (controlState.oscMixRaw - controlState.oscMixSmoothed) * OSC_MIX_SMOOTH_AMOUNT;

  controlState.osc2DetuneSmoothed +=
    (controlState.osc2DetuneRaw - controlState.osc2DetuneSmoothed) * OSC2_DETUNE_SMOOTH_AMOUNT;

  controlState.lfoDepthSmoothed +=
    (controlState.lfoDepthRaw - controlState.lfoDepthSmoothed) * LFO_DEPTH_SMOOTH_AMOUNT;

  controlState.lfoRateSmoothed +=
    (controlState.lfoRateRaw - controlState.lfoRateSmoothed) * LFO_RATE_SMOOTH_AMOUNT;

  controlState.driveSmoothed +=
    (controlState.driveRaw - controlState.driveSmoothed) * DRIVE_SMOOTH_AMOUNT;

  controlState.filterCutoffSmoothed +=
    (controlState.filterCutoffRaw - controlState.filterCutoffSmoothed) * FILTER_CUTOFF_SMOOTH_AMOUNT;

  controlState.filterEnvSmoothed +=
    (controlState.filterEnvRaw - controlState.filterEnvSmoothed) * FILTER_ENV_SMOOTH_AMOUNT;

  controlState.attackSmoothed +=
    (controlState.attackRaw - controlState.attackSmoothed) * ATTACK_SMOOTH_AMOUNT;

  controlState.releaseSmoothed +=
    (controlState.releaseRaw - controlState.releaseSmoothed) * RELEASE_SMOOTH_AMOUNT;

  controlState.masterSmoothed +=
    (controlState.masterRaw - controlState.masterSmoothed) * MASTER_SMOOTH_AMOUNT;
}

void deriveControlValues() {
  controlState.osc2DetuneCents =
    mapFloat(
      controlState.osc2DetuneSmoothed,
      0.0f,
      1.0f,
      OSC2_DETUNE_MIN_CENTS,
      OSC2_DETUNE_MAX_CENTS
    );

  controlState.lfoDepthCents =
    controlState.lfoDepthSmoothed * LFO_MAX_DETUNE_DEPTH_CENTS;

  controlState.lfoRateHz =
    mapExp(
      controlState.lfoRateSmoothed,
      LFO_RATE_MIN_HZ,
      LFO_RATE_MAX_HZ
    );

  // Exponential frequency mapping keeps the useful low/mid filter range
  // spread across more of the pot travel.
  controlState.filterCutoffHz =
    mapExp(
      controlState.filterCutoffSmoothed,
      FILTER_CUTOFF_MIN_HZ,
      FILTER_CUTOFF_MAX_HZ
    );

  controlState.filterEnvAmount = clamp01(controlState.filterEnvSmoothed);

  controlState.attackMs =
    mapFloat(
      controlState.attackSmoothed,
      0.0f,
      1.0f,
      ATTACK_MIN_MS,
      ATTACK_MAX_MS
    );

  controlState.releaseMs =
    mapFloat(
      controlState.releaseSmoothed,
      0.0f,
      1.0f,
      RELEASE_MIN_MS,
      RELEASE_MAX_MS
    );

  updateEnvelopeSteps();
}

void updateControls() {
  readControlInputs();
  smoothControls(false);
  deriveControlValues();
}

void initializeControlState() {
  // Let ADC/SPI/control rails settle.
  delay(30);

  // Throw away early reads. MCP/ADC startup can be a little goblin.
  for (int i = 0; i < 6; i++) {
    readControlInputs();
    delay(3);
  }

  smoothControls(true);
  deriveControlValues();

  // Explicitly force envelope timing from startup pot state.
  updateEnvelopeSteps();
}

bool updateControlsIfDue() {
  unsigned long now = millis();

  if (now - lastControlUpdateMs < CONTROL_UPDATE_INTERVAL_MS) {
    return false;
  }

  lastControlUpdateMs = now;
  updateControls();
  return true;
}

// -------------------- OLED status bridge --------------------
//
// Runs only after a slow control update. It only copies panel/state data
// into the display module; it never performs I2C work in the audio loop.

void updateDisplayStatusFromControls() {
  WeylandDisplayStatus status = {};

  switch (currentWaveform) {
    case WAVE_SINE:
      status.waveform = WEYLAND_DISPLAY_WAVE_SINE;
      break;

    case WAVE_TRIANGLE:
      status.waveform = WEYLAND_DISPLAY_WAVE_TRIANGLE;
      break;

    case WAVE_SQUARE:
      status.waveform = WEYLAND_DISPLAY_WAVE_SQUARE;
      break;

    case WAVE_SAW:
    default:
      status.waveform = WEYLAND_DISPLAY_WAVE_SAW;
      break;
  }

  status.osc2RangeSemitones = controlState.osc2RangeSemitones;

  switch (controlState.lfoTarget) {
    case LFO_TARGET_FILTER:
      status.lfoTarget = WEYLAND_DISPLAY_LFO_FILTER;
      break;

    case LFO_TARGET_OSC2:
      status.lfoTarget = WEYLAND_DISPLAY_LFO_OSC2;
      break;

    case LFO_TARGET_OFF:
    default:
      status.lfoTarget = WEYLAND_DISPLAY_LFO_OFF;
      break;
  }

  status.source =
    (midiSourceMode == MIDI_SOURCE_HOST)
      ? WEYLAND_DISPLAY_SOURCE_MID
      : WEYLAND_DISPLAY_SOURCE_USB;

  // ----------------------------------------------------------
  // Display values use the raw physical pot positions.
  //
  // Audio keeps using the existing smoothed controlState values.
  // This makes the OLED tag follow the panel immediately instead
  // of waiting for the synth smoothing tail to settle.
  // ----------------------------------------------------------

  status.oscMix = controlState.oscMixRaw;

  status.osc2DetuneCents =
    mapFloat(
      controlState.osc2DetuneRaw,
      0.0f,
      1.0f,
      OSC2_DETUNE_MIN_CENTS,
      OSC2_DETUNE_MAX_CENTS
    );

  status.lfoRateHz =
    mapExp(
      controlState.lfoRateRaw,
      LFO_RATE_MIN_HZ,
      LFO_RATE_MAX_HZ
    );

  status.lfoDepth = controlState.lfoDepthRaw;
  status.drive = controlState.driveRaw;

  status.cutoffHz =
    mapExp(
      controlState.filterCutoffRaw,
      FILTER_CUTOFF_MIN_HZ,
      FILTER_CUTOFF_MAX_HZ
    );

  status.filterEnv = controlState.filterEnvRaw;

  status.attackMs =
    mapFloat(
      controlState.attackRaw,
      0.0f,
      1.0f,
      ATTACK_MIN_MS,
      ATTACK_MAX_MS
    );

  status.releaseMs =
    mapFloat(
      controlState.releaseRaw,
      0.0f,
      1.0f,
      RELEASE_MIN_MS,
      RELEASE_MAX_MS
    );

  status.master = controlState.masterRaw;

  // ----------------------------------------------------------
  // Raw values are also passed through unchanged for the b4
  // touch detector. Without these fields every knob appears
  // permanently unchanged to the display module.
  // ----------------------------------------------------------

  status.oscMixRaw = controlState.oscMixRaw;
  status.osc2DetuneRaw = controlState.osc2DetuneRaw;

  status.lfoRateRaw = controlState.lfoRateRaw;
  status.lfoDepthRaw = controlState.lfoDepthRaw;

  status.driveRaw = controlState.driveRaw;
  status.cutoffRaw = controlState.filterCutoffRaw;
  status.filterEnvRaw = controlState.filterEnvRaw;

  status.attackRaw = controlState.attackRaw;
  status.releaseRaw = controlState.releaseRaw;
  status.masterRaw = controlState.masterRaw;

  // Retained for display-status compatibility. v0_28e uses a fixed 32 ms
  // scope timebase, so note-off cannot switch the horizontal scale.
  status.scopeFrequencyHz = noteState.finalFreqHz;

  weylandDisplaySetStatus(status);
}

// -------------------- LFO / osc2 --------------------

void updateOsc2RatioIfNeeded() {
  float totalOsc2Cents =
    ((float)controlState.osc2RangeSemitones * 100.0f) +
    currentOsc2DetuneCents;

  if (fabsf(totalOsc2Cents - cachedOsc2TotalCents) > 0.0001f) {
    currentOsc2Ratio = centsToRatio(totalOsc2Cents);
    cachedOsc2TotalCents = totalOsc2Cents;
  }
}

void setupLfo() {
  lfoPhase = 0.0f;
  currentLfoValue = 0.0f;

  modVibratoPhase = 0.0f;
  currentModVibratoCents = 0.0f;
  modWheelVibratoDepthCents = 0.0f;
  pitchBendCents = 0.0f;

  currentOsc2DetuneCents = controlState.osc2DetuneCents;
  cachedOsc2TotalCents = 999999.0f;
  updateOsc2RatioIfNeeded();

  lfoPhaseInc = controlState.lfoRateHz / (float)SAMPLE_RATE;
}

static inline void updateLfoOneSample() {
  lfoPhaseInc = controlState.lfoRateHz / (float)SAMPLE_RATE;

  currentLfoValue = sinf(2.0f * PI * lfoPhase);

  currentOsc2DetuneCents = controlState.osc2DetuneCents;

  if (controlState.lfoTarget == LFO_TARGET_OSC2) {
    currentOsc2DetuneCents +=
      currentLfoValue * controlState.lfoDepthCents;
  }

  updateOsc2RatioIfNeeded();

  lfoPhase += lfoPhaseInc;
  if (lfoPhase >= 1.0f) {
    lfoPhase -= 1.0f;
  }
}

static inline void updateModWheelVibratoOneSample() {
  float modWave = sinf(2.0f * PI * modVibratoPhase);

  currentModVibratoCents =
    modWave * modWheelVibratoDepthCents;

  modVibratoPhase += MOD_WHEEL_VIBRATO_RATE_HZ / (float)SAMPLE_RATE;
  if (modVibratoPhase >= 1.0f) {
    modVibratoPhase -= 1.0f;
  }
}

// -------------------- Drive --------------------

static inline float processDriveOneSample(float input) {
  float driveAmount = clamp01(controlState.driveSmoothed);

  float driveGain =
    mapFloat(
      driveAmount,
      0.0f,
      1.0f,
      DRIVE_MIN_GAIN,
      DRIVE_MAX_GAIN
    );

  float pre = input * driveGain;

  float soft = tanhf(pre);
  float hard = clampFloat(pre * 0.75f, -1.0f, 1.0f);

  float hardBlend = driveAmount * DRIVE_HARD_BLEND_MAX;
  float driven =
    (soft * (1.0f - hardBlend)) +
    (hard * hardBlend);

  float dryWet = powf(driveAmount, 0.65f);

  float out =
    (input * (1.0f - dryWet)) +
    (driven * dryWet);

  float postGain =
    mapFloat(
      driveAmount,
      0.0f,
      1.0f,
      1.0f,
      DRIVE_POST_GAIN_MAX
    );

  return clampFloat(out * postGain, -2.0f, 2.0f);
}

// -------------------- Filter --------------------

void updateDynamicFilterCoefficient() {
  float cutoff = controlState.filterCutoffHz;

  if (controlState.filterEnvAmount > 0.001f) {
    float envOctaves =
      envelope.level *
      controlState.filterEnvAmount *
      FILTER_ENV_MAX_OCTAVES;

    cutoff *= powf(2.0f, envOctaves);
  }

  if (controlState.lfoTarget == LFO_TARGET_FILTER) {
    float lfoOctaves =
      currentLfoValue *
      controlState.lfoDepthSmoothed *
      FILTER_LFO_MAX_OCTAVES;

    cutoff *= powf(2.0f, lfoOctaves);
  }

  cutoff = clampFloat(cutoff, 20.0f, 18000.0f);

  filterAlpha =
    1.0f - expf(-2.0f * PI * cutoff / (float)SAMPLE_RATE);
}

void setupFilter() {
  filterState = 0.0f;
  updateDynamicFilterCoefficient();
}

static inline float processLowpassOneSample(float input) {
  if (!FILTER_ENABLED) {
    return input;
  }

  filterState += filterAlpha * (input - filterState);
  return filterState;
}

// -------------------- Envelope --------------------

void updateEnvelopeSteps() {
  envelope.attackStep =
    1.0f / ((controlState.attackMs / 1000.0f) * SAMPLE_RATE);

  envelope.releaseStep =
    1.0f / ((controlState.releaseMs / 1000.0f) * SAMPLE_RATE);

  if (envelope.attackStep <= 0.0f) envelope.attackStep = 1.0f;
  if (envelope.releaseStep <= 0.0f) envelope.releaseStep = 1.0f;
}

void setupEnvelope() {
  envelope.level = 0.0f;
  envelope.stage = ENV_IDLE;

  // Re-apply startup pot-derived envelope times.
  updateEnvelopeSteps();
}

void setupSmoothing() {
  freqSmoothAlpha = smoothingAlphaFromMs(FREQ_SMOOTH_MS);
  velocitySmoothAlpha = smoothingAlphaFromMs(VELOCITY_SMOOTH_MS);

  oscMixAudioSmoothAlpha =
    smoothingAlphaFromMs(OSC_MIX_AUDIO_SMOOTH_MS);

  masterAudioSmoothAlpha =
    smoothingAlphaFromMs(MASTER_AUDIO_SMOOTH_MS);

  // Start from the settled panel state so boot does not fade in from stale values.
  oscMixAudioSmoothed = controlState.oscMixRaw;
  masterAudioSmoothed = controlState.masterRaw;
}

static inline void updateAudioControlSmoothingOneSample() {
  oscMixAudioSmoothed +=
    (controlState.oscMixRaw - oscMixAudioSmoothed) *
    oscMixAudioSmoothAlpha;

  masterAudioSmoothed +=
    (controlState.masterRaw - masterAudioSmoothed) *
    masterAudioSmoothAlpha;
}

void setupTuning() {
  masterTuneRatio = centsToRatio(MASTER_TUNE_CENTS);
}

void envelopeGateOn() {
  envelope.stage = ENV_ATTACK;
}

void envelopeGateOff() {
  envelope.stage = ENV_RELEASE;
}

static inline void updateEnvelopeOneSample() {
  switch (envelope.stage) {
    case ENV_IDLE:
      envelope.level = 0.0f;
      noteState.velocity = 0;
      noteState.targetVelocityGain = 0.0f;
      noteState.currentVelocityGain = 0.0f;
      break;

    case ENV_ATTACK:
      envelope.level += envelope.attackStep;
      if (envelope.level >= 1.0f) {
        envelope.level = 1.0f;
        envelope.stage = ENV_SUSTAIN;
      }
      break;

    case ENV_SUSTAIN:
      envelope.level = 1.0f;
      break;

    case ENV_RELEASE:
      envelope.level -= envelope.releaseStep;
      if (envelope.level <= 0.0f) {
        envelope.level = 0.0f;
        envelope.stage = ENV_IDLE;
        noteState.velocity = 0;
        noteState.targetVelocityGain = 0.0f;
        noteState.currentVelocityGain = 0.0f;
      }
      break;
  }
}

// -------------------- Voice --------------------

void updateSmoothVoiceOneSample() {
  noteState.currentFreqHz +=
    (noteState.targetFreqHz - noteState.currentFreqHz) * freqSmoothAlpha;

  noteState.currentVelocityGain +=
    (noteState.targetVelocityGain - noteState.currentVelocityGain) * velocitySmoothAlpha;
}

void updateFinalFrequency() {
  // Pitch bend and CC1 vibrato are global pitch modulation.
  // Both oscillators move together, while panel LFO can still detune osc2 separately.
  float globalPitchCents =
    pitchBendCents + currentModVibratoCents;

  float globalPitchRatio = centsToRatio(globalPitchCents);

  noteState.finalFreqHz =
    noteState.currentFreqHz *
    masterTuneRatio *
    globalPitchRatio;

  osc1PhaseInc = noteState.finalFreqHz / (float)SAMPLE_RATE;
  osc2PhaseInc = (noteState.finalFreqHz * currentOsc2Ratio) / (float)SAMPLE_RATE;
}

void setTargetNoteFrequency(float freqHz, bool immediate) {
  noteState.targetFreqHz = freqHz;

  if (immediate) {
    noteState.currentFreqHz = freqHz;
  }

  updateFinalFrequency();
}

void switchToNote(
  uint8_t noteNumber,
  uint8_t velocity,
  bool resetPhase,
  bool triggerEnvelope
) {
  noteState.activeMidiNote = noteNumber;
  noteState.velocity = velocity;

  float newFreq = midiNoteToFrequency(noteNumber);
  float newVelocityGain = clamp01((float)velocity / 127.0f);

  noteState.targetVelocityGain = newVelocityGain;

  if (resetPhase) {
    osc1Phase = 0.0f;
    osc2Phase = 0.25f;
    noteState.currentVelocityGain = newVelocityGain;
    setTargetNoteFrequency(newFreq, true);
  } else {
    setTargetNoteFrequency(newFreq, false);
  }

  if (triggerEnvelope) {
    envelopeGateOn();
  }

  noteState.gate = true;
}

void releaseActiveVoice() {
  noteState.gate = false;
  noteState.activeMidiNote = -1;
  envelopeGateOff();
}

float velocityToGain() {
  return clamp01(noteState.currentVelocityGain);
}

// -------------------- Held notes / sustain --------------------

int findHeldNoteIndex(uint8_t noteNumber) {
  for (int i = 0; i < heldNoteCount; i++) {
    if (heldNotes[i].note == noteNumber) {
      return i;
    }
  }

  return -1;
}

void removeHeldNoteAtIndex(int index) {
  if (index < 0 || index >= heldNoteCount) {
    return;
  }

  for (int i = index; i < heldNoteCount - 1; i++) {
    heldNotes[i] = heldNotes[i + 1];
  }

  heldNoteCount--;
}

void removeHeldNote(uint8_t noteNumber) {
  int index = findHeldNoteIndex(noteNumber);

  if (index >= 0) {
    removeHeldNoteAtIndex(index);
  }
}

void pushHeldNote(uint8_t noteNumber, uint8_t velocity) {
  // The mono stack treats repeated instances of a note as the newest press.
  removeHeldNote(noteNumber);

  if (heldNoteCount >= MAX_HELD_NOTES) {
    removeHeldNoteAtIndex(0);
  }

  heldNotes[heldNoteCount].note = noteNumber;
  heldNotes[heldNoteCount].velocity = velocity;
  heldNotes[heldNoteCount].keyDown = true;
  heldNoteCount++;
}

bool getNewestHeldNote(HeldNote *outNote) {
  if (heldNoteCount <= 0) {
    return false;
  }

  *outNote = heldNotes[heldNoteCount - 1];
  return true;
}

void removeSustainedReleasedNotes() {
  // Remove only notes whose keys were released while CC64 was down.
  for (int i = heldNoteCount - 1; i >= 0; --i) {
    if (!heldNotes[i].keyDown) {
      removeHeldNoteAtIndex(i);
    }
  }
}

void clearHeldNotes() {
  heldNoteCount = 0;
}

void handleMidiNoteOn(uint8_t noteNumber, uint8_t velocity) {
  if (velocity == 0) {
    handleMidiNoteOff(noteNumber);
    return;
  }

  bool hadHeldNotes = heldNoteCount > 0;

  bool isTrulySilent =
    (envelope.stage == ENV_IDLE) ||
    (envelope.level <= ENVELOPE_SILENCE_THRESHOLD);

  pushHeldNote(noteNumber, velocity);

  if (isTrulySilent) {
    switchToNote(noteNumber, velocity, true, true);
  } else if (hadHeldNotes) {
    switchToNote(noteNumber, velocity, false, false);
  } else {
    switchToNote(noteNumber, velocity, false, true);
  }
}

void handleMidiNoteOff(uint8_t noteNumber) {
  int index = findHeldNoteIndex(noteNumber);

  if (index < 0) {
    return;
  }

  if (sustainPedalDown) {
    // Keep it audible and eligible as the active voice until CC64 is released.
    heldNotes[index].keyDown = false;

    if (noteState.activeMidiNote == noteNumber) {
      return;
    }

    return;
  }

  removeHeldNoteAtIndex(index);

  if (noteState.activeMidiNote != noteNumber) {
    return;
  }

  HeldNote fallbackNote;

  if (getNewestHeldNote(&fallbackNote)) {
    switchToNote(fallbackNote.note, fallbackNote.velocity, false, false);
  } else {
    releaseActiveVoice();
  }
}

void handleSustainPedal(uint8_t value) {
  bool newSustainState = value >= 64;

  if (newSustainState == sustainPedalDown) {
    return;
  }

  sustainPedalDown = newSustainState;

  if (sustainPedalDown) {
    return;
  }

  // Pedal released: discard key-up notes and resolve the mono priority voice.
  removeSustainedReleasedNotes();

  if (noteState.activeMidiNote < 0) {
    return;
  }

  if (findHeldNoteIndex((uint8_t)noteState.activeMidiNote) >= 0) {
    return;
  }

  HeldNote fallbackNote;

  if (getNewestHeldNote(&fallbackNote)) {
    switchToNote(fallbackNote.note, fallbackNote.velocity, false, false);
  } else {
    releaseActiveVoice();
  }
}

void allNotesOff() {
  clearHeldNotes();

  if (noteState.activeMidiNote >= 0 || envelope.stage != ENV_IDLE) {
    releaseActiveVoice();
  }
}

void allSoundOffImmediate() {
  clearHeldNotes();

  // Reset external-performance state too. This prevents a source switch,
  // controller unplug, or panic packet from leaving latent bend/mod/pedal state.
  sustainPedalDown = false;
  pitchBendCents = 0.0f;
  modWheelVibratoDepthCents = 0.0f;
  currentModVibratoCents = 0.0f;

  noteState.gate = false;
  noteState.activeMidiNote = -1;
  noteState.velocity = 0;
  noteState.targetVelocityGain = 0.0f;
  noteState.currentVelocityGain = 0.0f;

  envelope.stage = ENV_IDLE;
  envelope.level = 0.0f;

  filterState = 0.0f;
}

const char* midiSourceModeName(MidiSourceMode mode) {
  return (mode == MIDI_SOURCE_HOST) ? "HOST" : "DEVICE";
}

MidiSourceMode readMidiSourceModePin() {
  // LOW is the HOST throw shorted to GND.
  // HIGH is the DEVICE throw left open, read through INPUT_PULLUP.
  return (digitalRead(MIDI_SOURCE_MODE_PIN) == LOW)
    ? MIDI_SOURCE_HOST
    : MIDI_SOURCE_DEVICE;
}

void setupMidiSourceModeGate() {
  pinMode(MIDI_SOURCE_MODE_PIN, INPUT_PULLUP);

  MidiSourceMode startupMode = readMidiSourceModePin();
  midiSourceMode = startupMode;
  midiSourceCandidate = startupMode;
  midiSourceCandidateSinceMs = millis();

  Serial.printf(
    "MIDI source GPIO%d: %s (LOW=HOST, HIGH=DEVICE)\n",
    MIDI_SOURCE_MODE_PIN,
    midiSourceModeName(midiSourceMode)
  );
}

void updateMidiSourceModeGate() {
  MidiSourceMode observedMode = readMidiSourceModePin();
  unsigned long now = millis();

  if (observedMode != midiSourceCandidate) {
    midiSourceCandidate = observedMode;
    midiSourceCandidateSinceMs = now;
    return;
  }

  if (midiSourceCandidate == midiSourceMode) {
    return;
  }

  if (now - midiSourceCandidateSinceMs < MIDI_SOURCE_DEBOUNCE_MS) {
    return;
  }

  // Cleanly end any notes originating from the old source before admitting
  // the new one. USB and MGP input queues are both continuously drained.
  allSoundOffImmediate();

  midiSourceMode = midiSourceCandidate;
  midiSourceModeSwitchCount++;

  Serial.printf(
    "MIDI source switched to %s\n",
    midiSourceModeName(midiSourceMode)
  );
}

void handleMidiPitchBend(uint8_t lsb, uint8_t msb) {
  const uint16_t bend14 =
    ((uint16_t)(msb & 0x7F) << 7) |
    (uint16_t)(lsb & 0x7F);

  // MIDI center is exactly 8192. Keep the positive and negative ends symmetric.
  float normalizedBend =
    (bend14 >= 8192)
      ? ((float)((int)bend14 - 8192) / 8191.0f)
      : ((float)((int)bend14 - 8192) / 8192.0f);

  pitchBendCents =
    normalizedBend *
    PITCH_BEND_RANGE_SEMITONES *
    100.0f;
}

void handleMidiControlChange(uint8_t controller, uint8_t value) {
  switch (controller) {
    case 1: // CC1: mod wheel -> independent global vibrato depth.
      modWheelVibratoDepthCents =
        ((float)(value & 0x7F) / 127.0f) *
        MOD_WHEEL_VIBRATO_MAX_CENTS;
      break;

    case 64: // CC64: sustain pedal.
      handleSustainPedal(value);
      break;

    case 120: // All Sound Off.
      allSoundOffImmediate();
      break;

    case 123: // All Notes Off.
      allNotesOff();
      break;

    default:
      break;
  }
}

void setupUsbMidi() {
  USB.productName("Weyland DOS-1");
  USB.manufacturerName("Tenhauser");

  MIDI.begin();
  USB.begin();
}

void handleMidiPacket(const midiEventPacket_t &packet) {
  // Always read USB packets so they cannot accumulate, but only DEVICE mode
  // is allowed to influence the synth.
  if (midiSourceMode != MIDI_SOURCE_DEVICE) {
    usbPacketsIgnoredHostMode++;
    return;
  }

  uint8_t status = packet.byte1;
  uint8_t data1 = packet.byte2;
  uint8_t data2 = packet.byte3;

  uint8_t messageType = status & 0xF0;
  uint8_t channel = status & 0x0F;

  if (MIDI_LISTEN_CHANNEL != 255 && channel != MIDI_LISTEN_CHANNEL) {
    return;
  }

  switch (messageType) {
    case 0x90:
      handleMidiNoteOn(data1, data2);
      break;

    case 0x80:
      handleMidiNoteOff(data1);
      break;

    case 0xB0:
      handleMidiControlChange(data1, data2);
      break;

    case 0xE0:
      handleMidiPitchBend(data1, data2);
      break;

    default:
      break;
  }
}

void updateMidiInput() {
  midiEventPacket_t packet;

  while (MIDI.readPacket(&packet)) {
    handleMidiPacket(packet);
  }
}

// -------------------- MGP raw UART bridge --------------------

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
  }
}

void setupMgpRawUart() {
  uart_config_t cfg = {};
  cfg.baud_rate = MGP_BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity    = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;

  ESP_ERROR_CHECK(uart_param_config(MGP_UART_NUM, &cfg));

  ESP_ERROR_CHECK(uart_set_pin(
    MGP_UART_NUM,
    UART_PIN_NO_CHANGE, // TX unused
    MGP_RX_PIN,
    UART_PIN_NO_CHANGE,
    UART_PIN_NO_CHANGE
  ));

  // Deliberately no uart_driver_install().
  // This avoids the ESP-IDF UART ISR/ringbuffer path that triggered INT_WDT
  // when combined with I2S/DMA in v0_24 diagnostics.
}

bool mgpChannelAccepted(uint8_t channel) {
  if (channel == MGP_ALL_CHANNELS) {
    return true;
  }

  if (MIDI_LISTEN_CHANNEL == 255) {
    return true;
  }

  return channel == MIDI_LISTEN_CHANNEL;
}

void handleMgpPacket(uint8_t type, const uint8_t *payload, uint8_t len) {
  mgpPacketsOk++;
  mgpLastType = type;

  // The raw UART parser keeps draining and validating packets in DEVICE mode,
  // but only HOST mode may apply them to the synth.
  if (midiSourceMode != MIDI_SOURCE_HOST) {
    mgpPacketsIgnoredDeviceMode++;
    return;
  }

  switch (type) {
    case MGP_NOTE_ON: {
      if (len != 3) {
        mgpUnknownType++;
        return;
      }

      uint8_t channel = payload[0];
      uint8_t note = payload[1];
      uint8_t velocity = payload[2];

      mgpLastChannel = channel;
      mgpLastNote = note;
      mgpLastVel = velocity;

      if (!mgpChannelAccepted(channel)) {
        return;
      }

      mgpNoteOnCount++;
      handleMidiNoteOn(note, velocity);
      break;
    }

    case MGP_NOTE_OFF: {
      if (len != 3) {
        mgpUnknownType++;
        return;
      }

      uint8_t channel = payload[0];
      uint8_t note = payload[1];
      uint8_t releaseVelocity = payload[2];

      mgpLastChannel = channel;
      mgpLastNote = note;
      mgpLastVel = releaseVelocity;

      if (!mgpChannelAccepted(channel)) {
        return;
      }

      mgpNoteOffCount++;
      handleMidiNoteOff(note);
      break;
    }

    case MGP_CONTROL_CHANGE: {
      if (len != 3) {
        mgpUnknownType++;
        return;
      }

      uint8_t channel = payload[0];
      uint8_t controller = payload[1];
      uint8_t value = payload[2];

      mgpLastChannel = channel;
      mgpLastNote = controller;
      mgpLastVel = value;

      if (!mgpChannelAccepted(channel)) {
        return;
      }

      mgpCcCount++;
      handleMidiControlChange(controller, value);
      break;
    }

    case MGP_PITCH_BEND: {
      if (len != 3) {
        mgpUnknownType++;
        return;
      }

      uint8_t channel = payload[0];
      uint8_t lsb = payload[1];
      uint8_t msb = payload[2];

      mgpLastChannel = channel;
      mgpLastNote = lsb;
      mgpLastVel = msb;

      if (!mgpChannelAccepted(channel)) {
        return;
      }

      mgpPitchBendCount++;
      handleMidiPitchBend(lsb, msb);
      break;
    }

    case MGP_ALL_NOTES_OFF: {
      if (len != 1) {
        mgpUnknownType++;
        return;
      }

      uint8_t channel = payload[0];
      mgpLastChannel = channel;

      if (!mgpChannelAccepted(channel)) {
        return;
      }

      mgpAllNotesOffCount++;
      allNotesOff();
      break;
    }

    case MGP_ALL_SOUND_OFF: {
      if (len != 1) {
        mgpUnknownType++;
        return;
      }

      uint8_t channel = payload[0];
      mgpLastChannel = channel;

      if (!mgpChannelAccepted(channel)) {
        return;
      }

      mgpAllSoundOffCount++;
      allSoundOffImmediate();
      break;
    }

    default:
      mgpUnknownType++;
      break;
  }
}

enum MgpParseState {
  MGP_WAIT_A5,
  MGP_WAIT_5A,
  MGP_READ_VER,
  MGP_READ_SEQ,
  MGP_READ_TYPE,
  MGP_READ_LEN,
  MGP_READ_PAYLOAD,
  MGP_READ_CHK
};

MgpParseState mgpParseState = MGP_WAIT_A5;

uint8_t mgpVer = 0;
uint8_t mgpSeq = 0;
uint8_t mgpType = 0;
uint8_t mgpLen = 0;
uint8_t mgpPayload[MGP_MAX_PAYLOAD];
uint8_t mgpPayloadIndex = 0;
uint8_t mgpChecksum = 0;

void resetMgpParser() {
  mgpParseState = MGP_WAIT_A5;
  mgpPayloadIndex = 0;
  mgpChecksum = 0;
}

void parseMgpByte(uint8_t b) {
  switch (mgpParseState) {
    case MGP_WAIT_A5:
      if (b == MGP_SYNC_A) {
        mgpParseState = MGP_WAIT_5A;
      }
      break;

    case MGP_WAIT_5A:
      if (b == MGP_SYNC_B) {
        mgpParseState = MGP_READ_VER;
      } else {
        mgpSyncLoss++;
        mgpParseState = MGP_WAIT_A5;
      }
      break;

    case MGP_READ_VER:
      mgpVer = b;
      mgpChecksum = b;

      if (mgpVer != MGP_VER) {
        mgpBadVer++;
        resetMgpParser();
      } else {
        mgpParseState = MGP_READ_SEQ;
      }
      break;

    case MGP_READ_SEQ:
      mgpSeq = b;
      mgpChecksum ^= b;

      if (mgpHaveSeq) {
        uint8_t expected = mgpLastSeq + 1;
        if (mgpSeq != expected) {
          mgpSeqJump++;
        }
      }

      mgpLastSeq = mgpSeq;
      mgpHaveSeq = true;
      mgpParseState = MGP_READ_TYPE;
      break;

    case MGP_READ_TYPE:
      mgpType = b;
      mgpChecksum ^= b;
      mgpParseState = MGP_READ_LEN;
      break;

    case MGP_READ_LEN:
      mgpLen = b;
      mgpChecksum ^= b;

      if (mgpLen > MGP_MAX_PAYLOAD) {
        mgpLenTooBig++;
        resetMgpParser();
      } else if (mgpLen == 0) {
        mgpParseState = MGP_READ_CHK;
      } else {
        mgpPayloadIndex = 0;
        mgpParseState = MGP_READ_PAYLOAD;
      }
      break;

    case MGP_READ_PAYLOAD:
      mgpPayload[mgpPayloadIndex++] = b;
      mgpChecksum ^= b;

      if (mgpPayloadIndex >= mgpLen) {
        mgpParseState = MGP_READ_CHK;
      }
      break;

    case MGP_READ_CHK:
      if (b == mgpChecksum) {
        handleMgpPacket(mgpType, mgpPayload, mgpLen);
      } else {
        mgpChkFail++;
      }

      resetMgpParser();
      break;
  }
}

void pollMgpRawUart() {
  while (true) {
    uint32_t status = READ_PERI_REG(UART_STATUS_REG(MGP_UART_NUM));
    uint32_t count = (status >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT_V;

    if (count == 0) {
      break;
    }

    if (count > mgpMaxFifoSeen) {
      mgpMaxFifoSeen = count;
    }

    uint8_t b = READ_PERI_REG(UART_FIFO_REG(MGP_UART_NUM)) & 0xFF;

    mgpRxBytes++;
    parseMgpByte(b);
  }
}

void printMgpDebugIfDue() {
  if (!MGP_DEBUG_PRINT) {
    return;
  }

  unsigned long now = millis();

  if (now - lastMgpDebugMs < MGP_DEBUG_INTERVAL_MS) {
    return;
  }

  lastMgpDebugMs = now;

  Serial.printf(
    "up=%lus mode=%s modeSw=%lu usbDrop=%lu mgpRx=%lu ok=%lu mgpDrop=%lu "
    "chkFail=%lu badVer=%lu lenBig=%lu syncLoss=%lu seqJump=%lu "
    "on=%lu off=%lu cc=%lu bend=%lu allOff=%lu soundOff=%lu unk=%lu maxFifo=%lu "
    "lastType=0x%02X ch=%u data1=%u data2=%u held=%d sus=%d bendC=%.1f modC=%.1f env=%d heap=%lu\n",
    now / 1000,
    midiSourceModeName(midiSourceMode),
    midiSourceModeSwitchCount,
    usbPacketsIgnoredHostMode,
    mgpRxBytes,
    mgpPacketsOk,
    mgpPacketsIgnoredDeviceMode,
    mgpChkFail,
    mgpBadVer,
    mgpLenTooBig,
    mgpSyncLoss,
    mgpSeqJump,
    mgpNoteOnCount,
    mgpNoteOffCount,
    mgpCcCount,
    mgpPitchBendCount,
    mgpAllNotesOffCount,
    mgpAllSoundOffCount,
    mgpUnknownType,
    mgpMaxFifoSeen,
    mgpLastType,
    mgpLastChannel,
    mgpLastNote,
    mgpLastVel,
    heldNoteCount,
    sustainPedalDown ? 1 : 0,
    pitchBendCents,
    modWheelVibratoDepthCents,
    (int)envelope.stage,
    (uint32_t)ESP.getFreeHeap()
  );
}

// -------------------- I2S --------------------

void setupI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
    I2S_NUM_0,
    I2S_ROLE_MASTER
  );

  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, nullptr));

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

    .slot_cfg = {
      .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
      .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
      .slot_mode = I2S_SLOT_MODE_STEREO,
      .slot_mask = I2S_STD_SLOT_BOTH,
      .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
      .ws_pol = false,
      .bit_shift = true,
      .left_align = true,
      .big_endian = false,
      .bit_order_lsb = false
    },

    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCK_PIN,
      .ws = (gpio_num_t)I2S_WS_PIN,
      .dout = (gpio_num_t)I2S_DOUT_PIN,
      .din = I2S_GPIO_UNUSED,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false
      }
    }
  };

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
}

// -------------------- Audio render --------------------

void fillAudioBuffer() {
  float waveGain = waveformGain(currentWaveform);

  for (int i = 0; i < AUDIO_FRAMES; i++) {
    updateLfoOneSample();
    updateModWheelVibratoOneSample();
    updateSmoothVoiceOneSample();
    updateFinalFrequency();
    updateEnvelopeOneSample();
    updateAudioControlSmoothingOneSample();

    if ((i & 0x07) == 0) {
      updateDynamicFilterCoefficient();
    }

    float osc1 = renderWave(osc1Phase);
    float osc2 = renderWave(osc2Phase);

    float oscMix = clamp01(oscMixAudioSmoothed);

    float rawSample =
      ((osc1 * (1.0f - oscMix)) + (osc2 * oscMix)) *
      waveGain;

    float drivenSample = processDriveOneSample(rawSample);
    float filteredSample = processLowpassOneSample(drivenSample);

    float finalGain =
      envelope.level *
      masterAudioSmoothed *
      velocityToGain();

    float finalSample =
      filteredSample *
      finalGain *
      SYNTH_HEADROOM_GAIN;

    finalSample = clampFloat(finalSample, -1.0f, 1.0f);

    // Scope tap: actual post-master, post-envelope signal.
    // This only writes a small display capture buffer; it never uses I2C.
    //weylandDisplayPushSample(finalSample);

    int16_t out = (int16_t)(finalSample * OUTPUT_AMPLITUDE);

    weylandDisplayPushPcm(out);

    audioBuffer[i * 2 + 0] = out;
    audioBuffer[i * 2 + 1] = out;

    osc1Phase += osc1PhaseInc;
    if (osc1Phase >= 1.0f) {
      osc1Phase -= 1.0f;
    }

    osc2Phase += osc2PhaseInc;
    if (osc2Phase >= 1.0f) {
      osc2Phase -= 1.0f;
    }
  }
}

// -------------------- Init --------------------

void initializeDefaultVoice() {
  float defaultFreq = midiNoteToFrequency(50);

  noteState.targetFreqHz = defaultFreq;
  noteState.currentFreqHz = defaultFreq;
  noteState.finalFreqHz = defaultFreq;
}

// -------------------- Arduino --------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  rtcBootCount++;
  esp_reset_reason_t resetReason = esp_reset_reason();

  Serial.println();
  Serial.println("============================================");
  Serial.println("Weyland_v0_28e_fixedTimebaseScope");
  Serial.println("Based on Weyland_v0_26_mgpPerformanceControls");
  Serial.println("MIDI source gate: GPIO1, LOW=HOST, HIGH=DEVICE");
  Serial.println("MGP: raw UART FIFO polling on GPIO15");
  Serial.println("NO uart_driver_install / NO UART ISR ringbuffer");
  Serial.println("============================================");
  Serial.printf("RTC boot count: %lu\n", rtcBootCount);
  Serial.printf("Reset reason: %d / %s\n", (int)resetReason, resetReasonName(resetReason));
  Serial.printf("MGP UART RX GPIO%d @ %d baud\n", MGP_RX_PIN, MGP_BAUD);
  Serial.println();

  setupMidiSourceModeGate();

  setupUsbMidi();
  setupMgpRawUart();

  setupControls();
  initializeControlState();

  setupEnvelope();
  setupSmoothing();
  setupTuning();
  setupLfo();
  setupFilter();

  // One more explicit startup pass after envelope/filter setup.
  deriveControlValues();
  updateEnvelopeSteps();
  updateDynamicFilterCoefficient();

  initializeDefaultVoice();

  lastControlUpdateMs = millis();

  const bool oledStarted = weylandDisplayBegin();
  Serial.printf(
    "OLED status display: %s\n",
    oledStarted ? "OK" : "NOT FOUND"
  );

  // Submit the settled startup state. The display task handles I2C itself.
  updateDisplayStatusFromControls();

  setupI2S();
}

void loop() {
  // Poll first so a source hand-off takes effect before either input is used.
  updateMidiSourceModeGate();

  // MGP is always drained. handleMgpPacket() applies it only in HOST mode.
  pollMgpRawUart();

  // USB is always drained. handleMidiPacket() applies it only in DEVICE mode.
  updateMidiInput();

  pollMgpRawUart();

  if (updateControlsIfDue()) {
    updateDisplayStatusFromControls();
  }

  pollMgpRawUart();

  fillAudioBuffer();

  pollMgpRawUart();

  size_t bytesWritten = 0;

  i2s_channel_write(
    tx_handle,
    audioBuffer,
    sizeof(audioBuffer),
    &bytesWritten,
    portMAX_DELAY
  );

  pollMgpRawUart();

  printMgpDebugIfDue();
}
