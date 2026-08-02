#include "WeylandDisplay.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// Weyland Display v0_28e — live status + fixed-32ms scope
//
// - GPIO16 SDA / GPIO17 SCL
// - SH1106G at 0x3C
// - 400 kHz I2C (keeps this scope integration conservative)
// - Low-priority Core 0 display task
// - Status updates come from the slow control path
// - Scope samples come from the actual final PCM output
// - No I2C, allocation, or display drawing in fillAudioBuffer()
// ============================================================

namespace {
  constexpr uint8_t OLED_SDA = 16;
  constexpr uint8_t OLED_SCL = 17;
  constexpr uint8_t OLED_ADDR = 0x3C;

  constexpr uint8_t OLED_WIDTH = 128;
  constexpr uint8_t OLED_HEIGHT = 64;

  constexpr uint32_t OLED_I2C_CLOCK_HZ = 400000;
  constexpr uint16_t DISPLAY_TASK_INTERVAL_MS = 50;  // Max ~20 FPS while scope is active.
  constexpr uint16_t SPLASH_MS = 900;

  constexpr uint8_t TAG_FIELD_CHARS = 13;
  constexpr uint32_t TAG_IDLE_TO_MASTER_MS = 1800;

  constexpr int TAG_X = 21;
  constexpr int TAG_Y = 0;
  constexpr int TAG_W = 84;
  constexpr int TAG_H = 9;
  constexpr int TAG_TEXT_X = 24;
  constexpr int TAG_TEXT_Y = 1;

  // User-settled scope width: 192 stored points.
  constexpr uint16_t SCOPE_CAPTURE_SAMPLES = 192;

  // Fixed horizontal timebase: 192 stored points x every 8th 48 kHz
  // audio sample = 1536 audio samples = exactly 32 ms on screen.
  // This makes low notes readable without changing timebase on note-off,
  // while high notes still become naturally denser.
  constexpr uint16_t SCOPE_FIXED_SAMPLE_STRIDE = 8;

  constexpr uint16_t SCOPE_SILENCE_CLEAR_SAMPLES = 960;
  constexpr float SCOPE_SILENCE_THRESHOLD = 0.0008f;

  constexpr int SCOPE_LEFT = 1;
  constexpr int SCOPE_RIGHT = 126;
  constexpr int SCOPE_TOP = 24;
  constexpr int SCOPE_BOTTOM = 63;
  constexpr int SCOPE_MID_Y = 44;

  // OUTPUT_AMPLITUDE is currently 24000, so the maximum PCM magnitude is
  // 24000 / 32767 = 0.732. The display has 19 px below the midline.
  // 19 / 0.732 = 25.9; 24 leaves a safety margin and does not auto-range.
  constexpr float SCOPE_VERTICAL_SCALE = 24.0f;

  Adafruit_SH1106G display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

  bool displayReady = false;
  bool haveStatus = false;
  bool displayDirty = false;

  WeylandDisplayStatus pendingStatus = {};
  WeylandDisplayStatus previousStatus = {};
  WeylandDisplayStatus touchReferenceStatus = {};
  char pendingTag[TAG_FIELD_CHARS + 1] = "MASTER 0";

  enum TagParameter : uint8_t {
    TAG_PARAMETER_OSC_MIX = 0,
    TAG_PARAMETER_OSC2_DETUNE,
    TAG_PARAMETER_LFO_RATE,
    TAG_PARAMETER_LFO_DEPTH,
    TAG_PARAMETER_DRIVE,
    TAG_PARAMETER_CUTOFF,
    TAG_PARAMETER_FILTER_ENV,
    TAG_PARAMETER_ATTACK,
    TAG_PARAMETER_RELEASE,
    TAG_PARAMETER_MASTER,
    TAG_PARAMETER_NONE
  };

  TagParameter activeTagParameter = TAG_PARAMETER_MASTER;
  uint32_t lastControlTouchMs = 0;

  portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
  TaskHandle_t displayTaskHandle = nullptr;

  // Audio side writes captureBuffer. Completed frames are copied into
  // displayBuffer under a short lock. The OLED task only reads displayBuffer.
  int16_t scopeCaptureBuffer[SCOPE_CAPTURE_SAMPLES] = {};
  int16_t scopeDisplayBuffer[SCOPE_CAPTURE_SAMPLES] = {};

  uint16_t scopeCaptureIndex = 0;
  uint16_t scopeStrideCounter = 0;
  uint16_t scopeQuietSamples = 0;
  uint16_t scopeSampleStride = SCOPE_FIXED_SAMPLE_STRIDE;

  bool scopeCapturing = false;
  bool scopeHasFrame = false;
  bool scopeFramePending = false;

  int16_t scopePreviousPcm = 0;

  portMUX_TYPE scopeMux = portMUX_INITIALIZER_UNLOCKED;

  float clamp01(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
  }

  const char* waveformLabel(uint8_t waveform) {
    switch (waveform) {
      case WEYLAND_DISPLAY_WAVE_SINE:     return "SIN";
      case WEYLAND_DISPLAY_WAVE_TRIANGLE: return "TRI";
      case WEYLAND_DISPLAY_WAVE_SQUARE:   return "SQR";
      case WEYLAND_DISPLAY_WAVE_SAW:      return "SAW";
      default:                            return "---";
    }
  }

  // Display-only inverse of the current audio waveformGain() values:
  // sine 1.50, triangle 1.75, saw 0.35, square 0.35.
  // This makes equalized waveforms occupy a comparable vertical range
  // without changing PCM output, master, envelope, filter, or drive.
  // Update these four values only if waveformGain() is retuned later.
  float scopeWaveVisualGain(uint8_t waveform) {
    switch (waveform) {
      case WEYLAND_DISPLAY_WAVE_SINE:     return 1.0f / 1.50f;
      case WEYLAND_DISPLAY_WAVE_TRIANGLE: return 1.0f / 1.75f;
      case WEYLAND_DISPLAY_WAVE_SAW:      return 1.0f / 0.35f;
      case WEYLAND_DISPLAY_WAVE_SQUARE:   return 1.0f / 0.35f;
      default:                            return 1.0f;
    }
  }

  const char* sourceLabel(uint8_t source) {
    switch (source) {
      case WEYLAND_DISPLAY_SOURCE_MID: return "MID";
      case WEYLAND_DISPLAY_SOURCE_DEV: return "DEV";
      case WEYLAND_DISPLAY_SOURCE_USB:
      default:                         return "USB";
    }
  }

  const char* octaveLabel(int8_t semitones) {
    if (semitones > 0) return "OCT +12";
    if (semitones < 0) return "OCT -12";
    return "OCT   0";
  }

  const char* lfoTargetLabel(uint8_t target) {
    switch (target) {
      case WEYLAND_DISPLAY_LFO_FILTER: return "LFO FILT";
      case WEYLAND_DISPLAY_LFO_OSC2:   return "LFO OSC2";
      case WEYLAND_DISPLAY_LFO_OFF:
      default:                         return "LFO  OFF";
    }
  }

  void formatPercentTag(const char* label, float value, char* output, size_t outputSize) {
    const int percent = static_cast<int>(lroundf(clamp01(value) * 100.0f));
    snprintf(output, outputSize, "%s %d", label, percent);
  }

  void formatDetuneTag(float cents, char* output, size_t outputSize) {
    const int shownCents = static_cast<int>(lroundf(cents));
    snprintf(output, outputSize, "OSC2 DT +%dc", shownCents);
  }

  void formatLfoRateTag(float rateHz, char* output, size_t outputSize) {
    if (rateHz < 0.075f) {
      snprintf(output, outputSize, "LFO RATE .05");
      return;
    }

    if (rateHz < 1.0f) {
      int tenths = static_cast<int>(rateHz * 10.0f);
      if (tenths < 1) tenths = 1;
      if (tenths > 9) tenths = 9;
      snprintf(output, outputSize, "LFO RATE .%dHz", tenths);
      return;
    }

    const int shownRate = static_cast<int>(lroundf(rateHz));
    snprintf(output, outputSize, "LFO RATE %dHz", shownRate);
  }

  void formatCutoffTag(float cutoffHz, char* output, size_t outputSize) {
    if (cutoffHz < 1000.0f) {
      snprintf(output, outputSize, "CUTOFF %dHz", static_cast<int>(lroundf(cutoffHz)));
      return;
    }

    snprintf(output, outputSize, "CUTOFF %.1fkHz", cutoffHz / 1000.0f);
  }

  void formatTimeTag(const char* label, float milliseconds, char* output, size_t outputSize) {
    if (milliseconds >= 1000.0f) {
      snprintf(output, outputSize, "%s %.1fs", label, milliseconds / 1000.0f);
      return;
    }

    snprintf(output, outputSize, "%s %dms", label, static_cast<int>(lroundf(milliseconds)));
  }

  void fillCenteredTagField(const char* source, char* destination) {
    for (uint8_t i = 0; i < TAG_FIELD_CHARS; i++) {
      destination[i] = ' ';
    }

    destination[TAG_FIELD_CHARS] = '\0';

    uint8_t sourceLength = strlen(source);
    if (sourceLength > TAG_FIELD_CHARS) {
      sourceLength = TAG_FIELD_CHARS;
    }

    const uint8_t leftPadding = (TAG_FIELD_CHARS - sourceLength) / 2;

    for (uint8_t i = 0; i < sourceLength; i++) {
      destination[leftPadding + i] = source[i];
    }
  }

  void drawInvertedTag(const char* text) {
    char field[TAG_FIELD_CHARS + 1];
    fillCenteredTagField(text, field);

    display.fillRect(TAG_X, TAG_Y, TAG_W, TAG_H, SH110X_WHITE);

    // Leaves one white pixel row above black tag text.
    display.setTextColor(SH110X_BLACK);
    display.setCursor(TAG_TEXT_X, TAG_TEXT_Y);
    display.print(field);

    display.setTextColor(SH110X_WHITE);
  }

  void drawSplash() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(7, 28);
    display.print("WEYLAND/DOS-1 v1.0");
    display.display();
  }

  void drawScope(
    const int16_t* localScope,
    bool hasFrame,
    uint8_t waveform
  ) {
    for (int x = 3; x < 126; x += 7) {
      display.drawPixel(x, SCOPE_MID_Y, SH110X_WHITE);
    }

    if (!hasFrame) {
      return;
    }

    // Draw around the real PCM zero line. Do not re-centre each captured
    // frame: a partial waveform cycle has a non-zero mean, and subtracting
    // that mean was visually flattening/clipping sine waves.
    const float visualGain = scopeWaveVisualGain(waveform);

    int previousX = SCOPE_LEFT;
    int previousY = SCOPE_MID_Y;

    for (int x = SCOPE_LEFT; x <= SCOPE_RIGHT; x++) {
      const float normalizedX =
        static_cast<float>(x - SCOPE_LEFT) /
        static_cast<float>(SCOPE_RIGHT - SCOPE_LEFT);

      const float sourcePosition =
        normalizedX * static_cast<float>(SCOPE_CAPTURE_SAMPLES - 1);

      const uint16_t indexA = static_cast<uint16_t>(sourcePosition);
      uint16_t indexB = indexA + 1;

      if (indexB >= SCOPE_CAPTURE_SAMPLES) {
        indexB = SCOPE_CAPTURE_SAMPLES - 1;
      }

      const float fraction = sourcePosition - static_cast<float>(indexA);
      const float sampleA = static_cast<float>(localScope[indexA]);
      const float sampleB = static_cast<float>(localScope[indexB]);
      const float pcmSample = sampleA + ((sampleB - sampleA) * fraction);
      const float sample =
        (pcmSample / 32767.0f) * visualGain;

      int y = SCOPE_MID_Y -
        static_cast<int>(roundf(sample * SCOPE_VERTICAL_SCALE));

      if (y < SCOPE_TOP) y = SCOPE_TOP;
      if (y > SCOPE_BOTTOM) y = SCOPE_BOTTOM;

      if (x > SCOPE_LEFT) {
        display.drawLine(previousX, previousY, x, y, SH110X_WHITE);
      }

      previousX = x;
      previousY = y;
    }
  }

  void drawStatusScreen(
    const WeylandDisplayStatus& status,
    const char* tag,
    const int16_t* scope,
    bool hasScopeFrame
  ) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);

    display.setCursor(0, 1);
    display.print(waveformLabel(status.waveform));

    drawInvertedTag(tag);

    display.setCursor(108, 1);
    display.print(sourceLabel(status.source));

    display.setCursor(0, 12);
    display.print(octaveLabel(status.osc2RangeSemitones));

    display.setCursor(80, 12);
    display.print(lfoTargetLabel(status.lfoTarget));

    drawScope(scope, hasScopeFrame, status.waveform);

    display.display();
  }

  void drawReadyScreen() {
    WeylandDisplayStatus readyStatus = {};
    readyStatus.waveform = WEYLAND_DISPLAY_WAVE_SAW;
    readyStatus.osc2RangeSemitones = 0;
    readyStatus.lfoTarget = WEYLAND_DISPLAY_LFO_OFF;
    readyStatus.source = WEYLAND_DISPLAY_SOURCE_USB;

    int16_t blankScope[SCOPE_CAPTURE_SAMPLES] = {};
    drawStatusScreen(readyStatus, "WEYLAND READY", blankScope, false);
  }

  void formatTagForParameter(
    TagParameter parameter,
    const WeylandDisplayStatus& status,
    char* output,
    size_t outputSize
  ) {
    switch (parameter) {
      case TAG_PARAMETER_OSC_MIX:
        formatPercentTag("OSC MIX", status.oscMix, output, outputSize);
        break;

      case TAG_PARAMETER_OSC2_DETUNE:
        formatDetuneTag(status.osc2DetuneCents, output, outputSize);
        break;

      case TAG_PARAMETER_LFO_RATE:
        formatLfoRateTag(status.lfoRateHz, output, outputSize);
        break;

      case TAG_PARAMETER_LFO_DEPTH:
        formatPercentTag("LFO DEPTH", status.lfoDepth, output, outputSize);
        break;

      case TAG_PARAMETER_DRIVE:
        formatPercentTag("DRIVE", status.drive, output, outputSize);
        break;

      case TAG_PARAMETER_CUTOFF:
        formatCutoffTag(status.cutoffHz, output, outputSize);
        break;

      case TAG_PARAMETER_FILTER_ENV:
        formatPercentTag("FILTENV", status.filterEnv, output, outputSize);
        break;

      case TAG_PARAMETER_ATTACK:
        formatTimeTag("ATTACK", status.attackMs, output, outputSize);
        break;

      case TAG_PARAMETER_RELEASE:
        formatTimeTag("RELEASE", status.releaseMs, output, outputSize);
        break;

      case TAG_PARAMETER_MASTER:
      default:
        formatPercentTag("MASTER", status.master, output, outputSize);
        break;
    }
  }

  TagParameter detectTouchedParameter(
    const WeylandDisplayStatus& current,
    const WeylandDisplayStatus& reference
  ) {
    // These thresholds are deliberately larger than MCP/direct-ADC jitter.
    // Comparison is against the last accepted position rather than the
    // immediately previous read, so slow knob movement still accumulates.
    constexpr float TOUCH_THRESHOLD = 0.012f;

    float bestDelta = TOUCH_THRESHOLD;
    TagParameter bestParameter = TAG_PARAMETER_NONE;

    auto consider = [&](TagParameter parameter, float delta) {
      if (delta >= bestDelta) {
        bestDelta = delta;
        bestParameter = parameter;
      }
    };

    consider(TAG_PARAMETER_OSC_MIX,
             fabsf(current.oscMixRaw - reference.oscMixRaw));
    consider(TAG_PARAMETER_OSC2_DETUNE,
             fabsf(current.osc2DetuneRaw - reference.osc2DetuneRaw));
    consider(TAG_PARAMETER_LFO_RATE,
             fabsf(current.lfoRateRaw - reference.lfoRateRaw));
    consider(TAG_PARAMETER_LFO_DEPTH,
             fabsf(current.lfoDepthRaw - reference.lfoDepthRaw));
    consider(TAG_PARAMETER_DRIVE,
             fabsf(current.driveRaw - reference.driveRaw));
    consider(TAG_PARAMETER_CUTOFF,
             fabsf(current.cutoffRaw - reference.cutoffRaw));
    consider(TAG_PARAMETER_FILTER_ENV,
             fabsf(current.filterEnvRaw - reference.filterEnvRaw));
    consider(TAG_PARAMETER_ATTACK,
             fabsf(current.attackRaw - reference.attackRaw));
    consider(TAG_PARAMETER_RELEASE,
             fabsf(current.releaseRaw - reference.releaseRaw));
    consider(TAG_PARAMETER_MASTER,
             fabsf(current.masterRaw - reference.masterRaw));

    return bestParameter;
  }

  bool structuralStatusChanged(
    const WeylandDisplayStatus& current,
    const WeylandDisplayStatus& previous
  ) {
    return current.waveform != previous.waveform ||
           current.osc2RangeSemitones != previous.osc2RangeSemitones ||
           current.lfoTarget != previous.lfoTarget ||
           current.source != previous.source;
  }

  void displayTask(void* parameter) {
    (void)parameter;

    WeylandDisplayStatus renderStatus = {};
    char renderTag[TAG_FIELD_CHARS + 1] = "WEYLAND READY";
    int16_t renderScope[SCOPE_CAPTURE_SAMPLES] = {};
    bool renderHasScopeFrame = false;

    while (true) {
      bool shouldDraw = false;

      portENTER_CRITICAL(&statusMux);

      if (displayDirty && haveStatus) {
        renderStatus = pendingStatus;
        strncpy(renderTag, pendingTag, sizeof(renderTag));
        renderTag[sizeof(renderTag) - 1] = '\0';
        displayDirty = false;
        shouldDraw = true;
      }

      portEXIT_CRITICAL(&statusMux);

      portENTER_CRITICAL(&scopeMux);

      if (scopeFramePending) {
        renderHasScopeFrame = scopeHasFrame;

        if (scopeHasFrame) {
          memcpy(renderScope, scopeDisplayBuffer, sizeof(renderScope));
        }

        scopeFramePending = false;
        shouldDraw = true;
      }

      portEXIT_CRITICAL(&scopeMux);

      if (shouldDraw) {
        drawStatusScreen(
          renderStatus,
          renderTag,
          renderScope,
          renderHasScopeFrame
        );
      }

      vTaskDelay(pdMS_TO_TICKS(DISPLAY_TASK_INTERVAL_MS));
    }
  }

  void clearScopeFrameFromAudio() {
    portENTER_CRITICAL(&scopeMux);

    if (scopeHasFrame) {
      scopeHasFrame = false;
      scopeFramePending = true;
    }

    portEXIT_CRITICAL(&scopeMux);
  }
}

bool weylandDisplayBegin() {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(OLED_I2C_CLOCK_HZ);

  if (!display.begin(OLED_ADDR, true)) {
    displayReady = false;
    return false;
  }

  displayReady = true;

  drawSplash();
  delay(SPLASH_MS);
  drawReadyScreen();

  const BaseType_t taskCreated = xTaskCreatePinnedToCore(
    displayTask,
    "WeylandOLED",
    4096,
    nullptr,
    1,
    &displayTaskHandle,
    0
  );

  if (taskCreated != pdPASS) {
    displayTaskHandle = nullptr;
  }

  return true;
}

void weylandDisplaySetStatus(const WeylandDisplayStatus& status) {
  char nextTag[TAG_FIELD_CHARS + 1];
  const uint32_t now = millis();

  portENTER_CRITICAL(&statusMux);

  // Fixed 32 ms scope timebase. Do not change stride on note-off/release.
  // scopeFrequencyHz remains in the status struct for compatibility only.
  scopeSampleStride = SCOPE_FIXED_SAMPLE_STRIDE;

  if (!haveStatus) {
    pendingStatus = status;
    previousStatus = status;
    touchReferenceStatus = status;

    activeTagParameter = TAG_PARAMETER_MASTER;
    lastControlTouchMs = now;
    formatTagForParameter(
      TAG_PARAMETER_MASTER,
      status,
      pendingTag,
      sizeof(pendingTag)
    );

    haveStatus = true;
    displayDirty = true;

    portEXIT_CRITICAL(&statusMux);
    return;
  }

  const TagParameter touchedParameter =
    detectTouchedParameter(status, touchReferenceStatus);

  if (touchedParameter != TAG_PARAMETER_NONE) {
    activeTagParameter = touchedParameter;
    lastControlTouchMs = now;

    // Reset all references together. Stable controls remain stable, and a
    // slowly turned control can still accumulate enough movement to win.
    touchReferenceStatus = status;
  } else if (activeTagParameter != TAG_PARAMETER_MASTER &&
             (uint32_t)(now - lastControlTouchMs) >= TAG_IDLE_TO_MASTER_MS) {
    activeTagParameter = TAG_PARAMETER_MASTER;
  }

  formatTagForParameter(
    activeTagParameter,
    status,
    nextTag,
    sizeof(nextTag)
  );

  const bool nextDirty =
    structuralStatusChanged(status, previousStatus) ||
    strcmp(nextTag, pendingTag) != 0;

  pendingStatus = status;
  previousStatus = status;

  if (strcmp(nextTag, pendingTag) != 0) {
    strncpy(pendingTag, nextTag, sizeof(pendingTag));
    pendingTag[sizeof(pendingTag) - 1] = '\0';
  }

  if (nextDirty) {
    displayDirty = true;
  }

  portEXIT_CRITICAL(&statusMux);
}

void weylandDisplayPushPcm(int16_t pcmSample) {
  // No I2C, allocation, or display drawing in the audio path.
  if (!displayReady) {
    return;
  }

  // Clear the old trace after a genuinely silent release.
  if (abs(static_cast<int32_t>(pcmSample)) < 27) {
    if (scopeQuietSamples < SCOPE_SILENCE_CLEAR_SAMPLES) {
      ++scopeQuietSamples;
    }
  } else {
    scopeQuietSamples = 0;
  }

  if (scopeQuietSamples >= SCOPE_SILENCE_CLEAR_SAMPLES) {
    scopeCapturing = false;
    scopeCaptureIndex = 0;
    scopeStrideCounter = 0;
    scopePreviousPcm = pcmSample;
    clearScopeFrameFromAudio();
    return;
  }

  // Rising zero-cross trigger, using raw PCM.
  if (!scopeCapturing &&
      scopePreviousPcm <= 0 &&
      pcmSample > 0) {
    scopeCapturing = true;
    scopeCaptureIndex = 0;
    scopeStrideCounter = 0;
  }

  if (scopeCapturing) {
    if (scopeStrideCounter == 0) {
      scopeCaptureBuffer[scopeCaptureIndex] = pcmSample;
      ++scopeCaptureIndex;

      if (scopeCaptureIndex >= SCOPE_CAPTURE_SAMPLES) {
        portENTER_CRITICAL(&scopeMux);

        memcpy(
          scopeDisplayBuffer,
          scopeCaptureBuffer,
          sizeof(scopeDisplayBuffer)
        );

        scopeHasFrame = true;
        scopeFramePending = true;

        portEXIT_CRITICAL(&scopeMux);

        scopeCapturing = false;
        scopeCaptureIndex = 0;
        scopeStrideCounter = 0;
      }
    }

    ++scopeStrideCounter;
    if (scopeStrideCounter >= scopeSampleStride) {
      scopeStrideCounter = 0;
    }
  }

  scopePreviousPcm = pcmSample;
}

bool weylandDisplayIsReady() {
  return displayReady;
}
