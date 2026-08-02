#include "Adafruit_TinyUSB.h"
#include "hardware/watchdog.h"

#ifndef USE_TINYUSB_HOST
  #error "Set Tools -> USB Stack -> Adafruit TinyUSB Host (Native)"
#endif

// ============================================================
// Weyland_Pico_USBHost_HardwareReset_v0_5
//
// USB MIDI host -> Weyland MGP bridge.
//
// Target ESP firmware:
//   Weyland_v0_27_modeGate
//   MGP_DEBUG_PRINT = false
//
// Hardware:
//   Pico GP0 -> 1k -> ESP GPIO15 RX
//   Pico GND -------- ESP GND
//
// Mode/reset wiring:
//   Existing mode node:
//     DPDT common <-> ESP GPIO1 <-> 10k pull-up to ESP 3V3
//
//   That node drives a BS170 reset clamp:
//     BS170 gate   -> mode node
//     BS170 source -> common GND
//     BS170 drain  -> Pico RUN
//
// Result:
//   HOST   = mode node LOW  -> BS170 OFF -> Pico RUN released
//   DEVICE = mode node HIGH -> BS170 ON  -> Pico RUN held LOW
//
// The Pico has no mode-sense input now.
// GP1 is unused and may be physically disconnected.
//
// Startup behavior:
//   The Pico can only run in HOST mode, because it is held in reset
//   while DEVICE mode is selected. On every HOST entry, it boots from
//   a clean state, waits for USB-A VBUS/controller power-up, then
//   starts TinyUSB host.
//
// Physical USB unplug/replug in HOST:
//   A normal remount is allowed first. If none occurs after 800 ms,
//   Pico reboots and starts the host stack fresh.
//
// LED:
//   Fast blink   = boot/startup wait
//   Slow blink   = HOST running, no USB device
//   Double blink = USB device mounted, but no MIDI interface
//   Solid        = USB MIDI interface mounted
//   Brief off    = incoming MIDI activity
// ============================================================

// -------------------- Hardware --------------------

constexpr uint8_t UART_TX_PIN = 0;  // GP0 -> ESP GPIO15 through 1k
constexpr uint8_t LED_PIN     = 25;
constexpr uint32_t UART_BAUD  = 115200;

// -------------------- Timing --------------------

constexpr uint32_t HOST_BOOT_SETTLE_MS     = 1200;
constexpr uint32_t HOST_BEGIN_RETRY_MS     = 600;
constexpr uint32_t UNMOUNT_REBOOT_DELAY_MS = 800;

// -------------------- USB host --------------------

Adafruit_USBH_Host USBHost;

bool hostStackStarted = false;
bool hostStartPending = false;
uint32_t hostStartAtMs = 0;

bool unplugRebootPending = false;
uint32_t unplugRebootAtMs = 0;

volatile bool anyUsbMounted = false;
volatile bool midiMounted = false;
volatile uint32_t lastMidiActivityMs = 0;

// -------------------- LED state --------------------

uint32_t lastLedMs = 0;
uint8_t blinkStep = 0;
bool ledState = false;

// -------------------- MGP v1 --------------------
// Frame: A5 5A VER SEQ TYPE LEN PAYLOAD... XOR
// XOR: VER ^ SEQ ^ TYPE ^ LEN ^ payload bytes

constexpr uint8_t MGP_SYNC_1  = 0xA5;
constexpr uint8_t MGP_SYNC_2  = 0x5A;
constexpr uint8_t MGP_VERSION = 0x01;

constexpr uint8_t MGP_NOTE_ON        = 0x01;
constexpr uint8_t MGP_NOTE_OFF       = 0x02;
constexpr uint8_t MGP_CONTROL_CHANGE = 0x03;
constexpr uint8_t MGP_PITCH_BEND     = 0x04;
constexpr uint8_t MGP_ALL_SOUND_OFF  = 0x11;

constexpr uint8_t MIDI_CC_MOD_WHEEL = 1;
constexpr uint8_t MIDI_CC_SUSTAIN   = 64;
constexpr uint8_t MGP_ALL_CHANNELS  = 0xFF;

uint8_t mgpSequence = 0;

// -------------------- MIDI parser state --------------------

uint8_t runningStatus = 0;
uint8_t messageData[2] = {0, 0};
uint8_t dataCount = 0;
uint8_t dataNeeded = 0;

// -------------------- Forward declarations --------------------

void requestPicoReboot();
void clearUsbState();
void resetMidiParser();
void scheduleHostStart(uint32_t delayMs);
void serviceHostStart();
void serviceUnplugRecovery();
void updateLed();

void sendMgpAllSoundOff();
void sendMgpNoteOn(uint8_t channel, uint8_t note, uint8_t velocity);
void sendMgpNoteOff(uint8_t channel, uint8_t note, uint8_t releaseVelocity);
void sendMgpControlChange(uint8_t channel, uint8_t controller, uint8_t value);
void sendMgpPitchBend(uint8_t channel, uint8_t lsb, uint8_t msb);

// -------------------- Host / reboot control --------------------

void requestPicoReboot() {
  // Do not wait for Serial1.flush() here. During a USB-host fault,
  // reset must remain the smallest, most deterministic recovery path.
  //
  // The 5 ms watchdog delay allows the final all-sound-off packet
  // a brief chance to leave UART FIFO before normal RP2040 reboot.
  watchdog_reboot(0, 0, 5);

  // No more USB/MIDI work after reboot has been requested.
  while (true) {
  }
}

void clearUsbState() {
  anyUsbMounted = false;
  midiMounted = false;
  lastMidiActivityMs = 0;
}

void resetMidiParser() {
  runningStatus = 0;
  messageData[0] = 0;
  messageData[1] = 0;
  dataCount = 0;
  dataNeeded = 0;
}

void scheduleHostStart(uint32_t delayMs) {
  hostStartPending = true;
  hostStartAtMs = millis() + delayMs;
}

void serviceHostStart() {
  if (!hostStartPending || hostStackStarted) {
    return;
  }

  if (static_cast<int32_t>(millis() - hostStartAtMs) < 0) {
    return;
  }

  hostStartPending = false;
  clearUsbState();
  resetMidiParser();

  if (USBHost.begin(0)) {
    hostStackStarted = true;
  } else {
    // Leave the Pico alive and retry after the host hardware settles.
    scheduleHostStart(HOST_BEGIN_RETRY_MS);
  }
}

void serviceUnplugRecovery() {
  if (!unplugRebootPending) {
    return;
  }

  if (midiMounted) {
    unplugRebootPending = false;
    return;
  }

  if (static_cast<int32_t>(millis() - unplugRebootAtMs) < 0) {
    return;
  }

  // No normal remount arrived after a physical unplug.
  sendMgpAllSoundOff();
  requestPicoReboot();
}

// -------------------- MGP sender --------------------

uint8_t mgpChecksum(
  uint8_t version,
  uint8_t sequence,
  uint8_t type,
  uint8_t length,
  const uint8_t* payload
) {
  uint8_t checksum = version ^ sequence ^ type ^ length;

  for (uint8_t i = 0; i < length; ++i) {
    checksum ^= payload[i];
  }

  return checksum;
}

void sendMgpPacket(uint8_t type, const uint8_t* payload, uint8_t length) {
  const uint8_t checksum =
    mgpChecksum(MGP_VERSION, mgpSequence, type, length, payload);

  Serial1.write(MGP_SYNC_1);
  Serial1.write(MGP_SYNC_2);
  Serial1.write(MGP_VERSION);
  Serial1.write(mgpSequence);
  Serial1.write(type);
  Serial1.write(length);

  for (uint8_t i = 0; i < length; ++i) {
    Serial1.write(payload[i]);
  }

  Serial1.write(checksum);
  ++mgpSequence;
}

void sendMgpNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
  const uint8_t payload[3] = {
    static_cast<uint8_t>(channel & 0x0F),
    static_cast<uint8_t>(note & 0x7F),
    static_cast<uint8_t>(velocity & 0x7F)
  };

  sendMgpPacket(MGP_NOTE_ON, payload, sizeof(payload));
}

void sendMgpNoteOff(uint8_t channel, uint8_t note, uint8_t releaseVelocity) {
  const uint8_t payload[3] = {
    static_cast<uint8_t>(channel & 0x0F),
    static_cast<uint8_t>(note & 0x7F),
    static_cast<uint8_t>(releaseVelocity & 0x7F)
  };

  sendMgpPacket(MGP_NOTE_OFF, payload, sizeof(payload));
}

void sendMgpControlChange(uint8_t channel, uint8_t controller, uint8_t value) {
  const uint8_t payload[3] = {
    static_cast<uint8_t>(channel & 0x0F),
    static_cast<uint8_t>(controller & 0x7F),
    static_cast<uint8_t>(value & 0x7F)
  };

  sendMgpPacket(MGP_CONTROL_CHANGE, payload, sizeof(payload));
}

void sendMgpPitchBend(uint8_t channel, uint8_t lsb, uint8_t msb) {
  const uint8_t payload[3] = {
    static_cast<uint8_t>(channel & 0x0F),
    static_cast<uint8_t>(lsb & 0x7F),
    static_cast<uint8_t>(msb & 0x7F)
  };

  sendMgpPacket(MGP_PITCH_BEND, payload, sizeof(payload));
}

void sendMgpAllSoundOff() {
  const uint8_t payload[1] = {MGP_ALL_CHANNELS};
  sendMgpPacket(MGP_ALL_SOUND_OFF, payload, sizeof(payload));
}

// -------------------- MIDI parser --------------------

uint8_t midiDataLength(uint8_t status) {
  switch (status & 0xF0) {
    case 0x80: // Note Off
    case 0x90: // Note On
    case 0xA0: // Poly aftertouch
    case 0xB0: // CC
    case 0xE0: // Pitch bend
      return 2;

    case 0xC0: // Program change
    case 0xD0: // Channel pressure
      return 1;

    default:
      return 0;
  }
}

void dispatchMidiChannelMessage(uint8_t status, uint8_t data1, uint8_t data2) {
  const uint8_t messageType = status & 0xF0;
  const uint8_t channel = status & 0x0F;

  switch (messageType) {
    case 0x90: // Note On
      if (data2 == 0) {
        sendMgpNoteOff(channel, data1, 0);
      } else {
        sendMgpNoteOn(channel, data1, data2);
      }
      break;

    case 0x80: // Note Off
      sendMgpNoteOff(channel, data1, data2);
      break;

    case 0xB0: // Control Change
      if (data1 == MIDI_CC_MOD_WHEEL || data1 == MIDI_CC_SUSTAIN) {
        sendMgpControlChange(channel, data1, data2);
      }
      break;

    case 0xE0: // Pitch bend, 14-bit LSB then MSB
      sendMgpPitchBend(channel, data1, data2);
      break;

    default:
      break;
  }
}

void processMidiByte(uint8_t midiByte) {
  // MIDI realtime bytes can occur anywhere in the stream.
  if (midiByte >= 0xF8) {
    return;
  }

  if (midiByte & 0x80) {
    // Ignore SysEx and system-common messages.
    if (midiByte >= 0xF0) {
      resetMidiParser();
      return;
    }

    runningStatus = midiByte;
    dataCount = 0;
    dataNeeded = midiDataLength(runningStatus);
    return;
  }

  if (runningStatus == 0 || dataNeeded == 0) {
    return;
  }

  messageData[dataCount++] = midiByte & 0x7F;

  if (dataCount >= dataNeeded) {
    const uint8_t data1 = messageData[0];
    const uint8_t data2 = (dataNeeded > 1) ? messageData[1] : 0;

    dispatchMidiChannelMessage(runningStatus, data1, data2);

    // Keep running status for standard MIDI streams.
    dataCount = 0;
  }
}

// -------------------- TinyUSB callbacks --------------------

void tuh_mount_cb(uint8_t daddr) {
  (void)daddr;

  anyUsbMounted = true;
  unplugRebootPending = false;
}

void tuh_umount_cb(uint8_t daddr) {
  (void)daddr;

  anyUsbMounted = false;
  midiMounted = false;
  sendMgpAllSoundOff();

  // A fast replug gets a chance to mount normally first.
  unplugRebootPending = true;
  unplugRebootAtMs = millis() + UNMOUNT_REBOOT_DELAY_MS;
}

void tuh_midi_mount_cb(uint8_t idx, const tuh_midi_mount_cb_t* mountInfo) {
  (void)idx;
  (void)mountInfo;

  midiMounted = true;
  unplugRebootPending = false;
}

void tuh_midi_umount_cb(uint8_t idx) {
  (void)idx;

  midiMounted = false;
  sendMgpAllSoundOff();

  unplugRebootPending = true;
  unplugRebootAtMs = millis() + UNMOUNT_REBOOT_DELAY_MS;
}

void tuh_midi_rx_cb(uint8_t idx, uint32_t xferredBytes) {
  (void)xferredBytes;

  uint8_t cableNum = 0;
  uint8_t midiData[64];

  // Drain all queued MIDI bytes every callback.
  while (true) {
    const uint32_t bytesRead = tuh_midi_stream_read(
      idx,
      &cableNum,
      midiData,
      sizeof(midiData)
    );

    if (bytesRead == 0) {
      break;
    }

    lastMidiActivityMs = millis();

    for (uint32_t i = 0; i < bytesRead; ++i) {
      processMidiByte(midiData[i]);
    }
  }
}

// -------------------- LED --------------------

void updateLed() {
  const uint32_t now = millis();

  if (hostStartPending || !hostStackStarted) {
    if (now - lastLedMs >= 100) {
      lastLedMs = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
    return;
  }

  if (midiMounted) {
    digitalWrite(
      LED_PIN,
      (now - lastMidiActivityMs < 70) ? LOW : HIGH
    );
    return;
  }

  if (!anyUsbMounted) {
    if (now - lastLedMs >= 500) {
      lastLedMs = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
    return;
  }

  if (now - lastLedMs >= 130) {
    lastLedMs = now;
    blinkStep = (blinkStep + 1) % 10;

    const bool on = (blinkStep == 0 || blinkStep == 2);
    digitalWrite(LED_PIN, on ? HIGH : LOW);
  }
}

// -------------------- Arduino --------------------

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial1.setTX(UART_TX_PIN);
  Serial1.begin(UART_BAUD);

  // Pico is only allowed out of RUN reset in HOST mode.
  // Give USB-A VBUS and the controller time to power up.
  scheduleHostStart(HOST_BOOT_SETTLE_MS);
}

void loop() {
  serviceHostStart();

  if (hostStackStarted) {
    USBHost.task(0);
  }

  serviceUnplugRecovery();
  updateLed();
}
