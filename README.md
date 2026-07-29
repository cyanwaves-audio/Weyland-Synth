# Weyland DOS-1

<p align="center">
  <img src="Weyland-02.JPG" alt="Weyland DOS-1 synthesizer" width="300">
</p>

Weyland DOS-1 is a digital monophonic synthesizer built from scratch around an ESP32-S3.

It features two oscillators, waveform selection, oscillator mix and detune, an AR envelope, a low-pass filter, drive, and an LFO assignable to oscillator pitch or filter cutoff. Audio is generated at 48 kHz and sent through a PCM5102A DAC.

Weyland accepts USB MIDI directly as a device or through an RP2040-based USB host bridge, allowing it to work with class-compliant USB MIDI controllers without a computer.

[Watch it in action here.](https://youtu.be/kIQRiqMi21E)
## Overview

The project includes the synthesizer firmware, USB MIDI host firmware, wiring documentation, component list, and files for the custom 3D-printed enclosure.

Weyland DOS-1 was designed as a complete playable instrument rather than a development board experiment: all synthesis parameters have dedicated physical controls, while a minimal OLED interface displays the current MIDI mode, oscillator range, and LFO assignment.

## Features

- Digital monophonic synthesizer running at 48 kHz
- Dual oscillators with shared waveform selection
- Oscillator mix, detune, and octave-range controls
- Low-pass filter with envelope modulation
- Attack–release amplitude envelope
- LFO with adjustable rate and depth
- LFO routing to oscillator pitch, filter cutoff, or off
- Adjustable drive and master output
- Velocity-sensitive MIDI response
- Last-note priority with held-note fallback
- Direct USB MIDI device mode
- USB MIDI host mode through an RP2040 bridge
- Minimal OLED status display
- Dedicated physical controls for every synthesis parameter
- PCM5102A I²S audio output
- Custom 3D-printed enclosure


## Architecture

Weyland is built around an ESP32-S3, which runs the synthesis engine, processes the physical controls, receives MIDI, updates the OLED, and sends digital audio to the DAC.

| Component | Function |
| --- | --- |
| ESP32-S3 | Synthesis engine, control processing, MIDI and display |
| RP2040 Pico | USB MIDI host and UART bridge |
| MCP3208 | Eight-channel ADC for the analog controls |
| PCM5102A | 48 kHz I²S digital-to-analog conversion |
| SH1106 OLED | MIDI mode, octave and LFO status display |

MIDI can reach the synthesizer through two paths:

- **Device mode:** A computer or USB host connects directly to the ESP32-S3.
- **Host mode:** A class-compliant USB MIDI controller connects to the RP2040, which forwards the MIDI data to the ESP32-S3 over UART.


## Controls

| Control | Function |
| --- | --- |
| WAVE | Selects the waveform used by both oscillators |
| OSC MIX | Blends oscillator 1 and oscillator 2 |
| OSC-2 DETUNE | Fine-tunes oscillator 2 relative to oscillator 1 |
| OSC -2 RANGE | Shifts oscillator 2 by −12, 0 or +12 semitones |
| LFO RATE | Sets the modulation speed |
| LFO DEPTH | Sets the modulation amount |
| LFO TARGET | Routes the LFO to the filter, oscillator 2 or off |
| DRIVE | Adds saturation to the signal |
| CUTOFF | Sets the low-pass filter cutoff |
| FILTER ENV | Sets the envelope modulation applied to the filter |
| ATTACK | Sets the amplitude-envelope attack time |
| RELEASE | Sets the amplitude-envelope release time |
| MASTER | Sets the output level |

Weyland is monophonic and uses last-note priority. When the current note is released, it returns to the newest note still being held.

## Hardware

### Core components

| Component | Purpose |
| --- | --- |
| ESP32-S3-WROOM-1 | Main processor and synthesis engine |
| Raspberry Pi Pico / RP2040 | USB MIDI host bridge |
| PCM5102A module | I²S audio output |
| MCP3208 | 12-bit ADC for eight analog controls |
| 1.3-inch SH1106 OLED | Status display |
| Potentiometers and switches | Synth parameter controls |
| Audio output jack | Line-level audio output |
| USB connectors | Power, direct MIDI and USB-host MIDI |
| Custom 3D-printed enclosure | Panel and electronics housing |

The complete bill of materials, including control types, connectors and supporting components, is provided in the project files.

## Wiring and Pin Mapping

### ESP32-S3 connections

| GPIO | Connection |
| ---: | --- |
| 1 | MIDI host/device mode selector |
| 4 | Oscillator mix |
| 5 | Oscillator 2 detune |
| 6, 7 | Oscillator 2 octave selector |
| 9, 10 | LFO target selector |
| 11–14 | Waveform selector |
| 15 | UART MIDI input from the RP2040 bridge |
| 16 | OLED SDA |
| 17 | OLED SCL |
| 35 | PCM5102A LRCK |
| 36 | PCM5102A DIN |
| 37 | PCM5102A BCK |
| 39 | MCP3208 CS |
| 40 | MCP3208 MOSI |
| 41 | MCP3208 MISO |
| 42 | MCP3208 SCK |

### MCP3208 channels

| Channel | Control |
| ---: | --- |
| CH0 | LFO rate |
| CH1 | LFO depth |
| CH2 | Drive |
| CH3 | Filter cutoff |
| CH4 | Filter envelope amount |
| CH5 | Attack |
| CH6 | Release |
| CH7 | Master output |

The supplied firmware compensates for the potentiometer orientation used in the original build. If the controls operate backwards in another build, swap the outer potentiometer connections or change the inversion behavior in the firmware.
