# Weyland DOS-1

Weyland DOS-1 is a digital monophonic synthesizer built from scratch around an ESP32-S3.

It features two oscillators, waveform selection, oscillator mix and detune, an AR envelope, a low-pass filter, drive, and an LFO assignable to oscillator pitch or filter cutoff. Audio is generated at 48 kHz and sent through a PCM5102A DAC.

Weyland accepts USB MIDI directly as a device or through an RP2040-based USB host bridge, allowing it to work with class-compliant USB MIDI controllers without a computer.

## Overview

The project includes the synthesizer firmware, USB MIDI host firmware, wiring documentation, component list, and files for the custom 3D-printed enclosure.

Weyland DOS-1 was designed as a complete playable instrument rather than a development board experiment: all synthesis parameters have dedicated physical controls, while a minimal OLED interface displays the current MIDI mode, oscillator range, and LFO assignment.
