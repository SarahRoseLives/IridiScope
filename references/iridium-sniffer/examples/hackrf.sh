#!/bin/sh
# HackRF One -- 8-bit ADC, 20 MHz max, built-in amp + bias tee
#
# LNA gain: 0-40 dB (8 dB steps)
# VGA gain: 0-62 dB (2 dB steps)
# RF amplifier: +14 dB (--hackrf-amp)
#
# With external LNA powered via bias tee:
#   iridium-sniffer -l -i hackrf-0000000000000000 -B --hackrf-lna=40 --hackrf-vga=20
#
# Without external LNA (RF amp on for extra sensitivity):
#   iridium-sniffer -l -i hackrf-0000000000000000 --hackrf-lna=40 --hackrf-vga=20 --hackrf-amp

exec iridium-sniffer -l -B --hackrf-lna=40 --hackrf-vga=20 --web "$@"
