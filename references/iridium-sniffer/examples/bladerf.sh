#!/bin/sh
# Nuand bladeRF 2.0 micro (xA4/xA9) -- 12-bit ADC (AD9361), 40 MHz max
#
# Gain range at L-band (1.3-4 GHz): -15 to 60 dB
# Recommended: 40 dB (gr-iridium default)
#
# Note: the bladeRF 2.0 has known reduced sensitivity at L-band compared
# to the USRP B2x0 despite using the same AD9361 transceiver. An external
# filtered LNA is recommended for best results.
#
# With built-in bias tee:
#   iridium-sniffer -l -i bladerf0 -B --bladerf-gain=40
#
# With external bias tee:
#   iridium-sniffer -l -i bladerf0 --bladerf-gain=40

exec iridium-sniffer -l -i bladerf0 --bladerf-gain=40 --web "$@"
