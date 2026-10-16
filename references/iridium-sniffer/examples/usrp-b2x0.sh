#!/bin/sh
# Ettus USRP B210 / B205mini -- 12-bit ADC, 56 MHz max
#
# Gain range: 0-76 dB (AD9361 combined LNA+mixer+VGA)
# Recommended: 40-50 dB for Iridium with external LNA
#
# Specify your serial with -i to avoid UHD probing all USB devices:
#   iridium-sniffer -l -i usrp-B210-SERIAL --usrp-gain=50
#   iridium-sniffer -l -i usrp-B205mini-SERIAL --usrp-gain=50
#
# Find your serial with: iridium-sniffer --list

exec iridium-sniffer -l --usrp-gain=50 --web "$@"
