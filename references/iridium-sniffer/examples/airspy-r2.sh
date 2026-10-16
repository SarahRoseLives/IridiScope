#!/bin/sh
# Airspy R2 / HydraSDR -- 12-bit ADC, 10 MHz max, via SoapySDR
#
# Gain range: 0-45 dB (3 stages: LNA 0-15, MIX 0-15, VGA 0-15)
# Recommended: 25-35 dB. Higher gains overload the ADC and raise the
# noise floor, reducing sensitivity. Tested sweet spot: 30 dB.
#
# Too low (18 dB):  weak signals missed, low burst rate
# Optimal (30 dB):  best burst rate and UW pass rate (78%)
# Too high (45 dB): ADC overload, 4x fewer bursts detected
#
# Aggregate gain (SoapySDR distributes across stages):
#   iridium-sniffer -l -i soapy-0 --soapy-gain=30
#
# Per-element gain (direct control of each stage):
#   iridium-sniffer -l -i soapy-0 \
#     --soapy-gain-element=LNA:10 \
#     --soapy-gain-element=MIX:9 \
#     --soapy-gain-element=VGA:10
#
# Discover available gain elements:
#   iridium-sniffer -l -i soapy-0 -v --diagnostic 2>&1 | grep "gain elements"

exec iridium-sniffer -l -i soapy-0 --soapy-gain=30 --web "$@"
