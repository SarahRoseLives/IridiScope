# IridiScope

A native Windows application for receiving, decoding and visualising **Iridium**
satellite signals. Load an IQ recording or plug in an SDR, and IridiScope shows a
live spectrum and waterfall, demodulates Iridium bursts, parses the messages,
decodes voice calls, reassembles ACARS/SBD traffic, and plots the satellites on
a world map.

---

## 📥 Official Windows builds

**Download the official Windows builds from
<https://sarahsforge.dev/products/iridiscope>.**

Builds published anywhere else are not official.

---

## Features

- Live spectrum + scrolling waterfall with configurable FFT size and averaging.
- Full Iridium burst detection, DQPSK demodulation and native message parser
  (IRA, IBC, IDA, IMS, ITL, SY, IP, VO).
- Satellite tracking and ring-alert positions on an equirectangular world map.
- Voice call capture with in-app playback, plus ACARS/SBD message decoding.
- Sources: IQ file, **SDRplay**, **HackRF** and **Airspy** (all locked to the
  Iridium band at 1623.326935 MHz), with gain and bias-tee controls.
- Automatic update notifications.

## Building

Requirements: Windows, [MSYS2](https://www.msys2.org/) with the `mingw-w64`
toolchain, CMake and Ninja, and PowerShell.

```powershell
# from the repository root
powershell -ExecutionPolicy Bypass -File scripts\bootstrap.ps1   # build the vendored FFTW
powershell -ExecutionPolicy Bypass -File scripts\build.ps1
```

The executable is produced at `build\IridiScope.exe`. SDR support is detected
automatically at configure time; if the SDRplay API, libhackrf or libairspy are
not found, that source is simply disabled.

Handy headless modes: `--info <file>`, `--scan <file> [seconds]`,
`--sdr-list`, `--hrf-list`, `--ap-list`, `--check-update`.

### macOS (Apple Silicon / Intel)

Requirements: Xcode command-line tools and [Homebrew](https://brew.sh).
For SDRplay devices, also install the SDRplay API for macOS.

```bash
bash scripts/build_macos.sh
open build-macos/IridiScope.app
```

The script installs CMake, Ninja, FFTW, libhackrf, libairspy and librtlsdr
from Homebrew and builds `build-macos/IridiScope.app` with the NEON DSP
kernels on Apple Silicon. When started from Finder, the window layout and voice
recordings are kept in `~/Library/Application Support/IridiScope`.

### Linux

```bash
sudo apt install cmake ninja-build libfftw3-dev libcurl4-openssl-dev \
  libgl-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
  libwayland-dev libxkbcommon-dev wayland-protocols \
  libhackrf-dev libairspy-dev librtlsdr-dev zenity
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build && ./build/IridiScope
```

On macOS and Linux the update check and aircraft lookup use libcurl, and the
file dialogs use the system picker (AppleScript on macOS, zenity or kdialog on
Linux).

## Credits

IridiScope is built on the work of others:

- **iridium-sniffer** (muccc) — burst detection and demodulation core.
- **iridium-toolkit** (Sec, schneider) — message-format reference.
- **osmo-ir77** (Sylvain Munaut / Osmocom) — Iridium AMBE voice vocoder.
- **Dear ImGui** (Omar Cornut) and **ImPlot** (Evan Pezent).
- **GLFW**, **FFTW**, **miniaudio**, **stb**.
- SDR backends: **SDRplay API**, **libhackrf** (Great Scott Gadgets),
  **libairspy** (Airspy).

SDR integration was modelled on **AirScope**. Map imagery courtesy of NASA.

## License

Released under the GNU General Public License v3.0.
