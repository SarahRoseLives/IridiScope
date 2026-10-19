#!/bin/bash
# Build IridiScope on macOS (Apple Silicon or Intel) with Homebrew.
#
#   bash scripts/build_macos.sh            # build build-macos/IridiScope.app
#   bash scripts/build_macos.sh --no-deps  # skip the Homebrew install step
#
# SDR support is detected automatically:
#   HackRF / Airspy / RTL-SDR -> installed below from Homebrew
#   SDRplay                    -> install the SDRplay API for macOS from
#                                 https://www.sdrplay.com/api (it goes to /usr/local)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build-macos"

if ! command -v brew >/dev/null 2>&1; then
    echo "Homebrew is required: https://brew.sh" >&2
    exit 1
fi

if [ "${1:-}" != "--no-deps" ]; then
    brew install cmake ninja pkg-config fftw hackrf airspy librtlsdr
fi

ARCH="$(uname -m)"          # arm64 on Apple Silicon, x86_64 on Intel
BREW_PREFIX="$(brew --prefix)"

cmake -S "$ROOT" -B "$BUILD" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_OSX_ARCHITECTURES="$ARCH" \
    -DCMAKE_PREFIX_PATH="$BREW_PREFIX;/usr/local"

cmake --build "$BUILD"

echo
echo "Built: $BUILD/IridiScope.app"
echo "Run:   open \"$BUILD/IridiScope.app\""
echo "CLI:   \"$BUILD/IridiScope.app/Contents/MacOS/IridiScope\" --info <file>"
echo "Layout and voice recordings are kept in ~/Library/Application Support/IridiScope"
