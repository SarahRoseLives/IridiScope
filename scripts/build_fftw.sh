#!/bin/bash
# Build the vendored single-precision FFTW (static) with the MSYS2 mingw64
# compiler. Run this from an MSYS2 shell:  bash scripts/build_fftw.sh
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/third_party/fftw/fftw-3.3.10"
TAR="$ROOT/third_party/fftw/fftw-3.3.10.tar.gz"
INST="$ROOT/third_party/fftw/install"

if [ -f "$INST/lib/libfftw3f.a" ]; then
    echo "FFTW already built at $INST"
    exit 0
fi

export PATH="/mingw64/bin:$PATH"

# Always build from a pristine tree.
rm -rf "$SRC"
tar -xzf "$TAR" -C "$ROOT/third_party/fftw"
cd "$SRC"

./configure \
    --enable-float --disable-shared --enable-static \
    --disable-fortran --disable-doc --disable-dependency-tracking \
    --build=x86_64-w64-mingw32 --host=x86_64-w64-mingw32 \
    --prefix="$(cygpath -m "$INST")" \
    CC=gcc

make -j"${NPROC:-4}"
make install
echo "FFTW installed to $INST"
