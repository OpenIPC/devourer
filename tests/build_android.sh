#!/usr/bin/env bash
#
# build_android.sh — cross-compile rxdemo for arm64-v8a Android (e.g. Meta Quest 3,
# a phone under Termux) for the issue #330 RX-ring test. Produces a stripped,
# self-contained bundle (rxdemo + libusb1.0.so) you `adb push` and run in Termux
# via `termux-usb -e`. See tests/quest_rxq.md.
#
# Requirements: an Android NDK (r26+), curl, cmake. Point NDK at it:
#   ANDROID_NDK=/path/to/android-ndk-r26d tests/build_android.sh
#
# Build knobs (env): ABI (default arm64-v8a), API (default 28), OUT (bundle dir).
#
set -euo pipefail

NDK="${ANDROID_NDK:?set ANDROID_NDK to your NDK root (r26+)}"
ABI="${ABI:-arm64-v8a}"
API="${API:-28}"
SRC="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${WORK:-/tmp/devourer-android}"
OUT="${OUT:-$WORK/bundle}"
LIBUSB_VER="${LIBUSB_VER:-1.0.27}"

mkdir -p "$WORK"; cd "$WORK"

# --- 1. libusb for the target ABI via ndk-build ------------------------------
if [ ! -d "libusb-$LIBUSB_VER" ]; then
  curl -sL -o "libusb-$LIBUSB_VER.tar.bz2" \
    "https://github.com/libusb/libusb/releases/download/v$LIBUSB_VER/libusb-$LIBUSB_VER.tar.bz2"
  tar xjf "libusb-$LIBUSB_VER.tar.bz2"
fi
( cd "libusb-$LIBUSB_VER/android/jni" && "$NDK/ndk-build" APP_ABI="$ABI" APP_PLATFORM="android-$API" -j"$(nproc)" )
SO=$(find "$WORK/libusb-$LIBUSB_VER/android/libs/$ABI" -name 'libusb1.0.so' | head -1)

# --- 2. stage a prefix + a pkg-config .pc the devourer CMake consumes ---------
PFX="$WORK/prefix-$ABI"
rm -rf "$PFX"; mkdir -p "$PFX/lib/pkgconfig" "$PFX/include/libusb-1.0"
cp "$SO" "$PFX/lib/libusb-1.0.so"                    # -lusb-1.0 wants this name
cp "$WORK/libusb-$LIBUSB_VER/libusb/libusb.h" "$PFX/include/libusb-1.0/"
cat > "$PFX/lib/pkgconfig/libusb-1.0.pc" <<EOF
prefix=$PFX
libdir=\${prefix}/lib
includedir=\${prefix}/include
Name: libusb-1.0
Description: libusb (android $ABI)
Version: $LIBUSB_VER
Libs: -L\${libdir} -lusb-1.0
Cflags: -I\${includedir}/libusb-1.0
EOF

# --- 3. configure + build rxdemo (8812AU-only) with the NDK toolchain ---------
# FIND_ROOT_PATH_MODE_LIBRARY=BOTH lets pkg_check_modules resolve the staged
# libusb outside the NDK sysroot; the tree links Android `log` itself (CMakeLists).
BUILD="$WORK/build-$ABI"; rm -rf "$BUILD"
PKG_CONFIG_LIBDIR="$PFX/lib/pkgconfig" PKG_CONFIG_SYSROOT_DIR="" \
cmake -S "$SRC" -B "$BUILD" \
  -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
  -DANDROID_ABI="$ABI" -DANDROID_PLATFORM="android-$API" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_FIND_ROOT_PATH="$PFX" -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH \
  -DDEVOURER_JAGUAR1=ON -DDEVOURER_8814=OFF \
  -DDEVOURER_JAGUAR2_8822B=OFF -DDEVOURER_JAGUAR2_8821C=OFF \
  -DDEVOURER_JAGUAR3_8822C=OFF -DDEVOURER_JAGUAR3_8822E=OFF
cmake --build "$BUILD" -j"$(nproc)" --target rxdemo

# --- 4. stage a stripped, shippable bundle -----------------------------------
STRIP=$(ls "$NDK"/toolchains/llvm/prebuilt/*/bin/llvm-strip | head -1)
rm -rf "$OUT"; mkdir -p "$OUT"
cp "$BUILD/rxdemo" "$OUT/rxdemo"; "$STRIP" "$OUT/rxdemo"
cp "$SO" "$OUT/libusb1.0.so"; "$STRIP" "$OUT/libusb1.0.so" 2>/dev/null || true

echo "=== bundle ($ABI) ==="; file "$OUT/rxdemo"; ls -lh "$OUT"
echo "push: adb push $OUT/rxdemo $OUT/libusb1.0.so /data/local/tmp/"
