#!/usr/bin/env bash
# Hand-build the RXQ USB-host APK (no gradle): javac -> d8 -> aapt package ->
# add dex + native libs -> zipalign -> apksigner. Autonomous vehicle for the
# issue-330 Quest run: the APK auto-grants USB permission via its
# USB_DEVICE_ATTACHED intent-filter (the PixelPilot mechanism) and execs the
# bundled devourer rxdemo, so nothing needs doing in the headset. See
# tests/quest_rxq.md.
#
# Prereqs: an Android SDK (build-tools 30.0.3 + android-29 platform), an NDK
# (r23+), and an arm64 rxdemo bundle from tests/build_android.sh. Point at them:
#   ANDROID_SDK=/opt/android-sdk ANDROID_NDK=.../ndk/23.1.7779620 \
#   RXQ_BUNDLE=/tmp/devourer-android/bundle tests/quest_apk/build_apk.sh
set -euo pipefail

SDK="${ANDROID_SDK:-${ANDROID_SDK_ROOT:-/opt/android-sdk}}"
BT="$SDK/build-tools/${BUILD_TOOLS:-30.0.3}"
AJAR="$SDK/platforms/${PLATFORM:-android-29}/android.jar"
NDK="${ANDROID_NDK:-$SDK/ndk/23.1.7779620}"
API="${API:-28}"
HERE="$(cd "$(dirname "$0")" && pwd)"
# RXQ_BUNDLE holds the arm64 rxdemo + libusb1.0.so (output of build_android.sh).
BUNDLE="${RXQ_BUNDLE:-/tmp/devourer-android/bundle}"
WORK="${RXQ_WORK:-/tmp/rxq-apk}"     # build artifacts + keystore, out of tree
OUT="$WORK/build"
KS="$WORK/debug.keystore"

rm -rf "$OUT"; mkdir -p "$OUT/obj" "$HERE/lib/arm64-v8a"

# 0. Native payload: rxdemo must be named lib*.so so Android extracts it
#    executable into nativeLibraryDir; its DT_NEEDED "libusb1.0.so" resolves
#    via LD_LIBRARY_PATH=nativeLibraryDir set by the launcher.
cp "$BUNDLE/rxdemo"       "$HERE/lib/arm64-v8a/librxdemo.so"
cp "$BUNDLE/libusb1.0.so" "$HERE/lib/arm64-v8a/libusb1.0.so"

# 0b. JNI fork()+exec() helper — ProcessBuilder can't pass a USB fd to a child,
#     so the app forks in-process via this .so.
CC=$(ls "$NDK"/toolchains/llvm/prebuilt/*/bin/aarch64-linux-android${API}-clang | head -1)
"$CC" -shared -fPIC -O2 -o "$HERE/lib/arm64-v8a/libspawn.so" "$HERE/jni/spawn.c" -llog

# 1. debug keystore (once)
if [ ! -f "$KS" ]; then
  keytool -genkeypair -keystore "$KS" -storepass android -keypass android \
    -alias rxq -keyalg RSA -keysize 2048 -validity 10000 \
    -dname "CN=RXQ,O=OpenIPC,C=US"
fi

# 2. compile + dex
javac -source 1.8 -target 1.8 -bootclasspath "$AJAR" \
  -d "$OUT/obj" "$HERE"/src/org/openipc/rxq/*.java
"$BT/d8" --min-api 28 --output "$OUT" "$OUT"/obj/org/openipc/rxq/*.class

# 3. package manifest + resources
"$BT/aapt" package -f -M "$HERE/AndroidManifest.xml" -S "$HERE/res" \
  -I "$AJAR" -F "$OUT/rxq.unsigned.apk"

# 4. add classes.dex + native libs (paths inside the apk preserved)
( cd "$OUT" && "$BT/aapt" add rxq.unsigned.apk classes.dex >/dev/null )
( cd "$HERE" && "$BT/aapt" add "$OUT/rxq.unsigned.apk" \
    lib/arm64-v8a/librxdemo.so lib/arm64-v8a/libusb1.0.so \
    lib/arm64-v8a/libspawn.so >/dev/null )

# 5. align + sign
"$BT/zipalign" -f 4 "$OUT/rxq.unsigned.apk" "$OUT/rxq.aligned.apk"
"$BT/apksigner" sign --ks "$KS" --ks-pass pass:android --key-pass pass:android \
  --out "$OUT/rxq.apk" "$OUT/rxq.aligned.apk"

echo "BUILT: $OUT/rxq.apk"
"$BT/apksigner" verify --print-certs "$OUT/rxq.apk" >/dev/null && echo "signature OK"
