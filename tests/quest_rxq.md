# Quest 3 RX-ring runbook (issue #330)

Confirmatory on-device test for the async-RX-ring residual loss. The Meta Quest 3
(Snapdragon XR2 Gen 2, Android) is a constrained mobile host — the most faithful
stand-in for the reporter's MediaTek phone available. The whole run is driven
**headlessly from the desktop over adb**; nothing happens in the headset.

Vehicle: a minimal auto-grant APK (`tests/quest_apk/`) modelled on how PixelPilot
avoids the USB permission dialog — a `USB_DEVICE_ATTACHED` intent-filter +
`device_filter` auto-grants USB permission when the adapter attaches, and a JNI
`fork()`+`execve()` helper hands the `UsbDeviceConnection` fd to the bundled
`rxdemo`. Results (`docs/experiments/issue-330-rx-ring-starvation.md`): the
ring-servicing mode shows no delivery benefit above the RF-baseline noise on the
real device either; the shippable product is the `rx.ring` telemetry as a field
diagnostic.

## Enablement (once)

1. **Developer mode** on the headset (Meta Quest mobile app / a developer org),
   then `Settings → System → Developer → USB debugging`.
2. **adb over Wi-Fi** — frees the single USB-C port for the adapter:
   ```sh
   adb tcpip 5555            # once over USB, then unplug
   adb connect <quest-ip>:5555
   ```
   The Quest's adb rides its **internal Wi-Fi**; check the band with
   `adb shell dumpsys wifi | grep frequencyMhz`. If it is 5 GHz (5180 = ch 36),
   the experiment must run on 2.4 GHz — a 5 GHz flood self-desenses the USB
   adapter inches away in the same chassis.
3. **Keep-awake** (off-head standby drops adb-Wi-Fi):
   ```sh
   adb shell am broadcast -a com.oculus.vrpowermanager.prox_close
   adb shell svc power stayon true
   ```
4. Plug the **RTL8812AU** into the Quest's USB-C (via OTG). Confirm the kernel
   enumerates it: `adb shell dumpsys usb | grep -A2 host_manager` should show
   `manufacturer=3034 product=34834` (0x0bda:0x8812).

## Build + install the APK

```sh
# 1. arm64 rxdemo bundle (rxdemo + libusb1.0.so)
ANDROID_NDK=/opt/android-sdk/ndk/23.1.7779620 tests/build_android.sh
# 2. hand-build + sign the APK (no gradle)
ANDROID_SDK=/opt/android-sdk ANDROID_NDK=/opt/android-sdk/ndk/23.1.7779620 \
  RXQ_BUNDLE=/tmp/devourer-android/bundle tests/quest_apk/build_apk.sh
adb install -r /tmp/rxq-apk/build/rxq.apk
```

First launch requests USB permission once; tap **"Always open …"** + **OK** (or
drive it headlessly: `adb shell uiautomator dump`, then `adb shell input tap` the
`alwaysUse` checkbox and `button1`). Thereafter every attach auto-grants.

## Run the matrix

The RX (Quest) side is launched per-mode via `am start` extras; the TX flood +
analysis run on the desktop. One command does the lot:

```sh
MODES="async reorder-pool spsc-fat" SINK_SPIN_US=500 DUR=20 \
  TX_VID=0x2357 TX_PID=0x0120 TX_GAP_US=200 \
  BURST_ON_MS=4 BURST_OFF_MS=30 POOL_SPARE=16 \
  tests/quest_rxq_run.sh
```

It keeps the headset awake, floods counter-stamped 6M QoS-Data, captures each
mode, pulls the JSONL, and prints per-mode delivery (from the 802.11 `seq_num`
sequence) + ring telemetry via `tests/quest_rxq_analyze.py`.

**Link tips** (2.4 GHz ch 6): use a 2T2R flooder — the 8821AU 1T1R nano couples
poorly to the Quest adapter; use **6M OFDM** — 1M CCK from the flooder airs but
the Quest decodes ~0. The jaguar3 dies wedge on repeated soft-kill (need a
replug); the 8821AU (jaguar1) is kill-robust if it reaches the Quest.

## Direct app control

```sh
adb shell am start -n org.openipc.rxq/.MainActivity \
  --es tag run --es rxmode reorder-pool --es channel 6 \
  --es urbs 4 --es poolspare 16 --es sinkspin 500
adb shell am start -n org.openipc.rxq/.MainActivity --es action stop
# output: /sdcard/Android/data/org.openipc.rxq/files/rxq_<tag>.jsonl ; status.txt
```
