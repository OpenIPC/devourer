cmake_minimum_required(VERSION 3.15)

# Set the toolchain file for vcpkg on Windows (only if VCPKG_ROOT is set;
# on mingw we get libusb from pkg-config directly).
if(WIN32 AND NOT "$ENV{VCPKG_ROOT}" STREQUAL "")
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
endif()

project(devourer)

enable_testing()

# ---------------------------------------------------------------------------
# Per-chip build options. All default ON (backward compatible: a plain
# `cmake -S . -B build` builds every chip). Turn off the ones you don't need to
# drop their firmware blobs + generated PHY tables and shrink the binary — e.g.
# an 8812AU-only build:
#   cmake -S . -B build -DDEVOURER_8814=OFF \
#         -DDEVOURER_JAGUAR2_8822B=OFF -DDEVOURER_JAGUAR2_8821C=OFF \
#         -DDEVOURER_JAGUAR3_8822C=OFF -DDEVOURER_JAGUAR3_8822E=OFF
# ---------------------------------------------------------------------------
option(DEVOURER_JAGUAR1       "RTL8812AU / 8811AU / 8821AU (Jaguar1 core)"      ON)
option(DEVOURER_8814          "RTL8814AU (4T4R; requires DEVOURER_JAGUAR1)"     ON)
option(DEVOURER_JAGUAR2_8822B "RTL8822BU (Jaguar2, rtl8822b, 2T2R USB)"         ON)
option(DEVOURER_JAGUAR2_8821C "RTL8811CU / 8821CU (Jaguar2, rtl8821c, 1T1R)"    ON)
option(DEVOURER_JAGUAR3_8822C "RTL8812CU / 8822CU (Jaguar3, rtl8822c)"          ON)
option(DEVOURER_JAGUAR3_8822E "RTL8812EU / 8822EU (Jaguar3, rtl8822e)"          ON)
# PCIe transport (vfio-pci). Linux-only. The transport implements the HalMAC-
# generation (Jaguar2/3) 88xx buffer-descriptor DMA rings; the RTL8821CE is
# the only chip with a PCIe bring-up implemented + hardware-validated so far,
# hence the DEVOURER_JAGUAR2_8821C requirement below. Other HalMAC PCIe parts
# (RTL8822BE / RTL8822CE) could ride the same rings with per-chip HAL gates;
# Jaguar1 PCIe parts (RTL8812AE/8821AE) CANNOT — they are pre-HalMAC silicon
# with the older rtlwifi-style own-bit descriptor DMA, a different engine.
# OFF by default — libusb-only builds are bit-identical.
option(DEVOURER_PCIE          "PCIe transport via vfio-pci (Linux; RTL8821CE)"  OFF)

# Compile-time diagnostics floor (src/logger.h). Calls below the floor —
# including their argument expressions at DVR_TRACE/DVR_DEBUG sites — compile
# to nothing. Empty (the default) keeps the NDEBUG-derived behavior: release
# builds drop trace/debug, debug builds keep everything. Example production
# firmware build: -DDEVOURER_LOG_MAX_LEVEL=WARN
set(DEVOURER_LOG_MAX_LEVEL "" CACHE STRING
    "Compile-time log floor: TRACE, DEBUG, INFO, WARN, ERROR or SILENT (empty = NDEBUG-derived)")
set_property(CACHE DEVOURER_LOG_MAX_LEVEL PROPERTY STRINGS
    "" TRACE DEBUG INFO WARN ERROR SILENT)
if(NOT DEVOURER_LOG_MAX_LEVEL STREQUAL "")
    set(_devourer_ll_valid TRACE DEBUG INFO WARN ERROR SILENT)
    if(NOT DEVOURER_LOG_MAX_LEVEL IN_LIST _devourer_ll_valid)
        message(FATAL_ERROR
            "DEVOURER_LOG_MAX_LEVEL=${DEVOURER_LOG_MAX_LEVEL} — must be one of "
            "TRACE, DEBUG, INFO, WARN, ERROR, SILENT (or empty).")
    endif()
endif()

if(DEVOURER_8814 AND NOT DEVOURER_JAGUAR1)
    message(FATAL_ERROR
        "DEVOURER_8814=ON requires DEVOURER_JAGUAR1=ON (8814 reuses the Jaguar1 HAL).")
endif()
# Convenience: the shared Jaguar2 HAL compiles when either chip variant is on.
if(DEVOURER_JAGUAR2_8822B OR DEVOURER_JAGUAR2_8821C)
    set(DEVOURER_JAGUAR2 ON)
else()
    set(DEVOURER_JAGUAR2 OFF)
endif()
# Convenience: the shared Jaguar3 port compiles when either variant is on.
if(DEVOURER_JAGUAR3_8822C OR DEVOURER_JAGUAR3_8822E)
    set(DEVOURER_JAGUAR3 ON)
else()
    set(DEVOURER_JAGUAR3 OFF)
endif()
if(NOT DEVOURER_JAGUAR1 AND NOT DEVOURER_JAGUAR2 AND NOT DEVOURER_JAGUAR3_8822C
   AND NOT DEVOURER_JAGUAR3_8822E)
    message(FATAL_ERROR
        "No chip support selected. Enable at least one of DEVOURER_JAGUAR1 / "
        "DEVOURER_JAGUAR2_8822B / DEVOURER_JAGUAR2_8821C / DEVOURER_JAGUAR3_8822C "
        "/ DEVOURER_JAGUAR3_8822E.")
endif()
if(DEVOURER_PCIE)
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
        message(FATAL_ERROR "DEVOURER_PCIE=ON requires Linux (vfio-pci).")
    endif()
    if(NOT DEVOURER_JAGUAR2_8821C)
        message(FATAL_ERROR
            "DEVOURER_PCIE=ON requires DEVOURER_JAGUAR2_8821C=ON: the RTL8821CE "
            "(Jaguar2 8821C) is currently the only chip with a PCIe bring-up "
            "implemented and hardware-validated, so a PCIe build without it has "
            "no usable chip. Not an architectural limit for the HalMAC families "
            "— RTL8822BE (Jaguar2) / RTL8822CE (Jaguar3) could reuse the same "
            "buffer-descriptor ring transport with per-chip HAL gates. Jaguar1 "
            "PCIe parts (RTL8812AE/8821AE) are out of reach of this transport: "
            "pre-HalMAC silicon with rtlwifi-style own-bit descriptor DMA, a "
            "different engine entirely.")
    endif()
endif()

# Find pkg-config and then use it to locate libusb.
find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

add_library(devourer
    # --- generation-agnostic core (always compiled) ---
    src/logger.h
    src/Event.h
    hal/basic_types.h
    hal/hal_com_reg.h
    src/ieee80211_radiotap.h
    src/ParsedRadioPacket.cpp
    src/Radiotap.c
    src/RadiotapBuilder.cpp
    src/RadiotapBuilder.h
    src/RtlTransport.h
    src/RtlAdapter.cpp
    src/RtlAdapter.h
    src/RtlUsbAdapter.h # compat alias shim (RtlUsbAdapter = RtlAdapter)
    src/UsbTransport.cpp
    src/UsbTransport.h
    src/UsbDeviceLock.cpp
    src/UsbDeviceLock.h
    src/UsbOpen.cpp
    src/UsbOpen.h
    src/SelectedChannel.h
    src/RateDefinitions.h
    src/RxPacket.h
    src/TxDescBits.h
    src/BeamformingSounder.h
    src/TxMode.cpp
    src/TxMode.h
    src/TxPower.cpp
    src/TxPower.h
    src/TxStats.h
    src/ThermalStatus.h
    src/TxCaps.h
    src/LinkHealth.cpp
    src/LinkHealth.h
    src/RxQuality.h
    src/IRtlDevice.h
    src/SignalStop.cpp
    src/SignalStop.h
    src/WiFiDriver.cpp
    src/WiFiDriver.h
    src/registry_priv.h
    # phydm check_positive table walker — shared by Jaguar1 and Jaguar2
    # (both use the older addr/value + check_positive table format).
    src/PhyTableLoader.cpp
    src/PhyTableLoader.h
)

target_compile_features(devourer PUBLIC cxx_std_20)

# PUBLIC: logger.h is header-only, so the library and its consumers must agree
# on the compile-time log floor (ODR).
if(NOT DEVOURER_LOG_MAX_LEVEL STREQUAL "")
    target_compile_definitions(devourer PUBLIC
        DEVOURER_LOG_MAX_LEVEL=DEVOURER_LL_${DEVOURER_LOG_MAX_LEVEL})
endif()

# Link devourer with libusb as found via pkg-config.
target_link_libraries(devourer PUBLIC PkgConfig::libusb)

target_include_directories(devourer PUBLIC hal)
target_include_directories(devourer PUBLIC hal/phydm/rtl8814a)
target_include_directories(devourer PUBLIC hal/phydm/rtl8822c)
target_include_directories(devourer PUBLIC hal/phydm/rtl8822e)
target_include_directories(devourer PUBLIC hal/phydm/rtl8822b)
target_include_directories(devourer PUBLIC hal/phydm/rtl8821c)
target_include_directories(devourer PUBLIC src)

# --- per-chip source groups (see options above) ---
if(DEVOURER_JAGUAR1)
    target_sources(devourer PRIVATE
        hal/hal8812a_fw.c hal/hal8812a_fw.h
        hal/hal8821a_fw.c hal/hal8821a_fw.h
        hal/Hal8812PwrSeq.c hal/Hal8812PwrSeq.h
        hal/Hal8821APwrSeq.c hal/Hal8821APwrSeq.h
        hal/Hal8812a_TxPwrTrack.cpp hal/Hal8812a_TxPwrTrack.h
        hal/Hal8812PhyReg.h hal/Hal8821PhyReg.h
        hal/rtl8812a_hal.h hal/rtl8812a_recv.h hal/rtl8812a_spec.h
        src/Firmware.h
        src/jaguar1/RtlJaguarDevice.cpp src/jaguar1/RtlJaguarDevice.h
        src/jaguar1/HalModule.cpp src/jaguar1/HalModule.h
        src/jaguar1/RadioManagementModule.cpp src/jaguar1/RadioManagementModule.h
        src/jaguar1/EepromManager.cpp src/jaguar1/EepromManager.h
        src/jaguar1/FirmwareManager.cpp src/jaguar1/FirmwareManager.h
        src/jaguar1/Iqk8812a.cpp src/jaguar1/Iqk8812a.h
        src/jaguar1/PhydmWatchdog.cpp src/jaguar1/PhydmWatchdog.h
        src/jaguar1/PowerTracking8812a.cpp src/jaguar1/PowerTracking8812a.h
        src/jaguar1/FrameParser.cpp src/jaguar1/FrameParser.h
        # NB: PhyTableLoader was promoted to src/ (shared with Jaguar2).
        src/jaguar1/BbDbgportReader.cpp src/jaguar1/BbDbgportReader.h)
    target_compile_definitions(devourer PUBLIC DEVOURER_HAVE_JAGUAR1=1)
endif()

if(DEVOURER_8814)
    target_sources(devourer PRIVATE
        hal/hal8814a_fw.c hal/hal8814a_fw.h
        hal/Hal8814PwrSeq.c hal/Hal8814PwrSeq.h
        hal/Hal8814PhyReg.h
        hal/phydm/rtl8814a/Hal8814_PhyTables.c
        hal/phydm/rtl8814a/Hal8814_PhyTables.h
        src/jaguar1/Iqk8814a.cpp src/jaguar1/Iqk8814a.h)
    target_compile_definitions(devourer PUBLIC DEVOURER_HAVE_8814=1)
endif()

# --- Jaguar2 (RTL8822BU / RTL8811CU-8821CU) ---
# Hybrid: HalMAC firmware/MAC-init/pwr-seq (like Jaguar3) + the older
# check_positive phydm table format (shared src/PhyTableLoader, like Jaguar1).
# Multi-chip via ChipVariant strategy dispatch (Jaguar2PhyTables / Jaguar2-
# Calibration), mirroring the Jaguar3 8822C/8822E split. The shared block below
# holds the orchestrator/HAL/halmac/strategy-trio + the per-variant dispatcher
# .cpp files (self-guarded on DEVOURER_HAVE_JAGUAR2_8822B/_8821C); each chip's
# firmware blob + generated tables live in its own variant block.
if(DEVOURER_JAGUAR2)
    target_sources(devourer PRIVATE
        src/jaguar2/RtlJaguar2Device.cpp src/jaguar2/RtlJaguar2Device.h
        src/jaguar2/HalJaguar2.cpp src/jaguar2/HalJaguar2.h
        src/jaguar2/HalmacJaguar2Fw.cpp src/jaguar2/HalmacJaguar2Fw.h
        src/jaguar2/HalmacJaguar2MacInit.cpp src/jaguar2/HalmacJaguar2MacInit.h
        src/jaguar2/HalmacJaguar2Regs.h
        src/jaguar2/FrameParserJaguar2.h
        src/jaguar2/ChipVariant.h
        src/jaguar2/Jaguar2PhyTables.h
        src/jaguar2/Jaguar2Calibration.h
        src/jaguar2/Phy8822bTables.cpp
        src/jaguar2/Halrf8822b.cpp src/jaguar2/Halrf8822b.h)
    target_compile_definitions(devourer PUBLIC DEVOURER_HAVE_JAGUAR2=1)
endif()

# 8822B variant data (2T2R): firmware blob + generated phydm tables + txpwr_lmt.
if(DEVOURER_JAGUAR2_8822B)
    target_sources(devourer PRIVATE
        hal/hal8822b_fw.c hal/hal8822b_fw.h
        hal/phydm/rtl8822b/Hal8822b_PhyTables.c hal/phydm/rtl8822b/Hal8822b_PhyTables.h)
    target_compile_definitions(devourer PUBLIC DEVOURER_HAVE_JAGUAR2_8822B=1)
endif()

# 8821C variant (1T1R): per-variant dispatcher impls (Phy8821cTables / Halrf8821c)
# + firmware blob + generated phydm tables. Mirrors the Jaguar3 8822E block.
if(DEVOURER_JAGUAR2_8821C)
    target_sources(devourer PRIVATE
        src/jaguar2/Phy8821cTables.cpp
        src/jaguar2/Halrf8821c.cpp src/jaguar2/Halrf8821c.h
        hal/hal8821c_fw.c hal/hal8821c_fw.h
        hal/phydm/rtl8821c/Hal8821c_PhyTables.c hal/phydm/rtl8821c/Hal8821c_PhyTables.h)
    target_compile_definitions(devourer PUBLIC DEVOURER_HAVE_JAGUAR2_8821C=1)
endif()

# --- PCIe transport (vfio-pci; see src/PcieTransport.h) ---
if(DEVOURER_PCIE)
    target_sources(devourer PRIVATE
        src/PcieTransport.cpp src/PcieTransport.h)
    target_compile_definitions(devourer PUBLIC DEVOURER_HAVE_PCIE=1)
endif()

if(DEVOURER_JAGUAR3)
    # Shared Jaguar3 port (compiled when either variant is on). Phy8822cTables.cpp
    # and Halrf8822c.cpp also carry the make_jaguar3_*() dispatchers, so they are
    # always built when Jaguar3 is on; their per-variant bodies are #if-guarded.
    target_sources(devourer PRIVATE
        src/jaguar3/Jaguar3PhyTables.h
        src/jaguar3/Jaguar3Calibration.h
        src/jaguar3/ChipVariant.h
        src/jaguar3/RtlJaguar3Device.cpp src/jaguar3/RtlJaguar3Device.h
        src/jaguar3/HalJaguar3.cpp src/jaguar3/HalJaguar3.h
        src/jaguar3/HalmacJaguar3Fw.cpp src/jaguar3/HalmacJaguar3Fw.h
        src/jaguar3/HalmacJaguar3MacInit.cpp src/jaguar3/HalmacJaguar3MacInit.h
        src/jaguar3/HalmacJaguar3Regs.h
        src/jaguar3/RadioManagementJaguar3.cpp src/jaguar3/RadioManagementJaguar3.h
        src/jaguar3/FrameParserJaguar3.h
        src/jaguar3/PhyTableLoaderJaguar3.cpp src/jaguar3/PhyTableLoaderJaguar3.h
        src/jaguar3/Phy8822cTables.cpp
        src/jaguar3/Halrf8822c.cpp src/jaguar3/Halrf8822c.h)
    target_compile_definitions(devourer PUBLIC DEVOURER_HAVE_JAGUAR3=1)
endif()

if(DEVOURER_JAGUAR3_8822C)
    target_sources(devourer PRIVATE
        hal/hal8822c_fw.c hal/hal8822c_fw.h
        hal/phydm/rtl8822c/Hal8822c_PhyTables.c hal/phydm/rtl8822c/Hal8822c_PhyTables.h
        hal/phydm/rtl8822c/Hal8822c_IqkNctl.c hal/phydm/rtl8822c/Hal8822c_IqkNctl.h)
    target_compile_definitions(devourer PUBLIC DEVOURER_HAVE_JAGUAR3_8822C=1)
endif()

if(DEVOURER_JAGUAR3_8822E)
    target_sources(devourer PRIVATE
        hal/hal8822e_fw.c hal/hal8822e_fw.h
        hal/phydm/rtl8822e/Hal8822e_PhyTables.c hal/phydm/rtl8822e/Hal8822e_PhyTables.h
        src/jaguar3/Phy8822eTables.cpp
        src/jaguar3/Halrf8822e.cpp src/jaguar3/Halrf8822e.h)
    target_compile_definitions(devourer PUBLIC DEVOURER_HAVE_JAGUAR3_8822E=1)
endif()


# pcieprobe — staged PCIe bring-up driver for the RTL8821CE milestones
# (chip-id/MMIO sanity -> power-on + EFUSE -> firmware DLFW). rxdemo's
# DEVOURER_PCIE_BDF branch is the full-RX consumer; this probe validates the
# layers below it one at a time. Linux/vfio only.
if(DEVOURER_PCIE)
    add_executable(pcieprobe
        examples/pcieprobe/main.cpp
    )
    target_link_libraries(pcieprobe PUBLIC devourer)
endif()

# reglat — register round-trip latency microbench (USB ctrl-xfer vs PCIe MMIO).
# Times RtlAdapter::rtw_read32; the PCIe path compiles only with DEVOURER_PCIE
# (inherited via the devourer PUBLIC DEVOURER_HAVE_PCIE define). See its header.
add_executable(reglat tests/reglat.cpp)
target_link_libraries(reglat PUBLIC devourer PRIVATE PkgConfig::libusb)

add_executable(rxdemo
    examples/rx/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(rxdemo PUBLIC devourer)
target_include_directories(rxdemo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)

add_executable(txdemo
    examples/tx/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(txdemo PUBLIC devourer PRIVATE PkgConfig::libusb)
target_include_directories(txdemo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)

# Pre-modulator subcarrier PoC: transmits PSDU bytes shaped by
# tools/precoder/encode_subcarriers.py so chosen OFDM data subcarriers carry
# chosen bits. Single-stream BPSK / HT MCS0 / BCC on 8812/8821/8811.
add_executable(precoder
    examples/precoder/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(precoder PUBLIC devourer PRIVATE PkgConfig::libusb)
target_include_directories(precoder PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)

# Stream-link TX: reads length-prefixed PSDU bodies from stdin and transmits
# one probe-request per body. Driven by tools/precoder/stream_tx.py — the
# C++/Python split keeps libusb out of the framing math.
add_executable(streamtx
    examples/streamtx/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(streamtx PUBLIC devourer PRIVATE PkgConfig::libusb)
# stream_stdin.h (shared binary-stdin framing) lives in examples/common/.
target_include_directories(streamtx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)

# SVC-T UEP injector: maps each HEVC NAL's temporal_id -> a robust/fast TxMode
# (svc_tx.h), so base/IDR layers fly robust and enhancement layers ride fast.
add_executable(svctx
    examples/svctx/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(svctx PUBLIC devourer PRIVATE PkgConfig::libusb)
target_include_directories(svctx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)

# tdma — burst-level bandwidth TDMA demo (docs/narrowband.md). Alternates
# narrowband/wide BURSTS via FastSetBandwidth; roles tx / rx-sync (lockstep,
# wallclock or TX-marker sync) / rx-camp (fixed band). tdma.h is header-only.
add_executable(tdma
    examples/tdma/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(tdma PUBLIC devourer PRIVATE PkgConfig::libusb)
target_include_directories(tdma PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/common
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/tdma)

# timesync — LTE-eNB-style over-the-air time distribution. One MASTER broadcasts
# its hardware TSF; SLAVES lock to it from the beacons alone (no GPS at the
# slaves). Reuses the tdma TD frame tag. timesync.h is header-only.
add_executable(timesync
    examples/timesync/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(timesync PUBLIC devourer PRIVATE PkgConfig::libusb)
target_include_directories(timesync PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/common
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/tdma
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/timesync)

# Stream-link DUPLEX: one binary, one chip, both directions. Combines the RX
# loop from rxdemo and the stdin-driven TX from streamtx —
# stdin = length-prefixed PSDU bodies, stdout = <devourer-stream> lines.
# tools/precoder/tun_p2p.py spawns this for --mode=duplex.
add_executable(duplex
    examples/duplex/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(duplex PUBLIC devourer PRIVATE PkgConfig::libusb)
target_include_directories(duplex PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)

# txpower — reference consumer of the runtime TX-power API
# (IRtlDevice::SetTxPowerOffsetQdb / SetTxPowerIndexOverride / GetTxPowerState /
# GetThermalStatus): opens one adapter, steps the knobs from CLI args (no env
# vars), and prints machine-readable state markers. The register-level
# validation driver for tests/txpwr_offset_regcheck.sh and the shape of an
# adaptive-link controller's power leg.
add_executable(txpower
    examples/txpower/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(txpower PUBLIC devourer PRIVATE PkgConfig::libusb)
target_include_directories(txpower PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)

# sense — Wi-Fi motion/presence sensor from beamforming reports. Drives
# two adapters (a sounder + a beamformee) on one host, self-captures the reports,
# decodes the per-tone Givens angles (src/BfReportDecode.h), and shows a live
# motion readout. See examples/sense/README.md and docs/beamforming-victim-sensing.md.
add_executable(sense
    examples/sense/main.cpp
    examples/common/env_config.cpp
)
target_link_libraries(sense PUBLIC devourer PRIVATE PkgConfig::libusb)
target_include_directories(sense PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)

# doctor — adapter health triage (src/AdapterHealth.h): EFUSE read-stability
# probe + firmware-boot outcome + RX smoke, classified into
# HEALTHY / SUSPECT / FAILING with reasons. Detects dying dongles that
# enumerate and init green but read stochastic EFUSE garbage / never boot the
# MCU (the #205 failure mode). Pure CLI; exit code carries the verdict. See
# docs/adapter-doctor.md.
add_executable(doctor
    examples/doctor/main.cpp
)
# caps_event.h (shared adapter.caps emitter) lives in examples/common/.
target_include_directories(doctor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)
target_link_libraries(doctor PUBLIC devourer PRIVATE PkgConfig::libusb)

# Headless regression guard for the binary-stdin framing shared by the two
# stream demos above (examples/common/stream_stdin.h). No libusb, no radio — just the
# set_stdin_binary() + read_exact() path, so a text-mode regression (e.g. the
# _setmode gate slipping back to _MSC_VER, which is invisible to a build-only
# check) fails `ctest` on the Windows/mingw jobs. See tests/stream_stdin_test.cmake.
add_executable(StreamStdinSelftest
    examples/common/stream_stdin_selftest.cpp
)
target_compile_features(StreamStdinSelftest PRIVATE cxx_std_20)
target_include_directories(StreamStdinSelftest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/common)

add_test(
    NAME stream_stdin_binary
    COMMAND ${CMAKE_COMMAND}
            -DSELFTEST_EXE=$<TARGET_FILE:StreamStdinSelftest>
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/stream_stdin_test.cmake
)

# Headless guard for the ToneMask tone-index math (src/ToneMask.h) — the fc
# derivation + subcarrier enumeration behind DEVOURER_RX_CSI_MASK /
# DEVOURER_RX_NBI. Pure math only; the register encodings are hardware-checked
# via the apply-time readback logging.
add_executable(ToneMaskSelftest
    tests/tone_mask_selftest.cpp
)
target_link_libraries(ToneMaskSelftest PRIVATE devourer)

add_test(NAME tone_mask_math COMMAND ToneMaskSelftest)

# Headless guard for the burst-TDMA example's pure logic (examples/tdma/tdma.h):
# the TsfClock host<->TSF least-squares fit, the frame-tag round-trip, and the
# schedule phase math. No hardware — the tsf_probe / tsf_tdoa_probe /
# tdma_tsf_test harnesses that need real adapters stay manual (out-of-band).
add_executable(TdmaTsfSelftest
    tests/tdma_tsf_selftest.cpp
)
target_link_libraries(TdmaTsfSelftest PRIVATE devourer)
target_include_directories(TdmaTsfSelftest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/tdma)

add_test(NAME tdma_tsf_math COMMAND TdmaTsfSelftest)

# Headless guard for the timesync example's pure logic (examples/timesync/
# timesync.h): the LinFit master<->local fit (ppm recovery + prediction), the
# inter-slave agreement bound, and the 32->64-bit TSF Recon wrap. No hardware —
# the master/slave demo that needs real adapters stays manual (out-of-band).
add_executable(TimesyncSelftest
    tests/timesync_selftest.cpp
)
target_link_libraries(TimesyncSelftest PRIVATE devourer)
target_include_directories(TimesyncSelftest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/common
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/tdma
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/timesync)

add_test(NAME timesync_math COMMAND TimesyncSelftest)

# Headless guard for the JSONL event emitter (src/Event.h) — the machine
# event stream every test script parses (docs/logging.md): first-field
# serialization guarantee, escaping, spill + truncation paths.
add_executable(LogEventSelftest
    tests/log_event_selftest.cpp
)
target_link_libraries(LogEventSelftest PRIVATE devourer)

add_test(NAME log_event_json COMMAND LogEventSelftest)

# Headless guard for the sweep/hop bin-list grammar (src/SweepSpec.h) — the
# channel-list / channel-range / MHz-range parsing behind DEVOURER_RX_SWEEP and
# DEVOURER_HOP_CHANNELS, so a grammar regression fails `ctest` instead of only
# surfacing on a radio.
add_executable(SweepSpecSelftest
    tests/sweep_spec_selftest.cpp
)
target_link_libraries(SweepSpecSelftest PRIVATE devourer)

add_test(NAME sweep_spec_math COMMAND SweepSpecSelftest)

# Headless guard for the TX-power offset quantization (src/TxPower.h) — the
# qdB->index-step rounding/clamping behind SetTxPowerOffsetQdb, so a rounding
# regression fails `ctest` instead of only surfacing as a wrong on-air level.
add_executable(TxPowerQuantSelftest
    tests/txpower_quant_selftest.cpp
)
target_link_libraries(TxPowerQuantSelftest PRIVATE devourer)

add_test(NAME txpower_quant_math COMMAND TxPowerQuantSelftest)

# Headless guard for the link-health classifier (src/LinkHealth.h) — the
# sensor-tuple -> verdict mapping behind <devourer-linkhealth>, with cases
# drawn from real on-air data (the saturation-knee + AWGN-interference sweeps).
# A misclassification fails `ctest` instead of only surfacing as bad advice.
add_executable(LinkHealthSelftest
    tests/link_health_selftest.cpp
)
target_link_libraries(LinkHealthSelftest PRIVATE devourer)

add_test(NAME link_health_classify COMMAND LinkHealthSelftest)

# Headless guard for the adapter-health classifier + EFUSE-stability probe
# loop (src/AdapterHealth.h) — the verdict mapping behind examples/doctor,
# with cases encoding the verdicts measured on a real dying unit vs its
# healthy twin (#205). A misgrade fails `ctest` instead of only surfacing as
# a wrong retire-or-keep call.
add_executable(AdapterHealthSelftest
    tests/adapter_health_selftest.cpp
)
target_link_libraries(AdapterHealthSelftest PRIVATE devourer)

add_test(NAME adapter_health_classify COMMAND AdapterHealthSelftest)

# Headless guard for the RX link-quality feed (src/RxQuality.h) — the aggregate
# math + passive noise-floor formula + verdict fuse behind GetRxQuality().
add_executable(RxQualitySelftest
    tests/rx_quality_selftest.cpp
)
target_link_libraries(RxQualitySelftest PRIVATE devourer)

add_test(NAME rx_quality_derive COMMAND RxQualitySelftest)

# Headless guard for the TX-capability derivation (src/TxCaps.h) — the
# STBC-needs-2-chains rule the send_packet 1T1R guard relies on.
add_executable(TxCapsSelftest
    tests/tx_caps_selftest.cpp
)
target_link_libraries(TxCapsSelftest PRIVATE devourer)

add_test(NAME tx_caps_derive COMMAND TxCapsSelftest)

# Headless guard for the adapter-capabilities API pure logic (src/AdapterCaps.h
# bw mask / gen names, the ActiveRxPaths classifier + accumulator in
# src/RxQuality.h, and the extended freq<->chan mapping in src/ChannelFreq.h).
add_executable(AdapterCapsSelftest
    tests/adapter_caps_selftest.cpp
)
target_link_libraries(AdapterCapsSelftest PRIVATE devourer)

add_test(NAME adapter_caps_derive COMMAND AdapterCapsSelftest)

# Headless guard for the Jaguar2 per-packet TX-power quantizer
# (jaguar2::txpkt_pwr_step_for_db) — dB delta -> the descriptor TXPWR_OFSET LUT
# step. Gated on Jaguar2 being built (the header is behind DEVOURER_HAVE_JAGUAR2).
if(DEVOURER_JAGUAR2_8822B OR DEVOURER_JAGUAR2_8821C)
    add_executable(TxPktPwrSelftest
        tests/txpkt_pwr_selftest.cpp
    )
    target_link_libraries(TxPktPwrSelftest PRIVATE devourer)
    target_include_directories(TxPktPwrSelftest PRIVATE src)
    add_test(NAME txpkt_pwr_quant COMMAND TxPktPwrSelftest)
endif()

# Headless guard for the beamforming-report decoder (src/BfReportDecode.h) — the
# LSB-first Givens-angle unpacking + per-tone variance behind sense. It
# decodes a real captured VHT MU report and checks the angles against an offline
# reference, so a port regression fails `ctest` instead of only on a radio.
add_executable(BfReportDecodeSelftest
    examples/sense/bf_report_decode_selftest.cpp
)
target_link_libraries(BfReportDecodeSelftest PRIVATE devourer)

add_test(NAME bf_report_decode COMMAND BfReportDecodeSelftest)

# Headless guard for the per-tone interference localizer's detection +
# frequency-mapping math (tests/rx_tone_localize.py). Pure Python, no hardware:
# synthetic psi-variance spikes / SNR notches must localize to the right tone
# group. The RF-side capture path is validated out-of-band (tests/rx_tone_localize.sh).
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(
        NAME rx_tone_localize_math
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/rx_tone_localize.py --self-test
    )
endif()
