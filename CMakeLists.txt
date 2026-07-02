cmake_minimum_required(VERSION 3.15)

# Set the toolchain file for vcpkg on Windows (only if VCPKG_ROOT is set;
# on mingw we get libusb from pkg-config directly).
if(WIN32 AND NOT "$ENV{VCPKG_ROOT}" STREQUAL "")
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
endif()

project(Rtl8812auNet)

enable_testing()

# ---------------------------------------------------------------------------
# Per-chip build options. All default ON (backward compatible: a plain
# `cmake -S . -B build` builds every chip). Turn off the ones you don't need to
# drop their firmware blobs + generated PHY tables and shrink the binary — e.g.
# an 8812AU-only build:
#   cmake -S . -B build -DDEVOURER_8814=OFF \
#         -DDEVOURER_JAGUAR3_8822C=OFF -DDEVOURER_JAGUAR3_8822E=OFF
# ---------------------------------------------------------------------------
option(DEVOURER_JAGUAR1       "RTL8812AU / 8811AU / 8821AU (Jaguar1 core)"      ON)
option(DEVOURER_8814          "RTL8814AU (4T4R; requires DEVOURER_JAGUAR1)"     ON)
option(DEVOURER_JAGUAR3_8822C "RTL8812CU / 8822CU (Jaguar3, rtl8822c)"          ON)
option(DEVOURER_JAGUAR3_8822E "RTL8812EU / 8822EU (Jaguar3, rtl8822e)"          ON)
option(DEVOURER_JAGUAR2       "RTL8822BU (Jaguar2, rtl8822b, 2T2R USB)"         ON)

if(DEVOURER_8814 AND NOT DEVOURER_JAGUAR1)
    message(FATAL_ERROR
        "DEVOURER_8814=ON requires DEVOURER_JAGUAR1=ON (8814 reuses the Jaguar1 HAL).")
endif()
if(NOT DEVOURER_JAGUAR1 AND NOT DEVOURER_JAGUAR3_8822C AND NOT DEVOURER_JAGUAR3_8822E
   AND NOT DEVOURER_JAGUAR2)
    message(FATAL_ERROR
        "No chip support selected. Enable at least one of DEVOURER_JAGUAR1 / "
        "DEVOURER_JAGUAR3_8822C / DEVOURER_JAGUAR3_8822E / DEVOURER_JAGUAR2.")
endif()
# Convenience: the shared Jaguar3 port compiles when either variant is on.
if(DEVOURER_JAGUAR3_8822C OR DEVOURER_JAGUAR3_8822E)
    set(DEVOURER_JAGUAR3 ON)
else()
    set(DEVOURER_JAGUAR3 OFF)
endif()

# Find pkg-config and then use it to locate libusb.
find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

add_library(WiFiDriver
    # --- generation-agnostic core (always compiled) ---
    src/logger.h
    hal/basic_types.h
    hal/hal_com_reg.h
    src/ieee80211_radiotap.h
    src/ParsedRadioPacket.cpp
    src/Radiotap.c
    src/RadiotapBuilder.cpp
    src/RadiotapBuilder.h
    src/RtlUsbAdapter.cpp
    src/RtlUsbAdapter.h
    src/SelectedChannel.h
    src/RateDefinitions.h
    src/RxPacket.h
    src/TxDescBits.h
    src/BeamformingSounder.h
    src/TxMode.cpp
    src/TxMode.h
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

target_compile_features(WiFiDriver PUBLIC cxx_std_20)

# Link WiFiDriver with libusb as found via pkg-config.
target_link_libraries(WiFiDriver PUBLIC PkgConfig::libusb)

target_include_directories(WiFiDriver PUBLIC hal)
target_include_directories(WiFiDriver PUBLIC hal/phydm/rtl8814a)
target_include_directories(WiFiDriver PUBLIC hal/phydm/rtl8822c)
target_include_directories(WiFiDriver PUBLIC hal/phydm/rtl8822e)
target_include_directories(WiFiDriver PUBLIC src)

# --- per-chip source groups (see options above) ---
if(DEVOURER_JAGUAR1)
    target_sources(WiFiDriver PRIVATE
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
    target_compile_definitions(WiFiDriver PUBLIC DEVOURER_HAVE_JAGUAR1=1)
endif()

if(DEVOURER_8814)
    target_sources(WiFiDriver PRIVATE
        hal/hal8814a_fw.c hal/hal8814a_fw.h
        hal/Hal8814PwrSeq.c hal/Hal8814PwrSeq.h
        hal/Hal8814PhyReg.h
        hal/phydm/rtl8814a/Hal8814_PhyTables.c
        hal/phydm/rtl8814a/Hal8814_PhyTables.h
        src/jaguar1/Iqk8814a.cpp src/jaguar1/Iqk8814a.h)
    target_compile_definitions(WiFiDriver PUBLIC DEVOURER_HAVE_8814=1)
endif()

if(DEVOURER_JAGUAR3)
    # Shared Jaguar3 port (compiled when either variant is on). Phy8822cTables.cpp
    # and Halrf8822c.cpp also carry the make_jaguar3_*() dispatchers, so they are
    # always built when Jaguar3 is on; their per-variant bodies are #if-guarded.
    target_sources(WiFiDriver PRIVATE
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
    target_compile_definitions(WiFiDriver PUBLIC DEVOURER_HAVE_JAGUAR3=1)
endif()

if(DEVOURER_JAGUAR3_8822C)
    target_sources(WiFiDriver PRIVATE
        hal/hal8822c_fw.c hal/hal8822c_fw.h
        hal/phydm/rtl8822c/Hal8822c_PhyTables.c hal/phydm/rtl8822c/Hal8822c_PhyTables.h
        hal/phydm/rtl8822c/Hal8822c_IqkNctl.c hal/phydm/rtl8822c/Hal8822c_IqkNctl.h)
    target_compile_definitions(WiFiDriver PUBLIC DEVOURER_HAVE_JAGUAR3_8822C=1)
endif()

if(DEVOURER_JAGUAR3_8822E)
    target_sources(WiFiDriver PRIVATE
        hal/hal8822e_fw.c hal/hal8822e_fw.h
        hal/phydm/rtl8822e/Hal8822e_PhyTables.c hal/phydm/rtl8822e/Hal8822e_PhyTables.h
        src/jaguar3/Phy8822eTables.cpp
        src/jaguar3/Halrf8822e.cpp src/jaguar3/Halrf8822e.h)
    target_compile_definitions(WiFiDriver PUBLIC DEVOURER_HAVE_JAGUAR3_8822E=1)
endif()

# --- Jaguar2 (RTL8822BU / chip 8822B) ---
# Hybrid: HalMAC firmware/MAC-init/pwr-seq (like Jaguar3) + the older
# check_positive phydm table format (shared src/PhyTableLoader, like Jaguar1).
# Single chip, so no per-variant strategy dispatch.
if(DEVOURER_JAGUAR2)
    target_sources(WiFiDriver PRIVATE
        src/jaguar2/RtlJaguar2Device.cpp src/jaguar2/RtlJaguar2Device.h)
    target_compile_definitions(WiFiDriver PUBLIC DEVOURER_HAVE_JAGUAR2=1)
endif()


add_executable(WiFiDriverDemo
    demo/main.cpp
)
target_link_libraries(WiFiDriverDemo PUBLIC WiFiDriver)

add_executable(WiFiDriverTxDemo
    txdemo/main.cpp
)
target_link_libraries(WiFiDriverTxDemo PUBLIC WiFiDriver PRIVATE PkgConfig::libusb)

# Pre-modulator subcarrier PoC: transmits PSDU bytes shaped by
# tools/precoder/encode_subcarriers.py so chosen OFDM data subcarriers carry
# chosen bits. Single-stream BPSK / HT MCS0 / BCC on 8812/8821/8811.
add_executable(PrecoderDemo
    txdemo/precoder_demo/main.cpp
)
target_link_libraries(PrecoderDemo PUBLIC WiFiDriver PRIVATE PkgConfig::libusb)

# Stream-link TX: reads length-prefixed PSDU bodies from stdin and transmits
# one probe-request per body. Driven by tools/precoder/stream_tx.py — the
# C++/Python split keeps libusb out of the framing math.
add_executable(StreamTxDemo
    txdemo/stream_tx_demo/main.cpp
)
target_link_libraries(StreamTxDemo PUBLIC WiFiDriver PRIVATE PkgConfig::libusb)
# stream_stdin.h (shared binary-stdin framing) lives in txdemo/.
target_include_directories(StreamTxDemo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/txdemo)

# SVC-T UEP injector: maps each HEVC NAL's temporal_id -> a robust/fast TxMode
# (svc_tx.h), so base/IDR layers fly robust and enhancement layers ride fast.
add_executable(SvcTxDemo
    txdemo/svc_tx_demo/main.cpp
)
target_link_libraries(SvcTxDemo PUBLIC WiFiDriver PRIVATE PkgConfig::libusb)
target_include_directories(SvcTxDemo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/txdemo)

# Stream-link DUPLEX: one binary, one chip, both directions. Combines the RX
# loop from WiFiDriverDemo and the stdin-driven TX from StreamTxDemo —
# stdin = length-prefixed PSDU bodies, stdout = <devourer-stream> lines.
# tools/precoder/tun_p2p.py spawns this for --mode=duplex.
add_executable(StreamDuplexDemo
    txdemo/stream_duplex_demo/main.cpp
)
target_link_libraries(StreamDuplexDemo PUBLIC WiFiDriver PRIVATE PkgConfig::libusb)
target_include_directories(StreamDuplexDemo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/txdemo)

# Headless regression guard for the binary-stdin framing shared by the two
# stream demos above (txdemo/stream_stdin.h). No libusb, no radio — just the
# set_stdin_binary() + read_exact() path, so a text-mode regression (e.g. the
# _setmode gate slipping back to _MSC_VER, which is invisible to a build-only
# check) fails `ctest` on the Windows/mingw jobs. See tests/stream_stdin_test.cmake.
add_executable(StreamStdinSelftest
    txdemo/stream_stdin_selftest.cpp
)
target_compile_features(StreamStdinSelftest PRIVATE cxx_std_20)
target_include_directories(StreamStdinSelftest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/txdemo)

add_test(
    NAME stream_stdin_binary
    COMMAND ${CMAKE_COMMAND}
            -DSELFTEST_EXE=$<TARGET_FILE:StreamStdinSelftest>
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/stream_stdin_test.cmake
)
