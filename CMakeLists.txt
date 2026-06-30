cmake_minimum_required(VERSION 3.15)

# Set the toolchain file for vcpkg on Windows (only if VCPKG_ROOT is set;
# on mingw we get libusb from pkg-config directly).
if(WIN32 AND NOT "$ENV{VCPKG_ROOT}" STREQUAL "")
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
endif()

project(Rtl8812auNet)

enable_testing()

# Find pkg-config and then use it to locate libusb.
find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

add_library(WiFiDriver
    src/logger.h

    hal/Hal8812PhyReg.h
    hal/Hal8812PwrSeq.c
    hal/Hal8812PwrSeq.h
    hal/Hal8812a_TxPwrTrack.cpp
    hal/Hal8812a_TxPwrTrack.h
    hal/Hal8814PwrSeq.c
    hal/Hal8814PwrSeq.h
    hal/Hal8821APwrSeq.c
    hal/Hal8821APwrSeq.h
    hal/Hal8821PhyReg.h
    hal/basic_types.h
    hal/hal8812a_fw.c
    hal/hal8812a_fw.h
    hal/hal8814a_fw.c
    hal/hal8814a_fw.h
    hal/hal8821a_fw.c
    hal/hal8821a_fw.h
    hal/hal8822c_fw.c
    hal/hal8822c_fw.h
    hal/hal_com_reg.h
    hal/rtl8812a_hal.h
    hal/rtl8812a_recv.h
    hal/rtl8812a_spec.h

    # RTL8814AU phydm-format register tables. The .c file is generated from the
    # aircrack-ng/rtl8814au tree by tools/extract_8814a_phy_tables.py; the
    # loader at src/PhyTableLoader walks the same conditional encoding the
    # upstream phydm parser does, without pulling in phydm itself.
    hal/phydm/rtl8814a/Hal8814_PhyTables.c
    hal/phydm/rtl8814a/Hal8814_PhyTables.h

    # RTL8822C (Jaguar3) phydm-format BB/AGC/RF tables. Generated from the
    # rtl88x2cu tree by tools/extract_8822c_phy_tables.py. These use the newer
    # "halbb" conditional encoding — walked by src/jaguar3/PhyTableLoader8822c,
    # NOT src/PhyTableLoader (which handles the Jaguar1 check_positive format).
    hal/phydm/rtl8822c/Hal8822c_PhyTables.c
    hal/phydm/rtl8822c/Hal8822c_PhyTables.h
    hal/phydm/rtl8822c/Hal8822c_IqkNctl.c
    hal/phydm/rtl8822c/Hal8822c_IqkNctl.h

    src/ieee80211_radiotap.h
    src/BbDbgportReader.cpp
    src/BbDbgportReader.h
    src/EepromManager.cpp
    src/EepromManager.h
    src/Firmware.h
    src/FirmwareManager.cpp
    src/FirmwareManager.h
    src/FrameParser.cpp
    src/FrameParser.h
    src/HalModule.cpp
    src/HalModule.h
    src/Iqk8812a.cpp
    src/Iqk8812a.h
    src/Iqk8814a.cpp
    src/Iqk8814a.h
    src/PhydmWatchdog.cpp
    src/PhydmWatchdog.h
    src/ParsedRadioPacket.cpp
    src/PhyTableLoader.cpp
    src/PhyTableLoader.h
    src/PowerTracking8812a.cpp
    src/PowerTracking8812a.h
    src/RadioManagementModule.cpp
    src/RadioManagementModule.h
    src/Radiotap.c
    src/RadiotapBuilder.cpp
    src/RadiotapBuilder.h
    src/RtlJaguarDevice.cpp
    src/RtlJaguarDevice.h
    src/RtlUsbAdapter.cpp
    src/RtlUsbAdapter.h
    src/SelectedChannel.h
    src/TxMode.cpp
    src/TxMode.h
    src/IRtlDevice.h
    src/SignalStop.cpp
    src/SignalStop.h
    src/WiFiDriver.cpp
    src/WiFiDriver.h
    src/registry_priv.h

    # Jaguar3 (RTL8822CU / RTL8812EU / RTL8822EU) port.
    src/jaguar3/RtlJaguar3Device.cpp
    src/jaguar3/RtlJaguar3Device.h
    src/jaguar3/Hal8822c.cpp
    src/jaguar3/Hal8822c.h
    src/jaguar3/Halmac8822cFw.cpp
    src/jaguar3/Halmac8822cFw.h
    src/jaguar3/Halmac8822cMacInit.cpp
    src/jaguar3/Halmac8822cMacInit.h
    src/jaguar3/Halrf8822cIqk.cpp
    src/jaguar3/Halrf8822cIqk.h
    src/jaguar3/Halmac8822cRegs.h
    src/jaguar3/RadioManagement8822c.cpp
    src/jaguar3/RadioManagement8822c.h
    src/jaguar3/FrameParser8822c.h
    src/jaguar3/PhyTableLoader8822c.cpp
    src/jaguar3/PhyTableLoader8822c.h
)

target_compile_features(WiFiDriver PUBLIC cxx_std_20)

# Link WiFiDriver with libusb as found via pkg-config.
target_link_libraries(WiFiDriver PUBLIC PkgConfig::libusb)

target_include_directories(WiFiDriver PUBLIC hal)
target_include_directories(WiFiDriver PUBLIC hal/phydm/rtl8814a)
target_include_directories(WiFiDriver PUBLIC hal/phydm/rtl8822c)
target_include_directories(WiFiDriver PUBLIC src)

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
