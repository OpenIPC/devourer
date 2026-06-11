cmake_minimum_required(VERSION 3.15)

# Set the toolchain file for vcpkg on Windows.
if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
endif()

project(Rtl8812auNet)

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
    src/WiFiDriver.cpp
    src/WiFiDriver.h
    src/registry_priv.h
)

target_compile_features(WiFiDriver PUBLIC cxx_std_20)

# Link WiFiDriver with libusb as found via pkg-config.
target_link_libraries(WiFiDriver PUBLIC PkgConfig::libusb)

target_include_directories(WiFiDriver PUBLIC hal)
target_include_directories(WiFiDriver PUBLIC hal/phydm/rtl8814a)
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

# Stream-link DUPLEX: one binary, one chip, both directions. Combines the RX
# loop from WiFiDriverDemo and the stdin-driven TX from StreamTxDemo —
# stdin = length-prefixed PSDU bodies, stdout = <devourer-stream> lines.
# tools/precoder/tun_p2p.py spawns this for --mode=duplex.
add_executable(StreamDuplexDemo
    txdemo/stream_duplex_demo/main.cpp
)
target_link_libraries(StreamDuplexDemo PUBLIC WiFiDriver PRIVATE PkgConfig::libusb)
