cmake_minimum_required(VERSION 3.15)

# This should go before project declaration.
if (WIN32)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
endif ()

project(Rtl8812auNet)

# the `pkg_check_modules` function is created with this call
find_package(PkgConfig REQUIRED)

# this calls create special `PkgConfig::LIBUSB` variables
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

find_package(spdlog CONFIG REQUIRED)

add_library(WiFiDriver
        src/logger.h

        hal/Hal8812PhyReg.h
        hal/Hal8812PwrSeq.c
        hal/Hal8812PwrSeq.h
        hal/basic_types.h
        hal/hal8812a_fw.c
        hal/hal8812a_fw.h
        hal/hal_com_reg.h
        hal/rtl8812a_hal.h
        hal/rtl8812a_recv.h
        hal/rtl8812a_spec.h

        src/EepromManager.cpp
        src/EepromManager.h
        src/Firmware.h
        src/FirmwareManager.cpp
        src/FirmwareManager.h
        src/FrameParser.cpp
        src/FrameParser.h
        src/HalModule.cpp
        src/HalModule.h
        src/ParsedRadioPacket.cpp
        src/RadioManagementModule.cpp
        src/RadioManagementModule.h
        src/Rtl8812aDevice.cpp
        src/Rtl8812aDevice.h
        src/RtlUsbAdapter.cpp
        src/RtlUsbAdapter.h
        src/SelectedChannel.h
        src/WiFiDriver.cpp
        src/WiFiDriver.h
        src/registry_priv.h
)

target_compile_features(WiFiDriver PUBLIC cxx_std_20)

target_link_libraries(WiFiDriver PUBLIC
        PkgConfig::LIBUSB
        # Use the header-only version
        spdlog::spdlog_header_only
)

target_include_directories(WiFiDriver PUBLIC hal)
target_include_directories(WiFiDriver PUBLIC src)

add_executable(WiFiDriverDemo
        demo/main.cpp)

target_link_libraries(WiFiDriverDemo PUBLIC WiFiDriver)
