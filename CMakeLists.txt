cmake_minimum_required(VERSION 3.16)
project(RTL_SDR_Scanner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3)
pkg_check_modules(RTLSDR REQUIRED librtlsdr)
pkg_check_modules(ALSA REQUIRED IMPORTED_TARGET alsa)

find_package(spdlog REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(${PROJECT_NAME}
    src/RTL_SDR.cpp
    lib/tools/tools.cpp
    lib/tools/pusher.cpp
    lib/tools/fft_engine.cpp
    lib/scanner/rtl_sdr_device.cpp
    lib/scanner/persistent_async_reader.cpp
    lib/scanner/scan_engine.cpp
    lib/ads_b/ads_b_demodulator.cpp
    lib/ads_b/adsb_engine.cpp
    lib/core/application.cpp
    lib/core/http_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${RTLSDR_INCLUDE_DIRS}
    include
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${RTLSDR_LIBRARIES}
    PkgConfig::FFTW3
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    pthread
)

target_link_directories(${PROJECT_NAME} PRIVATE ${RTLSDR_LIBRARY_DIRS})

add_executable(fm_broadcast_player
    example/fm_broadcast_player.cpp
)

target_include_directories(fm_broadcast_player PRIVATE
    ${RTLSDR_INCLUDE_DIRS}
)

target_link_libraries(fm_broadcast_player PRIVATE
    ${RTLSDR_LIBRARIES}
    PkgConfig::ALSA
    spdlog::spdlog
)

target_link_directories(fm_broadcast_player PRIVATE ${RTLSDR_LIBRARY_DIRS})
