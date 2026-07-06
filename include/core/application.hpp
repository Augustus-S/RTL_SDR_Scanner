#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include "ads_b/adsb_engine.hpp"
#include "core/http_controller.hpp"
#include "scanner/rtl_sdr_device.hpp"
#include "scanner/scan_engine.hpp"
#include "scanner/scan_profile.hpp"
#include "tools/pusher.hpp"

namespace rtl::core {

/**
 * @brief Initial application configuration parsed from CLI or the interactive menu.
 *
 * Frequency values are in Hz. scanSampleRateHz may be zero, in which case the
 * application selects the default sample rate for scanProfile.
 */
struct AppConfig {
    bool   adsbEnabled = false;
    bool   scanEnabled = false;
    double startFreqHz = 10e6;
    double endFreqHz   = 100e6;
    rtl::scanner::ScanProfile scanProfile = rtl::scanner::ScanProfile::BALANCED;
    std::uint32_t scanSampleRateHz = 0;
    rtl::scanner::ResetPolicy resetPolicy = rtl::scanner::ResetPolicy::ADAPTIVE;
};

class Application {
public:
    explicit Application(const AppConfig& config);
    ~Application();

    Application(const Application&)            = delete;
    Application& operator=(const Application&) = delete;

    int run();

    void requestExit();
    void requestShutdown();

private:
    void setupSignalHandlers();
    bool initDevice();
    void sendEvent(const std::string& eventType, const std::string& msg, bool scanOpen);
    void runHeartbeat();
    void runRadioLoop();

    AppConfig         config_;
    std::atomic<bool> running_{true};

    std::atomic<bool>          adsbEnabled_{false};
    std::atomic<bool>          scanEnabled_{false};
    std::mutex                 scanConfigMutex_;
    ScanRuntimeConfig          scanConfig_;
    std::atomic<int>           maxGain_{0};

    std::unique_ptr<rtl::scanner::RtlSdrDevice> device_;
    std::unique_ptr<rtl::tools::Pusher>         pusher_;
    std::unique_ptr<rtl::scanner::ScanEngine>   scanEngine_;
    std::unique_ptr<rtl::sda_b::ADSBEngine>     adsbEngine_;
    std::unique_ptr<HttpController>             httpController_;

    std::thread heartbeatThread_;
    std::thread radioThread_;
};

} // namespace rtl::core
