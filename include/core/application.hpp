#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include "ads_b/adsb_engine.hpp"
#include "core/http_controller.hpp"
#include "scanner/rtl_sdr_device.hpp"
#include "scanner/scan_engine.hpp"
#include "tools/pusher.hpp"

namespace rtl::core {

struct AppConfig {
    bool   adsbEnabled = false;
    bool   scanEnabled = false;
    double startFreqHz = 10e6;
    double endFreqHz   = 100e6;
};

class Application {
public:
    explicit Application(const AppConfig& config);
    ~Application();

    Application(const Application&)            = delete;
    Application& operator=(const Application&) = delete;

    int run();

    void requestShutdown() { running_ = false; }

private:
    void setupSignalHandlers();
    bool initDevice();
    void sendEvent(const std::string& eventType, const std::string& msg, bool scanOpen);
    void runHeartbeat();
    void runRadioLoop();

    AppConfig config_;
    std::atomic<bool> running_{true};

    std::atomic<bool>          adsbEnabled_{false};
    std::atomic<bool>          scanEnabled_{false};
    std::atomic<std::uint32_t> startFreq_{0};
    std::atomic<std::uint32_t> endFreq_{0};
    std::atomic<int>           maxGain_{0};

    std::unique_ptr<rtl::scanner::RtlSdrDevice> device_;
    std::unique_ptr<rtl::tools::Pusher>          pusher_;
    std::unique_ptr<rtl::scanner::ScanEngine>    scanEngine_;
    std::unique_ptr<rtl::sda_b::ADSBEngine>      adsbEngine_;
    std::unique_ptr<HttpController>              httpController_;

    std::thread heartbeatThread_;
    std::thread radioThread_;
};

} // namespace rtl::core