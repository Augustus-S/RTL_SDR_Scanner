#include "core/application.hpp"
#include "constants.hpp"
#include "tools/tools.hpp"
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <csignal>
#include <cmath>
#include <chrono>

/**
 * @file application.cpp
 * @brief Owns process lifetime, shared RTL-SDR device ownership, and worker threads.
 */

namespace rtl::core {

namespace {

Application* g_appInstance = nullptr;

void signalHandler(int) {
    if (g_appInstance) { g_appInstance->requestExit(); }
}

} // namespace

Application::Application(const AppConfig& config)
    : config_(config) {
    adsbEnabled_.store(config.adsbEnabled);
    scanEnabled_.store(config.scanEnabled);
    scanConfig_.startFreqHz  = static_cast<std::uint32_t>(config.startFreqHz);
    scanConfig_.endFreqHz    = static_cast<std::uint32_t>(config.endFreqHz);
    scanConfig_.profile      = config.scanProfile;
    scanConfig_.sampleRateHz = config.scanSampleRateHz == 0
                                   ? rtl::scanner::defaultSampleRateForProfile(config.scanProfile)
                                   : config.scanSampleRateHz;
    scanConfig_.resetPolicy  = config.resetPolicy;
}

Application::~Application() {
    requestShutdown();
    if (radioThread_.joinable()) radioThread_.join();
    if (heartbeatThread_.joinable()) heartbeatThread_.join();
}

void Application::requestExit() {
    running_ = false;
}

void Application::requestShutdown() {
    running_ = false;
    // The radio thread may be blocked inside an ADS-B async read or scanner read;
    // ask both engines to interrupt their current slice before joining threads.
    if (adsbEngine_) adsbEngine_->requestStop();
    if (scanEngine_) scanEngine_->requestStop();
}

void Application::setupSignalHandlers() {
    g_appInstance = this;
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
}

bool Application::initDevice() {
    int deviceCount = rtl::scanner::RtlSdrDevice::getDeviceCount();
    if (deviceCount == 0) {
        spdlog::error("No RTL-SDR devices found");
        sendEvent("scan_events", "RTL_SDR device not found.", false);
        return false;
    }

    device_ = std::make_unique<rtl::scanner::RtlSdrDevice>(0);
    if (!device_->isOpen()) {
        spdlog::error("Failed to open RTL-SDR device");
        sendEvent("scan_events", "RTL_SDR device not found.", false);
        return false;
    }

    device_->setMaxGain();
    int maxGain = device_->getCurrentGain();
    maxGain_.store(maxGain);

    // The single physical dongle is shared by scan and ADS-B modes. Set an
    // initial rate that matches the startup mode before the radio loop begins
    // reconfiguring the device per slice.
    std::uint32_t initialScanRate = rtl::constants::SCAN_SAMPLE_RATE;
    {
        std::lock_guard<std::mutex> lock(scanConfigMutex_);
        initialScanRate = scanConfig_.sampleRateHz;
    }
    device_->setSampleRate(scanEnabled_.load() ? initialScanRate : rtl::constants::SCAN_SAMPLE_RATE);
    device_->stabilize();

    spdlog::info("Device initialized, starting...");
    return true;
}

void Application::sendEvent(const std::string& eventType, const std::string& msg, bool scanOpen) {
    nlohmann::json eventJson;
    eventJson["id"]    = 200;
    eventJson["event"] = eventType;
    nlohmann::json dataObj;
    if (eventType == "scan_heartbeat") {
        int devState = 0;
        if (running_.load()) {
            devState |= 4;
            if (scanOpen) { devState |= 2; }
        }
        dataObj["dev_state"] = devState;
    } else {
        dataObj["msg"] = msg;
    }
    eventJson["data"] = dataObj;
    rtl::tools::post(std::string(rtl::constants::DATA_URL), eventJson.dump(), 1);
}

void Application::runHeartbeat() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!running_) break;
        sendEvent("scan_heartbeat", "", scanEnabled_.load());
    }
}

void Application::runRadioLoop() {
    while (running_) {
        if (!scanEnabled_.load()) {
            if (adsbEnabled_.load()) {
                spdlog::info("Entering continuous ADS-B mode");
                auto result = adsbEngine_->runSlice(std::chrono::milliseconds(0), [this] {
                    return running_.load() && adsbEnabled_.load() && !scanEnabled_.load();
                });
                if (result == rtl::sda_b::ADSBEngine::RunResult::DEVICE_ERROR) { running_ = false; }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }

        // Copy the HTTP-updatable scan configuration under one lock. The sweep
        // then runs against this stable snapshot even if /scan/param changes
        // while hardware reads are in progress.
        ScanRuntimeConfig scanConfig;
        {
            std::lock_guard<std::mutex> lock(scanConfigMutex_);
            scanConfig = scanConfig_;
        }
        scanEngine_->setFreqRange(scanConfig.startFreqHz, scanConfig.endFreqHz);
        scanEngine_->setScanProfile(scanConfig.profile);
        scanEngine_->setScanSampleRate(scanConfig.sampleRateHz);
        scanEngine_->setResetPolicy(scanConfig.resetPolicy);
        scanEngine_->start();
        {
            std::lock_guard<std::mutex> lock(scanConfigMutex_);
            // If the scanner fell back to a lower hardware-supported sample
            // rate, publish that active rate unless the user already submitted a
            // new scan configuration.
            if (scanConfig_.profile == scanConfig.profile && scanConfig_.sampleRateHz == scanConfig.sampleRateHz) {
                scanConfig_.sampleRateHz = scanEngine_->getScanSampleRate();
            }
        }
        auto sweepResult = scanEngine_->doOneSweep([this] {
            return running_.load() && scanEnabled_.load();
        });
        scanEngine_->stop();
        if (sweepResult == rtl::scanner::ScanEngine::SweepResult::DEVICE_ERROR) {
            running_ = false;
            break;
        }

        if (running_ && scanEnabled_.load() && adsbEnabled_.load()) {
            spdlog::info("Decoding ADS-B signal (time-slice)...");
            // Combined mode alternates full scan sweeps with short ADS-B slices
            // because one RTL-SDR device cannot tune both bands simultaneously.
            auto result = adsbEngine_->runSlice(std::chrono::seconds(10), [this] {
                return running_.load() && adsbEnabled_.load() && scanEnabled_.load();
            });
            if (result == rtl::sda_b::ADSBEngine::RunResult::DEVICE_ERROR) { running_ = false; }
        }
    }

    scanEngine_->stop();
}

int Application::run() {
    setupSignalHandlers();

    if (!initDevice()) return 1;

    spdlog::info("ADS-B: {}", adsbEnabled_.load() ? "enabled" : "disabled");
    spdlog::info("Scan: {}", scanEnabled_.load() ? "enabled" : "disabled");
    if (scanEnabled_.load()) {
        ScanRuntimeConfig scanConfig;
        {
            std::lock_guard<std::mutex> lock(scanConfigMutex_);
            scanConfig = scanConfig_;
        }
        spdlog::info(
            "Scan range: {:.1f} - {:.1f} MHz, profile={}, sample_rate={} MS/s, reset_policy={}",
            scanConfig.startFreqHz / 1e6,
            scanConfig.endFreqHz / 1e6,
            rtl::scanner::scanProfileName(scanConfig.profile),
            scanConfig.sampleRateHz / 1e6,
            rtl::scanner::resetPolicyName(scanConfig.resetPolicy));
    }

    pusher_     = std::make_unique<rtl::tools::Pusher>(rtl::constants::DATA_URL);
    scanEngine_ = std::make_unique<rtl::scanner::ScanEngine>(device_->getRawDev(), *pusher_);
    adsbEngine_ = std::make_unique<rtl::sda_b::ADSBEngine>(device_->getRawDev(), *pusher_, maxGain_.load());

    httpController_ = std::make_unique<HttpController>(
        running_,
        adsbEnabled_,
        scanEnabled_,
        scanConfigMutex_,
        scanConfig_);
    httpController_->setAdsbStopCallback([this] {
        if (adsbEngine_) adsbEngine_->requestStop();
    });
    httpController_->setScanStopCallback([this] {
        if (scanEngine_) scanEngine_->requestStop();
    });
    if (!httpController_->start(rtl::constants::CONTROL_PORT)) {
        spdlog::error("Failed to start HTTP controller");
        return 1;
    }

    // HTTP control starts before the workers so runtime start/stop commands can
    // change the atomic enable flags immediately after process startup.
    heartbeatThread_ = std::thread(&Application::runHeartbeat, this);
    radioThread_     = std::thread(&Application::runRadioLoop, this);

    while (running_) { std::this_thread::sleep_for(std::chrono::milliseconds(500)); }

    spdlog::info("Shutting down...");

    running_ = false;

    if (adsbEngine_) adsbEngine_->requestStop();
    if (scanEngine_) scanEngine_->requestStop();

    if (radioThread_.joinable()) radioThread_.join();
    if (heartbeatThread_.joinable()) heartbeatThread_.join();

    if (scanEngine_) scanEngine_->stop();

    httpController_->stop();

    device_.reset();
    pusher_.reset();

    spdlog::info("Stopped.");

    return 0;
}

} // namespace rtl::core
