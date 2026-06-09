#include "core/application.hpp"
#include "constants.hpp"
#include "tools/tools.hpp"
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <csignal>
#include <cmath>
#include <chrono>

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
    startFreq_.store(static_cast<std::uint32_t>(config.startFreqHz));
    endFreq_.store(static_cast<std::uint32_t>(config.endFreqHz));
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

    device_->setSampleRate(rtl::constants::SCAN_SAMPLE_RATE);
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

        scanEngine_->setFreqRange(startFreq_.load(), endFreq_.load());
        scanEngine_->start();
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
        spdlog::info("Scan range: {:.1f} - {:.1f} MHz", startFreq_.load() / 1e6, endFreq_.load() / 1e6);
    }

    pusher_     = std::make_unique<rtl::tools::Pusher>(rtl::constants::DATA_URL);
    scanEngine_ = std::make_unique<rtl::scanner::ScanEngine>(device_->getRawDev(), *pusher_);
    adsbEngine_ = std::make_unique<rtl::sda_b::ADSBEngine>(device_->getRawDev(), *pusher_, maxGain_.load());

    httpController_ = std::make_unique<HttpController>(running_, adsbEnabled_, scanEnabled_, startFreq_, endFreq_);
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
