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
    if (g_appInstance) {
        g_appInstance->requestShutdown();
    }
}

void printHelp(const char* progName) {
    std::cout << "RTL-SDR Scanner v2.0\n"
              << "====================\n\n"
              << "Usage: " << progName << " [options]\n\n"
              << "Options:\n"
              << "  --mode <1|2|3>        1=ADS-B only, 2=Scan only, 3=Both\n"
              << "  --start-freq <MHz>    Scan start frequency (10-1070 MHz)\n"
              << "  --end-freq <MHz>      Scan end frequency (10-1070 MHz)\n"
              << "  --adsb                Enable ADS-B decoding\n"
              << "  --scan                Enable frequency scanning\n"
              << "  --help, -h            Show this help message\n\n"
              << "Frequency range: 10 MHz - 1070 MHz\n"
              << "Maximum bandwidth: 100 MHz\n"
              << "Data output: 127.0.0.1:23568\n"
              << "Control port: 127.0.0.1:23569\n";
}

AppConfig parseArgs(int argc, char* argv[]) {
    AppConfig config;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp(argv[0]);
            exit(0);
        } else if (arg == "--mode" && i + 1 < argc) {
            int mode = std::atoi(argv[++i]);
            if (mode == 1)
                config.adsbEnabled = true;
            else if (mode == 2)
                config.scanEnabled = true;
            else if (mode == 3) {
                config.adsbEnabled = true;
                config.scanEnabled = true;
            }
        } else if (arg == "--start-freq" && i + 1 < argc) {
            config.startFreqHz = std::atof(argv[++i]) * 1e6;
        } else if (arg == "--end-freq" && i + 1 < argc) {
            config.endFreqHz = std::atof(argv[++i]) * 1e6;
        } else if (arg == "--adsb") {
            config.adsbEnabled = true;
        } else if (arg == "--scan") {
            config.scanEnabled = true;
        }
    }
    return config;
}

AppConfig interactiveMenu() {
    AppConfig config;

    std::cout << "\nRTL-SDR Scanner v2.0\n"
              << "====================\n"
              << "1. ADS-B Decode Only\n"
              << "2. Scan Only\n"
              << "3. ADS-B + Scan\n"
              << "4. Exit\n"
              << "\nSelect mode: ";

    int mode;
    std::cin >> mode;

    if (mode == 1) {
        config.adsbEnabled = true;
    } else if (mode == 2) {
        config.scanEnabled = true;
    } else if (mode == 3) {
        config.adsbEnabled = true;
        config.scanEnabled = true;
    } else {
        exit(0);
    }

    if (config.scanEnabled) {
        std::cout << "\nFrequency Configuration\n"
                  << "=======================\n"
                  << "Range: 10 MHz - 1070 MHz, Max bandwidth: 100 MHz\n";

        double startMhz, endMhz;
        while (true) {
            std::cout << "Enter start frequency (MHz): ";
            std::cin >> startMhz;
            if (startMhz < 10.0 || startMhz > 1070.0) {
                std::cout << "Error: Start frequency must be between 10 and 1070 MHz\n";
                continue;
            }
            break;
        }

        while (true) {
            std::cout << "Enter end frequency (MHz): ";
            std::cin >> endMhz;
            if (endMhz < 10.0 || endMhz > 1070.0) {
                std::cout << "Error: End frequency must be between 10 and 1070 MHz\n";
                continue;
            }
            if (endMhz <= startMhz) {
                std::cout << "Error: End frequency must be greater than start frequency\n";
                continue;
            }
            if ((endMhz - startMhz) > 100.0) {
                std::cout << "Error: Bandwidth cannot exceed 100 MHz\n";
                continue;
            }
            break;
        }

        config.startFreqHz = startMhz * 1e6;
        config.endFreqHz   = endMhz * 1e6;
    }

    return config;
}

bool validateConfig(const AppConfig& config) {
    if (!config.adsbEnabled && !config.scanEnabled) {
        spdlog::error("No functionality selected");
        return false;
    }

    if (config.scanEnabled) {
        if (!std::isfinite(config.startFreqHz) || !std::isfinite(config.endFreqHz)) {
            spdlog::error("Frequency must be finite");
            return false;
        }
        if (config.startFreqHz < rtl::constants::MIN_FREQ || config.startFreqHz > rtl::constants::MAX_FREQ) {
            spdlog::error("Start frequency out of range: {} MHz", config.startFreqHz / 1e6);
            return false;
        }
        if (config.endFreqHz < rtl::constants::MIN_FREQ || config.endFreqHz > rtl::constants::MAX_FREQ) {
            spdlog::error("End frequency out of range: {} MHz", config.endFreqHz / 1e6);
            return false;
        }
        if (config.endFreqHz <= config.startFreqHz) {
            spdlog::error("End frequency must be greater than start frequency");
            return false;
        }
        if ((config.endFreqHz - config.startFreqHz) > rtl::constants::MAX_BANDWIDTH) {
            spdlog::error("Bandwidth exceeds 100 MHz limit");
            return false;
        }
    }
    return true;
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
    running_ = false;
    if (radioThread_.joinable()) radioThread_.join();
    if (heartbeatThread_.joinable()) heartbeatThread_.join();
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
                adsbEngine_->runSlice(std::chrono::milliseconds(0));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }

        scanEngine_->setFreqRange(startFreq_.load(), endFreq_.load());
        scanEngine_->start();
        scanEngine_->doOneSweep();
        scanEngine_->stop();

        if (running_ && scanEnabled_.load() && adsbEnabled_.load()) {
            spdlog::info("Decoding ADS-B signal (time-slice)...");
            adsbEngine_->runSlice(std::chrono::seconds(10));
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

    httpController_ = std::make_unique<HttpController>(
        running_, adsbEnabled_, scanEnabled_, startFreq_, endFreq_);
    httpController_->start(rtl::constants::CONTROL_PORT);

    heartbeatThread_ = std::thread(&Application::runHeartbeat, this);
    radioThread_     = std::thread(&Application::runRadioLoop, this);

    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    spdlog::info("Shutting down...");

    running_ = false;

    if (adsbEngine_) adsbEngine_->requestStop();

    scanEngine_->stop();

    if (radioThread_.joinable()) radioThread_.join();
    if (heartbeatThread_.joinable()) heartbeatThread_.join();

    httpController_->stop();

    device_.reset();
    pusher_.reset();

    spdlog::info("Stopped.");

    return 0;
}

} // namespace rtl::core