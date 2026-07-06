#include "core/http_controller.hpp"
#include "constants.hpp"
#include <cmath>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <utility>

/**
 * @file http_controller.cpp
 * @brief Lightweight runtime control API for scan/ADS-B flags and scan config.
 */

namespace rtl::core {

HttpController::HttpController(
    std::atomic<bool>&          running,
    std::atomic<bool>&          adsbEnabled,
    std::atomic<bool>&          scanEnabled,
    std::mutex&                 scanConfigMutex,
    ScanRuntimeConfig&          scanConfig)
    : running_(running)
    , adsbEnabled_(adsbEnabled)
    , scanEnabled_(scanEnabled)
    , scanConfigMutex_(scanConfigMutex)
    , scanConfig_(scanConfig) {}

HttpController::~HttpController() {
    stop();
}

bool HttpController::start(int port) {
    registerRoutes();
    if (serverThread_.joinable()) return false;

    // cpp-httplib listen() is blocking, so keep it on its own thread and report
    // startup failure through the shared process-running flag.
    serverThread_ = std::thread([this, port]() {
        if (!srv_.listen("0.0.0.0", port)) {
            spdlog::error("HTTP server failed to bind 0.0.0.0:{}", port);
            running_ = false;
        }
    });

    for (int i = 0; i < 20; ++i) {
        if (srv_.is_running()) {
            spdlog::info("HTTP control server started on port {}", port);
            return true;
        }
        if (!running_.load()) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    spdlog::warn("HTTP control server startup was not confirmed within timeout");
    return srv_.is_running();
}

void HttpController::setAdsbStopCallback(std::function<void()> callback) {
    adsbStopCallback_ = std::move(callback);
}

void HttpController::setScanStopCallback(std::function<void()> callback) {
    scanStopCallback_ = std::move(callback);
}

void HttpController::stop() {
    srv_.stop();
    if (serverThread_.joinable()) serverThread_.join();
}

void HttpController::registerRoutes() {
    // Start/stop endpoints only flip intent flags. Long-running hardware reads
    // are interrupted through callbacks into the owning Application engines.
    srv_.Post("/adsb/start", [this](const httplib::Request&, httplib::Response& res) {
        adsbEnabled_.store(true);
        res.set_content(R"({"status":"ok","adsb":"started"})", "application/json");
    });

    srv_.Post("/adsb/stop", [this](const httplib::Request&, httplib::Response& res) {
        adsbEnabled_.store(false);
        if (adsbStopCallback_) adsbStopCallback_();
        res.set_content(R"({"status":"ok","adsb":"stopped"})", "application/json");
    });

    srv_.Post("/scan/start", [this](const httplib::Request&, httplib::Response& res) {
        scanEnabled_.store(true);
        if (adsbStopCallback_) adsbStopCallback_();
        res.set_content(R"({"status":"ok","scan":"started"})", "application/json");
    });

    srv_.Post("/scan/stop", [this](const httplib::Request&, httplib::Response& res) {
        scanEnabled_.store(false);
        if (scanStopCallback_) scanStopCallback_();
        res.set_content(R"({"status":"ok","scan":"stopped"})", "application/json");
    });

    srv_.Post("/scan/param", [this](const httplib::Request& req, httplib::Response& res) {
        constexpr std::size_t MAX_SCAN_PARAM_BODY = 8192;
        if (req.body.size() > MAX_SCAN_PARAM_BODY) {
            res.status = 413;
            res.set_content(R"({"status":"error","msg":"Request body too large"})", "application/json");
            return;
        }

        const auto logBody = req.body.size() > 512 ? req.body.substr(0, 512) + "...[truncated]" : req.body;
        spdlog::info("scan param update: {}", logBody);
        try {
            // Missing fields inherit from the current scan config. The final
            // assignment below commits all fields together so the radio loop
            // never observes a half-updated range/profile/sample-rate tuple.
            ScanRuntimeConfig currentConfig;
            {
                std::lock_guard<std::mutex> lock(scanConfigMutex_);
                currentConfig = scanConfig_;
            }

            auto   obj = nlohmann::json::parse(req.body);
            double sf  = obj.contains("start_freq") ? obj["start_freq"].get<double>() : currentConfig.startFreqHz;
            double ef  = obj.contains("end_freq") ? obj["end_freq"].get<double>() : currentConfig.endFreqHz;
            auto   profile = currentConfig.profile;
            auto   sampleRateHz = currentConfig.sampleRateHz;
            auto   resetPolicy = currentConfig.resetPolicy;
            const bool profileWasProvided = obj.contains("scan_profile");
            if (obj.contains("scan_profile")) {
                const auto value = obj["scan_profile"].get<std::string>();
                if (!rtl::scanner::parseScanProfile(value, profile)) {
                    res.status = 400;
                    res.set_content(R"({"status":"error","msg":"Invalid scan_profile"})", "application/json");
                    return;
                }
            }
            if (obj.contains("scan_sample_rate")) {
                const auto rawRate = obj["scan_sample_rate"].get<double>();
                if (!std::isfinite(rawRate) || rawRate <= 0.0) {
                    res.status = 400;
                    res.set_content(R"({"status":"error","msg":"Invalid scan_sample_rate"})", "application/json");
                    return;
                }
                // Match the CLI convention: small numeric values are MHz-style
                // rates such as 2.4, while larger values are already Hz.
                sampleRateHz = static_cast<std::uint32_t>(rawRate < 10000.0 ? rawRate * 1e6 : rawRate);
                if (!rtl::scanner::isSupportedScanSampleRate(sampleRateHz)) {
                    res.status = 400;
                    res.set_content(R"({"status":"error","msg":"Unsupported scan_sample_rate"})", "application/json");
                    return;
                }
            } else if (profileWasProvided) {
                sampleRateHz = rtl::scanner::defaultSampleRateForProfile(profile);
            }
            if (obj.contains("reset_policy")) {
                const auto value = obj["reset_policy"].get<std::string>();
                if (!rtl::scanner::parseResetPolicy(value, resetPolicy)) {
                    res.status = 400;
                    res.set_content(R"({"status":"error","msg":"Invalid reset_policy"})", "application/json");
                    return;
                }
            }

            if (!std::isfinite(sf) || !std::isfinite(ef)) {
                res.status = 400;
                res.set_content(R"({"status":"error","msg":"Frequency must be finite"})", "application/json");
                return;
            }
            if (sf < rtl::constants::MIN_FREQ || sf > rtl::constants::MAX_FREQ || ef < rtl::constants::MIN_FREQ
                || ef > rtl::constants::MAX_FREQ) {
                res.status = 400;
                res.set_content(R"({"status":"error","msg":"Frequency out of range"})", "application/json");
                return;
            }
            if (ef <= sf) {
                res.status = 400;
                res.set_content(R"({"status":"error","msg":"Invalid frequency range"})", "application/json");
                return;
            }
            if ((ef - sf) > rtl::constants::MAX_BANDWIDTH) {
                res.status = 400;
                res.set_content(R"({"status":"error","msg":"Bandwidth exceeds 100 MHz"})", "application/json");
                return;
            }

            ScanRuntimeConfig nextConfig;
            nextConfig.startFreqHz  = static_cast<std::uint32_t>(sf);
            nextConfig.endFreqHz    = static_cast<std::uint32_t>(ef);
            nextConfig.profile      = profile;
            nextConfig.sampleRateHz = sampleRateHz;
            nextConfig.resetPolicy  = resetPolicy;
            {
                std::lock_guard<std::mutex> lock(scanConfigMutex_);
                scanConfig_ = nextConfig;
            }

            nlohmann::json res_obj;
            res_obj["status"]           = "ok";
            res_obj["scan_profile"]     = rtl::scanner::scanProfileName(profile);
            res_obj["scan_sample_rate"] = sampleRateHz;
            res_obj["reset_policy"]     = rtl::scanner::resetPolicyName(resetPolicy);
            res.set_content(res_obj.dump(4), "application/json");

            spdlog::info(
                "scan_param updated: {} - {} MHz, profile={}, sample_rate={} MS/s, reset_policy={}",
                sf / 1e6,
                ef / 1e6,
                rtl::scanner::scanProfileName(profile),
                sampleRateHz / 1e6,
                rtl::scanner::resetPolicyName(resetPolicy));
        } catch (const std::exception& e) {
            spdlog::error("scan_param handler error: {}", e.what());
            nlohmann::json res_obj;
            res_obj["status"] = "error";
            res_obj["msg"]    = "Invalid scan parameter request";
            res.status        = 400;
            res.set_content(res_obj.dump(4), "application/json");
        }
    });

    srv_.Get("/status", [this](const httplib::Request&, httplib::Response& res) {
        ScanRuntimeConfig scanConfig;
        {
            std::lock_guard<std::mutex> lock(scanConfigMutex_);
            scanConfig = scanConfig_;
        }
        nlohmann::json status;
        status["adsb_enabled"] = adsbEnabled_.load();
        status["scan_enabled"] = scanEnabled_.load();
        status["start_freq"]   = scanConfig.startFreqHz;
        status["end_freq"]     = scanConfig.endFreqHz;
        status["scan_profile"] = rtl::scanner::scanProfileName(scanConfig.profile);
        status["scan_sample_rate"] = scanConfig.sampleRateHz;
        status["reset_policy"] = rtl::scanner::resetPolicyName(scanConfig.resetPolicy);
        res.set_content(status.dump(2), "application/json");
    });
}

} // namespace rtl::core
