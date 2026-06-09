#include "core/http_controller.hpp"
#include "constants.hpp"
#include <cmath>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace rtl::core {

HttpController::HttpController(
    std::atomic<bool>&         running,
    std::atomic<bool>&         adsbEnabled,
    std::atomic<bool>&         scanEnabled,
    std::atomic<std::uint32_t>& startFreq,
    std::atomic<std::uint32_t>& endFreq)
    : running_(running)
    , adsbEnabled_(adsbEnabled)
    , scanEnabled_(scanEnabled)
    , startFreq_(startFreq)
    , endFreq_(endFreq) {
}

HttpController::~HttpController() {
    stop();
}

bool HttpController::start(int port) {
    registerRoutes();

    serverThread_ = std::thread([this, port]() {
        if (!srv_.listen("0.0.0.0", port)) {
            spdlog::error("HTTP server failed to bind 0.0.0.0:{}", port);
            running_ = false;
        }
    });

    spdlog::info("HTTP control server started on port {}", port);
    return true;
}

void HttpController::stop() {
    srv_.stop();
    if (serverThread_.joinable()) serverThread_.join();
}

void HttpController::registerRoutes() {
    srv_.Post("/adsb/start", [this](const httplib::Request&, httplib::Response& res) {
        adsbEnabled_.store(true);
        res.set_content(R"({"status":"ok","adsb":"started"})", "application/json");
    });

    srv_.Post("/adsb/stop", [this](const httplib::Request&, httplib::Response& res) {
        adsbEnabled_.store(false);
        res.set_content(R"({"status":"ok","adsb":"stopped"})", "application/json");
    });

    srv_.Post("/scan/start", [this](const httplib::Request&, httplib::Response& res) {
        scanEnabled_.store(true);
        res.set_content(R"({"status":"ok","scan":"started"})", "application/json");
    });

    srv_.Post("/scan/stop", [this](const httplib::Request&, httplib::Response& res) {
        scanEnabled_.store(false);
        res.set_content(R"({"status":"ok","scan":"stopped"})", "application/json");
    });

    srv_.Post("/scan/param", [this](const httplib::Request& req, httplib::Response& res) {
        spdlog::info("scan param update: {}", req.body);
        try {
            auto   obj = nlohmann::json::parse(req.body);
            double sf  = obj.contains("start_freq") ? obj["start_freq"].get<double>() : startFreq_.load();
            double ef  = obj.contains("end_freq") ? obj["end_freq"].get<double>() : endFreq_.load();

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

            startFreq_.store(static_cast<std::uint32_t>(sf));
            endFreq_.store(static_cast<std::uint32_t>(ef));

            nlohmann::json res_obj;
            res_obj["status"] = "ok";
            res.set_content(res_obj.dump(4), "application/json");

            spdlog::info("scan_param updated: {} - {} MHz", sf / 1e6, ef / 1e6);
        } catch (const std::exception& e) {
            spdlog::error("scan_param handler error: {}", e.what());
            nlohmann::json res_obj;
            res_obj["status"] = "error";
            res_obj["msg"]    = e.what();
            res.status        = 400;
            res.set_content(res_obj.dump(4), "application/json");
        }
    });

    srv_.Get("/status", [this](const httplib::Request&, httplib::Response& res) {
        nlohmann::json status;
        status["adsb_enabled"] = adsbEnabled_.load();
        status["scan_enabled"] = scanEnabled_.load();
        status["start_freq"]   = startFreq_.load();
        status["end_freq"]     = endFreq_.load();
        res.set_content(status.dump(2), "application/json");
    });
}

} // namespace rtl::core