#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "httplib.h"
#include "scanner/scan_profile.hpp"

namespace rtl::core {

/**
 * @brief Runtime scan configuration shared by the HTTP controller and radio loop.
 *
 * Frequencies and sample rates are expressed in Hz. Callers should update and
 * read this structure while holding the associated mutex so a scan uses one
 * consistent configuration snapshot.
 */
struct ScanRuntimeConfig {
    std::uint32_t startFreqHz      = 0;
    std::uint32_t endFreqHz        = 0;
    rtl::scanner::ScanProfile profile = rtl::scanner::ScanProfile::BALANCED;
    std::uint32_t sampleRateHz     = 0;
    rtl::scanner::ResetPolicy resetPolicy = rtl::scanner::ResetPolicy::ADAPTIVE;
};

class HttpController {
public:
    HttpController(
        std::atomic<bool>&          running,
        std::atomic<bool>&          adsbEnabled,
        std::atomic<bool>&          scanEnabled,
        std::mutex&                 scanConfigMutex,
        ScanRuntimeConfig&          scanConfig);
    ~HttpController();

    HttpController(const HttpController&)            = delete;
    HttpController& operator=(const HttpController&) = delete;

    bool start(int port);
    void stop();
    void setAdsbStopCallback(std::function<void()> callback);
    void setScanStopCallback(std::function<void()> callback);

private:
    void registerRoutes();

    httplib::Server srv_;
    std::thread     serverThread_;

    std::atomic<bool>&          running_;
    std::atomic<bool>&          adsbEnabled_;
    std::atomic<bool>&          scanEnabled_;
    std::mutex&                 scanConfigMutex_;
    ScanRuntimeConfig&          scanConfig_;
    std::function<void()>       adsbStopCallback_;
    std::function<void()>       scanStopCallback_;
};

} // namespace rtl::core
