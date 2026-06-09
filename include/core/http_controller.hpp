#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "httplib.h"

namespace rtl::core {

class HttpController {
public:
    HttpController(
        std::atomic<bool>&          running,
        std::atomic<bool>&          adsbEnabled,
        std::atomic<bool>&          scanEnabled,
        std::atomic<std::uint32_t>& startFreq,
        std::atomic<std::uint32_t>& endFreq);
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
    std::atomic<std::uint32_t>& startFreq_;
    std::atomic<std::uint32_t>& endFreq_;
    std::function<void()>       adsbStopCallback_;
    std::function<void()>       scanStopCallback_;
};

} // namespace rtl::core
