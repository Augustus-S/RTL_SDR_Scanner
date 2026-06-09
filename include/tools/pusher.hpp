#pragma once

#include <atomic>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <nlohmann/json.hpp>
#include "ads_b/aircraft.hpp"
#include "constants.hpp"
#include "scanner/types.hpp"
#include "tools/tools.hpp"

/**
 * @file pusher.hpp
 * @brief Asynchronous HTTP data pusher for ADS-B aircraft and scan spectra.
 */

namespace rtl::tools {

/**
 * @brief Background HTTP pusher used by scanner and ADS-B modules.
 *
 * @note Push methods enqueue data and return quickly. The worker thread batches
 * aircraft updates and posts scan data to the configured service URL.
 */
class Pusher {
public:
    /**
     * @brief Create and start a background pusher.
     * @param url HTTP endpoint URL that accepts JSON POST requests.
     */
    explicit Pusher(const std::string& url = rtl::constants::DATA_URL);

    /**
     * @brief Stop the background worker and release resources.
     */
    ~Pusher();

    Pusher(const Pusher&)            = delete;
    Pusher& operator=(const Pusher&) = delete;

    /**
     * @brief Background worker loop.
     * @note This is started by the constructor and normally should not be
     * called directly by application code.
     */
    void work();

    /**
     * @brief Queue one decoded aircraft update.
     * @param aircraft Aircraft state object.
     */
    void pushData(const rtl::sda_b::Aircraft& aircraft);

    /**
     * @brief Push a complete scan spectrum and optional detected peaks.
     * @param plan Scanner plan describing the scan range.
     * @param fullSpectrum Spectrum power values in dBFS.
     * @param peaks Detected peak ranges.
     */
    void pushScanData(
        const rtl::scanner::ScanPlan&            plan,
        const std::vector<double>&               fullSpectrum,
        const std::vector<rtl::tools::PeakInfo>& peaks);

    /**
     * @brief Queue one scan JSON payload for asynchronous posting.
     * @param jsonData JSON object that already matches the external API schema.
     */
    void pushScanHop(nlohmann::json jsonData);

private:
    using AircraftQueue = std::deque<rtl::sda_b::Aircraft>;

    std::unordered_map<std::string, rtl::sda_b::Aircraft> filterLatestAircraft(AircraftQueue& queue);
    bool                                                  hasPendingData() const;

    std::atomic<bool> loop_{true};
    std::thread       workThread_;

    mutable std::mutex         lock_;
    AircraftQueue              aircraftQueue_;
    std::deque<nlohmann::json> scanQueue_;

    std::string url_;

    static constexpr std::size_t MAX_AIRCRAFT_QUEUE = 5000;
    static constexpr std::size_t MAX_SCAN_QUEUE     = 50;
};

} // namespace rtl::tools
