#include "tools/pusher.hpp"

#include <algorithm>
#include <spdlog/spdlog.h>

namespace rtl::tools {

Pusher::Pusher(const std::string& url) {
    url_        = url;
    workThread_ = std::thread(&Pusher::work, this);
}

Pusher::~Pusher() {
    loop_ = false;
    if (workThread_.joinable()) workThread_.join();
}

void Pusher::pushData(const rtl::sda_b::Aircraft& aircraft) {
    std::lock_guard<std::mutex> _(lock_);
    if (aircraftQueue_.size() >= MAX_AIRCRAFT_QUEUE) { aircraftQueue_.pop_front(); }
    aircraftQueue_.push_back(aircraft);
}

void Pusher::pushScanHop(nlohmann::json jsonData) {
    std::lock_guard<std::mutex> _(lock_);
    if (scanQueue_.size() >= MAX_SCAN_QUEUE) { scanQueue_.pop_front(); }
    scanQueue_.push_back(std::move(jsonData));
}

std::unordered_map<std::string, rtl::sda_b::Aircraft> Pusher::filterLatestAircraft(AircraftQueue& queue) {
    std::unordered_map<std::string, rtl::sda_b::Aircraft> latestData;

    for (auto& aircraft : queue) {
        auto it = latestData.find(aircraft.uuid);
        if (it == latestData.end()) {
            latestData.emplace(aircraft.uuid, std::move(aircraft));
        } else {
            auto& existing = it->second;
            if (aircraft.seq > existing.seq
                || (aircraft.seq == existing.seq && aircraft.lastTimeS > existing.lastTimeS)) {
                existing = std::move(aircraft);
            }
        }
    }

    return latestData;
}

void Pusher::work() {
    while (loop_) {
        for (int i = 0; i < 10 && loop_; ++i) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
        if (!loop_) break;

        std::unordered_map<std::string, rtl::sda_b::Aircraft> latestData;
        std::deque<nlohmann::json>                            scanBatch;

        {
            std::lock_guard<std::mutex> _(lock_);
            if (!aircraftQueue_.empty()) {
                latestData = filterLatestAircraft(aircraftQueue_);
                aircraftQueue_.clear();
                aircraftQueue_.shrink_to_fit();
            }
            if (!scanQueue_.empty()) { scanBatch.swap(scanQueue_); }
        }

        if (!latestData.empty()) {
            nlohmann::json::array_t dataList;
            for (auto& pair : latestData) { dataList.push_back(pair.second.toJson()); }

            nlohmann::json jsonData;
            jsonData["event"] = "ADSB_DATA_LIST";
            jsonData["data"]  = dataList;

            spdlog::info("pushData: {}", jsonData.dump());
            post(url_, jsonData.dump(), 3);
        }

        for (auto& scanJson : scanBatch) {
            spdlog::info("pushScanHop, batch_size={}", scanBatch.size());
            std::string result = post(url_, scanJson.dump(), 3);
            if (result.empty()) { spdlog::warn("pushScanHop post returned empty, data may be lost"); }
        }
    }
}

void Pusher::pushScanData(
    const rtl::scanner::ScanPlan&            plan,
    const std::vector<double>&               fullSpectrum,
    const std::vector<rtl::tools::PeakInfo>& peaks) {
    if (fullSpectrum.empty()) return;

    double maxVal = *std::max_element(fullSpectrum.begin(), fullSpectrum.end());
    double minVal = *std::min_element(fullSpectrum.begin(), fullSpectrum.end());

    nlohmann::json resultArr = nlohmann::json::array();
    for (const auto& p : peaks) {
        nlohmann::json r;
        r["cf"]                = p.cf;
        r["bw"]                = p.bw;
        r["avg_db"]            = p.avgDb;
        r["max_db"]            = p.maxDb;
        r["level"]             = p.level;
        r["position"]["start"] = p.posStart;
        r["position"]["stop"]  = p.posStop;
        resultArr.push_back(r);
    }

    nlohmann::json dataObj;
    dataObj["start_freq"] = plan.startFreq;
    dataObj["end_freq"]   = plan.endFreq;
    dataObj["max_value"]  = maxVal;
    dataObj["min_value"]  = minVal;
    dataObj["data"]       = fullSpectrum;
    dataObj["result"]     = resultArr;

    nlohmann::json jsonData;
    jsonData["event"] = "scan_data";
    jsonData["data"]  = dataObj;

    spdlog::info("pushScanData: {} peaks, {} bins", peaks.size(), fullSpectrum.size());
    post(url_, jsonData.dump(), 3);
}

} // namespace rtl::tools
