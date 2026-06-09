#include "scanner/scan_engine.hpp"
#include "constants.hpp"
#include "tools/tools.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <nlohmann/json.hpp>

namespace rtl::scanner {

ScanEngine::ScanEngine(rtlsdr_dev_t* dev, rtl::tools::Pusher& pusher)
    : dev_(dev)
    , pusher_(pusher)
    , fftEngine_(rtl::constants::FFT_SIZE) {
    bufferU8_.resize(rtl::constants::BUFFER_LEN);
    bufferIQ_.resize(rtl::constants::BUFFER_LEN / 2);
    bufferQ_.resize(rtl::constants::BUFFER_LEN / 2);
}

ScanEngine::~ScanEngine() {
    stop();
}

void ScanEngine::start() {
    if (running_.exchange(true)) return;

    rtlsdr_set_sample_rate(dev_, rtl::constants::SCAN_SAMPLE_RATE);
    reader_ = std::make_unique<PersistentAsyncReader>(dev_);
}

void ScanEngine::stop() {
    if (!running_.exchange(false)) return;

    if (reader_) {
        reader_->shutdown();
        reader_.reset();
    }
}

void ScanEngine::setFreqRange(std::uint32_t startHz, std::uint32_t endHz) {
    startFreq_.store(startHz);
    endFreq_.store(endHz);
}

std::pair<std::uint32_t, std::uint32_t> ScanEngine::getFreqRange() const {
    return {startFreq_.load(), endFreq_.load()};
}

bool ScanEngine::doOneSweep() {
    if (!running_ || !reader_) return false;

    std::uint32_t sweep_start_freq = startFreq_.load();
    std::uint32_t sweep_end_freq   = endFreq_.load();

    auto sweep_start = std::chrono::steady_clock::now();
    std::vector<SegmentData> segments;

    for (std::uint32_t cur_freq = sweep_start_freq;
         cur_freq <= sweep_end_freq && running_;
         cur_freq += rtl::constants::STEP_FREQ) {
        int need_ds = (cur_freq < rtl::constants::LOW_FREQ_THRESHOLD) ? 2 : 0;
        processOneHop(cur_freq, need_ds, segments);
    }

    if (!running_) return false;

    if (!segments.empty()) {
        spliceAndPush(segments, sweep_start_freq, sweep_end_freq);

        auto sweep_end     = std::chrono::steady_clock::now();
        auto sweep_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(sweep_end - sweep_start);
        spdlog::info("Sweep complete, took {} ms", sweep_elapsed.count());
    }

    return true;
}

void ScanEngine::processOneHop(std::uint32_t centerFreq, int directSampling, std::vector<SegmentData>& segments) {
    std::uint32_t n_read = 4 * rtl::constants::FFT_SIZE * 2;
    auto scan_result =
        reader_->read(bufferU8_.data(), &n_read, centerFreq, directSampling, rtl::constants::READ_TIMEOUT_MS);

    if (scan_result != PersistentAsyncReader::ReadResult::SUCCESS) {
        if (scan_result == PersistentAsyncReader::ReadResult::DEVICE_ERROR) {
            spdlog::error("Device error @ {} MHz, aborting", centerFreq / 1e6);
            running_ = false;
        } else {
            spdlog::warn("Read timeout @ {} MHz", centerFreq / 1e6);
        }
        return;
    }

    int samples = static_cast<int>(n_read / 2);
    samples     = std::min<int>(samples, static_cast<int>(bufferIQ_.size()));
    for (int i = 0; i < samples; ++i) {
        short I        = static_cast<short>(bufferU8_[2 * i]) - 127;
        short Q        = static_cast<short>(bufferU8_[2 * i + 1]) - 127;
        bufferIQ_[i]   = std::complex<short>(I, Q);
        bufferQ_[i]    = Q;
    }

    rtl::tools::removeDc(bufferIQ_.data(), samples);

    auto [fft_power_sum, groups_num] = fftEngine_.accumulatePower(bufferIQ_.data(), samples);

    if (groups_num <= 0) return;

    double rssi;
    if (centerFreq < rtl::constants::LOW_FREQ_THRESHOLD) {
        rssi = rtl::tools::calculateRssiDirectSampling(bufferQ_.data(), samples);
    } else {
        rssi = rtl::tools::calculateRssi(fft_power_sum, groups_num);
    }

    spdlog::info("Scan Freq: {} MHz, RSSI: {} dBFS", centerFreq / 1e6, rssi);
    std::vector<double> spectrum_db = rtl::tools::spectrumToDb(fft_power_sum, groups_num);
    rtl::tools::suppressDcSpike(spectrum_db);
    segments.push_back({std::move(spectrum_db), static_cast<double>(centerFreq)});
}

void ScanEngine::spliceAndPush(
    const std::vector<SegmentData>& segments, std::uint32_t sweepStartFreq, std::uint32_t sweepEndFreq) {
    auto [spliced_spectrum, spliced_freqs] = rtl::tools::spliceSpectrum(
        segments,
        static_cast<double>(rtl::constants::SCAN_SAMPLE_RATE),
        static_cast<double>(sweepStartFreq),
        static_cast<double>(sweepEndFreq));

    rtl::tools::suppressPeriodicSpurs(
        spliced_spectrum,
        static_cast<double>(sweepStartFreq),
        static_cast<double>(sweepEndFreq));

    double max_val = *std::max_element(spliced_spectrum.begin(), spliced_spectrum.end());
    double min_val = *std::min_element(spliced_spectrum.begin(), spliced_spectrum.end());

    nlohmann::json data_obj;
    data_obj["start_freq"] = sweepStartFreq;
    data_obj["end_freq"]   = sweepEndFreq;
    data_obj["max_value"]  = max_val;
    data_obj["min_value"]  = min_val;
    data_obj["data"]       = spliced_spectrum;

    nlohmann::json json_data;
    json_data["id"]    = 200;
    json_data["event"] = "scan_data";
    json_data["data"]  = data_obj;

    pusher_.pushScanHop(std::move(json_data));

    spdlog::info("Sweep complete, {} bins pushed", spliced_spectrum.size());
}

} // namespace rtl::scanner