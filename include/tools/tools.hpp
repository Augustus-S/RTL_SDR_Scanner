#pragma once

#include <cctype>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "httplib.h"
#include "scanner/types.hpp"

/**
 * @file tools.hpp
 * @brief Utility functions for logging, HTTP, spectrum processing and device reset helpers.
 */

namespace rtl::tools {

/**
 * @brief Spectrum peak metadata returned by detectPeaks().
 */
struct PeakInfo {
    double cf;       /**< Center frequency in Hz. */
    double bw;       /**< Bandwidth in Hz. */
    double avgDb;    /**< Average power in dBFS. */
    double maxDb;    /**< Peak power in dBFS. */
    double level;    /**< Relative level in range [0, 100]. */
    int    posStart; /**< Start bin index. */
    int    posStop;  /**< Stop bin index. */
};

/**
 * @brief Build a frequency sweep plan.
 * @param startFreq Start frequency in Hz.
 * @param endFreq End frequency in Hz.
 * @param rate Sample rate in samples per second.
 * @return Scanner plan containing hop centers and bin metadata.
 * @note This function performs planning only; it does not access the SDR device.
 */
rtl::scanner::ScanPlan buildScanPlan(double startFreq, double endFreq, double rate);

/**
 * @brief Remove DC offset from complex IQ samples in place.
 * @param buf Pointer to complex int16 IQ samples.
 * @param bufLen Number of complex samples.
 * @note Passing nullptr or zero length is a no-op.
 */
void removeDc(std::complex<short>* buf, std::size_t bufLen);



/**
 * @brief Splice per-hop spectra into one range spectrum.
 * @param segments Per-hop spectrum segments.
 * @param sampleRate Sample rate in samples per second.
 * @param startFreq Display start frequency in Hz.
 * @param endFreq Display end frequency in Hz.
 * @return Pair of power values in dBFS and corresponding frequencies in MHz.
 */
std::pair<std::vector<double>, std::vector<double>> spliceSpectrum(
    const std::vector<rtl::scanner::SegmentData>& segments, double sampleRate, double startFreq, double endFreq);

/**
 * @brief Convert accumulated linear FFT power to dBFS.
 * @param fftPowerSum Accumulated linear FFT power.
 * @param groupsNum Number of FFT groups used for accumulation.
 * @return dBFS spectrum vector.
 */
std::vector<double> spectrumToDb(const std::vector<double>& fftPowerSum, int groupsNum);

/** @brief Suppress DC-bin spikes in a spectrum vector. */
void suppressDcSpike(std::vector<double>& fftPower, int width = 10);

/** @brief Suppress symmetric IQ image artifacts in a linear power spectrum. */
void suppressSymmetricSpur(std::vector<double>& fftPower);

/** @brief Suppress image spurs in a linear power spectrum. */
void suppressImageSpur(std::vector<double>& fftPower, double thresholdDb = 4.0, double irrDb = 30.0);

/**
 * @brief Suppress periodic narrow spurs in a spliced dBFS spectrum.
 * @param spectrum Spectrum values in dBFS, modified in place.
 * @param startFreq Range start frequency in Hz.
 * @param endFreq Range end frequency in Hz.
 * @param spikeIntervalHz Expected spur interval in Hz.
 * @param width Number of bins around each detected spur to replace.
 * @param thresholdDb Threshold above local median in dB.
 */
void suppressPeriodicSpurs(
    std::vector<double>& spectrum,
    double               startFreq,
    double               endFreq,
    double               spikeIntervalHz = 1e6,
    int                  width           = 3,
    double               thresholdDb     = 3.0);

/**
 * @brief Detect continuous signal peaks above a threshold.
 * @param spectrum Spectrum values in dBFS.
 * @param freqStart Frequency of bin zero in Hz.
 * @param freqPerBin Frequency resolution in Hz per bin.
 * @param thresholdDb Minimum dBFS value for a bin to be considered active.
 * @return List of detected peak ranges.
 */
std::vector<PeakInfo>
    detectPeaks(const std::vector<double>& spectrum, double freqStart, double freqPerBin, double thresholdDb = -80.0);

/** @brief Calculate RSSI from accumulated FFT power. */
double calculateRssi(const std::vector<double>& fftPowerSum, int groupsNum);

/** @brief Calculate RSSI directly from one direct-sampling channel. */
double calculateRssiDirectSampling(const short* chData, std::size_t chLen);

/**
 * @brief Initialize the default spdlog logger.
 * @note Creates ./logs if needed and configures console plus rotating file sinks.
 */
inline void initSpdlog() {
    namespace fs = std::filesystem;

    auto logDir = fs::path("logs");
    if (!fs::exists(logDir)) { fs::create_directories(logDir); }

    auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    consoleSink->set_level(spdlog::level::info);

    auto fileSink =
        std::make_shared<spdlog::sinks::rotating_file_sink_mt>((logDir / "RTL_SDR.log").string(), 1024 * 1024 * 100, 3);

    std::vector<spdlog::sink_ptr> sinks{consoleSink, fileSink};
    auto                          logger = std::make_shared<spdlog::logger>("RTL_SDR", sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::info);
    spdlog::set_default_logger(logger);
    spdlog::flush_every(std::chrono::seconds(3));
}

/**
 * @brief Get the current Unix timestamp.
 * @param type Unit selector: "ms", "s", or "us". Unknown values return milliseconds.
 * @return Timestamp in the requested unit.
 */
inline std::int64_t getTimestamp(const std::string& type) {
    if (type == "ms") {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }
    if (type == "s") {
        return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();
    }
    if (type == "us") {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

namespace detail {

inline std::pair<std::string, std::string> formatUrl(const std::string& url) {
    std::regex  pattern("^(https?://)?([^/]+)(/.*)");
    std::smatch match;

    if (std::regex_search(url, match, pattern) && match.size() == 4) {
        std::string protocol = match[1].str();
        std::string domain   = match[2].str();
        std::string pathPart = match[3].str();
        if (protocol.empty()) protocol = "http://";
        return {protocol + domain, pathPart};
    }
    return {};
}

inline std::string readFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return "";
    std::string value;
    std::getline(file, value);
    return value;
}

inline bool writeSysfs(const std::string& file, const std::string& value) {
    std::ofstream ofs(file);
    if (!ofs.is_open()) {
        spdlog::error("无法打开: {}", file);
        return false;
    }
    ofs << value;
    return true;
}

inline std::string findUsbDevice(const std::string& vid, const std::string& pid) {
    namespace fs = std::filesystem;

    const std::string base = "/sys/bus/usb/devices";
    if (!fs::exists(base)) return "";

    for (const auto& entry : fs::directory_iterator(base)) {
        std::string name = entry.path().filename().string();
        if (name == "." || name == "..") continue;
        if (name.find(":") != std::string::npos) continue;

        std::string vendorFile  = (entry.path() / "idVendor").string();
        std::string productFile = (entry.path() / "idProduct").string();
        if (fs::exists(vendorFile) && fs::exists(productFile)) {
            std::string v = readFile(vendorFile);
            std::string p = readFile(productFile);
            if (v == vid && p == pid) return name;
        }
    }
    return "";
}

inline bool resetUsbDevice(const std::string& device) {
    const std::string unbind = "/sys/bus/usb/drivers/usb/unbind";
    const std::string bind   = "/sys/bus/usb/drivers/usb/bind";

    spdlog::info("执行 USB unbind...");
    if (!writeSysfs(unbind, device)) return false;

    std::this_thread::sleep_for(std::chrono::seconds(2));

    spdlog::info("执行 USB bind...");
    if (!writeSysfs(bind, device)) return false;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    spdlog::info("USB 重置完成");
    return true;
}

} // namespace detail

/**
 * @brief Send an HTTP POST request.
 * @param url Full URL including path.
 * @param data JSON or text body.
 * @param timeout Timeout in seconds.
 * @return Response body on HTTP 200/201, otherwise an empty string.
 */
inline std::string post(std::string url, const std::string& data, double timeout) {
    auto urlAndPath = detail::formatUrl(url);
    if (urlAndPath.second.empty()) {
        spdlog::warn("{} url不合法", url);
        return "";
    }

    url                      = urlAndPath.first;
    std::string      path    = urlAndPath.second;
    httplib::Headers headers = {
        {"Authorization", "Bearer token pass"}
    };
    httplib::Client cli(url);

    cli.set_connection_timeout(int(timeout), 1e6 * (timeout - int(timeout)));
    cli.set_keep_alive(false);
    cli.set_read_timeout(int(timeout), 1e6 * (timeout - int(timeout)));
    cli.set_write_timeout(int(timeout), 1e6 * (timeout - int(timeout)));

    try {
        auto res = cli.Post(path, headers, data.c_str(), "application/json");
        if (!res) {
            spdlog::warn("post {}{} 超时", url, path);
            return "";
        }
        if (res->status == 200 || res->status == 201) return res->body;
        spdlog::warn("{}{} status:{} HTTP post error", url, path, res->status);
    } catch (std::exception& e) { spdlog::warn("{}", e.what()); }

    spdlog::warn("post err");
    return "";
}

/**
 * @brief Send an HTTP GET request.
 * @param url Full URL including path.
 * @param timeout Timeout in seconds.
 * @return Response body on HTTP 200/201, otherwise an empty string.
 */
inline std::string get(const std::string& url, double timeout) {
    auto urlAndPath = detail::formatUrl(url);
    if (urlAndPath.second.empty()) {
        spdlog::info("{} url不合法", url);
        return "";
    }

    std::string      urlBase = urlAndPath.first;
    std::string      path    = urlAndPath.second;
    httplib::Headers headers = {
        {"Authorization", "Bearer token pass"}
    };
    httplib::Client cli(urlBase);

    cli.set_connection_timeout(int(timeout), 1e6 * (timeout - int(timeout)));
    cli.set_keep_alive(false);
    cli.set_read_timeout(int(timeout), 1e6 * (timeout - int(timeout)));
    cli.set_write_timeout(int(timeout), 1e6 * (timeout - int(timeout)));

    try {
        auto res = cli.Get(path, headers);
        if (!res) {
            spdlog::info("get {}{} 超时", urlBase, path);
            return "";
        }
        if (res->status == 200 || res->status == 201) return res->body;
        spdlog::info("{}{} status:{} HTTP get error", urlBase, path, res->status);
    } catch (std::exception& e) { spdlog::info("{}", e.what()); }

    spdlog::info("get err");
    return "";
}

/**
 * @brief Reset an RTL2838 USB device through Linux sysfs unbind/bind.
 * @return true when unbind and bind operations both succeed.
 * @note Requires sufficient permission to write /sys/bus/usb/drivers/usb.
 */
inline bool resetRtl2838() {
    const std::string vendorId   = "0bda";
    const std::string productId  = "2838";
    std::string       devicePath = detail::findUsbDevice(vendorId, productId);
    if (devicePath.empty()) {
        spdlog::error("未找到 RTL2838 USB 设备");
        return false;
    }
    spdlog::info("找到 RTL2838 设备: {}", devicePath);
    return detail::resetUsbDevice(devicePath);
}

/**
 * @brief Trim leading and trailing ASCII whitespace.
 * @param s Input string.
 * @return Trimmed string.
 */
inline std::string trim(const std::string& s) {
    std::size_t start = 0;
    std::size_t end   = s.size();
    while (start < end && std::isspace(static_cast<unsigned char>(s[start]))) ++start;
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) --end;
    return s.substr(start, end - start);
}

/**
 * @brief Execute a shell command and capture stdout.
 * @param cmd Command line passed to popen().
 * @return Captured stdout without the final trailing newline.
 * @warning Do not pass untrusted user input; this function invokes the shell.
 */
inline std::string getCmdResult(const std::string& cmd) {
    char  buf[10240] = {0};
    FILE* pf         = nullptr;
    if ((pf = popen(cmd.c_str(), "r")) == nullptr) return "";
    std::string result;
    while (fgets(buf, sizeof buf, pf)) result += buf;
    pclose(pf);
    if (!result.empty() && result.back() == '\n') result.pop_back();
    return result;
}

} // namespace rtl::tools
