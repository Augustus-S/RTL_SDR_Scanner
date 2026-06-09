#pragma once

#include <rtl-sdr.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

/**
 * @file persistent_async_reader.hpp
 * @brief Persistent worker thread for synchronous RTL-SDR reads with timeout-aware request tracking.
 */

namespace rtl::scanner {

/**
 * @brief Single-device read worker used by the spectrum scanner.
 *
 * @note The class does not own the rtlsdr_dev_t pointer. The caller must ensure
 * the device stays valid until shutdown() or destruction finishes.
 */
class PersistentAsyncReader {
public:
    /** @brief Read completion status. */
    enum class ReadResult {
        SUCCESS,
        TIMEOUT,
        DEVICE_ERROR,
        SHUTDOWN
    };

    /**
     * @brief Start a worker thread bound to an existing RTL-SDR device.
     * @param dev Raw librtlsdr device pointer. Ownership remains with caller.
     */
    explicit PersistentAsyncReader(rtlsdr_dev_t* dev);

    /** @brief Stop the worker thread. */
    ~PersistentAsyncReader();

    PersistentAsyncReader(const PersistentAsyncReader&)            = delete;
    PersistentAsyncReader& operator=(const PersistentAsyncReader&) = delete;

    /**
     * @brief Tune and read a block from the RTL-SDR device.
     * @param outBuf Caller-owned destination buffer.
     * @param outLen Input: capacity in bytes; output: bytes copied on success.
     * @param centerFreq Center frequency in Hz.
     * @param directSampling Direct sampling mode passed to librtlsdr.
     * @param timeoutMs Maximum wait time in milliseconds.
     * @return SUCCESS, TIMEOUT, DEVICE_ERROR or SHUTDOWN.
     * @note The request length must not exceed MAX_READ_BYTES.
     */
    ReadResult
        read(std::uint8_t* outBuf, std::uint32_t* outLen, std::uint32_t centerFreq, int directSampling, int timeoutMs);

    /**
     * @brief Stop the worker thread and discard pending commands.
     */
    void shutdown();

private:
    enum class Command {
        READ,
        SHUTDOWN
    };

    struct ReadRequest {
        Command                               cmd;
        std::uint64_t                         requestId      = 0;
        std::uint32_t                         centerFreq     = 0;
        int                                   directSampling = 0;
        std::chrono::steady_clock::time_point deadline;
        std::uint32_t                         expectedLen = 0;
    };

    void readerLoop();
    bool isDeviceAlive();

    rtlsdr_dev_t* dev_;

    std::thread             readerThread_;
    std::mutex              mtx_;
    std::condition_variable cmdCv_;
    std::condition_variable dataCv_;
    std::queue<ReadRequest> cmdQueue_;

    std::vector<std::uint8_t> internalBuf_;
    std::uint32_t             dataLen_            = 0;
    bool                      dataReady_          = false;
    ReadResult                dataResult_         = ReadResult::TIMEOUT;
    std::uint64_t             nextRequestId_      = 0;
    std::uint64_t             completedRequestId_ = 0;

    std::atomic<bool> running_{false};

    static constexpr int         STABILIZE_MS   = 20;
    static constexpr std::size_t MAX_READ_BYTES = 2 * 1024 * 1024;
};

} // namespace rtl::scanner
