#include "scanner/persistent_async_reader.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cstring>

/**
 * @file persistent_async_reader.cpp
 * @brief Timeout-aware RTL-SDR read worker used by scan sweeps.
 */

namespace rtl::scanner {

namespace {

struct AsyncReadContext {
    rtlsdr_dev_t*              dev         = nullptr;
    std::uint8_t*              buffer      = nullptr;
    std::uint32_t              capacity    = 0;
    std::uint32_t              bytesCopied = 0;
    std::atomic<bool>          complete{false};
};

// librtlsdr async callbacks run on the rtlsdr_read_async thread. The callback
// copies blocks into the request buffer until capacity is reached, then cancels
// the async loop so read() can return like a bounded blocking call.
void asyncReadCallback(unsigned char* buf, std::uint32_t len, void* ctx) {
    auto* state = static_cast<AsyncReadContext*>(ctx);
    if (!state || !buf || state->complete.load()) return;

    const auto remaining = state->capacity - state->bytesCopied;
    const auto copyLen   = std::min<std::uint32_t>(remaining, len);
    if (copyLen > 0) {
        std::memcpy(state->buffer + state->bytesCopied, buf, copyLen);
        state->bytesCopied += copyLen;
    }

    if (state->bytesCopied >= state->capacity) {
        state->complete.store(true);
        if (state->dev) rtlsdr_cancel_async(state->dev);
    }
}

} // namespace

PersistentAsyncReader::PersistentAsyncReader(rtlsdr_dev_t* dev)
    : dev_(dev) {
    running_      = true;
    readerThread_ = std::thread(&PersistentAsyncReader::readerLoop, this);
}

PersistentAsyncReader::~PersistentAsyncReader() {
    shutdown();
}

void PersistentAsyncReader::shutdown() {
    if (!running_.exchange(false)) return;
    // Cancels an in-flight async read; the command queue notification covers the
    // case where the worker is idle and waiting for the next read request.
    if (dev_) { rtlsdr_cancel_async(dev_); }

    {
        std::lock_guard<std::mutex> lock(mtx_);
        cmdQueue_ = {};
        cmdQueue_.push({Command::SHUTDOWN});
    }
    cmdCv_.notify_one();
    dataCv_.notify_all();

    if (readerThread_.joinable()) { readerThread_.join(); }
}

PersistentAsyncReader::ReadResult PersistentAsyncReader::read(
    uint8_t* outBuf,
    uint32_t* outLen,
    uint32_t centerFreq,
    int directSampling,
    int timeoutMs,
    int tuneSettleMs,
    ResetPolicy resetPolicy,
    bool forceReset) {
    if (!running_) return ReadResult::SHUTDOWN;
    if (!outBuf || !outLen || *outLen == 0) return ReadResult::DEVICE_ERROR;
    if (*outLen > MAX_READ_BYTES) {
        spdlog::error("Requested read length {} exceeds max {}", *outLen, MAX_READ_BYTES);
        return ReadResult::DEVICE_ERROR;
    }

    auto     deadline  = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    uint64_t requestId = 0;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        // Keep only the newest request. Scan sweeps are sequential, and stale
        // timed-out commands should not run after the caller has moved on.
        dataReady_ = false;
        requestId  = ++nextRequestId_;
        cmdQueue_  = {};
        cmdQueue_.push(
            {Command::READ, requestId, centerFreq, directSampling, tuneSettleMs, resetPolicy, forceReset, deadline, *outLen});
    }
    cmdCv_.notify_one();

    std::unique_lock<std::mutex> lock(mtx_);
    if (!dataCv_.wait_until(lock, deadline, [this, requestId] {
            return (dataReady_ && completedRequestId_ == requestId) || !running_;
        })) {
        lock.unlock();
        if (dev_) rtlsdr_cancel_async(dev_);
        return ReadResult::TIMEOUT;
    }

    if (!running_) return ReadResult::SHUTDOWN;

    if (dataResult_ == ReadResult::SUCCESS && dataLen_ > 0) {
        uint32_t copyLen = dataLen_;
        if (outBuf && outLen) {
            if (copyLen > *outLen) copyLen = *outLen;
            std::memcpy(outBuf, internalBuf_.data(), copyLen);
            *outLen = copyLen;
        }
    }

    return dataResult_;
}

void PersistentAsyncReader::readerLoop() {
    internalBuf_.resize(MAX_READ_BYTES);

    std::this_thread::sleep_for(std::chrono::milliseconds(STABILIZE_MS));

    while (running_) {
        ReadRequest req;
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cmdCv_.wait(lock, [this] {
                return !cmdQueue_.empty() || !running_;
            });

            if (!running_) break;

            req = cmdQueue_.front();
            cmdQueue_.pop();
        }

        if (req.cmd == Command::SHUTDOWN) break;

        if (req.cmd == Command::READ) {
            if (!dev_ || req.expectedLen > internalBuf_.size()) {
                std::lock_guard<std::mutex> lock(mtx_);
                if (req.requestId == nextRequestId_) {
                    dataResult_         = ReadResult::DEVICE_ERROR;
                    dataLen_            = 0;
                    dataReady_          = true;
                    completedRequestId_ = req.requestId;
                    dataCv_.notify_all();
                }
                continue;
            }

            // Direct sampling must be selected before tuning into the low-frequency
            // direct-sampling range; otherwise librtlsdr may reject the frequency.
            bool directSamplingChanged = req.directSampling != currentDirectSampling_;
            if (directSamplingChanged) {
                int dsRet = rtlsdr_set_direct_sampling(dev_, req.directSampling);
                if (dsRet < 0) {
                    std::lock_guard<std::mutex> lock(mtx_);
                    if (req.requestId == nextRequestId_) {
                        spdlog::error("Failed to set direct sampling mode {}: error {}", req.directSampling, dsRet);
                        dataResult_         = ReadResult::DEVICE_ERROR;
                        dataLen_            = 0;
                        dataReady_          = true;
                        completedRequestId_ = req.requestId;
                        dataCv_.notify_all();
                    }
                    continue;
                }
                currentDirectSampling_ = req.directSampling;
            }

            int tuneRet = rtlsdr_set_center_freq(dev_, req.centerFreq);
            if (tuneRet < 0) {
                std::lock_guard<std::mutex> lock(mtx_);
                if (req.requestId == nextRequestId_) {
                    spdlog::error("Failed to set center freq {}: error {}", req.centerFreq, tuneRet);
                    dataResult_         = ReadResult::DEVICE_ERROR;
                    dataLen_            = 0;
                    dataReady_          = true;
                    completedRequestId_ = req.requestId;
                    dataCv_.notify_all();
                }
                continue;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(req.tuneSettleMs, 0)));
            if (!running_) break;
            if (std::chrono::steady_clock::now() > req.deadline) {
                std::lock_guard<std::mutex> lock(mtx_);
                if (req.requestId == nextRequestId_) {
                    dataResult_         = ReadResult::TIMEOUT;
                    dataLen_            = 0;
                    dataReady_          = true;
                    completedRequestId_ = req.requestId;
                    dataCv_.notify_all();
                }
                continue;
            }

            // Adaptive reset avoids flushing on every hop, but still resets after
            // errors, forced boundaries, or direct-sampling mode changes.
            const bool shouldReset =
                req.resetPolicy == ResetPolicy::ALWAYS || req.forceReset || directSamplingChanged || resetAfterError_;
            if (shouldReset) {
                int resetRet = rtlsdr_reset_buffer(dev_);
                if (resetRet < 0) {
                    resetAfterError_ = true;
                    std::lock_guard<std::mutex> lock(mtx_);
                    if (req.requestId == nextRequestId_) {
                        spdlog::error("Failed to reset RTL-SDR buffer: error {}", resetRet);
                        dataResult_         = ReadResult::DEVICE_ERROR;
                        dataLen_            = 0;
                        dataReady_          = true;
                        completedRequestId_ = req.requestId;
                        dataCv_.notify_all();
                    }
                    continue;
                }
            }
            if (!running_) break;
            if (std::chrono::steady_clock::now() > req.deadline) {
                std::lock_guard<std::mutex> lock(mtx_);
                if (req.requestId == nextRequestId_) {
                    dataResult_         = ReadResult::TIMEOUT;
                    dataLen_            = 0;
                    dataReady_          = true;
                    completedRequestId_ = req.requestId;
                    dataCv_.notify_all();
                }
                continue;
            }

            AsyncReadContext asyncState;
            asyncState.dev      = dev_;
            asyncState.buffer   = internalBuf_.data();
            asyncState.capacity = req.expectedLen;

            int ret = rtlsdr_read_async(dev_, asyncReadCallback, &asyncState, 0, req.expectedLen);

            std::lock_guard<std::mutex> lock(mtx_);
            if (req.requestId == nextRequestId_) {
                if (!running_) {
                    dataResult_ = ReadResult::SHUTDOWN;
                    dataLen_    = 0;
                } else if (ret < 0 && !asyncState.complete.load()) {
                    dataResult_ = ReadResult::DEVICE_ERROR;
                    dataLen_    = 0;
                    resetAfterError_ = true;
                } else if (!asyncState.complete.load() || std::chrono::steady_clock::now() > req.deadline) {
                    dataResult_ = ReadResult::TIMEOUT;
                    dataLen_    = 0;
                    resetAfterError_ = true;
                } else {
                    dataResult_ = ReadResult::SUCCESS;
                    dataLen_    = asyncState.bytesCopied;
                    resetAfterError_ = false;
                }
                dataReady_          = true;
                completedRequestId_ = req.requestId;
                dataCv_.notify_all();
            }
        }
    }
}

} // namespace rtl::scanner
