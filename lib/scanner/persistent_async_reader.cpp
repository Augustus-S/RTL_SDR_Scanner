#include "scanner/persistent_async_reader.hpp"
#include <spdlog/spdlog.h>
#include <cstring>

namespace rtl::scanner {

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
    uint8_t* outBuf, uint32_t* outLen, uint32_t centerFreq, int directSampling, int timeoutMs) {
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
        dataReady_ = false;
        requestId  = ++nextRequestId_;
        cmdQueue_  = {};
        cmdQueue_.push({Command::READ, requestId, centerFreq, directSampling, deadline, *outLen});
    }
    cmdCv_.notify_one();

    std::unique_lock<std::mutex> lock(mtx_);
    if (!dataCv_.wait_until(lock, deadline, [this, requestId] {
            return (dataReady_ && completedRequestId_ == requestId) || !running_;
        })) {
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

            rtlsdr_set_center_freq(dev_, req.centerFreq);
            if (req.directSampling) {
                rtlsdr_set_direct_sampling(dev_, req.directSampling);
            } else {
                rtlsdr_set_direct_sampling(dev_, 0);
            }

            rtlsdr_reset_buffer(dev_);

            int nRead = 0;
            int ret   = rtlsdr_read_sync(dev_, internalBuf_.data(), req.expectedLen, &nRead);

            std::lock_guard<std::mutex> lock(mtx_);
            if (req.requestId == nextRequestId_) {
                if (ret < 0) {
                    dataResult_ = ReadResult::DEVICE_ERROR;
                    dataLen_    = 0;
                } else if (std::chrono::steady_clock::now() > req.deadline) {
                    dataResult_ = ReadResult::TIMEOUT;
                    dataLen_    = 0;
                } else {
                    dataResult_ = ReadResult::SUCCESS;
                    dataLen_    = static_cast<uint32_t>(nRead);
                }
                dataReady_          = true;
                completedRequestId_ = req.requestId;
                dataCv_.notify_all();
            }
        }
    }
}

bool PersistentAsyncReader::isDeviceAlive() {
    if (!dev_) return false;
    return rtlsdr_get_center_freq(dev_) > 0;
}

} // namespace rtl::scanner
