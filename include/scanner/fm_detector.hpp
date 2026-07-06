#pragma once

#include <cstdint>
#include <functional>
#include <vector>
#include "scanner/persistent_async_reader.hpp"

namespace rtl::scanner {

struct FmDetectionConfig {
    std::uint32_t fmBandMinHz                = 87500000;
    std::uint32_t fmBandMaxHz                = 108000000;
    double        minCandidateBwHz           = 80000.0;
    double        maxCandidateBwHz           = 350000.0;
    double        candidateThresholdDb       = 10.0;
    double        minSnrDb                   = 8.0;
    int           maxIqVerificationsPerSweep = 3;
    int           iqVerifyDurationMs         = 200;
    std::uint32_t verifyTuningOffsetHz       = 250000;
    double        pilotFreqHz                = 19000.0;
    double        rdsFreqHz                  = 57000.0;
    double        minPilotSnrDb              = 6.0;
    double        minAudioSnrDb              = 6.0;
    double        weakPilotSnrDb             = 4.0;
    double        strongSpectrumSnrDb        = 15.0;
};

struct FmCandidate {
    double startFreqHz = 0.0;
    double endFreqHz   = 0.0;
    double centerHz    = 0.0;
    double bandwidthHz = 0.0;
    double peakDb      = -200.0;
    double avgDb       = -200.0;
    double noiseDb     = -200.0;
    double snrDb       = 0.0;
    double confidence  = 0.0;
};

struct FmDetection {
    double startFreqHz = 0.0;
    double endFreqHz   = 0.0;
    double centerHz    = 0.0;
    double bandwidthHz = 0.0;
    double peakDb      = -200.0;
    double avgDb       = -200.0;
    double noiseDb     = -200.0;
    double snrDb       = 0.0;
    double confidence  = 0.0;
    bool   verified    = false;
    bool   stereo      = false;
    bool   rds         = false;
    double pilotSnrDb  = 0.0;
    double audioSnrDb  = 0.0;
    double deviationHz = 0.0;
    double paprDb      = 0.0;
    double amVariance  = 0.0;
    double fmRmsHz     = 0.0;
};

class FMDetector {
public:
    explicit FMDetector(FmDetectionConfig config = {});

    /**
     * @brief Find FM broadcast candidates in a sweep spectrum.
     * @param spectrum Spectrum power values in dBFS.
     * @param sweepStartHz Frequency represented by the first spectrum bin, in Hz.
     * @param sweepEndHz Frequency represented by the last spectrum bin, in Hz.
     */
    std::vector<FmCandidate>
        findCandidates(const std::vector<double>& spectrum, double sweepStartHz, double sweepEndHz) const;

    /**
     * @brief Re-tune and verify FM candidates using fresh IQ samples.
     * @param reader RTL-SDR reader used to collect verification IQ data.
     * @param candidates Candidate list from findCandidates().
     * @param sampleRateHz IQ sample rate in samples per second.
     * @param shouldContinue Optional cancellation predicate checked between candidates.
     */
    std::vector<FmDetection> verifyCandidates(
        PersistentAsyncReader&          reader,
        const std::vector<FmCandidate>& candidates,
        double                          sampleRateHz,
        const std::function<bool()>&    shouldContinue = {}) const;

    /**
     * @brief Detect and verify FM candidates using IQ from the current scan hop.
     * @param spectrum Hop spectrum power values in dBFS.
     * @param segmentStartHz Frequency represented by the first spectrum bin, in Hz.
     * @param segmentEndHz Frequency represented by the last spectrum bin, in Hz.
     * @param iq Interleaved unsigned 8-bit IQ samples from librtlsdr.
     * @param bytesRead Number of valid bytes in iq.
     * @param tunerCenterHz RTL-SDR center frequency for the IQ buffer, in Hz.
     * @param sampleRateHz IQ sample rate in samples per second.
     * @param maxIqVerifications Maximum candidates to verify in this hop.
     * @param verificationAttempts Optional output count of candidates attempted.
     */
    std::vector<FmDetection> detectInIqSegment(
        const std::vector<double>& spectrum,
        double                     segmentStartHz,
        double                     segmentEndHz,
        const std::uint8_t*        iq,
        std::uint32_t              bytesRead,
        double                     tunerCenterHz,
        double                     sampleRateHz,
        int                        maxIqVerifications,
        int*                       verificationAttempts = nullptr) const;

private:
    struct IqFeatures {
        bool   valid       = false;
        bool   stereo      = false;
        bool   rds         = false;
        double audioSnrDb  = 0.0;
        double pilotSnrDb  = 0.0;
        double rdsSnrDb    = 0.0;
        double deviationHz = 0.0;
        double paprDb      = 0.0;
        double amVariance  = 0.0;
        double fmRmsHz     = 0.0;
        double confidence  = 0.0;
    };

    IqFeatures
        analyzeIq(const std::uint8_t* iq, std::uint32_t bytesRead, double mixerOffsetHz, double sampleRateHz) const;

    FmDetectionConfig config_;
};

} // namespace rtl::scanner
