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

    std::vector<FmCandidate>
        findCandidates(const std::vector<double>& spectrum, double sweepStartHz, double sweepEndHz) const;

    std::vector<FmDetection> verifyCandidates(
        PersistentAsyncReader&          reader,
        const std::vector<FmCandidate>& candidates,
        const std::function<bool()>&    shouldContinue = {}) const;

    std::vector<FmDetection> detectInIqSegment(
        const std::vector<double>& spectrum,
        double                     segmentStartHz,
        double                     segmentEndHz,
        const std::uint8_t*        iq,
        std::uint32_t              bytesRead,
        double                     tunerCenterHz) const;

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

    IqFeatures analyzeIq(const std::uint8_t* iq, std::uint32_t bytesRead, double mixerOffsetHz) const;

    FmDetectionConfig config_;
};

} // namespace rtl::scanner
