#pragma once

#include <cstdint>
#include <vector>

namespace rtl::scanner {

struct AmDetectionConfig {
    double minCandidateBwHz     = 4000.0;
    double maxCandidateBwHz     = 60000.0;
    double candidateThresholdDb = 8.0;
    double minSnrDb             = 10.0;
    double minAudioSnrDb        = 5.0;
    double minModulationDepth   = 0.015;
    double maxFmRmsHz           = 12000.0;
    double fmBroadcastMinHz     = 87500000.0;
    double fmBroadcastMaxHz     = 108000000.0;
};

struct AmCandidate {
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

struct AmDetection {
    double startFreqHz     = 0.0;
    double endFreqHz       = 0.0;
    double centerHz        = 0.0;
    double bandwidthHz     = 0.0;
    double peakDb          = -200.0;
    double avgDb           = -200.0;
    double noiseDb         = -200.0;
    double snrDb           = 0.0;
    double confidence      = 0.0;
    bool   verified        = false;
    double audioSnrDb      = 0.0;
    double modulationDepth = 0.0;
    double carrierSnrDb    = 0.0;
    double fmRmsHz         = 0.0;
};

class AMDetector {
public:
    explicit AMDetector(AmDetectionConfig config = {});

    std::vector<AmCandidate>
        findCandidates(const std::vector<double>& spectrum, double sweepStartHz, double sweepEndHz) const;

    std::vector<AmDetection> detectInIqSegment(
        const std::vector<double>& spectrum,
        double                     segmentStartHz,
        double                     segmentEndHz,
        const std::uint8_t*        iq,
        std::uint32_t              bytesRead,
        double                     tunerCenterHz,
        int                        maxIqVerifications,
        int*                       verificationAttempts = nullptr) const;

private:
    struct IqFeatures {
        bool   valid           = false;
        double audioSnrDb      = 0.0;
        double modulationDepth = 0.0;
        double carrierSnrDb    = 0.0;
        double fmRmsHz         = 0.0;
        double confidence      = 0.0;
    };

    IqFeatures analyzeIq(const std::uint8_t* iq, std::uint32_t bytesRead, double mixerOffsetHz) const;

    AmDetectionConfig config_;
};

} // namespace rtl::scanner
