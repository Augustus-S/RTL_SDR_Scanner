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

    /**
     * @brief Find AM candidates in a sweep spectrum.
     * @param spectrum Spectrum power values in dBFS.
     * @param sweepStartHz Frequency represented by the first spectrum bin, in Hz.
     * @param sweepEndHz Frequency represented by the last spectrum bin, in Hz.
     */
    std::vector<AmCandidate>
        findCandidates(const std::vector<double>& spectrum, double sweepStartHz, double sweepEndHz) const;

    /**
     * @brief Detect and verify AM candidates using IQ from the current scan hop.
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
    std::vector<AmDetection> detectInIqSegment(
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
        bool   valid           = false;
        double audioSnrDb      = 0.0;
        double modulationDepth = 0.0;
        double carrierSnrDb    = 0.0;
        double fmRmsHz         = 0.0;
        double confidence      = 0.0;
    };

    IqFeatures
        analyzeIq(const std::uint8_t* iq, std::uint32_t bytesRead, double mixerOffsetHz, double sampleRateHz) const;

    AmDetectionConfig config_;
};

} // namespace rtl::scanner
