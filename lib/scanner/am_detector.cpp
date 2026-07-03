#include "scanner/am_detector.hpp"
#include "constants.hpp"
#include <algorithm>
#include <cmath>
#include <fftw3.h>

namespace rtl::scanner {

namespace {

double clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

double median(std::vector<double> values) {
    if (values.empty()) return -200.0;
    const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
}

double meanPowerDb(const std::vector<double>& values, int first, int last) {
    if (values.empty() || first > last) return -200.0;
    first = std::max(first, 0);
    last  = std::min(last, static_cast<int>(values.size()) - 1);
    if (first > last) return -200.0;

    double sum = 0.0;
    int    n   = 0;
    for (int i = first; i <= last; ++i) {
        sum += std::pow(10.0, values[static_cast<std::size_t>(i)] / 10.0);
        ++n;
    }
    if (n == 0) return -200.0;
    return 10.0 * std::log10(std::max(sum / n, 1e-20));
}

double bandPowerDb(const std::vector<double>& spectrum, double sampleRate, double loHz, double hiHz) {
    if (spectrum.empty() || hiHz <= loHz) return -200.0;
    const double binHz = sampleRate / static_cast<double>(spectrum.size() * 2);
    int          first = static_cast<int>(std::ceil(loHz / binHz));
    int          last  = static_cast<int>(std::floor(hiHz / binHz));
    return meanPowerDb(spectrum, first, last);
}

std::vector<double> makeHannSpectrum(const std::vector<double>& samples, int fftSize) {
    struct FftCache {
        int                 fftSize = 0;
        std::vector<double> window;
        double              windowPower = 0.0;
        double*             in          = nullptr;
        fftw_complex*       out         = nullptr;
        fftw_plan           plan        = nullptr;

        ~FftCache() {
            if (plan) fftw_destroy_plan(plan);
            fftw_free(in);
            fftw_free(out);
        }

        bool ensure(int size) {
            if (fftSize == size && plan && in && out) return true;
            if (plan) fftw_destroy_plan(plan);
            fftw_free(in);
            fftw_free(out);
            plan        = nullptr;
            in          = nullptr;
            out         = nullptr;
            fftSize     = size;
            windowPower = 0.0;
            window.resize(static_cast<std::size_t>(fftSize));
            for (int i = 0; i < fftSize; ++i) {
                window[static_cast<std::size_t>(i)] = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (fftSize - 1)));
                windowPower += window[static_cast<std::size_t>(i)] * window[static_cast<std::size_t>(i)];
            }
            in  = static_cast<double*>(fftw_malloc(sizeof(double) * fftSize));
            out = static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * (fftSize / 2 + 1)));
            if (!in || !out) return false;
            plan = fftw_plan_dft_r2c_1d(fftSize, in, out, FFTW_ESTIMATE);
            return plan != nullptr;
        }
    };

    std::vector<double> spectrum(static_cast<std::size_t>(fftSize / 2), -200.0);
    if (static_cast<int>(samples.size()) < fftSize) return spectrum;

    thread_local FftCache cache;
    if (!cache.ensure(fftSize)) return spectrum;

    const int           blocks = std::min(32, static_cast<int>(samples.size()) / fftSize);
    std::vector<double> power(static_cast<std::size_t>(fftSize / 2), 0.0);
    for (int b = 0; b < blocks; ++b) {
        for (int n = 0; n < fftSize; ++n) {
            cache.in[n] =
                samples[static_cast<std::size_t>(b * fftSize + n)] * cache.window[static_cast<std::size_t>(n)];
        }
        fftw_execute(cache.plan);
        for (int k = 0; k < fftSize / 2; ++k) {
            const double re                     = cache.out[k][0];
            const double im                     = cache.out[k][1];
            power[static_cast<std::size_t>(k)] += re * re + im * im;
        }
    }

    const double norm = std::max(cache.windowPower * blocks, 1e-12);
    for (int k = 0; k < fftSize / 2; ++k) {
        spectrum[static_cast<std::size_t>(k)] =
            10.0 * std::log10(std::max(power[static_cast<std::size_t>(k)] / norm, 1e-20));
    }

    return spectrum;
}

} // namespace

AMDetector::AMDetector(AmDetectionConfig config)
    : config_(config) {}

std::vector<AmCandidate>
    AMDetector::findCandidates(const std::vector<double>& spectrum, double sweepStartHz, double sweepEndHz) const {
    std::vector<AmCandidate> candidates;
    const int                n = static_cast<int>(spectrum.size());
    if (n < 8 || sweepEndHz <= sweepStartHz) return candidates;

    const double binHz = (sweepEndHz - sweepStartHz) / static_cast<double>(n - 1);
    if (binHz <= 0.0) return candidates;

    std::vector<double> smoothed(spectrum);
    for (int i = 1; i < n - 1; ++i) {
        smoothed[static_cast<std::size_t>(i)] =
            (spectrum[static_cast<std::size_t>(i - 1)] + 2.0 * spectrum[static_cast<std::size_t>(i)]
             + spectrum[static_cast<std::size_t>(i + 1)])
            / 4.0;
    }

    std::vector<double> finiteValues;
    finiteValues.reserve(spectrum.size());
    for (double value : smoothed) {
        if (std::isfinite(value) && value > -180.0) finiteValues.push_back(value);
    }
    const double globalNoise = median(finiteValues);

    auto localNoise = [&](int bin) {
        const int           radius = std::max(20, static_cast<int>(std::round(300000.0 / binHz)));
        const int           guard  = std::max(2, static_cast<int>(std::round(40000.0 / binHz)));
        std::vector<double> values;
        for (int i = bin - radius; i <= bin + radius; ++i) {
            if (i < 0 || i >= n || std::abs(i - bin) <= guard) continue;
            const double value = smoothed[static_cast<std::size_t>(i)];
            if (std::isfinite(value) && value > -180.0) values.push_back(value);
        }
        return values.empty() ? globalNoise : median(values);
    };

    std::vector<char> active(static_cast<std::size_t>(n), 0);
    for (int i = 0; i < n; ++i) {
        const double freqHz = sweepStartHz + i * binHz;
        if (freqHz >= config_.fmBroadcastMinHz && freqHz <= config_.fmBroadcastMaxHz) continue;
        const double noise = localNoise(i);
        if (smoothed[static_cast<std::size_t>(i)] >= noise + config_.candidateThresholdDb) {
            active[static_cast<std::size_t>(i)] = 1;
        }
    }

    const int mergeGapBins = std::max(1, static_cast<int>(std::round(8000.0 / binHz)));
    int       i            = 0;
    while (i < n) {
        while (i < n && !active[static_cast<std::size_t>(i)]) ++i;
        if (i >= n) break;

        int start = i;
        int stop  = i;
        int gap   = 0;
        ++i;
        while (i < n) {
            if (active[static_cast<std::size_t>(i)]) {
                stop = i;
                gap  = 0;
            } else if (++gap > mergeGapBins) {
                break;
            }
            ++i;
        }

        const double bandwidthHz = (stop - start + 1) * binHz;
        if (bandwidthHz < config_.minCandidateBwHz || bandwidthHz > config_.maxCandidateBwHz) continue;

        double peak     = -200.0;
        double weighted = 0.0;
        double weight   = 0.0;
        double sum      = 0.0;
        for (int j = start; j <= stop; ++j) {
            const double value = spectrum[static_cast<std::size_t>(j)];
            peak               = std::max(peak, value);
            sum               += value;
            const double p     = std::pow(10.0, value / 10.0);
            weighted          += (sweepStartHz + j * binHz) * p;
            weight            += p;
        }

        const double noise   = localNoise((start + stop) / 2);
        const double snr     = peak - noise;
        const double center  = weight > 0.0 ? weighted / weight : sweepStartHz + ((start + stop) * 0.5) * binHz;
        const double startHz = sweepStartHz + start * binHz;
        const double endHz   = sweepStartHz + stop * binHz;
        if (snr < config_.minSnrDb) continue;

        AmCandidate candidate;
        candidate.startFreqHz = startHz;
        candidate.endFreqHz   = endHz;
        candidate.centerHz    = center;
        candidate.bandwidthHz = bandwidthHz;
        candidate.peakDb      = peak;
        candidate.avgDb       = sum / static_cast<double>(stop - start + 1);
        candidate.noiseDb     = noise;
        candidate.snrDb       = snr;
        const double bwScore  = 1.0 - std::min(std::abs(bandwidthHz - 12000.0) / 50000.0, 1.0);
        const double snrScore = clamp01((snr - config_.minSnrDb) / 20.0);
        candidate.confidence  = clamp01(0.25 + 0.35 * bwScore + 0.40 * snrScore);
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(), [](const AmCandidate& a, const AmCandidate& b) {
        if (a.confidence == b.confidence) return a.snrDb > b.snrDb;
        return a.confidence > b.confidence;
    });
    return candidates;
}

std::vector<AmDetection> AMDetector::detectInIqSegment(
    const std::vector<double>& spectrum,
    double                     segmentStartHz,
    double                     segmentEndHz,
    const std::uint8_t*        iq,
    std::uint32_t              bytesRead,
    double                     tunerCenterHz,
    int                        maxIqVerifications,
    int*                       verificationAttempts) const {
    std::vector<AmDetection> detections;
    if (verificationAttempts) *verificationAttempts = 0;
    if (!iq || bytesRead < 4096 || maxIqVerifications <= 0) return detections;

    auto      candidates = findCandidates(spectrum, segmentStartHz, segmentEndHz);
    const int limit      = std::min<int>(maxIqVerifications, static_cast<int>(candidates.size()));
    if (verificationAttempts) *verificationAttempts = limit;
    for (int i = 0; i < limit; ++i) {
        const auto& candidate     = candidates[static_cast<std::size_t>(i)];
        const auto  mixerOffsetHz = tunerCenterHz - candidate.centerHz;
        auto        features      = analyzeIq(iq, bytesRead, mixerOffsetHz);
        if (!features.valid) continue;

        AmDetection detection;
        detection.startFreqHz     = candidate.startFreqHz;
        detection.endFreqHz       = candidate.endFreqHz;
        detection.centerHz        = candidate.centerHz;
        detection.bandwidthHz     = candidate.bandwidthHz;
        detection.peakDb          = candidate.peakDb;
        detection.avgDb           = candidate.avgDb;
        detection.noiseDb         = candidate.noiseDb;
        detection.snrDb           = candidate.snrDb;
        detection.confidence      = clamp01(0.50 * candidate.confidence + 0.50 * features.confidence);
        detection.verified        = true;
        detection.audioSnrDb      = features.audioSnrDb;
        detection.modulationDepth = features.modulationDepth;
        detection.carrierSnrDb    = features.carrierSnrDb;
        detection.fmRmsHz         = features.fmRmsHz;
        detections.push_back(detection);
    }
    return detections;
}

AMDetector::IqFeatures
    AMDetector::analyzeIq(const std::uint8_t* iq, std::uint32_t bytesRead, double mixerOffsetHz) const {
    IqFeatures features;
    if (!iq || bytesRead < 4096) return features;

    const double mixerPhaseInc = 2.0 * M_PI * mixerOffsetHz / static_cast<double>(rtl::constants::SCAN_SAMPLE_RATE);
    double       oscI          = 1.0;
    double       oscQ          = 0.0;
    const double stepI         = std::cos(mixerPhaseInc);
    const double stepQ         = std::sin(mixerPhaseInc);

    std::vector<double> envelope;
    envelope.reserve(bytesRead / 2);
    std::vector<double> carrierBaseband;
    carrierBaseband.reserve(bytesRead / 2);

    bool   havePrev = false;
    double prevI    = 0.0;
    double prevQ    = 0.0;
    double fmSumSq  = 0.0;
    int    fmCount  = 0;
    double ampSum   = 0.0;
    double ampSqSum = 0.0;

    for (std::uint32_t i = 0; i + 1 < bytesRead; i += 2) {
        const double rawI = (static_cast<double>(iq[static_cast<std::size_t>(i)]) - 127.5) / 127.5;
        const double rawQ = (static_cast<double>(iq[static_cast<std::size_t>(i + 1)]) - 127.5) / 127.5;
        const double curI = rawI * oscI - rawQ * oscQ;
        const double curQ = rawI * oscQ + rawQ * oscI;
        const double amp2 = curI * curI + curQ * curQ;
        const double amp  = std::sqrt(std::max(amp2, 0.0));

        envelope.push_back(amp);
        carrierBaseband.push_back(curI);
        ampSum   += amp;
        ampSqSum += amp2;

        const double nextOscI = oscI * stepI - oscQ * stepQ;
        const double nextOscQ = oscI * stepQ + oscQ * stepI;
        oscI                  = nextOscI;
        oscQ                  = nextOscQ;

        if (havePrev) {
            const double imag  = curQ * prevI - curI * prevQ;
            const double disc  = imag / (amp2 + 1e-12);
            fmSumSq           += disc * disc;
            ++fmCount;
        }
        prevI    = curI;
        prevQ    = curQ;
        havePrev = true;
    }

    if (envelope.size() < 4096 || ampSum <= 0.0) return features;

    const double meanAmp = ampSum / static_cast<double>(envelope.size());
    for (double& sample : envelope) { sample -= meanAmp; }

    const double meanAmp2 = ampSqSum / static_cast<double>(envelope.size());
    double       envSqSum = 0.0;
    for (double sample : envelope) { envSqSum += sample * sample; }
    const double envRms             = std::sqrt(envSqSum / static_cast<double>(envelope.size()));
    features.modulationDepth        = envRms / std::max(meanAmp, 1e-12);
    features.fmRmsHz                = fmCount > 0
                           ? std::sqrt(fmSumSq / static_cast<double>(fmCount))
                                 * static_cast<double>(rtl::constants::SCAN_SAMPLE_RATE) / (2.0 * M_PI)
                           : 0.0;

    constexpr int fftSize     = 2048;
    auto          envSpectrum = makeHannSpectrum(envelope, fftSize);
    auto          carSpectrum = makeHannSpectrum(carrierBaseband, fftSize);
    const double  audioDb     = bandPowerDb(envSpectrum, rtl::constants::SCAN_SAMPLE_RATE, 300.0, 8000.0);
    const double  refDb       = bandPowerDb(envSpectrum, rtl::constants::SCAN_SAMPLE_RATE, 40000.0, 100000.0);
    const double  carrierDb   = bandPowerDb(carSpectrum, rtl::constants::SCAN_SAMPLE_RATE, 0.0, 1500.0);
    const double  carrierRef  = bandPowerDb(carSpectrum, rtl::constants::SCAN_SAMPLE_RATE, 20000.0, 80000.0);
    features.audioSnrDb       = audioDb - refDb;
    features.carrierSnrDb     = carrierDb - carrierRef;

    const bool audioOk = features.audioSnrDb >= config_.minAudioSnrDb;
    const bool depthOk = features.modulationDepth >= config_.minModulationDepth;
    const bool fmOk    = features.fmRmsHz <= config_.maxFmRmsHz;
    if (!audioOk || !depthOk || !fmOk || meanAmp2 <= 0.0) return features;

    const double audioScore   = clamp01((features.audioSnrDb - config_.minAudioSnrDb) / 18.0);
    const double depthScore   = clamp01((features.modulationDepth - config_.minModulationDepth) / 0.20);
    const double carrierScore = clamp01((features.carrierSnrDb - 6.0) / 20.0);
    const double fmPenalty    = clamp01(features.fmRmsHz / config_.maxFmRmsHz) * 0.18;
    features.confidence       = clamp01(0.30 + 0.28 * audioScore + 0.24 * depthScore + 0.18 * carrierScore - fmPenalty);
    features.valid            = true;
    return features;
}

} // namespace rtl::scanner
