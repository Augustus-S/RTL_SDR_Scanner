#include "tools/tools.hpp"
#include "constants.hpp"

#include <spdlog/spdlog.h>
#include <numeric>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <fftw3.h>

namespace rtl::tools {

rtl::scanner::ScanPlan buildScanPlan(double startFreq, double endFreq, double rate) {
    rtl::scanner::ScanPlan plan;
    plan.startFreq  = startFreq;
    plan.endFreq    = endFreq;
    plan.rate       = rate;
    plan.freqPerBin = rate / rtl::constants::FFT_SIZE;

    constexpr double OVERLAP_RATIO = 0.25;
    plan.overlapBins               = static_cast<int>(rtl::constants::FFT_SIZE * OVERLAP_RATIO);
    plan.usablePerHop              = rtl::constants::FFT_SIZE - plan.overlapBins;

    double step_freq = rate * (1.0 - OVERLAP_RATIO);
    double span      = endFreq - startFreq;

    plan.numSteps = static_cast<int>(std::ceil(span / step_freq)) + 1;

    plan.centerFreqs.resize(plan.numSteps);
    for (int i = 0; i < plan.numSteps; ++i) {
        plan.centerFreqs[i] = startFreq + rate / 2.0 + static_cast<double>(i) * step_freq;
    }

    int total_bins   = (plan.numSteps - 1) * plan.usablePerHop + rtl::constants::FFT_SIZE;
    int max_bins     = static_cast<int>(span / plan.freqPerBin);
    plan.displayBins = std::min(total_bins, max_bins);

    return plan;
}

std::pair<std::vector<double>, std::vector<double>> spliceSpectrum(
    const std::vector<rtl::scanner::SegmentData>& segments, double sampleRate, double startFreq, double endFreq) {
    const double bin_width  = sampleRate / rtl::constants::FFT_SIZE;
    const int    total_bins = static_cast<int>(std::round((endFreq - startFreq) / bin_width)) + 1;

    std::vector<double> power_sum(total_bins, 0.0);
    std::vector<double> weight_sum(total_bins, 0.0);

    std::vector<double> window(rtl::constants::FFT_SIZE);
    for (int k = 0; k < rtl::constants::FFT_SIZE; ++k) {
        window[k] = 0.5 * (1.0 - std::cos(2.0 * M_PI * k / (rtl::constants::FFT_SIZE - 1)));
    }

    for (const auto& seg : segments) {
        if (static_cast<int>(seg.spectrum.size()) < rtl::constants::FFT_SIZE) continue;

        const double seg_start = seg.centerFreq - sampleRate / 2.0;
        const int    base_bin  = static_cast<int>(std::llround((seg_start - startFreq) / bin_width));

        for (int k = 0; k < rtl::constants::FFT_SIZE; ++k) {
            const int global = base_bin + k;
            if (global < 0 || global >= total_bins) continue;

            const double weight = window[k];
            if (weight <= 1e-12) continue;

            const double power = std::pow(10.0, seg.spectrum[k] / 10.0);

            power_sum[global]  += power * weight;
            weight_sum[global] += weight;
        }
    }

    std::vector<double> spectrum(total_bins, -200.0);
    std::vector<double> freqs(total_bins, 0.0);

    for (int i = 0; i < total_bins; ++i) {
        if (weight_sum[i] > 1e-12) {
            const double avg_power = power_sum[i] / weight_sum[i];
            spectrum[i]            = 10.0 * std::log10(std::max(avg_power, 1e-30));
        } else {
            spectrum[i] = -200.0;
        }
        freqs[i] = (startFreq + i * bin_width) / 1e6;
    }

    if (total_bins >= 5) {
        std::vector<double> smoothed = spectrum;
        for (int i = 2; i < total_bins - 2; ++i) {
            smoothed[i] = (1.0 * spectrum[i - 2] + 4.0 * spectrum[i - 1] + 6.0 * spectrum[i] + 4.0 * spectrum[i + 1]
                           + 1.0 * spectrum[i + 2])
                        / 16.0;
        }
        spectrum.swap(smoothed);
    }

    return {spectrum, freqs};
}

void removeDc(std::complex<short>* buf, size_t bufLen) {
    if (buf == nullptr || bufLen == 0) return;

    double sum_i = 0.0, sum_q = 0.0;
    for (size_t i = 0; i < bufLen; ++i) {
        sum_i += static_cast<double>(buf[i].real());
        sum_q += static_cast<double>(buf[i].imag());
    }
    double mean_i = sum_i / static_cast<double>(bufLen);
    double mean_q = sum_q / static_cast<double>(bufLen);

    for (size_t i = 0; i < bufLen; ++i) {
        buf[i] = std::complex<short>(
            static_cast<short>(std::lround(static_cast<double>(buf[i].real()) - mean_i)),
            static_cast<short>(std::lround(static_cast<double>(buf[i].imag()) - mean_q)));
    }
}

std::pair<std::vector<double>, int> calculateFft(const std::complex<short>* buf, size_t bufLen, double sampleRate) {
    std::vector<double> fft_out(rtl::constants::FFT_SIZE, 0.0);
    if (buf == nullptr || bufLen == 0) {
        spdlog::warn("CalculateFFT: null or empty buffer");
        return {fft_out, 0};
    }

    int groupsNum = static_cast<int>(bufLen) / rtl::constants::FFT_SIZE;
    if (groupsNum <= 0) {
        spdlog::warn("CalculateFFT: no FFT groups produced");
        return {fft_out, 0};
    }

    fftw_complex* in  = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * rtl::constants::FFT_SIZE);
    fftw_complex* out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * rtl::constants::FFT_SIZE);

    fftw_plan plan = fftw_plan_dft_1d(rtl::constants::FFT_SIZE, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    int half = rtl::constants::FFT_SIZE / 2;

    for (int g = 0; g < groupsNum; ++g) {
        for (int k = 0; k < rtl::constants::FFT_SIZE; ++k) {
            in[k][0] = static_cast<double>(buf[g * rtl::constants::FFT_SIZE + k].real());
            in[k][1] = static_cast<double>(buf[g * rtl::constants::FFT_SIZE + k].imag());
        }

        fftw_execute(plan);

        for (int k = 0; k < rtl::constants::FFT_SIZE; ++k) {
            int    shifted_k  = (k + half) % rtl::constants::FFT_SIZE;
            double real       = out[shifted_k][0];
            double imag       = out[shifted_k][1];
            fft_out[k]       += (real * real + imag * imag);
        }
    }

    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);

    return {fft_out, groupsNum};
}

std::vector<double> spectrumToDb(const std::vector<double>& fftPowerSum, int groupsNum) {
    std::vector<double> db(rtl::constants::FFT_SIZE, -200.0);
    if (groupsNum <= 0) return db;

    double           norm = static_cast<double>(groupsNum) * rtl::constants::FFT_SIZE * rtl::constants::FFT_SIZE
                          * rtl::constants::FULL_SCALE_REF;
    constexpr double eps  = 1e-18;
    for (int k = 0; k < rtl::constants::FFT_SIZE; ++k) {
        double p = fftPowerSum[k] / norm;
        db[k]    = 10.0 * std::log10(std::max(p, eps));
    }
    return db;
}

void suppressDcSpike(std::vector<double>& fftPower, int width) {
    int N = static_cast<int>(fftPower.size());
    if (N < 32 || width < 1) return;

    auto median_of = [](std::vector<double>& v) -> double {
        if (v.empty()) return 0.0;
        size_t mid = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + mid, v.end());
        return v[mid];
    };

    auto suppress_if_spike = [&](int bin) {
        int guard = width + 3;
        if (bin < guard + 4 || bin >= N - guard - 4) return;

        std::vector<double> left_ref(4);
        for (int k = 0; k < 4; ++k) left_ref[k] = fftPower[bin - guard - k];
        double left_median = median_of(left_ref);

        std::vector<double> right_ref(4);
        for (int k = 0; k < 4; ++k) right_ref[k] = fftPower[bin + guard + k];
        double right_median = median_of(right_ref);

        double peak_max = 0.0;
        for (int k = -width; k <= width; ++k) { peak_max = std::max(peak_max, fftPower[bin + k]); }

        double ref = (left_median + right_median) * 0.5;
        if (ref <= 0.0) return;

        if (peak_max > ref * 2.0) {
            for (int k = -width; k <= width; ++k) {
                double ratio      = static_cast<double>(k + width) / (2 * width);
                fftPower[bin + k] = left_median * (1.0 - ratio) + right_median * ratio;
            }
        }
    };

    suppress_if_spike(0);
    suppress_if_spike(N / 2);
}

void suppressPeriodicSpurs(
    std::vector<double>& spectrum,
    double               startFreq,
    double               endFreq,
    double               spikeIntervalHz,
    int                  width,
    double               thresholdDb) {
    int N = static_cast<int>(spectrum.size());
    if (N < 32 || width < 1) return;

    const double bin_width  = (endFreq - startFreq) / (N - 1);
    const int    spike_bins = static_cast<int>(std::round(spikeIntervalHz / bin_width));
    if (spike_bins < 16) return;

    auto median_of = [](std::vector<double>& v) -> double {
        if (v.empty()) return 0.0;
        size_t mid = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + mid, v.end());
        return v[mid];
    };

    auto suppress_if_spike = [&](int bin) {
        int guard = width + 3;
        if (bin < guard + 4 || bin >= N - guard - 4) return;

        std::vector<double> left_ref(4);
        for (int k = 0; k < 4; ++k) left_ref[k] = spectrum[bin - guard - k];
        double left_median = median_of(left_ref);

        std::vector<double> right_ref(4);
        for (int k = 0; k < 4; ++k) right_ref[k] = spectrum[bin + guard + k];
        double right_median = median_of(right_ref);

        double peak_max = -200.0;
        for (int k = -width; k <= width; ++k) { peak_max = std::max(peak_max, spectrum[bin + k]); }

        double ref = (left_median + right_median) * 0.5;

        if (peak_max > ref + thresholdDb) {
            for (int k = -width; k <= width; ++k) {
                double ratio      = static_cast<double>(k + width) / (2 * width);
                spectrum[bin + k] = left_median * (1.0 - ratio) + right_median * ratio;
            }
        }
    };

    double first_spike_freq = std::ceil(startFreq / spikeIntervalHz) * spikeIntervalHz;

    for (double freq = first_spike_freq; freq <= endFreq; freq += spikeIntervalHz) {
        int bin = static_cast<int>(std::round((freq - startFreq) / bin_width));
        if (bin >= 0 && bin < N) { suppress_if_spike(bin); }
    }
}

void suppressSymmetricSpur(std::vector<double>& fftPower) {
    int N = static_cast<int>(fftPower.size());
    if (N < 32) return;

    auto median_of = [](std::vector<double>& v) -> double {
        if (v.empty()) return 0.0;
        size_t mid = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + mid, v.end());
        return v[mid];
    };

    auto local_median = [&](int center, int radius, int exclude) -> double {
        std::vector<double> vals;
        for (int d = -radius; d <= radius; ++d) {
            if (d == exclude || center + d < 0 || center + d >= N) continue;
            vals.push_back(fftPower[center + d]);
        }
        return vals.empty() ? 0.0 : median_of(vals);
    };

    std::vector<char> suppressed(N, 0);

    for (int k = 5; k < N / 2 - 5; ++k) {
        int mk = N - k;

        if (suppressed[k] || suppressed[mk]) continue;

        double pk = fftPower[k];
        double pm = fftPower[mk];
        if (pk <= 0 || pm <= 0) continue;

        double ratio = pk / pm;
        if (pk < pm) ratio = 1.0 / ratio;

        if (ratio > 2.0) continue;

        double avg_power = (pk + pm) * 0.5;

        double ref_k   = local_median(k, 6, 0);
        double ref_mk  = local_median(mk, 6, 0);
        double ref_avg = (ref_k + ref_mk) * 0.5;

        if (avg_power < ref_avg * 2.0) continue;

        std::vector<double> rep_vals_k, rep_vals_mk;
        rep_vals_k.reserve(3);
        rep_vals_mk.reserve(3);
        rep_vals_k.push_back(ref_k);
        rep_vals_mk.push_back(ref_mk);
        if (k - 10 >= 0) rep_vals_k.push_back(fftPower[k - 10]);
        if (k + 10 < N) rep_vals_k.push_back(fftPower[k + 10]);
        if (mk - 10 >= 0) rep_vals_mk.push_back(fftPower[mk - 10]);
        if (mk + 10 < N) rep_vals_mk.push_back(fftPower[mk + 10]);

        fftPower[k]    = median_of(rep_vals_k);
        fftPower[mk]   = median_of(rep_vals_mk);
        suppressed[k]  = 1;
        suppressed[mk] = 1;
    }
}

void suppressImageSpur(std::vector<double>& fftPower, double thresholdDb, double irr_db) {
    int N = static_cast<int>(fftPower.size());
    if (N < 32) return;

    (void)irr_db;

    std::vector<double> sorted(fftPower.begin(), fftPower.end());
    size_t              q25 = sorted.size() / 4;
    std::nth_element(sorted.begin(), sorted.begin() + q25, sorted.end());
    double noise_floor = sorted[q25];
    if (noise_floor <= 0) return;

    double threshold_ratio = std::pow(10.0, thresholdDb / 10.0);
    double elevated_thresh = noise_floor * 4.0;

    auto median_of = [](std::vector<double>& v) -> double {
        if (v.empty()) return 0.0;
        size_t mid = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + mid, v.end());
        return v[mid];
    };

    for (int k = 1; k < N / 2; ++k) {
        int    mirror = N - k;
        double p_k    = fftPower[k];
        double p_m    = fftPower[mirror];

        if (p_k <= 0 || p_m <= 0) continue;

        if (p_k < elevated_thresh && p_m < elevated_thresh) continue;

        int    weak_bin;
        double p_weak, p_strong;
        if (p_k >= p_m) {
            p_strong = p_k;
            weak_bin = mirror;
            p_weak   = p_m;
        } else {
            p_strong = p_m;
            weak_bin = k;
            p_weak   = p_k;
        }

        if (p_strong / p_weak < threshold_ratio) continue;

        std::vector<double> neighbors;
        neighbors.reserve(8);
        for (int d = 4; d <= 7; ++d) {
            if (weak_bin - d >= 0) neighbors.push_back(fftPower[weak_bin - d]);
            if (weak_bin + d < N) neighbors.push_back(fftPower[weak_bin + d]);
        }
        double replacement = neighbors.empty() ? noise_floor : median_of(neighbors);

        for (int d = -2; d <= 2; ++d) {
            int b = weak_bin + d;
            if (b >= 0 && b < N && fftPower[b] > replacement) { fftPower[b] = replacement; }
        }
    }
}

double calculateRssi(const std::vector<double>& fftPowerSum, int groupsNum) {
    if (groupsNum <= 0) return -120.0;

    double total_power = std::accumulate(fftPowerSum.begin(), fftPowerSum.end(), 0.0);

    double mean_power =
        total_power / (static_cast<double>(groupsNum) * rtl::constants::FFT_SIZE * rtl::constants::FFT_SIZE);

    double normalized_power = mean_power / rtl::constants::FULL_SCALE_REF;

    constexpr double eps       = 1e-18;
    double           rssi_dbfs = 10.0 * std::log10(std::max(normalized_power, eps));
    return rssi_dbfs;
}

double calculateRssiDirectSampling(const short* chData, size_t chLen) {
    if (chData == nullptr || chLen == 0) return -120.0;

    double power = 0.0;
    for (size_t i = 0; i < chLen; ++i) {
        double v  = static_cast<double>(chData[i]);
        power    += v * v;
    }

    double mean_power       = power / static_cast<double>(chLen);
    double normalized_power = mean_power / rtl::constants::FULL_SCALE_REF;

    constexpr double eps       = 1e-18;
    double           rssi_dbfs = 10.0 * std::log10(std::max(normalized_power, eps));
    return rssi_dbfs;
}

std::vector<PeakInfo>
    detectPeaks(const std::vector<double>& spectrum, double freqStart, double freqPerBin, double thresholdDb) {
    std::vector<PeakInfo> peaks;
    int                   N = static_cast<int>(spectrum.size());
    if (N < 2) return peaks;

    double max_val = *std::max_element(spectrum.begin(), spectrum.end());
    double min_val = *std::min_element(spectrum.begin(), spectrum.end());
    double range   = max_val - min_val;
    if (range <= 0) range = 1.0;

    int i = 0;
    while (i < N) {
        if (spectrum[i] < thresholdDb) {
            i++;
            continue;
        }

        int start = i;
        while (i < N && spectrum[i] >= thresholdDb) i++;
        int stop = i - 1;

        if (stop - start < 2) continue;

        double sum  = 0.0;
        double peak = -200.0;
        for (int j = start; j <= stop; ++j) {
            sum += spectrum[j];
            if (spectrum[j] > peak) peak = spectrum[j];
        }
        double avg = sum / (stop - start + 1);

        PeakInfo info;
        info.posStart = start;
        info.posStop  = stop;
        info.cf       = freqStart + (start + stop) / 2.0 * freqPerBin;
        info.bw       = (stop - start + 1) * freqPerBin;
        info.avgDb    = avg;
        info.maxDb    = peak;
        info.level    = (peak - min_val) / range * 100.0;

        peaks.push_back(info);
    }

    return peaks;
}

} // namespace rtl::tools
