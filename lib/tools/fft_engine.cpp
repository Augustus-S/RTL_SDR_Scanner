#include "tools/fft_engine.hpp"
#include "constants.hpp"
#include <spdlog/spdlog.h>

namespace rtl::tools {

FftEngine::FftEngine(int fftSize)
    : fftSize_(fftSize)
    , half_(fftSize / 2) {
    in_  = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * fftSize_);
    out_ = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * fftSize_);
    plan_ = fftw_plan_dft_1d(fftSize_, in_, out_, FFTW_FORWARD, FFTW_MEASURE);
}

FftEngine::~FftEngine() {
    fftw_destroy_plan(plan_);
    fftw_free(in_);
    fftw_free(out_);
}

std::pair<std::vector<double>, int> FftEngine::accumulatePower(const std::complex<short>* buf, std::size_t bufLen) {
    std::vector<double> fft_out(fftSize_, 0.0);
    if (buf == nullptr || bufLen == 0) {
        spdlog::warn("FftEngine: null or empty buffer");
        return {fft_out, 0};
    }

    int groupsNum = static_cast<int>(bufLen) / fftSize_;
    if (groupsNum <= 0) {
        spdlog::warn("FftEngine: no FFT groups produced");
        return {fft_out, 0};
    }

    for (int g = 0; g < groupsNum; ++g) {
        for (int k = 0; k < fftSize_; ++k) {
            in_[k][0] = static_cast<double>(buf[g * fftSize_ + k].real());
            in_[k][1] = static_cast<double>(buf[g * fftSize_ + k].imag());
        }

        fftw_execute(plan_);

        for (int k = 0; k < fftSize_; ++k) {
            int    shifted_k  = (k + half_) % fftSize_;
            double real       = out_[shifted_k][0];
            double imag       = out_[shifted_k][1];
            fft_out[k]       += (real * real + imag * imag);
        }
    }

    return {fft_out, groupsNum};
}

} // namespace rtl::tools