#pragma once

#include <complex>
#include <cstddef>
#include <utility>
#include <vector>
#include <fftw3.h>

namespace rtl::tools {

class FftEngine {
public:
    explicit FftEngine(int fftSize);
    ~FftEngine();

    FftEngine(const FftEngine&)            = delete;
    FftEngine& operator=(const FftEngine&) = delete;

    std::pair<std::vector<double>, int> accumulatePower(const std::complex<short>* buf, std::size_t bufLen);

private:
    int           fftSize_;
    int           half_;
    fftw_complex* in_;
    fftw_complex* out_;
    fftw_plan     plan_;
};

} // namespace rtl::tools