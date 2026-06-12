#include <rtl-sdr.h>
#include <alsa/asoundlib.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr std::uint32_t kDefaultDeviceIndex = 0;
constexpr std::uint32_t kDefaultFrequencyHz = 100000000;
constexpr std::uint32_t kSdrSampleRate      = 1200000;
constexpr std::uint32_t kAudioSampleRate    = 48000;
constexpr int           kCompositeDecimation = 5;
constexpr std::uint32_t kCompositeSampleRate = kSdrSampleRate / kCompositeDecimation;
constexpr int           kAudioDecimation    = kCompositeSampleRate / kAudioSampleRate;
constexpr int           kDecimation         = kCompositeDecimation * kAudioDecimation;
constexpr std::int32_t  kTuningOffsetHz     = 250000;
constexpr int           kReadBytes          = 16 * 16384;
constexpr int           kRfFirTaps          = 97;
constexpr double        kRfCutoffHz         = 120000.0;
constexpr int           kCompositeFirTaps   = 97;
constexpr double        kCompositeCutoffHz  = 100000.0;
constexpr int           kAudioFirTaps       = 161;
constexpr double        kAudioCutoffHz      = 15000.0;
constexpr double        kStereoPilotHz      = 19000.0;
constexpr double        kStereoSubcarrierHz = 38000.0;
constexpr double        kFmDeviationHz      = 75000.0;
constexpr std::uint32_t kAlsaLatencyUs      = 500000;
constexpr std::size_t   kAudioPrebufferFrames = kAudioSampleRate / 2;

std::atomic_bool gStop{false};

void handleSignal(int) {
    gStop.store(true);
}

void initLogging() {
    namespace fs = std::filesystem;

    auto logDir = fs::path("logs");
    if (!fs::exists(logDir)) { fs::create_directories(logDir); }

    auto consoleSink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    consoleSink->set_level(spdlog::level::info);

    auto fileSink =
        std::make_shared<spdlog::sinks::rotating_file_sink_mt>((logDir / "RTL_SDR.log").string(), 1024 * 1024 * 100, 3);
    fileSink->set_level(spdlog::level::info);

    std::vector<spdlog::sink_ptr> sinks{consoleSink, fileSink};
    auto                          logger = std::make_shared<spdlog::logger>("fm_broadcast_player", sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::info);
    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");

    spdlog::set_default_logger(logger);
    spdlog::flush_every(std::chrono::seconds(3));
}

struct Config {
    std::uint32_t deviceIndex  = kDefaultDeviceIndex;
    std::uint32_t frequencyHz  = kDefaultFrequencyHz;
    int           gainTenthsDb = -1;
    int           ppm          = 0;
    double        volume       = 0.8;
    double        deemphasisUs = 50.0;
    double        durationSec  = 0.0;
    bool          audioOn      = false;
    bool          offsetTuning = true;
    bool          stereoOn     = true;
    bool          debugOn      = false;
    bool          listGains    = false;
    bool          captureIqOnly = false;
};

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " [options] > audio.s16le\n\n"
        << "Options:\n"
        << "  --freq <MHz>          FM broadcast frequency, default 100.0\n"
        << "  --device <index>      RTL-SDR device index, default 0\n"
        << "  --gain <dB|auto>      Tuner gain in dB, default auto\n"
        << "  --list-gains          List supported tuner gains and exit\n"
        << "  --ppm <value>         Frequency correction PPM, default 0\n"
        << "  --volume <value>      Audio scale, default 0.8\n"
        << "  --deemphasis <us>     50 or 75, default 50\n"
        << "  --duration <sec>      Stop after seconds, default run until Ctrl-C\n"
        << "  --audio <on|off>      Play through system default audio device, default off\n"
        << "  --offset <on|off>     Offset tune away from the RTL-SDR DC spike, default on\n"
        << "  --stereo <on|off>     Decode FM stereo, default on\n"
        << "  --debug <on|off>      Save IQ and print DSP statistics, default off\n"
        << "  --capture-iq-only <on|off>\n"
        << "                         With --debug on, save raw IQ without demodulating, default off\n"
        << "  --help, -h            Show this help\n\n"
        << "Play example:\n"
        << "  " << argv0 << " --freq 100.0 --audio on\n"
        << "  " << argv0 << " --freq 100.0 --audio off | aplay -r 48000 -f S16_LE -c 2\n";
}

bool parseUint32(const char* text, std::uint32_t& value) {
    char* end = nullptr;
    auto  v   = std::strtoul(text, &end, 10);
    if (!end || *end != '\0') return false;
    value = static_cast<std::uint32_t>(v);
    return true;
}

bool parseArgs(int argc, char* argv[], Config& cfg) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto        needValue = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                spdlog::error("Missing value for {}", name);
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--list-gains") {
            cfg.listGains = true;
        } else if (arg == "--freq") {
            const char* value = needValue("--freq");
            if (!value) return false;
            cfg.frequencyHz = static_cast<std::uint32_t>(std::llround(std::atof(value) * 1000000.0));
        } else if (arg == "--device") {
            const char* value = needValue("--device");
            if (!value || !parseUint32(value, cfg.deviceIndex)) return false;
        } else if (arg == "--gain") {
            const char* value = needValue("--gain");
            if (!value) return false;
            if (std::string(value) == "auto") {
                cfg.gainTenthsDb = -1;
            } else {
                cfg.gainTenthsDb = static_cast<int>(std::llround(std::atof(value) * 10.0));
            }
        } else if (arg == "--ppm") {
            const char* value = needValue("--ppm");
            if (!value) return false;
            cfg.ppm = std::atoi(value);
        } else if (arg == "--volume") {
            const char* value = needValue("--volume");
            if (!value) return false;
            cfg.volume = std::atof(value);
        } else if (arg == "--deemphasis") {
            const char* value = needValue("--deemphasis");
            if (!value) return false;
            cfg.deemphasisUs = std::atof(value);
        } else if (arg == "--duration") {
            const char* value = needValue("--duration");
            if (!value) return false;
            cfg.durationSec = std::atof(value);
        } else if (arg == "--audio") {
            const char* value = needValue("--audio");
            if (!value) return false;
            std::string mode = value;
            if (mode == "on") {
                cfg.audioOn = true;
            } else if (mode == "off") {
                cfg.audioOn = false;
            } else {
                spdlog::error("--audio must be on or off");
                return false;
            }
        } else if (arg == "--offset") {
            const char* value = needValue("--offset");
            if (!value) return false;
            std::string mode = value;
            if (mode == "on") {
                cfg.offsetTuning = true;
            } else if (mode == "off") {
                cfg.offsetTuning = false;
            } else {
                spdlog::error("--offset must be on or off");
                return false;
            }
        } else if (arg == "--stereo") {
            const char* value = needValue("--stereo");
            if (!value) return false;
            std::string mode = value;
            if (mode == "on") {
                cfg.stereoOn = true;
            } else if (mode == "off") {
                cfg.stereoOn = false;
            } else {
                spdlog::error("--stereo must be on or off");
                return false;
            }
        } else if (arg == "--debug") {
            const char* value = needValue("--debug");
            if (!value) return false;
            std::string mode = value;
            if (mode == "on") {
                cfg.debugOn = true;
            } else if (mode == "off") {
                cfg.debugOn = false;
            } else {
                spdlog::error("--debug must be on or off");
                return false;
            }
        } else if (arg == "--capture-iq-only") {
            const char* value = needValue("--capture-iq-only");
            if (!value) return false;
            std::string mode = value;
            if (mode == "on") {
                cfg.captureIqOnly = true;
                cfg.debugOn = true;
            } else if (mode == "off") {
                cfg.captureIqOnly = false;
            } else {
                spdlog::error("--capture-iq-only must be on or off");
                return false;
            }
        } else {
            spdlog::error("Unknown option: {}", arg);
            return false;
        }
    }

    if (cfg.frequencyHz < 87000000 || cfg.frequencyHz > 109000000) {
        spdlog::error("FM broadcast frequency should be near 87-109 MHz");
        return false;
    }
    if (cfg.deemphasisUs <= 0.0 || cfg.volume <= 0.0) {
        spdlog::error("De-emphasis and volume must be positive");
        return false;
    }
    if (cfg.captureIqOnly && cfg.audioOn) {
        spdlog::error("--capture-iq-only on cannot be combined with --audio on");
        return false;
    }
    return true;
}

std::string makeTimestampedIqFilename() {
    const auto now = std::chrono::system_clock::now();
    const auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm    tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d%H%M%S") << "_FM.iq";
    return oss.str();
}

class DebugStats {
public:
    explicit DebugStats(bool enabled)
        : enabled_(enabled)
        , lastReport_(std::chrono::steady_clock::now()) {}

    bool enabled() const {
        return enabled_;
    }

    void addRaw(double i, double q) {
        if (!enabled_) return;
        rawPowerSum_ += i * i + q * q;
        ++rawCount_;
    }

    void addRf(double i, double q) {
        if (!enabled_) return;
        rfPowerSum_ += i * i + q * q;
        ++rfCount_;
    }

    void addDiscriminator(double value) {
        if (!enabled_) return;
        discPowerSum_ += value * value;
        discPeak_ = std::max(discPeak_, std::abs(value));
        ++discCount_;
    }

    void addComposite(double value) {
        if (!enabled_) return;
        compositePowerSum_ += value * value;
        compositePeak_ = std::max(compositePeak_, std::abs(value));
        ++compositeCount_;
    }

    void addAudio(double sum, double diff) {
        if (!enabled_) return;
        sumPowerSum_ += sum * sum;
        diffPowerSum_ += diff * diff;
        ++audioCount_;
    }

    void addPcm(const std::vector<std::int16_t>& pcm, int channels) {
        if (!enabled_ || channels <= 0) return;
        for (auto sample : pcm) { pcmPeak_ = std::max(pcmPeak_, std::abs(static_cast<int>(sample))); }
        outputFrames_ += pcm.size() / static_cast<std::size_t>(channels);
    }

    void addIqBytes(std::size_t bytes) {
        if (!enabled_) return;
        iqBytes_ += bytes;
    }

    void reportIfDue(std::uint32_t sampleRate, std::uint32_t audioRate) {
        if (!enabled_) return;
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReport_);
        if (elapsed.count() < 1000) return;

        const double seconds = elapsed.count() / 1000.0;
        spdlog::info(
            "debug stats: iq_rate={:.0f}S/s audio_rate={:.0f}f/s raw_rms={:.5f} rf_rms={:.5f} disc_rms={:.5f} disc_peak={:.5f} composite_rms={:.5f} composite_peak={:.5f} sum_rms={:.5f} diff_rms={:.5f} pcm_peak={} iq_bytes={}",
            (iqBytes_ / 2.0) / seconds,
            outputFrames_ / seconds,
            rms(rawPowerSum_, rawCount_),
            rms(rfPowerSum_, rfCount_),
            rms(discPowerSum_, discCount_),
            discPeak_,
            rms(compositePowerSum_, compositeCount_),
            compositePeak_,
            rms(sumPowerSum_, audioCount_),
            rms(diffPowerSum_, audioCount_),
            pcmPeak_,
            iqBytes_);

        if ((iqBytes_ / 2.0) / seconds < sampleRate * 0.90) {
            spdlog::warn("debug stats: input sample rate is below expected {} S/s", sampleRate);
        }
        if (outputFrames_ / seconds < audioRate * 0.90) {
            spdlog::warn("debug stats: output audio frame rate is below expected {} frames/s", audioRate);
        }

        reset(now);
    }

private:
    static double rms(double powerSum, std::uint64_t count) {
        return count == 0 ? 0.0 : std::sqrt(powerSum / static_cast<double>(count));
    }

    void reset(std::chrono::steady_clock::time_point now) {
        lastReport_ = now;
        rawPowerSum_ = 0.0;
        rfPowerSum_ = 0.0;
        discPowerSum_ = 0.0;
        compositePowerSum_ = 0.0;
        sumPowerSum_ = 0.0;
        diffPowerSum_ = 0.0;
        rawCount_ = 0;
        rfCount_ = 0;
        discCount_ = 0;
        compositeCount_ = 0;
        audioCount_ = 0;
        outputFrames_ = 0;
        iqBytes_ = 0;
        discPeak_ = 0.0;
        compositePeak_ = 0.0;
        pcmPeak_ = 0;
    }

    bool enabled_ = false;
    std::chrono::steady_clock::time_point lastReport_;
    double rawPowerSum_ = 0.0;
    double rfPowerSum_ = 0.0;
    double discPowerSum_ = 0.0;
    double compositePowerSum_ = 0.0;
    double sumPowerSum_ = 0.0;
    double diffPowerSum_ = 0.0;
    double discPeak_ = 0.0;
    double compositePeak_ = 0.0;
    int pcmPeak_ = 0;
    std::uint64_t rawCount_ = 0;
    std::uint64_t rfCount_ = 0;
    std::uint64_t discCount_ = 0;
    std::uint64_t compositeCount_ = 0;
    std::uint64_t audioCount_ = 0;
    std::uint64_t outputFrames_ = 0;
    std::uint64_t iqBytes_ = 0;
};

std::vector<double> makeLowPassFir(int taps, double cutoffHz, double sampleRate) {
    std::vector<double> h(static_cast<std::size_t>(taps));
    const int           mid = taps / 2;
    const double        fc  = cutoffHz / sampleRate;
    double              sum = 0.0;

    for (int n = 0; n < taps; ++n) {
        const int    m      = n - mid;
        const double sinc   = (m == 0) ? 2.0 * fc : std::sin(2.0 * M_PI * fc * m) / (M_PI * m);
        const double window = 0.54 - 0.46 * std::cos(2.0 * M_PI * n / (taps - 1));
        h[static_cast<std::size_t>(n)] = sinc * window;
        sum += h[static_cast<std::size_t>(n)];
    }

    for (double& v : h) v /= sum;
    return h;
}

class WbfmDemodulator {
public:
    explicit WbfmDemodulator(const Config& cfg, DebugStats* debugStats)
        : rfFir_(makeLowPassFir(kRfFirTaps, kRfCutoffHz, kSdrSampleRate))
        , rfRingI_(kRfFirTaps, 0.0)
        , rfRingQ_(kRfFirTaps, 0.0)
        , compositeFir_(makeLowPassFir(kCompositeFirTaps, kCompositeCutoffHz, kSdrSampleRate))
        , compositeRing_(kCompositeFirTaps, 0.0)
        , sumFir_(makeLowPassFir(kAudioFirTaps, kAudioCutoffHz, kCompositeSampleRate))
        , sumRing_(kAudioFirTaps, 0.0)
        , diffFir_(makeLowPassFir(kAudioFirTaps, kAudioCutoffHz, kCompositeSampleRate))
        , diffRing_(kAudioFirTaps, 0.0)
        , mixerPhaseInc_(cfg.offsetTuning ? 2.0 * M_PI * kTuningOffsetHz / kSdrSampleRate : 0.0)
        , deemphasisAlpha_(1.0 / (1.0 + (cfg.deemphasisUs * 1e-6) * kAudioSampleRate))
        , pilotNominalPhaseInc_(2.0 * M_PI * kStereoPilotHz / kCompositeSampleRate)
        , stereoOn_(cfg.stereoOn)
        , debugStats_(debugStats)
        , volume_(cfg.volume) {}

    int outputChannels() const {
        return stereoOn_ ? 2 : 1;
    }

    void processIq(const std::uint8_t* iq, int bytes, std::vector<std::int16_t>& pcm) {
        pcm.clear();
        pcm.reserve(static_cast<std::size_t>(bytes / (2 * kDecimation) * outputChannels()));

        for (int idx = 0; idx + 1 < bytes; idx += 2) {
            const double rawI = (static_cast<double>(iq[idx]) - 127.5) / 127.5;
            const double rawQ = (static_cast<double>(iq[idx + 1]) - 127.5) / 127.5;
            if (debugStats_) { debugStats_->addRaw(rawI, rawQ); }
            const double cosPhase = std::cos(mixerPhase_);
            const double sinPhase = std::sin(mixerPhase_);
            const double i = rawI * cosPhase - rawQ * sinPhase;
            const double q = rawI * sinPhase + rawQ * cosPhase;

            mixerPhase_ += mixerPhaseInc_;
            if (mixerPhase_ >= 2.0 * M_PI) mixerPhase_ -= 2.0 * M_PI;

            pushRfSample(i, q);
            const auto [filteredI, filteredQ] = rfOutput();
            if (debugStats_) { debugStats_->addRf(filteredI, filteredQ); }

            if (!havePrev_) {
                prevI_    = filteredI;
                prevQ_    = filteredQ;
                havePrev_ = true;
                continue;
            }

            const double real  = filteredI * prevI_ + filteredQ * prevQ_;
            const double imag  = filteredQ * prevI_ - filteredI * prevQ_;
            const double angle = std::atan2(imag, real);
            prevI_             = filteredI;
            prevQ_             = filteredQ;

            const double discriminator = angle * kSdrSampleRate / (2.0 * M_PI * kFmDeviationHz);
            if (debugStats_) { debugStats_->addDiscriminator(discriminator); }
            pushCompositeSample(discriminator);
            if (++compositeDecimPhase_ < kCompositeDecimation) continue;
            compositeDecimPhase_ = 0;

            const double composite = compositeOutput();
            if (debugStats_) { debugStats_->addComposite(composite); }
            updatePilotPll(composite);
            pushSumSample(composite);
            const double stereoDiffBaseband = 2.0 * composite * std::cos(2.0 * pilotPhase_);
            pushDiffSample(stereoDiffBaseband);
            if (++audioDecimPhase_ < kAudioDecimation) continue;
            audioDecimPhase_ = 0;

            const double sum  = dcBlockSum(sumOutput());
            const double diff = dcBlockDiff(diffOutput());
            if (debugStats_) { debugStats_->addAudio(sum, diff); }

            if (stereoOn_) {
                double left  = 0.5 * (sum + diff);
                double right = 0.5 * (sum - diff);
                leftDeemphState_ += deemphasisAlpha_ * (left - leftDeemphState_);
                rightDeemphState_ += deemphasisAlpha_ * (right - rightDeemphState_);
                left  = std::clamp(leftDeemphState_ * volume_, -1.0, 1.0);
                right = std::clamp(rightDeemphState_ * volume_, -1.0, 1.0);
                pcm.push_back(toPcm(left));
                pcm.push_back(toPcm(right));
            } else {
                monoDeemphState_ += deemphasisAlpha_ * (sum - monoDeemphState_);
                const double mono = std::clamp(monoDeemphState_ * volume_, -1.0, 1.0);
                pcm.push_back(toPcm(mono));
            }
        }
    }

private:
    static std::int16_t toPcm(double sample) {
        return static_cast<std::int16_t>(std::lrint(sample * 32767.0));
    }

    static double wrapPhase(double phase) {
        while (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
        while (phase < 0.0) phase += 2.0 * M_PI;
        return phase;
    }

    void pushRfSample(double i, double q) {
        rfRingI_[rfRingPos_] = i;
        rfRingQ_[rfRingPos_] = q;
        rfRingPos_           = (rfRingPos_ + 1) % rfRingI_.size();
    }

    std::pair<double, double> rfOutput() const {
        double outI = 0.0;
        double outQ = 0.0;
        auto   pos  = rfRingPos_;
        for (std::size_t n = 0; n < rfFir_.size(); ++n) {
            pos = (pos == 0) ? rfRingI_.size() - 1 : pos - 1;
            outI += rfFir_[n] * rfRingI_[pos];
            outQ += rfFir_[n] * rfRingQ_[pos];
        }
        return {outI, outQ};
    }

    void pushCompositeSample(double value) {
        compositeRing_[compositeRingPos_] = value;
        compositeRingPos_                 = (compositeRingPos_ + 1) % compositeRing_.size();
    }

    double compositeOutput() const {
        double acc = 0.0;
        auto   pos = compositeRingPos_;
        for (std::size_t n = 0; n < compositeFir_.size(); ++n) {
            pos = (pos == 0) ? compositeRing_.size() - 1 : pos - 1;
            acc += compositeFir_[n] * compositeRing_[pos];
        }
        return acc;
    }

    void updatePilotPll(double composite) {
        if (!stereoOn_) return;

        constexpr double alpha = 0.0015;
        constexpr double beta  = 0.000001;

        const double error = composite * std::sin(pilotPhase_);
        pilotFreq_ += beta * error;
        pilotFreq_ = std::clamp(pilotFreq_, -0.002, 0.002);
        pilotPhase_ = wrapPhase(pilotPhase_ + pilotNominalPhaseInc_ + pilotFreq_ + alpha * error);
    }

    void pushSumSample(double value) {
        sumRing_[sumRingPos_] = value;
        sumRingPos_           = (sumRingPos_ + 1) % sumRing_.size();
    }

    double sumOutput() const {
        double acc = 0.0;
        auto   pos = sumRingPos_;
        for (std::size_t n = 0; n < sumFir_.size(); ++n) {
            pos = (pos == 0) ? sumRing_.size() - 1 : pos - 1;
            acc += sumFir_[n] * sumRing_[pos];
        }
        return acc;
    }

    void pushDiffSample(double value) {
        diffRing_[diffRingPos_] = value;
        diffRingPos_            = (diffRingPos_ + 1) % diffRing_.size();
    }

    double diffOutput() const {
        double acc = 0.0;
        auto   pos = diffRingPos_;
        for (std::size_t n = 0; n < diffFir_.size(); ++n) {
            pos = (pos == 0) ? diffRing_.size() - 1 : pos - 1;
            acc += diffFir_[n] * diffRing_[pos];
        }
        return acc;
    }

    double dcBlockSum(double x) {
        constexpr double r = 0.995;
        const double     y = x - dcSumPrevX_ + r * dcSumPrevY_;
        dcSumPrevX_        = x;
        dcSumPrevY_        = y;
        return y;
    }

    double dcBlockDiff(double x) {
        constexpr double r = 0.995;
        const double     y = x - dcDiffPrevX_ + r * dcDiffPrevY_;
        dcDiffPrevX_       = x;
        dcDiffPrevY_       = y;
        return y;
    }

    std::vector<double> rfFir_;
    std::vector<double> rfRingI_;
    std::vector<double> rfRingQ_;
    std::vector<double> compositeFir_;
    std::vector<double> compositeRing_;
    std::vector<double> sumFir_;
    std::vector<double> sumRing_;
    std::vector<double> diffFir_;
    std::vector<double> diffRing_;
    std::size_t         rfRingPos_        = 0;
    std::size_t         compositeRingPos_ = 0;
    std::size_t         sumRingPos_       = 0;
    std::size_t         diffRingPos_      = 0;
    int                 compositeDecimPhase_ = 0;
    int                 audioDecimPhase_ = 0;
    bool                havePrev_ = false;
    double              mixerPhase_ = 0.0;
    double              mixerPhaseInc_ = 0.0;
    double              prevI_ = 0.0;
    double              prevQ_ = 0.0;
    double              dcSumPrevX_ = 0.0;
    double              dcSumPrevY_ = 0.0;
    double              dcDiffPrevX_ = 0.0;
    double              dcDiffPrevY_ = 0.0;
    double              monoDeemphState_ = 0.0;
    double              leftDeemphState_ = 0.0;
    double              rightDeemphState_ = 0.0;
    double              deemphasisAlpha_;
    double              pilotPhase_ = 0.0;
    double              pilotFreq_ = 0.0;
    double              pilotNominalPhaseInc_;
    bool                stereoOn_;
    DebugStats*         debugStats_ = nullptr;
    double              volume_;
};

class AlsaAudioSink {
public:
    explicit AlsaAudioSink(int channels)
        : channels_(channels) {
        int err = snd_pcm_open(&pcm_, "default", SND_PCM_STREAM_PLAYBACK, 0);
        if (err < 0) {
            spdlog::error("Failed to open default ALSA playback device: {}", snd_strerror(err));
            return;
        }

        err = snd_pcm_set_params(pcm_,
                                 SND_PCM_FORMAT_S16_LE,
                                 SND_PCM_ACCESS_RW_INTERLEAVED,
                                 channels_,
                                 kAudioSampleRate,
                                 1,
                                 kAlsaLatencyUs);
        if (err < 0) {
            spdlog::error("Failed to configure ALSA playback device: {}", snd_strerror(err));
            snd_pcm_close(pcm_);
            pcm_ = nullptr;
            return;
        }
    }

    ~AlsaAudioSink() {
        if (!pcm_) return;
        snd_pcm_drain(pcm_);
        snd_pcm_close(pcm_);
    }

    AlsaAudioSink(const AlsaAudioSink&)            = delete;
    AlsaAudioSink& operator=(const AlsaAudioSink&) = delete;

    bool isOpen() const {
        return pcm_ != nullptr;
    }

    bool write(const std::int16_t* samples, std::size_t frames) {
        std::size_t offset = 0;
        while (offset < frames) {
            snd_pcm_sframes_t written = snd_pcm_writei(pcm_, samples + offset * channels_, frames - offset);
            if (written == -EPIPE) {
                int err = snd_pcm_prepare(pcm_);
                if (err < 0) {
                    spdlog::error("Failed to recover ALSA underrun: {}", snd_strerror(err));
                    return false;
                }
                ++underrunCount_;
                if (underrunCount_ == 1 || underrunCount_ % 20 == 0) {
                    spdlog::warn("Recovered ALSA playback underrun, count={}", underrunCount_);
                }
                continue;
            }
            if (written < 0) {
                written = snd_pcm_recover(pcm_, static_cast<int>(written), 1);
                if (written < 0) {
                    spdlog::error("ALSA playback write failed: {}", snd_strerror(static_cast<int>(written)));
                    return false;
                }
                continue;
            }
            if (written == 0) continue;
            offset += static_cast<std::size_t>(written);
        }
        return true;
    }

private:
    snd_pcm_t* pcm_          = nullptr;
    int        channels_     = 1;
    std::size_t underrunCount_ = 0;
};

bool configureDevice(rtlsdr_dev_t* dev, const Config& cfg) {
    int ret = rtlsdr_set_sample_rate(dev, kSdrSampleRate);
    if (ret < 0) {
        spdlog::error("Failed to set sample rate {}: error {}", kSdrSampleRate, ret);
        return false;
    }
    const std::uint32_t tunerFrequencyHz =
        cfg.offsetTuning ? static_cast<std::uint32_t>(cfg.frequencyHz + kTuningOffsetHz) : cfg.frequencyHz;
    ret = rtlsdr_set_center_freq(dev, tunerFrequencyHz);
    if (ret < 0) {
        spdlog::error("Failed to set center frequency {} Hz: error {}", tunerFrequencyHz, ret);
        return false;
    }
    if (cfg.offsetTuning) {
        spdlog::info("Offset tuning enabled: station {} Hz, tuner {} Hz, digital mixer +{} Hz",
                     cfg.frequencyHz,
                     tunerFrequencyHz,
                     kTuningOffsetHz);
    }
    ret = rtlsdr_set_freq_correction(dev, cfg.ppm);
    if (ret < 0 && ret != -2) {
        spdlog::error("Failed to set frequency correction {} ppm: error {}", cfg.ppm, ret);
        return false;
    }
    if (ret == -2) { spdlog::warn("RTL-SDR frequency correction returned -2; continuing with requested {} ppm", cfg.ppm); }
    if (cfg.gainTenthsDb < 0) {
        rtlsdr_set_tuner_gain_mode(dev, 0);
        spdlog::info("Using automatic tuner gain");
    } else {
        int gainToSet = cfg.gainTenthsDb;
        int gainCount = rtlsdr_get_tuner_gains(dev, nullptr);
        if (gainCount > 0) {
            std::vector<int> gains(static_cast<std::size_t>(gainCount));
            int retGains = rtlsdr_get_tuner_gains(dev, gains.data());
            if (retGains > 0) {
                gains.resize(static_cast<std::size_t>(retGains));
                gainToSet = *std::min_element(gains.begin(), gains.end(), [&](int a, int b) {
                    return std::abs(a - cfg.gainTenthsDb) < std::abs(b - cfg.gainTenthsDb);
                });
                if (gainToSet != cfg.gainTenthsDb) {
                    spdlog::info("Requested gain {:.1f} dB, using nearest supported gain {:.1f} dB",
                                 cfg.gainTenthsDb / 10.0,
                                 gainToSet / 10.0);
                }
            }
        }

        rtlsdr_set_tuner_gain_mode(dev, 1);
        ret = rtlsdr_set_tuner_gain(dev, gainToSet);
        if (ret < 0) {
            spdlog::error("Failed to set tuner gain {} tenths dB: error {}", gainToSet, ret);
            return false;
        }
        spdlog::info("Using tuner gain {:.1f} dB", gainToSet / 10.0);
    }
    ret = rtlsdr_reset_buffer(dev);
    if (ret < 0) {
        spdlog::error("Failed to reset RTL-SDR buffer: error {}", ret);
        return false;
    }
    return true;
}

void listTunerGains(rtlsdr_dev_t* dev) {
    int gainCount = rtlsdr_get_tuner_gains(dev, nullptr);
    if (gainCount <= 0) {
        spdlog::warn("No tuner gain list available; this tuner may only support automatic gain");
        return;
    }

    std::vector<int> gains(static_cast<std::size_t>(gainCount));
    int ret = rtlsdr_get_tuner_gains(dev, gains.data());
    if (ret <= 0) {
        spdlog::error("Failed to read tuner gains: {}", ret);
        return;
    }
    gains.resize(static_cast<std::size_t>(ret));

    std::string text;
    for (std::size_t i = 0; i < gains.size(); ++i) {
        if (i > 0) text += ", ";
        text += fmt::format("{:.1f}", gains[i] / 10.0);
    }
    spdlog::info("Supported tuner gains in dB: {}", text);
    spdlog::info("Use --gain auto for AGC, or --gain <dB>; unsupported values use the nearest supported gain");
}

} // namespace

int main(int argc, char* argv[]) {
    initLogging();

    Config cfg;
    if (!parseArgs(argc, argv, cfg)) {
        printUsage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    if (rtlsdr_get_device_count() == 0) {
        spdlog::error("No RTL-SDR devices found");
        return 1;
    }

    rtlsdr_dev_t* dev = nullptr;
    if (rtlsdr_open(&dev, cfg.deviceIndex) < 0 || !dev) {
        spdlog::error("Failed to open RTL-SDR device {}", cfg.deviceIndex);
        return 1;
    }
    spdlog::info("Opened RTL-SDR device {}", cfg.deviceIndex);

    if (cfg.listGains) {
        listTunerGains(dev);
        rtlsdr_close(dev);
        return 0;
    }

    if (!configureDevice(dev, cfg)) {
        rtlsdr_close(dev);
        return 1;
    }

    DebugStats debugStats(cfg.debugOn);

    std::ofstream iqDump;
    if (cfg.debugOn) {
        const auto iqFilename = makeTimestampedIqFilename();
        iqDump.open(iqFilename, std::ios::binary);
        if (!iqDump) {
            spdlog::error("Failed to open IQ dump file: {}", iqFilename);
            rtlsdr_close(dev);
            return 1;
        }
        spdlog::info("Debug mode enabled, saving raw RTL-SDR IQ to {}", iqFilename);
        if (cfg.captureIqOnly) {
            spdlog::info("Capture-only mode enabled; demodulation and audio output are disabled");
        }
    }

    WbfmDemodulator demod(cfg, cfg.debugOn ? &debugStats : nullptr);
    const int       outputChannels = demod.outputChannels();

    spdlog::info("Receiving WBFM at {:.3f} MHz, {} S/s -> {} Hz {}ch s16le, audio {}, offset tuning {}, stereo {}. Press Ctrl-C to stop.",
                 cfg.frequencyHz / 1000000.0,
                 kSdrSampleRate,
                 kAudioSampleRate,
                 outputChannels,
                 cfg.audioOn ? "on" : "off",
                 cfg.offsetTuning ? "on" : "off",
                 cfg.stereoOn ? "on" : "off");

    std::unique_ptr<AlsaAudioSink> audioSink;
    if (cfg.audioOn) {
        audioSink = std::make_unique<AlsaAudioSink>(outputChannels);
        if (!audioSink->isOpen()) {
            rtlsdr_close(dev);
            return 1;
        }
    }

    std::vector<std::uint8_t> iq(kReadBytes);
    std::vector<std::int16_t> pcm;
    std::vector<std::int16_t> audioPrebuffer;
    if (cfg.audioOn) { audioPrebuffer.reserve((kAudioPrebufferFrames + kAudioSampleRate / 10) * outputChannels); }
    const std::uint64_t       maxAudioFrames =
        (cfg.durationSec > 0.0) ? static_cast<std::uint64_t>(cfg.durationSec * kAudioSampleRate) : 0;
    const std::uint64_t       maxIqBytes =
        (cfg.captureIqOnly && cfg.durationSec > 0.0)
            ? static_cast<std::uint64_t>(cfg.durationSec * kSdrSampleRate * 2.0)
            : 0;
    std::uint64_t capturedIqBytes = 0;
    std::uint64_t writtenAudioFrames = 0;
    bool          audioStarted        = !cfg.audioOn;

    while (!gStop.load()) {
        int nRead = 0;
        int ret   = rtlsdr_read_sync(dev, iq.data(), static_cast<int>(iq.size()), &nRead);
        if (ret < 0) {
            spdlog::error("RTL-SDR read failed: {}", ret);
            break;
        }
        if (nRead <= 0) continue;
        if (cfg.debugOn) {
            iqDump.write(reinterpret_cast<const char*>(iq.data()), nRead);
            if (!iqDump) {
                spdlog::error("Failed to write IQ dump");
                break;
            }
            debugStats.addIqBytes(static_cast<std::size_t>(nRead));
            capturedIqBytes += static_cast<std::uint64_t>(nRead);
        }

        if (cfg.captureIqOnly) {
            debugStats.reportIfDue(kSdrSampleRate, kAudioSampleRate);
            if (maxIqBytes > 0 && capturedIqBytes >= maxIqBytes) break;
            continue;
        }

        demod.processIq(iq.data(), nRead, pcm);
        if (pcm.empty()) continue;
        if (cfg.debugOn) { debugStats.addPcm(pcm, outputChannels); }

        const auto pendingAudioFrames = cfg.audioOn && !audioStarted ? audioPrebuffer.size() / outputChannels : 0;
        const auto pcmFrames = pcm.size() / outputChannels;
        if (maxAudioFrames > 0 && writtenAudioFrames + pendingAudioFrames + pcmFrames > maxAudioFrames) {
            const auto allowedFrames =
                static_cast<std::size_t>(maxAudioFrames - writtenAudioFrames - pendingAudioFrames);
            pcm.resize(allowedFrames * outputChannels);
        }
        if (pcm.empty()) break;

        if (cfg.audioOn && !audioStarted) {
            audioPrebuffer.insert(audioPrebuffer.end(), pcm.begin(), pcm.end());
            const auto prebufferFrames = audioPrebuffer.size() / outputChannels;

            if (prebufferFrames < kAudioPrebufferFrames &&
                !(maxAudioFrames > 0 && writtenAudioFrames + prebufferFrames >= maxAudioFrames)) {
                continue;
            }

            spdlog::info("Starting ALSA playback after prebuffering {} frames", prebufferFrames);
            if (!audioSink->write(audioPrebuffer.data(), prebufferFrames)) break;
            writtenAudioFrames += prebufferFrames;
            audioPrebuffer.clear();
            audioStarted = true;

            if (maxAudioFrames > 0 && writtenAudioFrames >= maxAudioFrames) break;
            continue;
        }

        const auto framesToWrite = pcm.size() / outputChannels;
        if (cfg.audioOn) {
            if (!audioSink->write(pcm.data(), framesToWrite)) break;
        } else {
            std::cout.write(reinterpret_cast<const char*>(pcm.data()),
                            static_cast<std::streamsize>(pcm.size() * sizeof(std::int16_t)));
            if (!std::cout) break;
        }
        writtenAudioFrames += framesToWrite;

        if (cfg.debugOn) { debugStats.reportIfDue(kSdrSampleRate, kAudioSampleRate); }

        if (maxAudioFrames > 0 && writtenAudioFrames >= maxAudioFrames) break;
    }

    if (iqDump.is_open()) {
        iqDump.close();
        spdlog::info("Closed IQ dump file");
    }
    rtlsdr_close(dev);
    spdlog::info("Stopped FM broadcast player");
    return 0;
}
