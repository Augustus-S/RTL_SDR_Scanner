#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace rtl::scanner {

enum class ScanProfile : int {
    FAST = 0,
    BALANCED = 1,
    ACCURATE = 2
};

enum class ResetPolicy : int {
    ADAPTIVE = 0,
    ALWAYS = 1
};

struct ScanProfileConfig {
    ScanProfile   profile              = ScanProfile::BALANCED;
    const char*   name                 = "balanced";
    std::uint32_t sampleRateHz         = 2400000;
    std::uint32_t stepHz               = 1800000;
    int           tuneSettleMs         = 20;
    int           maxFmIqVerifications = 12;
    int           maxAmIqVerifications = 12;
};

inline std::uint32_t defaultSampleRateForProfile(ScanProfile profile) {
    return profile == ScanProfile::FAST ? 3200000 : 2400000;
}

inline std::uint32_t stepForProfile(ScanProfile profile, std::uint32_t sampleRateHz) {
    switch (profile) {
    case ScanProfile::FAST:
        return sampleRateHz;
    case ScanProfile::ACCURATE:
        return sampleRateHz / 2;
    case ScanProfile::BALANCED:
    default:
        return (sampleRateHz * 3) / 4;
    }
}

inline ScanProfileConfig configForProfile(ScanProfile profile, std::uint32_t sampleRateHz = 0) {
    if (sampleRateHz == 0) sampleRateHz = defaultSampleRateForProfile(profile);
    switch (profile) {
    case ScanProfile::FAST:
        return {ScanProfile::FAST, "fast", sampleRateHz, stepForProfile(profile, sampleRateHz), 12, 6, 6};
    case ScanProfile::ACCURATE:
        return {ScanProfile::ACCURATE, "accurate", sampleRateHz, stepForProfile(profile, sampleRateHz), 50, 24, 24};
    case ScanProfile::BALANCED:
    default:
        return {ScanProfile::BALANCED, "balanced", sampleRateHz, stepForProfile(profile, sampleRateHz), 20, 12, 12};
    }
}

inline const char* scanProfileName(ScanProfile profile) {
    return configForProfile(profile).name;
}

inline bool parseScanProfile(std::string_view value, ScanProfile& out) {
    if (value == "fast") {
        out = ScanProfile::FAST;
        return true;
    }
    if (value == "balanced") {
        out = ScanProfile::BALANCED;
        return true;
    }
    if (value == "accurate") {
        out = ScanProfile::ACCURATE;
        return true;
    }
    return false;
}

inline std::string scanProfileChoices() {
    return "fast|balanced|accurate";
}

inline const char* resetPolicyName(ResetPolicy policy) {
    switch (policy) {
    case ResetPolicy::ALWAYS:
        return "always";
    case ResetPolicy::ADAPTIVE:
    default:
        return "adaptive";
    }
}

inline bool parseResetPolicy(std::string_view value, ResetPolicy& out) {
    if (value == "adaptive") {
        out = ResetPolicy::ADAPTIVE;
        return true;
    }
    if (value == "always") {
        out = ResetPolicy::ALWAYS;
        return true;
    }
    return false;
}

inline std::string resetPolicyChoices() {
    return "adaptive|always";
}

inline bool isSupportedScanSampleRate(std::uint32_t sampleRateHz) {
    return sampleRateHz == 2000000 || sampleRateHz == 2400000 || sampleRateHz == 3200000;
}

} // namespace rtl::scanner
