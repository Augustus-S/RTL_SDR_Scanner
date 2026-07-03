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

struct ScanProfileConfig {
    ScanProfile   profile              = ScanProfile::BALANCED;
    const char*   name                 = "balanced";
    std::uint32_t stepHz               = 1500000;
    int           tuneSettleMs         = 20;
    int           maxFmIqVerifications = 12;
    int           maxAmIqVerifications = 12;
};

inline ScanProfileConfig configForProfile(ScanProfile profile) {
    switch (profile) {
    case ScanProfile::FAST:
        return {ScanProfile::FAST, "fast", 2000000, 12, 6, 6};
    case ScanProfile::ACCURATE:
        return {ScanProfile::ACCURATE, "accurate", 1000000, 50, 24, 24};
    case ScanProfile::BALANCED:
    default:
        return {ScanProfile::BALANCED, "balanced", 1500000, 20, 12, 12};
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

} // namespace rtl::scanner
