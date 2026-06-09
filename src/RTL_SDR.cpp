#include <iostream>
#include <cstdlib>
#include <cmath>
#include "core/application.hpp"
#include "constants.hpp"
#include "tools/tools.hpp"

namespace {

rtl::core::AppConfig parseArgs(int argc, char* argv[]) {
    rtl::core::AppConfig config;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "RTL-SDR Scanner v2.0\n"
                      << "====================\n\n"
                      << "Usage: " << argv[0] << " [options]\n\n"
                      << "Options:\n"
                      << "  --mode <1|2|3>        1=ADS-B only, 2=Scan only, 3=Both\n"
                      << "  --start-freq <MHz>    Scan start frequency (10-1070 MHz)\n"
                      << "  --end-freq <MHz>      Scan end frequency (10-1070 MHz)\n"
                      << "  --adsb                Enable ADS-B decoding\n"
                      << "  --scan                Enable frequency scanning\n"
                      << "  --help, -h            Show this help message\n\n"
                      << "Frequency range: 10 MHz - 1070 MHz\n"
                      << "Maximum bandwidth: 100 MHz\n"
                      << "Data output: 127.0.0.1:23568\n"
                      << "Control port: 127.0.0.1:23569\n";
            exit(0);
        } else if (arg == "--mode" && i + 1 < argc) {
            int mode = std::atoi(argv[++i]);
            if (mode == 1)
                config.adsbEnabled = true;
            else if (mode == 2)
                config.scanEnabled = true;
            else if (mode == 3) {
                config.adsbEnabled = true;
                config.scanEnabled = true;
            }
        } else if (arg == "--start-freq" && i + 1 < argc) {
            config.startFreqHz = std::atof(argv[++i]) * 1e6;
        } else if (arg == "--end-freq" && i + 1 < argc) {
            config.endFreqHz = std::atof(argv[++i]) * 1e6;
        } else if (arg == "--adsb") {
            config.adsbEnabled = true;
        } else if (arg == "--scan") {
            config.scanEnabled = true;
        }
    }
    return config;
}

rtl::core::AppConfig interactiveMenu() {
    rtl::core::AppConfig config;

    std::cout << "\nRTL-SDR Scanner v2.0\n"
              << "====================\n"
              << "1. ADS-B Decode Only\n"
              << "2. Scan Only\n"
              << "3. ADS-B + Scan\n"
              << "4. Exit\n"
              << "\nSelect mode: ";

    int mode;
    std::cin >> mode;

    if (mode == 1) {
        config.adsbEnabled = true;
    } else if (mode == 2) {
        config.scanEnabled = true;
    } else if (mode == 3) {
        config.adsbEnabled = true;
        config.scanEnabled = true;
    } else {
        exit(0);
    }

    if (config.scanEnabled) {
        std::cout << "\nFrequency Configuration\n"
                  << "=======================\n"
                  << "Range: 10 MHz - 1070 MHz, Max bandwidth: 100 MHz\n";

        double startMhz, endMhz;
        while (true) {
            std::cout << "Enter start frequency (MHz): ";
            std::cin >> startMhz;
            if (startMhz < 10.0 || startMhz > 1070.0) {
                std::cout << "Error: Start frequency must be between 10 and 1070 MHz\n";
                continue;
            }
            break;
        }

        while (true) {
            std::cout << "Enter end frequency (MHz): ";
            std::cin >> endMhz;
            if (endMhz < 10.0 || endMhz > 1070.0) {
                std::cout << "Error: End frequency must be between 10 and 1070 MHz\n";
                continue;
            }
            if (endMhz <= startMhz) {
                std::cout << "Error: End frequency must be greater than start frequency\n";
                continue;
            }
            if ((endMhz - startMhz) > 100.0) {
                std::cout << "Error: Bandwidth cannot exceed 100 MHz\n";
                continue;
            }
            break;
        }

        config.startFreqHz = startMhz * 1e6;
        config.endFreqHz   = endMhz * 1e6;
    }

    return config;
}

bool validateConfig(const rtl::core::AppConfig& config) {
    if (!config.adsbEnabled && !config.scanEnabled) {
        std::cerr << "Error: No functionality selected\n";
        return false;
    }
    if (config.scanEnabled) {
        if (!std::isfinite(config.startFreqHz) || !std::isfinite(config.endFreqHz)) {
            std::cerr << "Error: Frequency must be finite\n";
            return false;
        }
        if (config.startFreqHz < rtl::constants::MIN_FREQ || config.startFreqHz > rtl::constants::MAX_FREQ) {
            std::cerr << "Error: Start frequency out of range\n";
            return false;
        }
        if (config.endFreqHz < rtl::constants::MIN_FREQ || config.endFreqHz > rtl::constants::MAX_FREQ) {
            std::cerr << "Error: End frequency out of range\n";
            return false;
        }
        if (config.endFreqHz <= config.startFreqHz) {
            std::cerr << "Error: End frequency must be greater than start frequency\n";
            return false;
        }
        if ((config.endFreqHz - config.startFreqHz) > rtl::constants::MAX_BANDWIDTH) {
            std::cerr << "Error: Bandwidth exceeds 100 MHz limit\n";
            return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    rtl::tools::initSpdlog();

    rtl::core::AppConfig config;
    if (argc > 1) {
        config = parseArgs(argc, argv);
    } else {
        config = interactiveMenu();
    }

    if (!validateConfig(config)) return 1;

    rtl::core::Application app(config);
    return app.run();
}
