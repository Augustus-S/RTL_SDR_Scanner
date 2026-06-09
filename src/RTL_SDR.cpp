#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <complex>
#include <algorithm>
#include <string>
#include <atomic>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <rtl-sdr.h>
#include <spdlog/spdlog.h>
#include "ads_b/ads_b_demodulator.hpp"
#include "constants.hpp"
#include "scanner/persistent_async_reader.hpp"
#include "tools/pusher.hpp"
#include "tools/tools.hpp"

namespace constants = rtl::constants;
using rtl::scanner::PersistentAsyncReader;
using rtl::scanner::SegmentData;
using rtl::sda_b::ADSBDemodulator;
using rtl::tools::Pusher;

static std::atomic<bool>     g_running{true};
static std::atomic<int>      g_max_gain{0};
static std::atomic<bool>     adsb_enabled{false};
static std::atomic<bool>     scan_enabled{false};
static std::atomic<uint32_t> start_freq{constants::MIN_FREQ};
static std::atomic<uint32_t> end_freq{static_cast<uint32_t>(100e6)};
static rtlsdr_dev_t*         g_dev = nullptr;

static void signal_handler(int) {
    g_running = false;
}

struct ProgramConfig {
    bool   adsb_enabled = false;
    bool   scan_enabled = false;
    double start_freq   = 10e6;
    double end_freq     = 100e6;
};

static void print_help(const char* prog_name) {
    std::cout << "RTL-SDR Scanner v2.0\n"
              << "====================\n\n"
              << "Usage: " << prog_name << " [options]\n\n"
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
}

static ProgramConfig parse_args(int argc, char* argv[]) {
    ProgramConfig config;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_help(argv[0]);
            exit(0);
        } else if (arg == "--mode" && i + 1 < argc) {
            int mode = std::atoi(argv[++i]);
            if (mode == 1)
                config.adsb_enabled = true;
            else if (mode == 2)
                config.scan_enabled = true;
            else if (mode == 3) {
                config.adsb_enabled = true;
                config.scan_enabled = true;
            }
        } else if (arg == "--start-freq" && i + 1 < argc) {
            config.start_freq = std::atof(argv[++i]) * 1e6;
        } else if (arg == "--end-freq" && i + 1 < argc) {
            config.end_freq = std::atof(argv[++i]) * 1e6;
        } else if (arg == "--adsb") {
            config.adsb_enabled = true;
        } else if (arg == "--scan") {
            config.scan_enabled = true;
        }
    }
    return config;
}

static ProgramConfig interactive_menu() {
    ProgramConfig config;

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
        config.adsb_enabled = true;
    } else if (mode == 2) {
        config.scan_enabled = true;
    } else if (mode == 3) {
        config.adsb_enabled = true;
        config.scan_enabled = true;
    } else {
        exit(0);
    }

    if (config.scan_enabled) {
        std::cout << "\nFrequency Configuration\n"
                  << "=======================\n"
                  << "Range: 10 MHz - 1070 MHz, Max bandwidth: 100 MHz\n";

        double start_mhz, end_mhz;
        while (true) {
            std::cout << "Enter start frequency (MHz): ";
            std::cin >> start_mhz;
            if (start_mhz < 10.0 || start_mhz > 1070.0) {
                std::cout << "Error: Start frequency must be between 10 and 1070 MHz\n";
                continue;
            }
            break;
        }

        while (true) {
            std::cout << "Enter end frequency (MHz): ";
            std::cin >> end_mhz;
            if (end_mhz < 10.0 || end_mhz > 1070.0) {
                std::cout << "Error: End frequency must be between 10 and 1070 MHz\n";
                continue;
            }
            if (end_mhz <= start_mhz) {
                std::cout << "Error: End frequency must be greater than start frequency\n";
                continue;
            }
            if ((end_mhz - start_mhz) > 100.0) {
                std::cout << "Error: Bandwidth cannot exceed 100 MHz\n";
                continue;
            }
            break;
        }

        config.start_freq = start_mhz * 1e6;
        config.end_freq   = end_mhz * 1e6;
    }

    return config;
}

static bool validate_config(const ProgramConfig& config) {
    if (!config.adsb_enabled && !config.scan_enabled) {
        spdlog::error("No functionality selected");
        return false;
    }

    if (config.scan_enabled) {
        if (!std::isfinite(config.start_freq) || !std::isfinite(config.end_freq)) {
            spdlog::error("Frequency must be finite");
            return false;
        }
        if (config.start_freq < constants::MIN_FREQ || config.start_freq > constants::MAX_FREQ) {
            spdlog::error("Start frequency out of range: {} MHz", config.start_freq / 1e6);
            return false;
        }
        if (config.end_freq < constants::MIN_FREQ || config.end_freq > constants::MAX_FREQ) {
            spdlog::error("End frequency out of range: {} MHz", config.end_freq / 1e6);
            return false;
        }
        if (config.end_freq <= config.start_freq) {
            spdlog::error("End frequency must be greater than start frequency");
            return false;
        }
        if ((config.end_freq - config.start_freq) > constants::MAX_BANDWIDTH) {
            spdlog::error("Bandwidth exceeds 100 MHz limit");
            return false;
        }
    }
    return true;
}

static void send_event(const std::string& event_type, const std::string& msg, bool scan_open) {
    nlohmann::json event_json;
    event_json["id"]    = 200;
    event_json["event"] = event_type;
    nlohmann::json data_obj;
    if (event_type == "scan_heartbeat") {
        int dev_state = 0;
        if (g_running.load()) {
            dev_state |= 4;
            if (scan_open) { dev_state |= 2; }
        }
        data_obj["dev_state"] = dev_state;
    } else {
        data_obj["msg"] = msg;
    }
    event_json["data"] = data_obj;
    rtl::tools::post(std::string(constants::DATA_URL), event_json.dump(), 1);
}

static void adsb_async_callback(unsigned char* buf, uint32_t len, void* ctx) {
    auto* server = static_cast<ADSBDemodulator*>(ctx);
    server->processIq(buf, len);
}

int main(int argc, char* argv[]) {
    rtl::tools::initSpdlog();

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    ProgramConfig config;
    if (argc > 1) {
        config = parse_args(argc, argv);
    } else {
        config = interactive_menu();
    }

    if (!validate_config(config)) { return 1; }

    adsb_enabled.store(config.adsb_enabled);
    scan_enabled.store(config.scan_enabled);
    start_freq.store(static_cast<uint32_t>(config.start_freq));
    end_freq.store(static_cast<uint32_t>(config.end_freq));

    spdlog::info("ADS-B: {}", adsb_enabled.load() ? "enabled" : "disabled");
    spdlog::info("Scan: {}", scan_enabled.load() ? "enabled" : "disabled");
    if (scan_enabled.load()) {
        spdlog::info("Scan range: {:.1f} - {:.1f} MHz", start_freq.load() / 1e6, end_freq.load() / 1e6);
    }

    rtlsdr_dev_t* dev = nullptr;

    int device_count = rtlsdr_get_device_count();
    if (device_count == 0) {
        spdlog::error("No RTL-SDR devices found");
        send_event("scan_events", "RTL_SDR device not found.", false);
        return 1;
    }

    if (rtlsdr_open(&dev, 0) < 0) {
        spdlog::error("Failed to open RTL-SDR device");
        send_event("scan_events", "RTL_SDR device not found.", false);
        return 1;
    }
    g_dev = dev;

    {
        int gains[100] = {};
        int num_gains  = rtlsdr_get_tuner_gains(dev, gains);
        if (num_gains > 0) {
            int max_gain = gains[num_gains - 1];
            g_max_gain.store(max_gain);
            rtlsdr_set_tuner_gain_mode(dev, 1);
            rtlsdr_set_tuner_gain(dev, max_gain);
            spdlog::info("Max available gain: {} dB", max_gain / 10.0);
        }
    }

    rtlsdr_set_sample_rate(dev, constants::SCAN_SAMPLE_RATE);

    rtlsdr_reset_buffer(dev);
    {
        std::vector<uint8_t> dummy_buf(16384);
        int                  dummy_len = 0;
        for (int i = 0; i < 100; ++i) { rtlsdr_read_sync(dev, dummy_buf.data(), 16384, &dummy_len); }
    }
    spdlog::info("Device stabilized, starting...");

    Pusher          scan_pusher(constants::DATA_URL);
    ADSBDemodulator adsb_demod(scan_pusher);

    std::thread radio_worker([&]() {
        std::unique_ptr<PersistentAsyncReader> reader;
        std::vector<uint8_t>                   buffer_u8(constants::BUFFER_LEN);
        std::vector<std::complex<short>>       buffer_iq(constants::BUFFER_LEN / 2);
        std::vector<short>                     buffer_q(constants::BUFFER_LEN / 2);

        auto ensure_reader = [&]() {
            if (!reader) {
                rtlsdr_set_sample_rate(dev, constants::SCAN_SAMPLE_RATE);
                reader = std::make_unique<PersistentAsyncReader>(dev);
            }
        };

        auto stop_reader = [&]() {
            if (reader) {
                reader->shutdown();
                reader.reset();
            }
        };

        auto run_adsb_slice = [&](std::chrono::milliseconds max_duration) {
            stop_reader();
            rtlsdr_set_sample_rate(dev, constants::ADSB_SAMPLE_RATE);
            rtlsdr_set_direct_sampling(dev, 0);
            rtlsdr_set_tuner_gain_mode(dev, 1);
            rtlsdr_set_tuner_gain(dev, g_max_gain.load());
            rtlsdr_set_center_freq(dev, constants::ADSB_FREQ);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            rtlsdr_reset_buffer(dev);

            std::atomic<int> adsb_async_ret{0};
            std::thread      adsb_thread([&]() {
                int ret = rtlsdr_read_async(dev, adsb_async_callback, &adsb_demod, 12, constants::ADSB_READ_LEN);
                adsb_async_ret.store(ret);
            });

            auto adsb_start = std::chrono::steady_clock::now();
            while (g_running && adsb_enabled.load()) {
                if (max_duration.count() > 0 && std::chrono::steady_clock::now() - adsb_start >= max_duration) break;
                if (max_duration.count() == 0 && scan_enabled.load()) break;
                if (adsb_async_ret.load() != 0) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (adsb_async_ret.load() < 0) {
                spdlog::error("rtlsdr_read_async failed: {}", adsb_async_ret.load());
                g_running = false;
            } else if (adsb_async_ret.load() == 0) {
                rtlsdr_cancel_async(dev);
            }
            if (adsb_thread.joinable()) adsb_thread.join();
        };

        while (g_running) {
            if (!scan_enabled.load()) {
                if (adsb_enabled.load()) {
                    spdlog::info("Entering continuous ADS-B mode");
                    run_adsb_slice(std::chrono::milliseconds(0));
                } else {
                    stop_reader();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }

            ensure_reader();
            uint32_t sweep_start_freq = start_freq.load();
            uint32_t sweep_end_freq   = end_freq.load();

            auto                     sweep_start = std::chrono::steady_clock::now();
            std::vector<SegmentData> segments;

            for (uint32_t cur_freq  = sweep_start_freq; cur_freq <= sweep_end_freq && g_running;
                 cur_freq          += constants::STEP_FREQ) {
                if (!scan_enabled.load()) break;
                int need_ds = (cur_freq < constants::LOW_FREQ_THRESHOLD) ? 2 : 0;

                uint32_t n_read = 4 * constants::FFT_SIZE * 2;
                auto     scan_result =
                    reader->read(buffer_u8.data(), &n_read, cur_freq, need_ds, constants::READ_TIMEOUT_MS);

                if (scan_result != PersistentAsyncReader::ReadResult::SUCCESS) {
                    if (scan_result == PersistentAsyncReader::ReadResult::DEVICE_ERROR) {
                        spdlog::error("Device error @ {} MHz, aborting", cur_freq / 1e6);
                        send_event("scan_events", "Device error @ " + std::to_string(cur_freq / 1e6) + " MHz", true);
                        g_running = false;
                        break;
                    }
                    spdlog::warn("Read timeout @ {} MHz", cur_freq / 1e6);
                    continue;
                }

                int samples = static_cast<int>(n_read / 2);
                samples     = std::min<int>(samples, static_cast<int>(buffer_iq.size()));
                for (int i = 0; i < samples; ++i) {
                    short I      = static_cast<short>(buffer_u8[2 * i]) - 127;
                    short Q      = static_cast<short>(buffer_u8[2 * i + 1]) - 127;
                    buffer_iq[i] = std::complex<short>(I, Q);
                    buffer_q[i]  = Q;
                }

                rtl::tools::removeDc(buffer_iq.data(), samples);

                auto [fft_power_sum, groups_num] =
                    rtl::tools::calculateFft(buffer_iq.data(), samples, constants::SCAN_SAMPLE_RATE);

                double rssi;
                if (cur_freq < constants::LOW_FREQ_THRESHOLD) {
                    rssi = rtl::tools::calculateRssiDirectSampling(buffer_q.data(), samples);
                } else {
                    rssi = rtl::tools::calculateRssi(fft_power_sum, groups_num);
                }

                spdlog::info("Scan Freq: {} MHz, RSSI: {} dBFS", cur_freq / 1e6, rssi);
                std::vector<double> spectrum_db = rtl::tools::spectrumToDb(fft_power_sum, groups_num);
                rtl::tools::suppressDcSpike(spectrum_db);
                segments.push_back({std::move(spectrum_db), static_cast<double>(cur_freq)});
            }

            if (!g_running) break;

            if (!segments.empty()) {
                auto [spliced_spectrum, spliced_freqs] = rtl::tools::spliceSpectrum(
                    segments,
                    static_cast<double>(constants::SCAN_SAMPLE_RATE),
                    static_cast<double>(sweep_start_freq),
                    static_cast<double>(sweep_end_freq));

                rtl::tools::suppressPeriodicSpurs(
                    spliced_spectrum, static_cast<double>(sweep_start_freq), static_cast<double>(sweep_end_freq));

                double max_val = *std::max_element(spliced_spectrum.begin(), spliced_spectrum.end());
                double min_val = *std::min_element(spliced_spectrum.begin(), spliced_spectrum.end());

                nlohmann::json data_obj;
                data_obj["start_freq"] = sweep_start_freq;
                data_obj["end_freq"]   = sweep_end_freq;
                data_obj["max_value"]  = max_val;
                data_obj["min_value"]  = min_val;
                data_obj["data"]       = spliced_spectrum;

                nlohmann::json json_data;
                json_data["id"]    = 200;
                json_data["event"] = "scan_data";
                json_data["data"]  = data_obj;

                scan_pusher.pushScanHop(std::move(json_data));

                auto sweep_end     = std::chrono::steady_clock::now();
                auto sweep_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(sweep_end - sweep_start);
                spdlog::info(
                    "Sweep complete, {} bins pushed, took {} ms", spliced_spectrum.size(), sweep_elapsed.count());
            }

            if (g_running && scan_enabled.load() && adsb_enabled.load()) {
                spdlog::info("Decoding ADS-B signal (time-slice)...");
                run_adsb_slice(std::chrono::seconds(10));
            }
        }

        stop_reader();
    });

    std::thread heartbeat_sender([&]() {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (!g_running) break;
            send_event("scan_heartbeat", "", scan_enabled.load());
        }
    });

    httplib::Server srv;

    srv.Post("/adsb/start", [&](const httplib::Request& req, httplib::Response& res) {
        adsb_enabled.store(true);
        res.set_content(R"({"status":"ok","adsb":"started"})", "application/json");
    });

    srv.Post("/adsb/stop", [&](const httplib::Request& req, httplib::Response& res) {
        adsb_enabled.store(false);
        res.set_content(R"({"status":"ok","adsb":"stopped"})", "application/json");
    });

    srv.Post("/scan/start", [&](const httplib::Request& req, httplib::Response& res) {
        scan_enabled.store(true);
        res.set_content(R"({"status":"ok","scan":"started"})", "application/json");
    });

    srv.Post("/scan/stop", [&](const httplib::Request& req, httplib::Response& res) {
        scan_enabled.store(false);
        res.set_content(R"({"status":"ok","scan":"stopped"})", "application/json");
    });

    srv.Post("/scan/param", [&](const httplib::Request& req, httplib::Response& res) {
        spdlog::info("配置扫频: {}", req.body);
        try {
            auto   obj = nlohmann::json::parse(req.body);
            double sf  = obj.contains("start_freq") ? obj["start_freq"].get<double>() : start_freq.load();
            double ef  = obj.contains("end_freq") ? obj["end_freq"].get<double>() : end_freq.load();

            if (!std::isfinite(sf) || !std::isfinite(ef)) {
                res.status = 400;
                res.set_content(R"({"status":"error","msg":"Frequency must be finite"})", "application/json");
                return;
            }
            if (sf < constants::MIN_FREQ || sf > constants::MAX_FREQ || ef < constants::MIN_FREQ
                || ef > constants::MAX_FREQ) {
                res.status = 400;
                res.set_content(R"({"status":"error","msg":"Frequency out of range"})", "application/json");
                return;
            }
            if (ef <= sf) {
                res.status = 400;
                res.set_content(R"({"status":"error","msg":"Invalid frequency range"})", "application/json");
                return;
            }
            if ((ef - sf) > constants::MAX_BANDWIDTH) {
                res.status = 400;
                res.set_content(R"({"status":"error","msg":"Bandwidth exceeds 100 MHz"})", "application/json");
                return;
            }

            start_freq.store(static_cast<uint32_t>(sf));
            end_freq.store(static_cast<uint32_t>(ef));

            nlohmann::json res_obj;
            res_obj["status"] = "ok";
            res.set_content(res_obj.dump(4), "application/json");

            spdlog::info("scan_param updated: {} - {} MHz", sf / 1e6, ef / 1e6);
        } catch (const std::exception& e) {
            spdlog::error("scan_param handler error: {}", e.what());
            nlohmann::json res_obj;
            res_obj["status"] = "error";
            res_obj["msg"]    = e.what();
            res.status        = 400;
            res.set_content(res_obj.dump(4), "application/json");
        }
    });

    srv.Get("/status", [&](const httplib::Request& req, httplib::Response& res) {
        nlohmann::json status;
        status["adsb_enabled"] = adsb_enabled.load();
        status["scan_enabled"] = scan_enabled.load();
        status["start_freq"]   = start_freq.load();
        status["end_freq"]     = end_freq.load();
        res.set_content(status.dump(2), "application/json");
    });

    std::thread server_thread([&srv]() {
        if (!srv.listen("0.0.0.0", constants::CONTROL_PORT)) {
            spdlog::error("HTTP server failed to bind 0.0.0.0:{}", constants::CONTROL_PORT);
            g_running = false;
        }
    });

    spdlog::info("HTTP control server started on port {}", constants::CONTROL_PORT);

    while (g_running) { std::this_thread::sleep_for(std::chrono::milliseconds(500)); }

    g_running = false;
    rtlsdr_cancel_async(dev);
    if (radio_worker.joinable()) radio_worker.join();
    if (heartbeat_sender.joinable()) heartbeat_sender.join();
    srv.stop();
    if (server_thread.joinable()) server_thread.join();

    g_dev = nullptr;
    rtlsdr_close(dev);

    spdlog::info("Stopped.");

    return 0;
}
