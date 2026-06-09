#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include "tools/pusher.hpp"

/**
 * @file ads_b_demodulator.hpp
 * @brief ADS-B / Mode-S IQ demodulator and decoder.
 */

namespace rtl::sda_b {

struct ADSMessage;

/**
 * @brief Decode ADS-B/Mode-S messages from RTL-SDR unsigned 8-bit IQ buffers.
 *
 * @note Input IQ data is expected to be interleaved I/Q bytes centered around
 * 127, sampled at rtl::constants::ADSB_SAMPLE_RATE.
 */
class ADSBDemodulator {
public:
    /**
     * @brief Construct an ADS-B demodulator.
     * @param pusher Data pusher used for decoded aircraft updates. It must
     * outlive this demodulator.
     */
    explicit ADSBDemodulator(rtl::tools::Pusher& pusher);

    /** @brief Release internal buffers. */
    ~ADSBDemodulator();

    ADSBDemodulator(const ADSBDemodulator&)            = delete;
    ADSBDemodulator& operator=(const ADSBDemodulator&) = delete;

    /**
     * @brief Process one IQ buffer.
     * @param data Pointer to interleaved unsigned 8-bit IQ samples.
     * @param len Buffer length in bytes.
     * @note Passing nullptr or zero length is ignored. Buffers longer than the
     * internal block size are truncated.
     */
    void processIq(const std::uint8_t* data, std::uint32_t len);

    /**
     * @brief Remove aircraft not seen recently.
     * @param ttlSec Time-to-live in seconds.
     */
    void removeStaleAircrafts(int ttlSec = 60);

private:
    rtl::tools::Pusher& pusher_;

    std::unique_ptr<std::uint8_t[]>  data_;
    std::unique_ptr<std::uint16_t[]> magnitude_;
    std::unique_ptr<std::uint16_t[]> maglut_;
    std::uint32_t                    dataLen_ = 0;

    std::unique_ptr<std::uint32_t[]> icaoCache_;

    struct ErrorInfo {
        std::uint32_t syndrome;
        int           bits;
        int           pos[2];
    };

    static constexpr int NERRORINFO = 5778;
    ErrorInfo            bitErrorTable_[NERRORINFO];

    struct TrackedAircraft;
    std::unordered_map<std::uint32_t, TrackedAircraft> aircrafts_;

    double refLat_   = 0;
    double refLon_   = 0;
    int    refCount_ = 0;

    bool fixErrors_  = true;
    bool checkCrc_   = true;
    bool aggressive_ = false;

    void initBuffers();
    void initMagLut();
    void initErrorInfo();

    static std::uint32_t computeCrc(const std::uint8_t* msg, int bits);
    static std::uint32_t checksum(const std::uint8_t* msg, int bits);
    int                  fixBitErrors(std::uint8_t* msg, int bits, int maxfix, int* fixedbits);
    static int           compareErrorInfo(const void* a, const void* b);

    static std::uint32_t icaoCacheHash(std::uint32_t a);
    void                 addRecentlySeenIcao(std::uint32_t addr);
    bool                 wasIcaoRecentlySeen(std::uint32_t addr);

    void       decodeModesMessage(ADSMessage* mm, std::uint8_t* msg);
    static int messageLenByType(int type);
    static int decodeAc13Field(const std::uint8_t* msg, int* unit);
    static int decodeAc12Field(const std::uint8_t* msg, int* unit);
    int        bruteForceAp(std::uint8_t* msg, ADSMessage* mm);

    void                 computeMagnitudeVector();
    void                 detectModeS(std::uint16_t* m, std::uint32_t mlen);
    void                 applyPhaseCorrection(std::uint16_t* m, bool hasPreviousSample);
    static std::uint16_t scaleSample(std::uint16_t v, std::uint16_t scale);

    TrackedAircraft* findAircraft(std::uint32_t addr);
    TrackedAircraft* receiveData(ADSMessage* mm);
    void             useMessage(ADSMessage* mm);
    void             checkAndPush(TrackedAircraft* a);

    static int    cprModFunction(int a, int b);
    static int    cprNlFunction(double lat);
    static int    cprNFunction(double lat, int isodd);
    static double cprDlonFunction(double lat, int isodd);
    void          decodeCpr(TrackedAircraft* a);
    void          decodeCprSurface(TrackedAircraft* a, int fflag, int rawLat, int rawLon);
    static int    decodeMovementField(int movement);
};

} // namespace rtl::sda_b
