#include "ads_b/ads_b_demodulator.hpp"
#include "tools/pusher.hpp"
#include "ads_b/aircraft.hpp"
#include "tools/tools.hpp"

#include <cstring>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace rtl::sda_b {

static constexpr int MODES_PREAMBLE_US     = 8;   // Mode-S preamble duration in microseconds.
static constexpr int MODES_LONG_MSG_BITS   = 112; // Bit length of extended squitter / long Mode-S messages.
static constexpr int MODES_SHORT_MSG_BITS  = 56;  // Bit length of short Mode-S messages.
static constexpr int MODES_FULL_LEN        = MODES_PREAMBLE_US + MODES_LONG_MSG_BITS; // Preamble plus longest payload.
static constexpr int MODES_LONG_MSG_BYTES  = MODES_LONG_MSG_BITS / 8;                 // Byte count for a long message.
static constexpr int MODES_SHORT_MSG_BYTES = MODES_SHORT_MSG_BITS / 8;                // Byte count for a short message.
static constexpr int MODES_DATA_LEN        = 16 * 16384; // IQ bytes processed per ADS-B block.
static constexpr int MODES_ICAO_CACHE_LEN  = 1024;       // Recently seen ICAO cache size; must be a power of two.
static constexpr int MODES_ICAO_CACHE_TTL  = 60;         // Recently seen ICAO cache expiry in seconds.
static constexpr int MODES_MAX_BITERRORS   = 2;          // Maximum bit errors supported by the correction table.
static constexpr int MODES_UNIT_FEET       = 0;          // Altitude unit flag for feet.
static constexpr int MODES_UNIT_METERS     = 1;          // Altitude unit flag for meters.

struct ADSMessage {
    uint8_t  msg[MODES_LONG_MSG_BYTES]; // Raw decoded Mode-S message bytes.
    int      msgbits  = 0;              // Actual message length in bits, 56 or 112.
    int      msgtype  = 0;              // Downlink format (DF), stored from the first five bits.
    int      crcok    = 0;              // Non-zero when CRC or AP recovery validates the message.
    uint32_t crc      = 0;              // CRC syndrome / checksum remainder.
    int      errorbit = -1;             // First corrected bit position, or -1 if no correction was applied.
    int      aa1 = 0, aa2 = 0, aa3 = 0; // Three bytes of the ICAO aircraft address.
    int      phase_corrected = 0;       // Non-zero when sample phase correction was used.

    int ca  = 0; // Capability field for DF11/17/18 messages.
    int iid = 0; // Interrogator identifier recovered from DF11 all-call replies.

    int  metype           = 0;  // ADS-B message type code.
    int  mesub            = 0;  // ADS-B message subtype.
    int  heading_is_valid = 0;  // Non-zero when heading contains valid airborne velocity heading.
    int  heading          = 0;  // Heading or ground track in degrees.
    int  aircraft_type    = 0;  // Aircraft category decoded from identification messages.
    int  fflag            = 0;  // CPR odd/even frame flag.
    int  tflag            = 0;  // CPR time synchronization flag.
    int  rawLatitude      = 0;  // Encoded 17-bit CPR latitude.
    int  rawLongitude     = 0;  // Encoded 17-bit CPR longitude.
    char flight[9]        = {}; // Callsign, eight characters plus NUL terminator.
    int  ew_dir           = 0;  // East/west velocity sign bit.
    int  ew_velocity      = 0;  // East/west velocity component in knots.
    int  ns_dir           = 0;  // North/south velocity sign bit.
    int  ns_velocity      = 0;  // North/south velocity component in knots.
    int  vert_rate_source = 0;  // Vertical rate source bit.
    int  vert_rate_sign   = 0;  // Vertical rate sign bit.
    int  vert_rate        = 0;  // Encoded vertical rate value.
    int  velocity         = 0;  // Horizontal speed magnitude in knots.

    int movement           = 0; // Surface movement field.
    int movement_valid     = 0; // Non-zero when movement field is present.
    int ground_track       = 0; // Surface ground track in degrees.
    int ground_track_valid = 0; // Non-zero when ground track is present.

    int fs       = 0; // Flight status field.
    int dr       = 0; // Downlink request field.
    int um       = 0; // Utility message field.
    int identity = 0; // Mode-A squawk identity.

    int altitude = 0; // Decoded altitude in feet before conversion to public output.
    int unit     = 0; // Altitude unit flag, MODES_UNIT_FEET or MODES_UNIT_METERS.
};

struct ADSBDemodulator::TrackedAircraft {
    uint32_t  addr       = 0;                    // ICAO address as an integer.
    char      hexaddr[7] = {};                   // ICAO address formatted as six hex characters plus NUL.
    char      flight[9]  = {};                   // Latest decoded callsign.
    int       altitude   = 0;                    // Latest altitude in feet.
    int       speed      = 0;                    // Latest speed in knots.
    int       track      = 0;                    // Latest heading or ground track in degrees.
    time_t    seen       = 0;                    // Last time any valid message was received.
    long      messages   = 0;                    // Number of accepted messages for this aircraft.
    int       odd_cprlat = 0, odd_cprlon = 0;    // Last odd airborne CPR coordinates.
    int       even_cprlat = 0, even_cprlon = 0;  // Last even airborne CPR coordinates.
    double    lat = 0, lon = 0;                  // Decoded WGS84 position in degrees.
    long long odd_cprtime = 0, even_cprtime = 0; // Arrival time of odd/even CPR frames in milliseconds.
    int       vert_rate      = 0;                // Latest vertical rate in ADS-B units.
    uint64_t  vert_rate_time = 0;                // Timestamp of latest vertical rate update in milliseconds.
    int       on_ground      = 0;                // Non-zero when surface position messages indicate ground state.

    double   last_push_lat = 0, last_push_lon = 0; // Last pushed position, used to suppress duplicate output.
    int      last_push_altitude = 0;               // Last pushed altitude in feet.
    int      last_push_speed    = 0;               // Last pushed speed in knots.
    uint64_t last_push_time     = 0;               // Last push timestamp in milliseconds.
    char     last_flight[9]     = {};              // Last pushed callsign.
    int      last_track         = 0;               // Last pushed heading/track.
};

// Per-bit Mode-S CRC polynomial contribution table indexed by message bit position.
static uint32_t modes_checksum_table[112] = {
    0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178, 0x2c38bc, 0x161c5e, 0x0b0e2f,
    0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14, 0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936,
    0x4e5c9b, 0xd8d449, 0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22, 0x3f6d11,
    0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7, 0x91c77f, 0xb719bb, 0xa476d9, 0xadc168,
    0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612, 0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c,
    0x030ace, 0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53, 0xea04ad, 0x8af852,
    0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441, 0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00,
    0x383600, 0x1c1b00, 0x0e0d80, 0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000};

ADSBDemodulator::ADSBDemodulator(rtl::tools::Pusher& pusher)
    : pusher_(pusher) {
    initBuffers();
    initMagLut();
    initErrorInfo();
}

ADSBDemodulator::~ADSBDemodulator() = default;

void ADSBDemodulator::initBuffers() {
    dataLen_   = MODES_DATA_LEN + (MODES_FULL_LEN - 1) * 4;  // IQ bytes plus overlap from the previous block.
    data_      = std::make_unique<uint8_t[]>(dataLen_);      // Rolling unsigned IQ byte buffer.
    magnitude_ = std::make_unique<uint16_t[]>(dataLen_ / 2); // Magnitude samples derived from I/Q byte pairs.
    icaoCache_ = std::make_unique<uint32_t[]>(MODES_ICAO_CACHE_LEN * 2); // ICAO and timestamp pairs.

    std::memset(data_.get(), 127, dataLen_);
    std::memset(icaoCache_.get(), 0, sizeof(uint32_t) * MODES_ICAO_CACHE_LEN * 2);
}

void ADSBDemodulator::initMagLut() {
    maglut_ = std::make_unique<uint16_t[]>(129 * 129); // Lookup table for sqrt(I^2 + Q^2), indexed by abs(I), abs(Q).
    for (int i = 0; i <= 128; i++) {
        for (int q = 0; q <= 128; q++) {
            maglut_[i * 129 + q] = static_cast<uint16_t>(std::round(std::sqrt(i * i + q * q) * 360));
        }
    }
}

void ADSBDemodulator::initErrorInfo() {
    uint8_t msg[MODES_LONG_MSG_BYTES]; // Synthetic all-zero long message used to generate error syndromes.
    std::memset(msg, 0, MODES_LONG_MSG_BYTES);
    std::memset(bitErrorTable_, 0, sizeof(bitErrorTable_));
    int n = 0; // Number of populated entries in the bit error lookup table.

    for (int i = 5; i < MODES_LONG_MSG_BITS; i++) {
        int bp0       = (i >> 3);           // Byte position of the first synthetic flipped bit.
        int mask0     = 1 << (7 - (i & 7)); // Bit mask for the first synthetic flipped bit.
        msg[bp0]     ^= mask0;
        uint32_t crc  = checksum(msg, MODES_LONG_MSG_BITS); // Syndrome produced by this bit error pattern.

        bitErrorTable_[n].syndrome = crc;
        bitErrorTable_[n].bits     = 1;
        bitErrorTable_[n].pos[0]   = i;
        bitErrorTable_[n].pos[1]   = -1;
        n++;

        for (int j = i + 1; j < MODES_LONG_MSG_BITS; j++) {
            int bp1    = (j >> 3);           // Byte position of the second synthetic flipped bit.
            int mask1  = 1 << (7 - (j & 7)); // Bit mask for the second synthetic flipped bit.
            msg[bp1]  ^= mask1;
            crc        = checksum(msg, MODES_LONG_MSG_BITS);
            if (n >= NERRORINFO) break;
            bitErrorTable_[n].syndrome = crc;
            bitErrorTable_[n].bits     = 2;
            bitErrorTable_[n].pos[0]   = i;
            bitErrorTable_[n].pos[1]   = j;
            n++;
            msg[bp1] ^= mask1;
        }
        msg[bp0] ^= mask0;
    }
    qsort(bitErrorTable_, NERRORINFO, sizeof(ErrorInfo), compareErrorInfo);
}

void ADSBDemodulator::processIq(const uint8_t* data, uint32_t len) {
    if (!data_ || !data || len == 0) return;
    if (len > MODES_DATA_LEN) len = MODES_DATA_LEN;

    uint32_t tail_size = (MODES_FULL_LEN - 1) * 4; // Overlap retained to catch messages crossing block boundaries.
    std::memmove(data_.get(), data_.get() + MODES_DATA_LEN, tail_size);
    std::memcpy(data_.get() + tail_size, data, len);
    if (len < MODES_DATA_LEN) { std::memset(data_.get() + tail_size + len, 127, MODES_DATA_LEN - len); }

    computeMagnitudeVector();
    detectModeS(magnitude_.get(), dataLen_ / 2);
}

void ADSBDemodulator::removeStaleAircrafts(int ttl_sec) {
    time_t now = time(nullptr); // Current wall-clock time used for stale aircraft eviction.
    for (auto it = aircrafts_.begin(); it != aircrafts_.end();) {
        if ((now - it->second.seen) > ttl_sec) {
            it = aircrafts_.erase(it);
        } else {
            ++it;
        }
    }
}

uint32_t ADSBDemodulator::computeCrc(const uint8_t* msg, int bits) {
    uint32_t crc    = 0;                              // Accumulated CRC value.
    int      offset = (bits == 112) ? 0 : (112 - 56); // Table offset for short-message CRC calculation.
    for (int j = 0; j < bits - 24; j++) {
        int byte    = j / 8;          // Byte containing message bit j.
        int bit     = j % 8;          // Bit offset inside the byte.
        int bitmask = 1 << (7 - bit); // Mask for the current message bit.
        if (msg[byte] & bitmask) crc ^= modes_checksum_table[j + offset];
    }
    return crc & 0x00FFFFFF;
}

uint32_t ADSBDemodulator::checksum(const uint8_t* msg, int bits) {
    uint32_t crc = computeCrc(msg, bits); // Calculated CRC for message payload bits.
    uint32_t rem =                        // Remainder transmitted in the final 24 bits of the message.
        ((uint32_t)msg[(bits / 8) - 3] << 16) | ((uint32_t)msg[(bits / 8) - 2] << 8) | (uint32_t)msg[(bits / 8) - 1];
    return (crc ^ rem) & 0x00FFFFFF;
}

int ADSBDemodulator::compareErrorInfo(const void* a, const void* b) {
    const auto* ea = (const ErrorInfo*)a;
    const auto* eb = (const ErrorInfo*)b;
    if (ea->syndrome < eb->syndrome) return -1;
    if (ea->syndrome > eb->syndrome) return 1;
    return 0;
}

int ADSBDemodulator::fixBitErrors(uint8_t* msg, int bits, int maxfix, int* fixedbits) {
    ErrorInfo ei{};                    // Search key containing the message syndrome.
    ei.syndrome = checksum(msg, bits); // Syndrome to match against known bit error patterns.
    auto* pei   = (ErrorInfo*)bsearch(&ei, bitErrorTable_, NERRORINFO, sizeof(ErrorInfo), compareErrorInfo);
    if (!pei || pei->bits > maxfix) return 0;

    int offset = MODES_LONG_MSG_BITS - bits; // Difference between table bit positions and actual message length.
    for (int i = 0; i < pei->bits; i++) {
        int bitpos = pei->pos[i] - offset; // Corrected bit position in the current message.
        if (bitpos < 0 || bitpos >= bits) return 0;
    }

    int res = 0; // Number of corrected bits.
    for (int i = 0; i < pei->bits; i++) {
        int bitpos        = pei->pos[i] - offset; // Corrected bit position in the current message.
        msg[bitpos >> 3] ^= (1 << (7 - (bitpos & 7)));
        if (fixedbits)
            fixedbits[res++] = bitpos;
        else
            res++;
    }
    return res;
}

uint32_t ADSBDemodulator::icaoCacheHash(uint32_t a) {
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a);
    return a & (MODES_ICAO_CACHE_LEN - 1);
}

void ADSBDemodulator::addRecentlySeenIcao(uint32_t addr) {
    uint32_t h            = icaoCacheHash(addr); // Cache slot for this ICAO address.
    icaoCache_[h * 2]     = addr;
    icaoCache_[h * 2 + 1] = (uint32_t)time(nullptr);
}

bool ADSBDemodulator::wasIcaoRecentlySeen(uint32_t addr) {
    uint32_t h = icaoCacheHash(addr);   // Cache slot for this ICAO address.
    uint32_t a = icaoCache_[h * 2];     // Cached ICAO address.
    uint32_t t = icaoCache_[h * 2 + 1]; // Cached timestamp in seconds.
    return a && a == addr && time(nullptr) - t <= MODES_ICAO_CACHE_TTL;
}

int ADSBDemodulator::messageLenByType(int type) {
    if (type == 16 || type == 17 || type == 18 || type == 19 || type == 20 || type == 21) return MODES_LONG_MSG_BITS;
    return MODES_SHORT_MSG_BITS;
}

int ADSBDemodulator::decodeAc13Field(const uint8_t* msg, int* unit) {
    int m_bit = msg[3] & (1 << 6); // M bit: metric altitude encoding indicator.
    int q_bit = msg[3] & (1 << 4); // Q bit: 25-foot altitude quantization indicator.
    if (!m_bit && q_bit) {
        *unit = MODES_UNIT_FEET;
        int n = ((msg[2] & 31) << 6) | ((msg[3] & 0x80) >> 2) | ((msg[3] & 0x20) >> 1)
              | (msg[3] & 15); // Encoded altitude value before scale and offset.
        return n * 25 - 1000;
    }
    *unit = MODES_UNIT_FEET;
    return 0;
}

int ADSBDemodulator::decodeAc12Field(const uint8_t* msg, int* unit) {
    if (msg[5] & 1) {
        *unit = MODES_UNIT_FEET;
        int n = ((msg[5] >> 1) << 4) | ((msg[6] & 0xF0) >> 4); // Encoded ADS-B altitude value.
        return n * 25 - 1000;
    }
    *unit = MODES_UNIT_FEET;
    return 0;
}

int ADSBDemodulator::bruteForceAp(uint8_t* msg, ADSMessage* mm) {
    int msgtype = mm->msgtype; // Downlink format used to decide whether AP recovery is valid.
    int msgbits = mm->msgbits; // Message length in bits.
    if (msgtype == 0 || msgtype == 4 || msgtype == 5 || msgtype == 16 || msgtype == 20 || msgtype == 21
        || msgtype == 24) {
        uint8_t aux[MODES_LONG_MSG_BYTES]; // Working copy used to recover address parity without mutating msg.
        std::memcpy(aux, msg, msgbits / 8);
        uint32_t crc       = computeCrc(aux, msgbits); // CRC used to remove address parity from the last three bytes.
        int      lastbyte  = (msgbits / 8) - 1;        // Index of the last message byte.
        aux[lastbyte]     ^= crc & 0xff;
        aux[lastbyte - 1] ^= (crc >> 8) & 0xff;
        aux[lastbyte - 2] ^= (crc >> 16) & 0xff;
        uint32_t addr =
            aux[lastbyte] | (aux[lastbyte - 1] << 8) | (aux[lastbyte - 2] << 16); // Recovered ICAO candidate.
        if (wasIcaoRecentlySeen(addr)) {
            mm->aa1 = aux[lastbyte - 2];
            mm->aa2 = aux[lastbyte - 1];
            mm->aa3 = aux[lastbyte];
            return 1;
        }
    }
    return 0;
}

void ADSBDemodulator::decodeModesMessage(ADSMessage* mm, uint8_t* msg) {
    static const char ais_charset[] =
        "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????"; // Callsign character lookup table.

    std::memcpy(mm->msg, msg, MODES_LONG_MSG_BYTES);
    msg = mm->msg;

    mm->msgtype = msg[0] >> 3;
    mm->msgbits = messageLenByType(mm->msgtype);
    mm->crc     = checksum(msg, mm->msgbits);

    mm->errorbit = -1;
    mm->iid      = 0;
    mm->crcok    = (mm->crc == 0);

    if (!mm->crcok && (mm->msgtype == 11 || mm->msgtype == 17 || mm->msgtype == 18)) {
        int maxfix = aggressive_ ? MODES_MAX_BITERRORS : 1; // Maximum number of bit flips allowed for this decode.
        int fixedbits[MODES_MAX_BITERRORS];                 // Positions corrected by fixBitErrors().
        int nfixed = fixBitErrors(msg, mm->msgbits, maxfix, fixedbits); // Number of corrected bits.
        if (nfixed > 0) {
            mm->crc      = checksum(msg, mm->msgbits);
            mm->crcok    = (mm->crc == 0);
            mm->errorbit = fixedbits[0];
        }
    }

    mm->ca     = msg[0] & 7;
    mm->aa1    = msg[1];
    mm->aa2    = msg[2];
    mm->aa3    = msg[3];
    mm->metype = msg[4] >> 3;
    mm->mesub  = msg[4] & 7;
    mm->fs     = msg[0] & 7;
    mm->dr     = msg[1] >> 3 & 31;
    mm->um     = ((msg[1] & 7) << 3) | msg[2] >> 5;

    {
        int a = ((msg[3] & 0x80) >> 5) | ((msg[2] & 0x02) >> 0) | ((msg[2] & 0x08) >> 3); // First octal squawk digit.
        int b = ((msg[3] & 0x02) << 1) | ((msg[3] & 0x08) >> 2) | ((msg[3] & 0x20) >> 5); // Second octal squawk digit.
        int c = ((msg[2] & 0x01) << 2) | ((msg[2] & 0x04) >> 1) | ((msg[2] & 0x10) >> 4); // Third octal squawk digit.
        int d = ((msg[3] & 0x01) << 2) | ((msg[3] & 0x04) >> 1) | ((msg[3] & 0x10) >> 4); // Fourth octal squawk digit.
        mm->identity = a * 1000 + b * 100 + c * 10 + d;
    }

    if (mm->msgtype != 11 && mm->msgtype != 17 && mm->msgtype != 18) {
        if (bruteForceAp(msg, mm)) {
            mm->crcok = 1;
        } else {
            mm->crcok = 0;
        }
    } else {
        uint32_t addr = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3; // ICAO address from address bytes.
        if (mm->crcok && mm->errorbit == -1) addRecentlySeenIcao(addr);
        if (mm->msgtype == 11 && !mm->crcok && mm->crc < 80) {
            if (wasIcaoRecentlySeen(addr)) {
                mm->iid   = mm->crc;
                mm->crcok = 1;
            }
        }
    }

    if (mm->msgtype == 0 || mm->msgtype == 4 || mm->msgtype == 16 || mm->msgtype == 20)
        mm->altitude = decodeAc13Field(msg, &mm->unit);

    if (mm->msgtype == 17 || mm->msgtype == 18) {
        if (mm->metype >= 1 && mm->metype <= 4) {
            mm->aircraft_type = mm->metype - 1;
            mm->flight[0]     = ais_charset[msg[5] >> 2];
            mm->flight[1]     = ais_charset[((msg[5] & 3) << 4) | (msg[6] >> 4)];
            mm->flight[2]     = ais_charset[((msg[6] & 15) << 2) | (msg[7] >> 6)];
            mm->flight[3]     = ais_charset[msg[7] & 63];
            mm->flight[4]     = ais_charset[msg[8] >> 2];
            mm->flight[5]     = ais_charset[((msg[8] & 3) << 4) | (msg[9] >> 4)];
            mm->flight[6]     = ais_charset[((msg[9] & 15) << 2) | (msg[10] >> 6)];
            mm->flight[7]     = ais_charset[msg[10] & 63];
            mm->flight[8]     = '\0';
        } else if (mm->metype >= 5 && mm->metype <= 8) {
            mm->movement           = ((msg[4] & 0x07) << 4) | (msg[5] >> 4);
            mm->movement_valid     = (mm->movement != 0);
            mm->ground_track_valid = (msg[5] >> 3) & 1;
            mm->ground_track       = ((msg[5] & 0x07) << 4) | (msg[6] >> 4);
            mm->ground_track       = mm->ground_track * 360 / 128;
            if (mm->ground_track > 360) mm->ground_track -= 360;
            mm->fflag        = (msg[6] >> 2) & 1;
            mm->tflag        = (msg[6] >> 3) & 1;
            mm->rawLatitude  = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1);
            mm->rawLongitude = ((msg[8] & 1) << 16) | (msg[9] << 8) | msg[10];
        } else if (mm->metype >= 9 && mm->metype <= 18) {
            mm->fflag        = msg[6] & (1 << 2);
            mm->tflag        = msg[6] & (1 << 3);
            mm->altitude     = decodeAc12Field(msg, &mm->unit);
            mm->rawLatitude  = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1);
            mm->rawLongitude = ((msg[8] & 1) << 16) | (msg[9] << 8) | msg[10];
        } else if (mm->metype == 19 && mm->mesub >= 1 && mm->mesub <= 4) {
            if (mm->mesub == 1 || mm->mesub == 2) {
                mm->ew_dir           = (msg[5] & 4) >> 2;
                mm->ew_velocity      = ((msg[5] & 3) << 8) | msg[6];
                mm->ns_dir           = (msg[7] & 0x80) >> 7;
                mm->ns_velocity      = ((msg[7] & 0x7f) << 3) | ((msg[8] & 0xe0) >> 5);
                mm->vert_rate_source = (msg[8] & 0x10) >> 4;
                mm->vert_rate_sign   = (msg[8] & 0x8) >> 3;
                mm->vert_rate        = ((msg[8] & 7) << 6) | ((msg[9] & 0xfc) >> 2);
                mm->velocity =
                    (int)std::sqrt((double)(mm->ns_velocity * mm->ns_velocity + mm->ew_velocity * mm->ew_velocity));
                if (mm->velocity) {
                    int ewv = mm->ew_velocity, nsv = mm->ns_velocity; // Signed velocity components used for heading.
                    if (mm->ew_dir) ewv *= -1;
                    if (mm->ns_dir) nsv *= -1;
                    double heading = std::atan2((double)ewv, (double)nsv) * 360 / (M_PI * 2); // Heading in degrees.
                    if (heading < 0) heading += 360;
                    mm->heading = (int)heading;
                }
            } else if (mm->mesub == 3 || mm->mesub == 4) {
                mm->heading_is_valid = msg[5] & (1 << 2);
                mm->heading          = (int)(360.0 / 128.) * (((msg[5] & 3) << 5) | (msg[6] >> 3));
            }
        }
    }
    mm->phase_corrected = 0;
}

void ADSBDemodulator::computeMagnitudeVector() {
    auto* m = magnitude_.get(); // Destination magnitude vector.
    auto* p = data_.get();      // Source interleaved unsigned IQ byte buffer.
    for (uint32_t j = 0; j < dataLen_; j += 2) {
        int i = p[j] - 127;     // Signed I sample centered around zero.
        int q = p[j + 1] - 127; // Signed Q sample centered around zero.
        if (i < 0) i = -i;
        if (q < 0) q = -q;
        m[j / 2] = maglut_[i * 129 + q];
    }
}

uint16_t ADSBDemodulator::scaleSample(uint16_t v, uint16_t scale) {
    uint32_t result = (uint32_t)v * scale / 16384; // Scaled magnitude before uint16 saturation.
    return (result > 65535) ? 65535 : (uint16_t)result;
}

void ADSBDemodulator::applyPhaseCorrection(uint16_t* m, bool hasPreviousSample) {
    if (!m || !hasPreviousSample) return;

    uint32_t onTime = m[0] + m[2] + m[7] + m[9]; // Energy on expected preamble pulse samples.
    uint32_t early  = (m[-1] + m[6]) * 2;        // Energy suggesting samples are early.
    uint32_t late   = (m[3] + m[10]) * 2;        // Energy suggesting samples are late.

    if (early > late) {
        if (early + onTime == 0) return;
        uint16_t scaleUp   = 16384 + (uint16_t)(16384 * early / (early + onTime)); // Gain for corrected samples.
        uint16_t scaleDown = 16384 - (uint16_t)(16384 * early / (early + onTime)); // Attenuation for opposite samples.
        m[MODES_PREAMBLE_US * 2 + MODES_LONG_MSG_BITS * 2 - 1] =
            scaleSample(m[MODES_PREAMBLE_US * 2 + MODES_LONG_MSG_BITS * 2 - 1], scaleUp);
        for (int j = MODES_PREAMBLE_US * 2 + MODES_LONG_MSG_BITS * 2 - 2; j > MODES_PREAMBLE_US * 2; j -= 2) {
            if (m[j] > m[j + 1])
                m[j - 1] = scaleSample(m[j - 1], scaleDown);
            else
                m[j - 1] = scaleSample(m[j - 1], scaleUp);
        }
    } else {
        if (late + onTime == 0) return;
        uint16_t scaleUp   = 16384 + (uint16_t)(16384 * late / (late + onTime)); // Gain for corrected samples.
        uint16_t scaleDown = 16384 - (uint16_t)(16384 * late / (late + onTime)); // Attenuation for opposite samples.
        m[MODES_PREAMBLE_US * 2] = scaleSample(m[MODES_PREAMBLE_US * 2], scaleUp);
        for (int j = MODES_PREAMBLE_US * 2; j < MODES_PREAMBLE_US * 2 + MODES_LONG_MSG_BITS * 2 - 2; j += 2) {
            if (m[j] > m[j + 1])
                m[j + 2] = scaleSample(m[j + 2], scaleUp);
            else
                m[j + 2] = scaleSample(m[j + 2], scaleDown);
        }
    }
}

void ADSBDemodulator::detectModeS(uint16_t* m, uint32_t mlen) {
    uint8_t  bits[MODES_LONG_MSG_BITS];    // Soft-decoded bits, where 2 marks an ambiguous bit.
    uint8_t  msg[MODES_LONG_MSG_BYTES];    // Packed decoded message bytes.
    uint16_t aux[MODES_LONG_MSG_BITS * 2]; // Backup samples restored after phase correction trial.
    int      use_correction = 0;           // Non-zero to retry the current preamble with phase correction.

    for (uint32_t j = 0; j < mlen - MODES_FULL_LEN * 2; j++) {
        int good_message = 0; // Non-zero when the candidate decoded and passed CRC.
        int high         = 0; // Preamble high-pulse reference level.

        if (use_correction) goto good_preamble;

        if (!(m[j] > m[j + 1] && m[j + 1] < m[j + 2] && m[j + 2] > m[j + 3] && m[j + 3] < m[j] && m[j + 4] < m[j]
              && m[j + 5] < m[j] && m[j + 6] < m[j] && m[j + 7] > m[j + 8] && m[j + 8] < m[j + 9]
              && m[j + 9] > m[j + 6]))
            continue;

        high = (m[j] + m[j + 2] + m[j + 7] + m[j + 9]) / 6;
        if (m[j + 4] >= high || m[j + 5] >= high) continue;
        if (m[j + 11] >= high || m[j + 12] >= high || m[j + 13] >= high || m[j + 14] >= high) continue;

good_preamble:
        if (use_correction) {
            std::memcpy(aux, m + j + MODES_PREAMBLE_US * 2, sizeof(aux));
            applyPhaseCorrection(m + j, j > 0);
        }

        int errors = 0; // Number of ambiguous bits in the short-message portion.
        for (int i = 0; i < MODES_LONG_MSG_BITS * 2; i += 2) {
            int low   = m[j + i + MODES_PREAMBLE_US * 2];     // First half-bit magnitude.
            int high2 = m[j + i + MODES_PREAMBLE_US * 2 + 1]; // Second half-bit magnitude.
            int delta = low - high2;                          // Absolute distance between half-bit magnitudes.
            if (delta < 0) delta = -delta;
            if (i > 0 && delta < 256) {
                bits[i / 2] = bits[i / 2 - 1];
            } else if (low == high2) {
                bits[i / 2] = 2;
                if (i < MODES_SHORT_MSG_BITS * 2) errors++;
            } else if (low > high2) {
                bits[i / 2] = 1;
            } else {
                bits[i / 2] = 0;
            }
        }

        if (use_correction) std::memcpy(m + j + MODES_PREAMBLE_US * 2, aux, sizeof(aux));

        for (int i = 0; i < MODES_LONG_MSG_BITS; i += 8) {
            msg[i / 8] = bits[i + 0] << 7 | bits[i + 1] << 6 | bits[i + 2] << 5 | bits[i + 3] << 4 | bits[i + 4] << 3
                       | bits[i + 5] << 2 | bits[i + 6] << 1 | bits[i + 7];
        }

        int msgtype = msg[0] >> 3;                   // Downlink format decoded from the candidate message.
        int msglen  = messageLenByType(msgtype) / 8; // Candidate message length in bytes.

        int delta = 0; // Average half-bit magnitude separation used to reject low-confidence candidates.
        for (int i = 0; i < msglen * 8 * 2; i += 2)
            delta += abs(m[j + i + MODES_PREAMBLE_US * 2] - m[j + i + MODES_PREAMBLE_US * 2 + 1]);
        delta /= msglen * 4;
        if (delta < 10 * 255) {
            use_correction = 0;
            continue;
        }

        if (errors == 0 || (aggressive_ && errors < 3)) {
            ADSMessage mm{}; // Parsed candidate message.
            decodeModesMessage(&mm, msg);
            if (mm.crcok) {
                j            += (MODES_PREAMBLE_US + msglen * 8) * 2;
                good_message  = 1;
                if (use_correction) mm.phase_corrected = 1;
            }
            useMessage(&mm);
        }

        if (!good_message && !use_correction) {
            j--;
            use_correction = 1;
        } else {
            use_correction = 0;
        }
    }
}

ADSBDemodulator::TrackedAircraft* ADSBDemodulator::findAircraft(uint32_t addr) {
    auto it = aircrafts_.find(addr); // Iterator for tracked aircraft lookup by ICAO address.
    return (it != aircrafts_.end()) ? &it->second : nullptr;
}

ADSBDemodulator::TrackedAircraft* ADSBDemodulator::receiveData(ADSMessage* mm) {
    if (checkCrc_ && mm->crcok == 0) return nullptr;
    uint32_t addr = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3; // ICAO address from parsed message bytes.

    auto [it, inserted] = aircrafts_.emplace(addr, TrackedAircraft{}); // Existing or newly inserted track entry.
    auto* a             = &it->second;                                 // Mutable tracked aircraft state.
    if (inserted) {
        a->addr = addr;
        std::snprintf(a->hexaddr, sizeof(a->hexaddr), "%06x", (int)addr);
        a->seen = time(nullptr);
    }

    a->seen = time(nullptr);
    a->messages++;

    if (mm->msgtype == 0 || mm->msgtype == 4 || mm->msgtype == 20) {
        a->altitude = mm->altitude;
    } else if (mm->msgtype == 17 || mm->msgtype == 18) {
        if (mm->metype >= 1 && mm->metype <= 4) {
            std::memcpy(a->flight, mm->flight, sizeof(a->flight));
        } else if (mm->metype >= 9 && mm->metype <= 18) {
            a->altitude  = mm->altitude;
            a->on_ground = 0;
            if (mm->fflag) {
                a->odd_cprlat  = mm->rawLatitude;
                a->odd_cprlon  = mm->rawLongitude;
                a->odd_cprtime = rtl::tools::getTimestamp("ms");
            } else {
                a->even_cprlat  = mm->rawLatitude;
                a->even_cprlon  = mm->rawLongitude;
                a->even_cprtime = rtl::tools::getTimestamp("ms");
            }
            if (llabs(a->even_cprtime - a->odd_cprtime) <= 10000) {
                double prev_lat = a->lat, prev_lon = a->lon; // Position before CPR decode, used to detect updates.
                decodeCpr(a);
                if (a->lat != prev_lat || a->lon != prev_lon) {
                    if (refCount_ == 0) {
                        refLat_ = a->lat;
                        refLon_ = a->lon;
                    } else {
                        refLat_ += (a->lat - refLat_) / (refCount_ + 1);
                        refLon_ += (a->lon - refLon_) / (refCount_ + 1);
                    }
                    if (refCount_ < 10000) refCount_++;
                }
            }
        } else if (mm->metype >= 5 && mm->metype <= 8) {
            if (refCount_) {
                if (mm->ground_track_valid) a->track = mm->ground_track;
                if (mm->movement_valid) a->speed = decodeMovementField(mm->movement);
                a->altitude  = 0;
                a->on_ground = 1;
                decodeCprSurface(a, mm->fflag, mm->rawLatitude, mm->rawLongitude);
            }
        } else if (mm->metype == 19) {
            if (mm->mesub == 1 || mm->mesub == 2) {
                a->speed = mm->velocity;
                a->track = mm->heading;
                if (mm->vert_rate != 0) {
                    a->vert_rate      = mm->vert_rate_sign ? -mm->vert_rate : mm->vert_rate;
                    a->vert_rate_time = rtl::tools::getTimestamp("ms");
                }
            }
        }
    }
    return a;
}

void ADSBDemodulator::useMessage(ADSMessage* mm) {
    auto* a = receiveData(mm); // Tracked aircraft updated by this message, or nullptr if rejected.
    if (a) {
        spdlog::info(
            "ADS-B: {} flt={} alt={}ft spd={}kts lat={} lon={} msg={}",
            a->hexaddr,
            a->flight[0] ? a->flight : "-",
            a->altitude,
            a->speed,
            a->lat,
            a->lon,
            a->messages);
        checkAndPush(a);
    }
}

void ADSBDemodulator::checkAndPush(TrackedAircraft* a) {
    if (!a || a->lat < 1 || a->lon < 1) return;

    int64_t now_ms    = rtl::tools::getTimestamp("ms"); // Current timestamp for rate-limiting pushes.
    int     need_push = 0;                              // Non-zero when aircraft state changed enough to publish.
    if (std::fabs(a->lat - a->last_push_lat) > 0.0001 || std::fabs(a->lon - a->last_push_lon) > 0.0001) need_push = 1;
    if (a->altitude != a->last_push_altitude) need_push = 1;
    if (std::strncmp(a->flight, a->last_flight, sizeof(a->flight)) != 0) need_push = 1;
    if (a->speed != a->last_push_speed) need_push = 1;
    if (a->track != a->last_track) need_push = 1;
    if (now_ms - a->last_push_time > 1000) need_push = 1;
    if (!need_push) return;

    a->last_push_lat      = a->lat;
    a->last_push_lon      = a->lon;
    a->last_push_altitude = a->altitude;
    a->last_push_speed    = a->speed;
    a->last_push_time     = now_ms;
    std::strncpy(a->last_flight, a->flight, sizeof(a->last_flight) - 1);
    a->last_flight[sizeof(a->last_flight) - 1] = '\0';
    a->last_track                              = a->track;

    rtl::sda_b::Aircraft aircraft; // Public output DTO converted to SI units where applicable.
    aircraft.uuid            = a->hexaddr;
    aircraft.flight          = rtl::tools::trim(std::string(a->flight, strnlen(a->flight, sizeof(a->flight))));
    aircraft.lon             = a->lon;
    aircraft.lat             = a->lat;
    aircraft.alt             = a->altitude * 0.3048;
    aircraft.yaw             = a->track;
    aircraft.seq             = a->messages;
    aircraft.onGround        = a->on_ground;
    aircraft.horizontalSpeed = a->speed * 0.514444;
    aircraft.vertSpeed       = a->vert_rate * 0.00508;
    aircraft.lastTimeS       = a->seen;

    pusher_.pushData(aircraft);
}

int ADSBDemodulator::cprModFunction(int a, int b) {
    int res = a % b; // Positive modulo result required by CPR equations.
    if (res < 0) res += b;
    return res;
}

int ADSBDemodulator::cprNlFunction(double lat) {
    if (lat < 0) lat = -lat;
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    return 1;
}

int ADSBDemodulator::cprNFunction(double lat, int isodd) {
    int nl = cprNlFunction(lat) - isodd; // Longitude zone count adjusted for odd/even CPR frame.
    return (nl < 1) ? 1 : nl;
}

double ADSBDemodulator::cprDlonFunction(double lat, int isodd) {
    return 360.0 / cprNFunction(lat, isodd);
}

void ADSBDemodulator::decodeCpr(TrackedAircraft* a) {
    const double AirDlat0 = 360.0 / 60;                       // Even airborne CPR latitude zone size.
    const double AirDlat1 = 360.0 / 59;                       // Odd airborne CPR latitude zone size.
    double       lat0 = a->even_cprlat, lat1 = a->odd_cprlat; // Encoded even/odd CPR latitude values.
    double       lon0 = a->even_cprlon, lon1 = a->odd_cprlon; // Encoded even/odd CPR longitude values.

    int    j     = (int)std::floor(((59 * lat0 - 60 * lat1) / 131072.0) + 0.5); // Global CPR latitude index.
    double rlat0 = AirDlat0 * (cprModFunction(j, 60) + lat0 / 131072.0);        // Even-frame resolved latitude.
    double rlat1 = AirDlat1 * (cprModFunction(j, 59) + lat1 / 131072.0);        // Odd-frame resolved latitude.
    if (rlat0 >= 270) rlat0 -= 360;
    if (rlat1 >= 270) rlat1 -= 360;

    if (cprNlFunction(rlat0) != cprNlFunction(rlat1)) return;

    if (a->even_cprtime > a->odd_cprtime) {
        int ni = cprNFunction(rlat0, 0); // Longitude zone count for the newer even frame.
        int m =
            (int)std::floor((((lon0 * (cprNlFunction(rlat0) - 1)) - (lon1 * cprNlFunction(rlat0))) / 131072.0) + 0.5);
        // m is the global CPR longitude index for the even frame.
        a->lon = cprDlonFunction(rlat0, 0) * (cprModFunction(m, ni) + lon0 / 131072.0);
        a->lat = rlat0;
    } else {
        int ni = cprNFunction(rlat1, 1); // Longitude zone count for the newer odd frame.
        int m =
            (int)std::floor((((lon0 * (cprNlFunction(rlat1) - 1)) - (lon1 * cprNlFunction(rlat1))) / 131072.0) + 0.5);
        // m is the global CPR longitude index for the odd frame.
        a->lon = cprDlonFunction(rlat1, 1) * (cprModFunction(m, ni) + lon1 / 131072.0);
        a->lat = rlat1;
    }
    if (a->lon > 180) a->lon -= 360;
}

void ADSBDemodulator::decodeCprSurface(TrackedAircraft* a, int fflag, int rawLat, int rawLon) {
    if (refCount_ == 0) return;
    const double SurfDlat0 = 90.0 / 60;                     // Even surface CPR latitude zone size.
    const double SurfDlat1 = 90.0 / 59;                     // Odd surface CPR latitude zone size.
    double       dlat      = fflag ? SurfDlat1 : SurfDlat0; // Latitude zone size selected by frame flag.

    int    j   = (int)std::floor(refLat_ / dlat) // Local surface CPR latitude index near the reference position.
               + (int)std::floor(0.5 + cprModFunction((int)refLat_, (int)dlat) / dlat - (double)rawLat / 131072.0);
    double lat = dlat * (j + (double)rawLat / 131072.0); // Locally resolved surface latitude.

    if (std::fabs(lat - refLat_) > 45) {
        if (lat > refLat_)
            lat -= 90;
        else
            lat += 90;
    }
    if (lat < -90 || lat > 90) return;

    int ni = cprNFunction(lat, fflag); // Longitude zone count for resolved surface latitude.
    if (ni == 0) ni = 1;
    int    m   = (int)std::floor(refLon_ / (90.0 / ni)) // Local surface CPR longitude index.
               + (int)std::floor(
                     0.5 + cprModFunction((int)refLon_, (int)(90.0 / ni)) / (90.0 / ni) - (double)rawLon / 131072.0);
    double lon = (90.0 / ni) * (m + (double)rawLon / 131072.0); // Locally resolved surface longitude.

    while (lon > refLon_ + 45) lon -= 90;
    while (lon < refLon_ - 45) lon += 90;
    if (lon > 180) lon -= 360;
    if (lon < -180) lon += 360;

    a->lat = lat;
    a->lon = lon;
}

int ADSBDemodulator::decodeMovementField(int movement) {
    if (movement == 0) return -1;
    if (movement == 1) return 0;
    if (movement <= 8) return (movement - 2) * 0.125 + 0.125;
    if (movement <= 12) return (movement - 9) * 0.25 + 1;
    if (movement <= 38) return (movement - 13) * 0.5 + 2;
    if (movement <= 93) return (movement - 39) + 15;
    if (movement <= 108) return (movement - 94) * 2 + 70;
    if (movement <= 123) return (movement - 109) * 5 + 100;
    return 175;
}

} // namespace rtl::sda_b
