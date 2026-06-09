#pragma once

#include <cstdint>
#include <string>
#include <nlohmann/json.hpp>

/**
 * @file aircraft.hpp
 * @brief Public ADS-B aircraft data model.
 */

namespace rtl::sda_b {

/**
 * @brief Latest decoded state for one ADS-B aircraft.
 *
 * @note Position fields are expressed in WGS84 degrees. Altitude is meters,
 * horizontal speed is meters per second, vertical speed is meters per second.
 */
struct Aircraft {
    std::string  uuid;                    /**< ICAO address in hexadecimal form. */
    std::string  flight;                  /**< Flight/callsign, if present in ADS-B messages. */
    double       lat             = 0;     /**< Latitude in degrees. */
    double       lon             = 0;     /**< Longitude in degrees. */
    double       alt             = 0;     /**< Altitude in meters. */
    double       yaw             = 0;     /**< Heading/track in degrees. */
    int          seq             = 0;     /**< Monotonic message count for this aircraft. */
    bool         onGround        = false; /**< Whether the aircraft reports ground movement. */
    double       horizontalSpeed = 0;     /**< Horizontal speed in meters per second. */
    double       vertSpeed       = 0;     /**< Vertical speed in meters per second. */
    std::int64_t lastTimeS       = 0;     /**< Last seen time as Unix seconds. */

    /**
     * @brief Convert this aircraft state to the JSON payload used by the data service.
     * @return JSON object containing position, speed, heading and metadata fields.
     */
    nlohmann::json toJson() const {
        nlohmann::json j;
        j["uuid"]             = uuid;
        j["flight"]           = flight;
        j["lng"]              = lon;
        j["lat"]              = lat;
        j["alt"]              = alt;
        j["yaw"]              = yaw;
        j["seq"]              = seq;
        j["on_ground"]        = onGround;
        j["horizontal_speed"] = horizontalSpeed;
        j["vert_speed"]       = vertSpeed;
        j["last_time_s"]      = lastTimeS;
        return j;
    }
};

} // namespace rtl::sda_b
