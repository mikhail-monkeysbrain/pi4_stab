#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace virtual_rangefinder {

inline double invalidHeight() {
    return std::numeric_limits<double>::quiet_NaN();
}

struct State {
    double baro_raw_m = invalidHeight();
    double lidar_anchor_m = invalidHeight();
    double baro_offset_m = invalidHeight();
    double baro_virtual_m = invalidHeight();

    bool baro_valid = false;
    bool offset_valid = false;
    bool virtual_valid = false;

    uint64_t sync_count = 0;

    void updateBaro(double altitude_m, bool valid) {
        baro_raw_m = altitude_m;
        baro_valid = valid && std::isfinite(altitude_m);
        updateVirtual();
    }

    bool synchronize(double lidar_m, bool lidar_valid) {
        offset_valid = false;

        if (!lidar_valid || !baro_valid ||
            !std::isfinite(lidar_m) || lidar_m <= 0.0) {
            updateVirtual();
            return false;
        }

        lidar_anchor_m = lidar_m;
        baro_offset_m = baro_raw_m - lidar_anchor_m;
        offset_valid = std::isfinite(baro_offset_m);

        if (offset_valid)
            ++sync_count;

        updateVirtual();
        return offset_valid;
    }

    void invalidateAnchor() {
        offset_valid = false;
        updateVirtual();
    }

    void updateVirtual() {
        baro_virtual_m = invalidHeight();
        virtual_valid = false;

        if (!baro_valid || !offset_valid)
            return;

        baro_virtual_m = baro_raw_m - baro_offset_m;
        virtual_valid =
            std::isfinite(baro_virtual_m) &&
            baro_virtual_m > 0.0;
    }
};

} // namespace virtual_rangefinder
