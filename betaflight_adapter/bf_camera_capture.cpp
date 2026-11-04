#include "bf_camera.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    const std::string device =
        argc > 1 ? argv[1] : "/dev/video0";

    try {
        bf_camera::Camera camera;
        camera.open(device, 640, 480, 60);

        std::cout << "CAMERA_OPEN_OK"
                  << " fps=" << camera.actualFps()
                  << std::endl;

        int received = 0;
        int timeouts = 0;
        uint64_t missing = 0;
        int64_t previous_ns = 0;
        uint32_t previous_seq = 0;

        double min_dt = 1e9;
        double max_dt = 0;
        double sum_dt = 0;
        int dt_count = 0;

        const auto start = std::chrono::steady_clock::now();

        while (std::chrono::steady_clock::now() - start <
               std::chrono::seconds(10)) {

            bf_camera::Frame frame;

            if (!camera.read(frame, 1000)) {
                ++timeouts;
                continue;
            }

            if (previous_ns > 0) {
                const double dt =
                    (frame.timestamp_ns - previous_ns) * 1e-6;

                min_dt = std::min(min_dt, dt);
                max_dt = std::max(max_dt, dt);
                sum_dt += dt;
                ++dt_count;

                if (frame.sequence > previous_seq + 1)
                    missing += frame.sequence - previous_seq - 1;
            }

            previous_ns = frame.timestamp_ns;
            previous_seq = frame.sequence;

            // BF_TIMESTAMP_DEBUG_V1
            if (received < 5) {
                std::cout
                    << "FRAME_TIMESTAMP"
                    << " seq=" << frame.sequence
                    << " ns=" << frame.timestamp_ns
                    << std::endl;
            }

            ++received;
        }

        std::cout << "CAMERA_RESULT"
                  << " frames=" << received
                  << " missing=" << missing
                  << " timeouts=" << timeouts
                  << " min_dt_ms=" << (dt_count ? min_dt : 0)
                  << " mean_dt_ms="
                  << (dt_count ? sum_dt / dt_count : 0)
                  << " max_dt_ms=" << max_dt
                  << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "CAMERA_ERROR: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
