#include "ardupilotmega/mavlink.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>

using Clock = std::chrono::steady_clock;

int main() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5760);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    if (connect(fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
        perror("MAVLink connect");
        return 1;
    }

    FILE* out = fopen("/home/vio/baro_ekf_120s.csv", "w");
    if (!out) {
        perror("CSV");
        close(fd);
        return 1;
    }

    fprintf(out, "time_s,source,pressure_hpa,temperature_c,ekf_z_m,ekf_alt_m\n");
    setvbuf(out, nullptr, _IONBF, 0);

    mavlink_message_t msg{};
    mavlink_status_t status{};
    mavlink_message_t cmd{};
    uint8_t buffer[4096];
    uint8_t tx[300];

    bool requested = false;
    int baro_count = 0, ekf_count = 0;
    uint8_t fc_sys = 0;
    uint8_t fc_comp = 0;

    auto start = Clock::now();

    printf("Recording 120 seconds...\n");
    fflush(stdout);

    while (std::chrono::duration<double>(
               Clock::now() - start).count() < 120.0) {

        pollfd pfd{fd, POLLIN, 0};
        int pr = poll(&pfd, 1, 100);

        if (pr < 0) {
            perror("poll");
            break;
        }
        if (pr == 0) continue;

        ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
        if (n <= 0) {
            printf("MAVLink connection closed\n");
            break;
        }

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(
                    MAVLINK_COMM_0, buffer[i], &msg, &status))
                continue;

            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT &&
                msg.compid == MAV_COMP_ID_AUTOPILOT1 &&
                !requested) {

                fc_sys = msg.sysid;
                fc_comp = msg.compid;

                mavlink_msg_command_long_pack(
                    245, 190, &cmd,
                    fc_sys, fc_comp,
                    MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    MAVLINK_MSG_ID_SCALED_PRESSURE,
                    200000,
                    0, 0, 0, 0, 0
                );

                uint16_t len = mavlink_msg_to_send_buffer(tx, &cmd);
                send(fd, tx, len, 0);
                requested = true;

                printf("Requested SCALED_PRESSURE 5 Hz from FC %u\n",
                       fc_sys);
            }

            double t = std::chrono::duration<double>(
                Clock::now() - start).count();

            if (msg.msgid == MAVLINK_MSG_ID_SCALED_PRESSURE) {
                mavlink_scaled_pressure_t q{};
                mavlink_msg_scaled_pressure_decode(&msg, &q);

                fprintf(out, "%.6f,BARO,%.6f,%.3f,,\n",
                        t, q.press_abs, q.temperature / 100.0);
                ++baro_count;
            }

            if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                mavlink_local_position_ned_t q{};
                mavlink_msg_local_position_ned_decode(&msg, &q);

                fprintf(out, "%.6f,EKF,,,%.6f,%.6f\n",
                        t, q.z, -q.z);
                ++ekf_count;
            }
        }
    }

    fflush(out);
    fclose(out);
    close(fd);

    printf("\n===== RESULT =====\n");
    printf("BARO samples: %d\n", baro_count);
    printf("EKF samples:  %d\n", ekf_count);
    printf("CSV: /home/vio/baro_ekf_120s.csv\n");

    return (baro_count > 0 && ekf_count > 0) ? 0 : 2;
}
