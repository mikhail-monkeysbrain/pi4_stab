#include <gpiod.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

static constexpr unsigned GPIO = 12;
static constexpr uint64_t BIT_NS = 1000000000ULL / 9600;
static constexpr uint64_t CAPTURE_NS = 400000000ULL;

static int run(const char *cmd) {
    int status = std::system(cmd);
    return status != -1 && WIFEXITED(status)
        ? WEXITSTATUS(status) : -1;
}

struct Restore {
    ~Restore() {
        if (run("pinctrl set 12 a4") != 0)
            std::cerr << "WARNING: GPIO12 TXD5 restore failed\n";
    }
};

static std::array<uint8_t,16> make_packet(char command) {
    std::array<uint8_t,16> p{};
    p[0] = 0x0F;
    p[1] = static_cast<uint8_t>(command);
    unsigned sum = 0;
    for (int i = 1; i < 14; ++i) sum += p[i];
    p[14] = sum & 0xFF;
    return p;
}

static bool valid(const std::array<uint8_t,16>& p) {
    if (p[0] != 0x0F || p[15] != 0) return false;
    unsigned sum = 0;
    for (int i = 1; i < 14; ++i) sum += p[i];
    return p[14] == (sum & 0xFF);
}

static uint16_t u16(const std::array<uint8_t,16>& p, int i) {
    return uint16_t(p[i]) | (uint16_t(p[i+1]) << 8);
}

struct Edge {
    uint64_t t;
    int level;
};

static int level_at(const std::vector<Edge>& edges,
                    uint64_t t) {
    int level = 1;
    for (const auto& e : edges) {
        if (e.t > t) break;
        level = e.level;
    }
    return level;
}

static std::vector<uint8_t>
decode(const std::vector<Edge>& edges) {
    std::vector<uint8_t> out;
    uint64_t last_end = 0;

    for (const auto& e : edges) {
        if (e.level != 0 || e.t < last_end) continue;

        // UART start bit: линия должна оставаться LOW
        // в середине стартового бита.
        if (level_at(edges, e.t + BIT_NS / 2) != 0)
            continue;

        uint8_t value = 0;
        for (int bit = 0; bit < 8; ++bit) {
            uint64_t sample =
                e.t + BIT_NS * (3 + 2 * bit) / 2;
            if (level_at(edges, sample))
                value |= uint8_t(1U << bit);
        }

        uint64_t stop_t = e.t + BIT_NS * 19 / 2;
        if (level_at(edges, stop_t) != 1)
            continue;

        out.push_back(value);
        last_end = e.t + BIT_NS * 10;
    }
    return out;
}

static void print_frames(const std::vector<uint8_t>& bytes) {
    std::cout << "Decoded bytes: " << bytes.size() << "\nRAW:";
    for (auto b : bytes) printf(" %02X", b);
    std::cout << "\n";

    int good = 0;
    for (size_t i = 0; i + 16 <= bytes.size(); ++i) {
        if (bytes[i] != 0x0F) continue;

        std::array<uint8_t,16> p{};
        std::copy_n(bytes.begin() + i, 16, p.begin());

        if (!valid(p)) continue;
        ++good;

        std::cout << "VALID IRC TRAMP RESPONSE: "
                  << char(p[1]) << "\n";

        if (p[1] == 'r') {
            std::cout << "  Min frequency: " << u16(p,2)
                      << " MHz\n";
            std::cout << "  Max frequency: " << u16(p,4)
                      << " MHz\n";
            std::cout << "  Max power: " << u16(p,6)
                      << " mW\n";
        } else if (p[1] == 'v') {
            std::cout << "  Frequency: " << u16(p,2)
                      << " MHz\n";
            std::cout << "  Configured power: " << u16(p,4)
                      << " mW\n";
            std::cout << "  Control mode: " << unsigned(p[6])
                      << "\n";
            std::cout << "  Pit mode field: " << unsigned(p[7])
                      << "\n";
        }
    }
    std::cout << "Valid frames: " << good << "\n";
}

int main(int argc, char** argv) {
    if (argc != 2 ||
        (std::strcmp(argv[1],"r") != 0 &&
         std::strcmp(argv[1],"v") != 0)) {
        std::cerr << "Usage: sudo " << argv[0]
                  << " r|v\n";
        return 1;
    }

    Restore restore;
    char command = argv[1][0];

    int fd = open("/dev/ttyAMA5", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open UART5");
        return 1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return 1;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag |= CLOCAL | CREAD | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB);
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return 1;
    }

    auto packet = make_packet(command);
    tcflush(fd, TCIOFLUSH);

    ssize_t written = write(fd, packet.data(), packet.size());
    if (written != static_cast<ssize_t>(packet.size())) {
        perror("UART write");
        close(fd);
        return 1;
    }

    if (tcdrain(fd) != 0) {
        perror("tcdrain");
        close(fd);
        return 1;
    }

    // Отключаем аппаратную функцию TXD5.
    // UART fd закрывается после завершения передачи.
    close(fd);

    if (run("pinctrl set 12 ip") != 0) {
        std::cerr << "Cannot switch GPIO12 to input\n";
        return 1;
    }

    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        perror("gpiod_chip_open");
        return 1;
    }

    gpiod_line_settings* settings = gpiod_line_settings_new();
    gpiod_line_config* config = gpiod_line_config_new();
    gpiod_request_config* reqcfg = gpiod_request_config_new();

    if (!settings || !config || !reqcfg) {
        std::cerr << "Cannot allocate gpiod configuration\n";
        return 1;
    }

    gpiod_line_settings_set_direction(
        settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(
        settings, GPIOD_LINE_EDGE_BOTH);
    gpiod_line_settings_set_event_clock(
        settings, GPIOD_LINE_CLOCK_MONOTONIC);

    unsigned int offset = GPIO;
    if (gpiod_line_config_add_line_settings(
            config, &offset, 1, settings) != 0) {
        perror("gpiod_line_config_add_line_settings");
        return 1;
    }

    gpiod_request_config_set_consumer(reqcfg, "tramp-onewire");
    gpiod_line_request* request =
        gpiod_chip_request_lines(chip, reqcfg, config);

    if (!request) {
        perror("gpiod_chip_request_lines");
        return 1;
    }

    gpiod_edge_event_buffer* buffer =
        gpiod_edge_event_buffer_new(256);

    if (!buffer) {
        std::cerr << "Cannot allocate event buffer\n";
        return 1;
    }

    std::vector<Edge> edges;
    auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start <
           std::chrono::nanoseconds(CAPTURE_NS)) {

        int ready = gpiod_line_request_wait_edge_events(
            request, 20000000LL);

        if (ready < 0) {
            perror("wait_edge_events");
            break;
        }
        if (ready == 0) continue;

        int n = gpiod_line_request_read_edge_events(
            request, buffer, 256);

        if (n < 0) {
            perror("read_edge_events");
            break;
        }

        for (int i = 0; i < n; ++i) {
            auto* ev = gpiod_edge_event_buffer_get_event(buffer, i);
            if (!ev) continue;

            edges.push_back({
                gpiod_edge_event_get_timestamp_ns(ev),
                gpiod_edge_event_get_event_type(ev) ==
                    GPIOD_EDGE_EVENT_RISING_EDGE ? 1 : 0
            });
        }
    }

    gpiod_edge_event_buffer_free(buffer);
    gpiod_line_request_release(request);
    gpiod_request_config_free(reqcfg);
    gpiod_line_config_free(config);
    gpiod_line_settings_free(settings);
    gpiod_chip_close(chip);

    std::sort(edges.begin(), edges.end(),
        [](const Edge& a, const Edge& b) {
            return a.t < b.t;
        });

    std::cout << "Captured edges: " << edges.size() << "\n";
    print_frames(decode(edges));
    return 0;
}
