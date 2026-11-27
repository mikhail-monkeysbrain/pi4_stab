#include <poll.h>
#include <algorithm>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <stdexcept>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

using Clock = std::chrono::steady_clock;

static volatile sig_atomic_t interrupted = 0;

static void on_signal(int) {
    interrupted = 1;
}


class GPIO {
    int chip = -1;
    int line = -1;

    void config(bool output) {
        gpio_v2_line_config c{};
        c.flags = output
            ? GPIO_V2_LINE_FLAG_OUTPUT
            : (GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_UP);

        if (output) {
            c.num_attrs = 1;
            c.attrs[0].attr.id =
                GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
            c.attrs[0].attr.values = 1;
            c.attrs[0].mask = 1;
        }

        if (ioctl(line,
            GPIO_V2_LINE_SET_CONFIG_IOCTL, &c))
            throw std::runtime_error("GPIO configure failed");
    }

public:
    GPIO() {
        chip = open("/dev/gpiochip0",
                    O_RDWR | O_CLOEXEC);
        if (chip < 0) return;

        gpiochip_info ci{};
        if (ioctl(chip, GPIO_GET_CHIPINFO_IOCTL, &ci))
            return;

        if (strcmp(ci.label, "300b000.pinctrl"))
            return;

        gpio_v2_line_request r{};
        r.offsets[0] = 261;
        r.num_lines = 1;
        r.event_buffer_size = 1024;
        r.config.flags = GPIO_V2_LINE_FLAG_INPUT;

        strcpy(r.consumer, "tramp-orangepi");

        if (ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &r))
            return;

        line = r.fd;
    }

    ~GPIO() {
        if (line >= 0) {
            try { input(); } catch (...) {}
            close(line);
        }
        if (chip >= 0) close(chip);
    }

    bool ok() const { return line >= 0; }

    void output() { config(true); }
    void input() { config(false); }
    void restore() { input(); }

    void tx(bool high) {
        gpio_v2_line_values v{};
        v.mask = 1;
        v.bits = high ? 1 : 0;

        if (ioctl(line,
            GPIO_V2_LINE_SET_VALUES_IOCTL, &v))
            throw std::runtime_error("GPIO TX failed");
    }


    void edges() {
        gpio_v2_line_config c{};
        c.flags = GPIO_V2_LINE_FLAG_INPUT |
                  GPIO_V2_LINE_FLAG_BIAS_PULL_UP |
                  GPIO_V2_LINE_FLAG_EDGE_RISING |
                  GPIO_V2_LINE_FLAG_EDGE_FALLING;

        if (ioctl(line,
            GPIO_V2_LINE_SET_CONFIG_IOCTL, &c))
            throw std::runtime_error("GPIO edge config failed");
    }

    int event_fd() const { return line; }

    bool level() const {
        gpio_v2_line_values v{};
        v.mask = 1;

        if (ioctl(line,
            GPIO_V2_LINE_GET_VALUES_IOCTL, &v))
            throw std::runtime_error("GPIO RX failed");

        return (v.bits & 1) != 0;
    }
};

static double seconds(Clock::time_point a,
                      Clock::time_point b) {
    return std::chrono::duration<double>(b - a).count();
}

static void wait_until(Clock::time_point target) {
    while (!interrupted && Clock::now() < target) {
        // Busy wait for accurate sampling.
    }
}

static std::array<uint8_t, 16> query(char command) {
    std::array<uint8_t, 16> p{};
    p[0] = 0x0F;
    p[1] = static_cast<uint8_t>(command);

    unsigned sum = 0;
    for (int i = 1; i < 14; ++i) sum += p[i];
    p[14] = static_cast<uint8_t>(sum & 0xFF);
    return p;
}

static void print_hex(const std::vector<uint8_t>& bytes) {
    std::cout << std::hex;
    for (uint8_t b : bytes) {
        const char* h = "0123456789ABCDEF";
        std::cout << h[b >> 4] << h[b & 15] << ' ';
    }
    std::cout << std::dec << '\n';
}

int main(int argc, char** argv) {
    if (argc != 2 ||
        std::strlen(argv[1]) != 1 ||
        (argv[1][0] != 'r' &&
         argv[1][0] != 'v' &&
         argv[1][0] != 's')) {
        std::cerr << "Usage: " << argv[0]
                  << " r|v|s\n";
        return 2;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    GPIO gpio;
    if (!gpio.ok()) {
        std::cerr << "Cannot request PI5\n";
        return 1;
    }


    auto packet = query(argv[1][0]);

    std::cout << "TX: ";
    print_hex({packet.begin(), packet.end()});
    std::cout.flush();

    sched_param sp{};
    sp.sched_priority = 20;
    sched_setscheduler(0, SCHED_FIFO, &sp);

    gpio.output();

    wait_until(Clock::now() +
               std::chrono::milliseconds(3));

    constexpr double tx_bit = 1.0 / 9600.0;
    auto start = Clock::now();

    auto at = [&](double bits) {
        return start +
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(
                    bits * tx_bit));
    };

    for (unsigned n = 0; n < packet.size(); ++n) {
        if (interrupted) {
            gpio.input();
            return 4;
        }

        wait_until(at(n * 10.0));
        gpio.tx(false);

        for (int i = 0; i < 8; ++i) {
            wait_until(at(n * 10.0 + i + 1));
            gpio.tx((packet[n] >> i) & 1);
        }

        wait_until(at(n * 10.0 + 9));
        gpio.tx(true);
    }

    wait_until(at(packet.size() * 10.0));

    gpio.edges();

    struct Edge {
        uint64_t ns;
        bool high;
    };

    std::vector<Edge> edges;
    auto deadline = Clock::now() +
                    std::chrono::milliseconds(200);

    unsigned lost = 0;
    uint32_t previous_seq = 0;

    while (!interrupted && Clock::now() < deadline) {
        pollfd p{};
        p.fd = gpio.event_fd();
        p.events = POLLIN;

        int ready = poll(&p, 1, 5);
        if (ready < 0) break;
        if (ready == 0) continue;

        gpio_v2_line_event ev[128]{};

        ssize_t n = read(p.fd, ev, sizeof(ev));
        if (n <= 0) break;

        for (size_t i = 0;
             i < size_t(n) / sizeof(ev[0]); ++i) {

            if (previous_seq &&
                ev[i].seqno != previous_seq + 1)
                ++lost;

            previous_seq = ev[i].seqno;

            edges.push_back({
                ev[i].timestamp_ns,
                ev[i].id == GPIO_V2_LINE_EVENT_RISING_EDGE
            });
        }
    }

    gpio.restore();

    constexpr double bit_ns = 1e9 / 9600.0;

    auto level_at = [&](uint64_t t) {
        bool high = true;
        for (const auto& e : edges) {
            if (e.ns > t) break;
            high = e.high;
        }
        return high;
    };

    std::vector<uint8_t> received;
    unsigned framing_errors = 0;
    uint64_t next_start = 0;

    for (const auto& e : edges) {
        if (e.high || e.ns < next_start)
            continue;

        if (level_at(e.ns + uint64_t(bit_ns * 0.5)))
            continue;

        uint8_t value = 0;

        for (int i = 0; i < 8; ++i) {
            if (level_at(e.ns +
                uint64_t(bit_ns * (1.5 + i))))
                value |= uint8_t(1u << i);
        }

        if (!level_at(e.ns + uint64_t(bit_ns * 9.5))) {
            ++framing_errors;
            continue;
        }

        received.push_back(value);

        next_start = e.ns + uint64_t(bit_ns * 9.75);
    }

    std::cout << "Captured edges: "
              << edges.size() << '\n';
    std::cout << "Lost event sequences: "
              << lost << '\n';

    if (lost) return 5;

    std::cout << "RX bytes: " << received.size() << '\n';
    std::cout << "Framing errors: "
              << framing_errors << '\n';
    std::cout << "RX: ";
    print_hex(received);

    bool valid = false;

    for (size_t i = 0; i + 16 <= received.size(); ++i) {
        if (received[i] != 0x0F) continue;

        unsigned sum = 0;
        for (size_t j = 1; j < 14; ++j)
            sum += received[i + j];

        if ((sum & 0xFF) == received[i + 14] && received[i + 15] == 0 && received[i + 1] == packet[1]) {
            valid = true;
            std::cout << "VALID TRAMP FRAME at offset "
                      << i << '\n';
        }
    }

    if (!valid)
        std::cout << "No validated 16-byte Tramp frame\n";

    return valid ? 0 : 3;
}
