#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

using Clock = std::chrono::steady_clock;

static volatile sig_atomic_t stop_requested = 0;

static void on_signal(int) {
    stop_requested = 1;
}

class GPIO {
public:
    static constexpr size_t MAP_SIZE = 0x30000;
    static constexpr size_t STATUS = 0x60;
    static constexpr size_t CTRL = 0x64;

    static constexpr uint32_t FUNC_MASK = 0x1f;
    static constexpr uint32_t OUT_MASK = 3u << 12;
    static constexpr uint32_t OE_MASK = 3u << 14;
    static constexpr uint32_t IN_PAD = 1u << 17;

    GPIO() {
        fd = open("/dev/gpiomem0", O_RDWR | O_SYNC);
        if (fd < 0) return;

        void* p = mmap(nullptr, MAP_SIZE,
                       PROT_READ | PROT_WRITE,
                       MAP_SHARED, fd, 0);
        if (p == MAP_FAILED) return;

        regs = static_cast<volatile uint32_t*>(p);
        original = read(CTRL);

        if ((original & FUNC_MASK) != 2 ||
            (original & OE_MASK) != 0) {
            fprintf(stderr, "Unexpected GPIO12 configuration\n");
            return;
        }

        valid = true;
    }

    ~GPIO() {
        restore();
        if (regs) munmap((void*)regs, MAP_SIZE);
        if (fd >= 0) close(fd);
    }

    bool ok() const { return valid; }

    void begin() {
        if (!valid) return;

        // First disable output, then select SIO.
        uint32_t disabled =
            (original & ~OE_MASK) | (2u << 14);
        write(CTRL, disabled);

        base = disabled & ~(FUNC_MASK | OUT_MASK);
        base = (base & ~FUNC_MASK) | 5u;

        // Prepare HIGH while output is disabled.
        write(CTRL, base | (3u << 12));

        // Enable physical output.
        output_base = base & ~OE_MASK;
        output_base |= 3u << 14;

        write(CTRL, output_base | (3u << 12));
        active = true;
    }

    void tx(bool high) {
        write(CTRL, output_base |
                    ((high ? 3u : 2u) << 12));
    }

    void input() {
        // Release DATA without switching away from SIO.
        write(CTRL, base | (3u << 12));
    }

    bool level() const {
        return (read(STATUS) & IN_PAD) != 0;
    }

    void restore() {
        if (!valid || !active) return;

        // Disable output before restoring UART function.
        write(CTRL, base | (3u << 12));
        write(CTRL, original);
        active = false;
    }

private:
    uint32_t read(size_t offset) const {
        return regs[offset / 4];
    }

    void write(size_t offset, uint32_t value) {
        regs[offset / 4] = value;
        __sync_synchronize();
    }

    int fd = -1;
    volatile uint32_t* regs = nullptr;
    uint32_t original = 0;
    uint32_t base = 0;
    uint32_t output_base = 0;
    bool valid = false;
    bool active = false;
};

static void wait_until(Clock::time_point t) {
    while (!stop_requested && Clock::now() < t) {}
}

static Clock::duration bits(double count) {
    return std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(count / 9600.0));
}

static uint64_t tx_checked = 0;
static uint64_t tx_matches = 0;
static uint64_t tx_high = 0;
static uint64_t tx_low = 0;

static void transmit(GPIO& gpio,
                     const std::array<uint8_t, 16>& packet) {
    auto start = Clock::now();

    auto check_bit = [&](Clock::time_point t, bool expected) {
        wait_until(t);
        bool actual = gpio.level();
        ++tx_checked;
        if (actual == expected) ++tx_matches;
        if (actual) ++tx_high;
        else ++tx_low;
    };

    for (size_t n = 0; n < packet.size(); ++n) {
        if (stop_requested) break;

        auto byte_start = start + bits(n * 10.0);

        wait_until(byte_start);
        gpio.tx(false);
        check_bit(byte_start + bits(0.5), false);

        for (int i = 0; i < 8; ++i) {
            wait_until(byte_start + bits(i + 1.0));
            bool expected = (packet[n] >> i) & 1;
            gpio.tx(expected);
            check_bit(byte_start + bits(i + 1.5), expected);
        }

        wait_until(byte_start + bits(9.0));
        gpio.tx(true);
        check_bit(byte_start + bits(9.5), true);
    }

    wait_until(start + bits(160.0));

    // Immediately release DATA.
    gpio.input();
}

int main(int argc, char** argv) {
    if (argc != 2 ||
        argv[1][1] != '\0' ||
        (argv[1][0] != 'v' && argv[1][0] != 'r')) {
        fprintf(stderr, "Usage: %s v|r\n", argv[0]);
        return 2;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    GPIO gpio;
    if (!gpio.ok()) return 1;

    std::array<uint8_t, 16> packet{};
    packet[0] = 0x0F;
    packet[1] = static_cast<uint8_t>(argv[1][0]);
    packet[14] = packet[1];

    sched_param sp{};
    sp.sched_priority = 50;

    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "SCHED_FIFO unavailable\n");

    printf("TX:");
    for (auto b : packet) printf(" %02X", b);
    printf("\n");
    fflush(stdout);

    gpio.begin();

    // A short idle HIGH period.
    auto idle_end = Clock::now() + bits(20);
    wait_until(idle_end);

    transmit(gpio, packet);

    auto rx_start = Clock::now();
    auto rx_end = rx_start + std::chrono::milliseconds(150);

    std::vector<uint8_t> received;
    uint64_t samples = 0, low = 0, edges = 0;
    unsigned framing_errors = 0;

    bool previous = gpio.level();

    while (!stop_requested &&
           Clock::now() < rx_end &&
           received.size() < 64) {

        bool current = gpio.level();
        ++samples;
        if (!current) ++low;
        if (current != previous) ++edges;

        if (previous && !current) {
            auto start = Clock::now();

            wait_until(start + bits(0.5));
            if (gpio.level()) {
                previous = gpio.level();
                continue;
            }

            uint8_t value = 0;

            for (int i = 0; i < 8; ++i) {
                wait_until(start + bits(1.5 + i));
                if (gpio.level())
                    value |= uint8_t(1u << i);
            }

            wait_until(start + bits(9.5));

            if (gpio.level())
                received.push_back(value);
            else
                ++framing_errors;

            previous = gpio.level();
        } else {
            previous = current;
        }
    }

    gpio.restore();

    printf("TX checked: %llu\\n",
           (unsigned long long)tx_checked);
    printf("TX matches: %llu\\n",
           (unsigned long long)tx_matches);
    printf("TX HIGH: %llu\\n",
           (unsigned long long)tx_high);
    printf("TX LOW: %llu\\n",
           (unsigned long long)tx_low);

    printf("Samples: %llu\n",
           (unsigned long long)samples);
    printf("LOW samples: %llu\n",
           (unsigned long long)low);
    printf("Edges: %llu\n",
           (unsigned long long)edges);
    printf("RX bytes: %zu\n", received.size());
    printf("Framing errors: %u\n", framing_errors);

    printf("RX:");
    for (auto b : received) printf(" %02X", b);
    printf("\n");

    bool valid = false;
    for (size_t i = 0; i + 16 <= received.size(); ++i) {
        if (received[i] != 0x0F) continue;

        unsigned sum = 0;
        for (size_t j = 1; j < 14; ++j)
            sum += received[i + j];

        if ((sum & 255) == received[i + 14]) {
            valid = true;
            printf("VALID TRAMP FRAME at offset %zu\n", i);
        }
    }

    if (!valid)
        printf("No validated 16-byte Tramp frame\n");

    return valid ? 0 : 3;
}
