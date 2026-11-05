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
#include <string>

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
        std::chrono::duration<double>(count / 4800.0));
}


static void transmit(GPIO& gpio,
                     const std::vector<uint8_t>& packet) {
    auto start = Clock::now();

    for (size_t n = 0; n < packet.size(); ++n) {
        if (stop_requested) break;

        auto byte_start = start + bits(n * 11.0);

        wait_until(byte_start);
        gpio.tx(false);

        for (int i = 0; i < 8; ++i) {
            wait_until(byte_start + bits(i + 1.0));
            gpio.tx((packet[n] >> i) & 1);
        }

        // SmartAudio: two stop bits.
        wait_until(byte_start + bits(9.0));
        gpio.tx(true);
    }

    wait_until(start + bits(packet.size() * 11.0));
    gpio.input();
}

int main(int argc, char** argv) {
    bool setting_frequency = false;
    int requested_frequency = 0;

    if (argc == 3 &&
        std::string(argv[1]) == "--set-frequency") {
        char* end = nullptr;
        long value = strtol(argv[2], &end, 10);

        if (!argv[2][0] || *end != '\0' ||
            value < 5000 || value > 6000) {
            fprintf(stderr, "Invalid frequency\n");
            return 2;
        }

        requested_frequency = static_cast<int>(value);
        setting_frequency = true;
    } else if (argc != 1) {
        fprintf(stderr, "Usage: %s\n", argv[0]);
        return 2;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    GPIO gpio;
    if (!gpio.ok()) return 1;

    // SmartAudio GET SETTINGS, read-only.
    std::vector<uint8_t> packet{
        0xAA, 0x55, 0x03, 0x00, 0x9F
    };

    if (setting_frequency) {
        packet = {
            0xAA, 0x55, 0x09, 0x02,
            static_cast<uint8_t>(requested_frequency >> 8),
            static_cast<uint8_t>(requested_frequency & 0xFF)
        };

        uint8_t crc = 0;
        for (uint8_t value : packet) {
            crc ^= value;
            for (int i = 0; i < 8; ++i) {
                crc = (crc & 0x80)
                    ? uint8_t((crc << 1) ^ 0xD5)
                    : uint8_t(crc << 1);
            }
        }
        packet.push_back(crc);
    }

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



    // SmartAudio CRC8, polynomial 0xD5.
    // CRC covers command, length and payload.
    auto crc8 = [](const uint8_t* data, size_t len) {
        uint8_t crc = 0;

        for (size_t i = 0; i < len; ++i) {
            crc ^= data[i];

            for (int bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x80)
                    ? uint8_t((crc << 1) ^ 0xD5)
                    : uint8_t(crc << 1);
            }
        }

        return crc;
    };


    bool found = false;

    for (size_t i = 0; i + 5 <= received.size(); ++i) {
        if (received[i] != 0xAA ||
            received[i + 1] != 0x55)
            continue;

        const uint8_t cmd = received[i + 2];
        const uint8_t len = received[i + 3];

        // SmartAudio response length includes CRC.
        if (len < 1 || len > 33)
            continue;

        const size_t frame_size = 4u + len;
        if (i + frame_size > received.size())
            continue;

        const size_t crc_index = i + 3u + len;

        const uint8_t calculated =
            crc8(received.data() + i + 2, len + 1u);

        const uint8_t actual = received[crc_index];

        printf("FRAME cmd=0x%02X len=%u CRC=%02X/%02X\n",
               cmd, len, calculated, actual);

        if (calculated != actual) {
            printf("CRC ERROR\n");
            continue;
        }

        printf("CRC OK\n");

        if (cmd == 0x11 || setting_frequency) {
            printf("SMARTAUDIO CRC VALID RESPONSE\n");
            found = true;
            break;
        }
    }

    if (!found)
        printf("No complete SmartAudio settings response\n");

    return found ? 0 : 3;
}
