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
public:
    static constexpr size_t MAP_SIZE = 0x30000;
    static constexpr size_t GPIO12_STATUS = 0x60;
    static constexpr size_t GPIO12_CTRL = 0x64;

    static constexpr uint32_t FUNC_MASK = 0x1f;
    static constexpr uint32_t FUNC_TXD4 = 2;
    static constexpr uint32_t OEOVER_MASK = 3u << 14;
    static constexpr uint32_t OEOVER_DISABLE = 2u << 14;
    static constexpr uint32_t INFROMPAD = 1u << 17;

    GPIO() {
        fd = open("/dev/gpiomem0", O_RDWR | O_SYNC);
        if (fd < 0) return;

        void* p = mmap(nullptr, MAP_SIZE,
                       PROT_READ | PROT_WRITE,
                       MAP_SHARED, fd, 0);

        if (p == MAP_FAILED) return;

        regs = static_cast<volatile uint32_t*>(p);

        saved_ctrl = read(GPIO12_CTRL);

        // Never transmit if GPIO12 is not configured as TXD4.
        if ((saved_ctrl & FUNC_MASK) != FUNC_TXD4) {
            std::cerr << "GPIO12 is not TXD4 (ALT2)\n";
            return;
        }

        // Do not overwrite an existing output-enable override.
        if ((saved_ctrl & OEOVER_MASK) != 0) {
            std::cerr << "GPIO12 has unexpected OEOVER\n";
            return;
        }

        valid = true;
    }

    ~GPIO() {
        restore();

        if (regs)
            munmap((void*)regs, MAP_SIZE);

        if (fd >= 0)
            close(fd);
    }

    bool ok() const { return valid; }

    void input() {
        if (!valid || changed) return;

        // RP1: force physical output disabled first.
        uint32_t disabled =
            (saved_ctrl & ~OEOVER_MASK) | OEOVER_DISABLE;
        write(GPIO12_CTRL, disabled);

        // Select SIO while keeping output forcibly disabled.
        uint32_t sio_input =
            (disabled & ~FUNC_MASK) | 5u;
        write(GPIO12_CTRL, sio_input);

        changed = true;
    }

    void restore() {
        if (!valid || !changed) return;

        // Restore the exact original GPIO12 control value.
        write(GPIO12_CTRL, saved_ctrl);
        changed = false;
    }

    bool level() const {
        if (!valid) return true;
        return (read(GPIO12_STATUS) & INFROMPAD) != 0;
    }

    void debug(const char* label) const {
        std::cerr << label
                  << " CTRL=0x" << std::hex
                  << read(GPIO12_CTRL)
                  << " STATUS=0x"
                  << read(GPIO12_STATUS)
                  << std::dec << "\n";
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
    uint32_t saved_ctrl = 0;
    bool valid = false;
    bool changed = false;
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
        std::cerr << "Cannot mmap /dev/gpiomem\n";
        return 1;
    }

    int serial = open("/dev/ttyAMA4",
                      O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (serial < 0) {
        perror("open UART4");
        return 1;
    }

    termios oldt{}, t{};
    if (tcgetattr(serial, &oldt) != 0) {
        perror("tcgetattr");
        close(serial);
        return 1;
    }

    t = oldt;
    cfmakeraw(&t);
    cfsetispeed(&t, B9600);
    cfsetospeed(&t, B9600);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~CRTSCTS;
    t.c_cflag &= ~CSTOPB;
    t.c_cflag &= ~PARENB;
    t.c_cflag &= ~CSIZE;
    t.c_cflag |= CS8;

    if (tcsetattr(serial, TCSANOW, &t) != 0) {
        perror("tcsetattr");
        close(serial);
        return 1;
    }

    // Best-effort CPU scheduling; failure is not fatal.
    sched_param sp{};
    sp.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        std::cerr << "Note: SCHED_FIFO unavailable\n";

    auto packet = query(argv[1][0]);

    tcflush(serial, TCIOFLUSH);

    std::cout << "TX: ";
    print_hex({packet.begin(), packet.end()});
    std::cout.flush();

    ssize_t written = write(serial, packet.data(),
                            packet.size());

    if (written != static_cast<ssize_t>(packet.size())) {
        std::cerr << "UART write failed\n";
        tcsetattr(serial, TCSANOW, &oldt);
        close(serial);
        return 1;
    }

    if (tcdrain(serial) != 0) {
        perror("tcdrain");
        tcsetattr(serial, TCSANOW, &oldt);
        close(serial);
        return 1;
    }

    // Disable GPIO12 TX output after UART4 transmission.
    gpio.input();
    auto rx_start = Clock::now();

    constexpr double bit = 1.0 / 9600.0;
    constexpr double timeout = 0.150;

    std::vector<uint8_t> received;
    unsigned framing_errors = 0;
    bool previous = gpio.level();

    uint64_t samples = 0;
    uint64_t rising_edges = 0;
    uint64_t falling_edges = 0;
    uint64_t high_samples = 0;
    uint64_t low_samples = 0;

    while (!interrupted &&
           seconds(rx_start, Clock::now()) < timeout &&
           received.size() < 64) {

        bool current = gpio.level();

        ++samples;
        if (current)
            ++high_samples;
        else
            ++low_samples;

        if (!previous && current)
            ++rising_edges;
        if (previous && !current)
            ++falling_edges;

        if (previous && !current) {
            auto start = Clock::now();

            // Validate start bit at its midpoint.
            wait_until(start +
                std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(bit * 0.5)));

            if (gpio.level()) {
                previous = gpio.level();
                continue;
            }

            uint8_t value = 0;

            for (int i = 0; i < 8; ++i) {
                auto target = start +
                    std::chrono::duration_cast<Clock::duration>(
                        std::chrono::duration<double>(
                            bit * (1.5 + i)));

                wait_until(target);

                if (gpio.level())
                    value |= uint8_t(1u << i);
            }

            auto stop = start +
                std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(bit * 9.5));

            wait_until(stop);

            if (gpio.level()) {
                received.push_back(value);
            } else {
                ++framing_errors;
            }

            previous = gpio.level();
        } else {
            previous = current;
        }
    }

    gpio.restore();

    std::cout << "===== GPIO12 EDGE STATS =====\\n";
    std::cout << "Samples: " << samples << '\n';
    std::cout << "HIGH samples: " << high_samples << '\n';
    std::cout << "LOW samples: " << low_samples << '\n';
    std::cout << "Rising edges: " << rising_edges << '\n';
    std::cout << "Falling edges: " << falling_edges << '\n';

    tcsetattr(serial, TCSANOW, &oldt);
    close(serial);

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

        if ((sum & 0xFF) == received[i + 14]) {
            valid = true;
            std::cout << "VALID TRAMP FRAME at offset "
                      << i << '\n';
        }
    }

    if (!valid)
        std::cout << "No validated 16-byte Tramp frame\n";

    return valid ? 0 : 3;
}
