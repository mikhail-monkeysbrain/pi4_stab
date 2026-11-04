#include "bf_serial_port.hpp"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

int main() {
    const int master = posix_openpt(
        O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC
    );

    assert(master >= 0);
    assert(grantpt(master) == 0);
    assert(unlockpt(master) == 0);

    const char* slave = ptsname(master);
    assert(slave != nullptr);

    {
        bf_serial::Port port(slave, B115200);

        const uint8_t incoming[] = {0x59, 0x59, 0x7B, 0x00};

        assert(::write(master, incoming, sizeof(incoming)) ==
               static_cast<ssize_t>(sizeof(incoming)));

        uint8_t received[32]{};
        ssize_t n = 0;

        for (int i = 0; i < 1000 && n == 0; ++i) {
            n = port.read(received, sizeof(received));
            if (n == 0)
                usleep(1000);
        }

        assert(n == 4);
        assert(std::memcmp(
            received, incoming, sizeof(incoming)
        ) == 0);

        std::cout << "PASS: nonblocking RX\n";

        const uint8_t outgoing[] = {
            '$', 'X', '<', 0x00, 0x01, 0x1F
        };

        port.enqueue(outgoing, sizeof(outgoing));

        assert(port.pendingTx() == sizeof(outgoing));

        const ssize_t written = port.flushTx();

        assert(written == static_cast<ssize_t>(
            sizeof(outgoing)
        ));
        assert(port.pendingTx() == 0);

        uint8_t echoed[32]{};
        ssize_t received_n = 0;

        for (int i = 0; i < 1000 && received_n == 0; ++i) {
            received_n = ::read(
                master, echoed, sizeof(echoed)
            );

            if (received_n < 0 &&
                (errno == EAGAIN || errno == EWOULDBLOCK)) {
                received_n = 0;
                usleep(1000);
            }
        }

        assert(received_n ==
               static_cast<ssize_t>(sizeof(outgoing)));

        assert(std::memcmp(
            echoed, outgoing, sizeof(outgoing)
        ) == 0);

        std::cout << "PASS: queued TX\n";

        bool overflow_detected = false;
        uint8_t oversized[4097]{};

        try {
            port.enqueue(oversized, sizeof(oversized));
        } catch (const std::runtime_error&) {
            overflow_detected = true;
        }

        assert(overflow_detected);
        std::cout << "PASS: TX queue limit\n";
    }

    ::close(master);

    std::cout << "PASS: PTY transport\n";
    std::cout << "PASS: no physical UART access\n";

    return 0;
}
