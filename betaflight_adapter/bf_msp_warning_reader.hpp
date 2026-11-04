#pragma once

#include "bf_msp_encoder.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace bf_msp {

struct OsdWarning {
    uint8_t attributes = 0;
    std::string text;
};

class WarningReader {
public:
    void feed(const uint8_t* data, std::size_t size) {
        buffer_.insert(buffer_.end(), data, data + size);

        if (buffer_.size() > 4096)
            buffer_.erase(
                buffer_.begin(),
                buffer_.end() - 4096
            );

        parse();
    }

    bool pop(OsdWarning& result) {
        if (warnings_.empty())
            return false;

        result = warnings_.front();
        warnings_.pop_front();
        return true;
    }

private:
    std::vector<uint8_t> buffer_;
    std::deque<OsdWarning> warnings_;

    void parse() {
        while (buffer_.size() >= 3) {
            if (buffer_[0] != '$' ||
                buffer_[1] != 'X' ||
                buffer_[2] != '>') {
                buffer_.erase(buffer_.begin());
                continue;
            }

            if (buffer_.size() < 9)
                return;

            const uint16_t command =
                uint16_t(buffer_[4]) |
                (uint16_t(buffer_[5]) << 8);

            const uint16_t length =
                uint16_t(buffer_[6]) |
                (uint16_t(buffer_[7]) << 8);

            if (length > 256) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            const size_t total = size_t(length) + 9;

            if (buffer_.size() < total)
                return;

            std::vector<uint8_t> body(
                buffer_.begin() + 3,
                buffer_.begin() + total - 1
            );

            if (crc8(body) != buffer_[total - 1]) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            if (command == 0x3005 && length >= 2) {
                const uint8_t* p = buffer_.data() + 8;
                const size_t text_len = p[1];

                if (text_len <= size_t(length) - 2) {
                    OsdWarning warning;
                    warning.attributes = p[0];

                    warning.text.assign(
                        reinterpret_cast<const char*>(p + 2),
                        text_len
                    );

                    warnings_.push_back(warning);
                }
            }

            buffer_.erase(
                buffer_.begin(),
                buffer_.begin() + total
            );
        }
    }
};

} // namespace bf_msp
