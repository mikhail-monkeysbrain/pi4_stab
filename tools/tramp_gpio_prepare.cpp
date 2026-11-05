#include <gpiod.h>
#include <cstdlib>
#include <iostream>

int main() {
    // GPIO12 сначала должен быть освобождён от TXD5.
    if (std::system("pinctrl set 12 ip") != 0) {
        std::cerr << "Cannot switch GPIO12 to input\n";
        return 1;
    }

    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        perror("gpiod_chip_open");
        std::system("pinctrl set 12 a4");
        return 1;
    }

    auto* settings = gpiod_line_settings_new();
    auto* config = gpiod_line_config_new();
    auto* reqcfg = gpiod_request_config_new();

    gpiod_line_settings_set_direction(
        settings, GPIOD_LINE_DIRECTION_INPUT);

    unsigned offset = 12;
    int result = gpiod_line_config_add_line_settings(
        config, &offset, 1, settings);

    gpiod_line_request* request = nullptr;

    if (result == 0) {
        request = gpiod_chip_request_lines(
            chip, reqcfg, config);
    }

    if (request) {
        std::cout << "GPIO12 REQUEST OK\n";
        std::cout << "Single-wire GPIO access available\n";
        gpiod_line_request_release(request);
    } else {
        perror("GPIO12 request");
    }

    gpiod_request_config_free(reqcfg);
    gpiod_line_config_free(config);
    gpiod_line_settings_free(settings);
    gpiod_chip_close(chip);

    if (std::system("pinctrl set 12 a4") != 0) {
        std::cerr << "WARNING: TXD5 restore failed\n";
        return 1;
    }

    std::cout << "GPIO12 restored to TXD5\n";
    return request ? 0 : 1;
}
