#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

int main() {
    int fd = open("/dev/gpiochip0", O_RDWR);
    if (fd < 0) {
        perror("GPIO OPEN");
        return 1;
    }

    gpiochip_info ci{};
    if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &ci) ||
        strcmp(ci.label, "300b000.pinctrl")) {
        close(fd);
        return 2;
    }

    for (int pull = 0; pull < 2; ++pull) {
        gpio_v2_line_request req{};
        req.offsets[0] = 261;
        req.num_lines = 1;
        req.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_EDGE_FALLING |
            (pull ? GPIO_V2_LINE_FLAG_BIAS_PULL_UP :
                    GPIO_V2_LINE_FLAG_BIAS_DISABLED);

        strcpy(req.consumer, "tramp-check");

        if (ioctl(fd, GPIO_V2_GET_LINE_IOCTL, &req)) {
            perror("GPIO REQUEST");
            close(fd);
            return 3;
        }

        usleep(100000);

        gpio_v2_line_values v{};
        v.mask = 1;

        if (ioctl(req.fd,
                  GPIO_V2_LINE_GET_VALUES_IOCTL, &v)) {
            perror("GPIO READ");
            close(req.fd);
            close(fd);
            return 4;
        }

        printf("PI5 %s: level=%llu\n",
               pull ? "PULL_UP" : "NO_PULL",
               (unsigned long long)(v.bits & 1));

        close(req.fd);
    }

    close(fd);
    return 0;
}
