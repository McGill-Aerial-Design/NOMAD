#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <time.h>

void sleep_us(long us) {
    struct timespec ts;
    ts.tv_sec = us / 1000000;
    ts.tv_nsec = (us % 1000000) * 1000;
    nanosleep(&ts, NULL);
}

int main(int argc, char *argv[]) {
    int pulse_us = 1500;
    int duration_s = 3;

    if (argc > 1) pulse_us = atoi(argv[1]);
    if (argc > 2) duration_s = atoi(argv[2]);

    int fd = open("/dev/gpiochip0", O_RDONLY);
    if (fd < 0) { perror("open gpiochip"); return 1; }

    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffsets[0] = 85;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0] = 0;
    req.lines = 1;
    strcpy(req.consumer_label, "servo_test");

    if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        perror("ioctl get line");
        close(fd);
        return 1;
    }

    struct gpiohandle_data data;
    int period_us = 20000;
    int pulses = duration_s * 50;

    printf("Sending pulses: count=%d width=%dus\n", pulses, pulse_us);

    for (int i = 0; i < pulses; i++) {
        data.values[0] = 1;
        ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        sleep_us(pulse_us);

        data.values[0] = 0;
        ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        sleep_us(period_us - pulse_us);
    }

    printf("Done.\n");
    close(req.fd);
    close(fd);
    return 0;
}
