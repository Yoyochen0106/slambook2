
#include "boost/format.hpp"

#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/joystick.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

static float map(float f1, float f2, float t1, float t2, float v) {
    return (v - f1) * (t2 - t1) / (f2 - f1) + t1;
}

namespace myslam {

Servo::Servo() {
}

Servo::Start() {
    if (started_) {
        return;
    }
    started_ = true;

    evdev_thread_running_.store(true);
    evdev_thread_ = std::thread(&ThreadLoop_evdev);

    uart_thread_running_.store(true);
    uart_thread_ = std::thread(&ThreadLoop_uart);
}

Servo::Stop() {
    evdev_thread_running_.store(false);
    uart_thread_running_.store(false);
    evdev_thread_.join();
    uart_thread_.join();
}

Servo::ThreadLoop_evdev() {

    // Return code
    int rc;
    int js_fd;
    struct libevdev *dev;

    js_fd = open(event_device_file_name_, O_RDONLY | O_NONBLOCK);

    dev = libevdev_new();
    rc = libevdev_set_fd(dev, js_fd);

    while (evdev_thread_running_.load()) {
        struct input_event ev;
        int hasEvent = libevdev_has_event_pending(dev);
        if (hasEvent) {
            int result = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
            if (result == LIBEVDEV_READ_STATUS_SUCCESS) {
                if (ev.type == EV_ABS && ev.code == ABS_Y) {
                    // Top -> Bottom
                    float spd_new = map(0.0f, 255.0f, 1.0f, -1.0f, (float)ev.value);
                    {
                        lock_guard<mutex> lock_cmd(cmd_mutex);
                        spd = spd_new;
                    }
                }
                else if (ev.type == EV_ABS && ev.code == ABS_Z) {
                    // Left -> Right
                    float dir_new = map(0.0f, 255.0f, -1.0f, 1.0f, (float)ev.value);
                    {
                        lock_guard<mutex> lock_cmd(cmd_mutex);
                        dir = dir_new;
                    }
                }
            }
        } else {
            std::chrono::milliseconds timespan(10);
            std::this_thread::sleep_for(timespan);
        }
    }

    libevdev_free(dev);
    close(js_fd);
}

Servo::ThreadLoop_uart() {

    int uart_fd = open(uart_device_file_name_, O_RDWR);

    long next_send = get_now().count();

    while (evdev_thread_running_.load()) {

        long wait_time = next_send - get_now().count();
        if (wait_time > 0) {
            this_thread::sleep_for(ms(wait_time));
        }
        
        if (!uartIOThread_running) {
            return;
        }

        float dir_tmp;
        float spd_tmp;
        {
            lock_guard<mutex> lock_cmd(cmd_mutex);
            dir_tmp = dir;
            spd_tmp = spd;
        }
        ms now = get_now();
        int dir_code = (int)map(-1.0f, 1.0f, -100.0f, 100.0f, dir_tmp);
        int spd_code = (int)map(-1.0f, 1.0f, -100.0f, 100.0f, spd_tmp);
        dir_code += 0x80;
        spd_code += 0x80;
        int length = sprintf(buffer, "$%02X%02X\n", dir_code, spd_code);
        write(uart_fd, buffer, length);

        buffer[length-1] = '\0';
        printf("sent: %s, %d, %ld, %f, %f\n", buffer, length, now.count(), dir_tmp, spd_tmp);

        next_send += 50;

    }

    close(uart_fd);

}

}  // namespace myslam
