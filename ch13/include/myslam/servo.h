#pragma once
#ifndef MYSLAM_SERVO_H
#define MYSLAM_SERVO_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

class Servo {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Servo> Ptr;

    Servo();

    void Start();

    void Stop();

    void SetFileNames(std::string event_device, std::string uart_device) {
        event_device_filename_ = event_device;
        uart_device_filename_ = uart_device;
    }

private:

    void ThreadLoop_evdev();

    void ThreadLoop_uart();

    bool started_ = false;

    std::thread evdev_thread_;
    std::atomic<bool> evdev_thread_running_;

    std::thread uart_thread_;
    std::atomic<bool> uart_thread_running_;

    float dir = 0.0f;
    float spd = 0.0f;
    std::mutex data_mutex;

    char buffer[64];

    // parameters

    std::string event_device_filename_;
    std::string uart_device_filename_

};

}  // namespace myslam

#endif  // MYSLAM_SERVO_H
