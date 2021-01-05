
#include "main.h"

using namespace std;
using namespace std::chrono;

struct libevdev *dev;
int js_fd;
int uart_fd;

float dir = 0.0f;
float spd = 0.0f;
mutex cmd_mutex;

char buffer[64];
bool uartIOThread_running;
thread uartIOThread;

typedef milliseconds ms;

ms get_now() {
    return duration_cast<ms>(system_clock::now().time_since_epoch());
}

float map(float f1, float f2, float t1, float t2, float v) {
    return (v - f1) * (t2 - t1) / (f2 - f1) + t1;
}

void uartIOThread_Func() {

    long next_send = get_now().count();

    while (uartIOThread_running) {

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
}

int main() {

    int rc;
    
    dev = libevdev_new();

    js_fd = open("/dev/input/event7", O_RDONLY | O_NONBLOCK);
    cout << "js_fd: " << js_fd << endl;
    rc = libevdev_set_fd(dev, js_fd);
    
    uart_fd = open("/dev/ttyTHS2", O_RDWR);
    cout << "uart_fd: " << uart_fd << endl;

    uartIOThread_running = true;
    uartIOThread = thread(uartIOThread_Func);

    do {
        struct input_event ev;
        int hasEvent = libevdev_has_event_pending(dev);
        if (hasEvent) {
            int rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
            if (rc == 0) {
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
            //cout << "Waiting" << endl;
            std::chrono::milliseconds timespan(100);
            std::this_thread::sleep_for(timespan);
        }
    } while (rc == 0 || rc == 1 || rc == -EAGAIN);

    uartIOThread_running = false;
    uartIOThread.join();

    libevdev_free(dev);
    close(js_fd);
    close(uart_fd);

    return 0;
}

