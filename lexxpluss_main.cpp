#include <Arduino.h>
#include <Arduino_CRC32.h>
#include "serial_message.hpp"
#include "lexxpluss_main.hpp"

namespace {

class power_controller {
public:
    void init() {
        heartbeat_timer.start();
        pinMode(13, OUTPUT);
    }
    void poll() {
        auto elapsed_ms = heartbeat_timer.read_ms();
        if (elapsed_ms > 10000)
            set_enable(false);
        if (is_connector_overheat())
            set_enable(false);
    }
    void ping() {
        heartbeat_timer.reset();
    }
    void set_enable(bool enable) {
        if (enable) {
            if (!is_connector_overheat())
                digitalWrite(13, 1);
            voltage_timer.reset();
            voltage_timer.start();
        } else {
            digitalWrite(13, 0);
            voltage_timer.stop();
            voltage_timer.reset();
        }
        last_enable = enable;
    }
    bool get_enable() const {
        return last_enable;
    }
private:
    bool is_connector_overheat() const {
        return (analogRead(A0) > 512 || //@@
                analogRead(A1) > 512);  //@@
    }
    simpletimer heartbeat_timer, voltage_timer;
    bool last_enable{false};
};

class charger {
public:
    void init() {
        Serial1.begin(4800, SERIAL_8N1);
        power.init();
        msg.init();
    }
    void poll() {
        power.poll();
        while (Serial1.available()) {
            int c = Serial1.read();
            if (msg.decode(c)) {
                uint8_t param[3];
                uint8_t command = msg.get_command(param);
                handle_command(command, param);
            }
        }
        delay(30);
    }
private:
    void handle_command(uint8_t command, uint8_t param[3]) {
        switch (command) {
        case serial_message::HEARTBEAT:
            heartbeat(param);
            break;
        default:
            break;
        }
    }
    void heartbeat(const uint8_t param[3]) {
        power.set_enable(param[1] != 0);
        uint8_t buf[8], send_param[3]{param[0], power.get_enable()};
        serial_message::compose(buf, serial_message::HEARTBEAT, send_param);
        Serial1.write(buf, sizeof buf);
        power.ping();
    }
    power_controller power;
    serial_message msg;
} impl;

}

namespace lexxpluss {

void setup()
{
    impl.init();
}

void loop()
{
    impl.poll();
}

}

// vim: expandtab shiftwidth=4:
