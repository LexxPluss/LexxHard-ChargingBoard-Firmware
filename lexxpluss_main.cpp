#include <Arduino.h>
#include "serial_message.hpp"
#include "simpletimer.hpp"
#include "lexxpluss_main.hpp"

namespace {

class manual_switch {
public:
    enum class STATE {
        RELEASED, PUSHED, LONG_PUSHED
    };
    void init() const {
        pinMode(PIN_SW, INPUT);
        pinMode(PIN_LED, OUTPUT);
    }
    void poll() {
        int now{digitalRead(PIN_SW)};
        if (prev != now) {
            prev = now;
            timer.reset();
            timer.start();
        } else if (now == 0) {
            auto elapsed_ms{timer.read_ms()};
            if (elapsed_ms > 5000)
                state == STATE::LONG_PUSHED;
            else if (elapsed_ms > 500)
                state == STATE::PUSHED;
        } else if (now == 1) {
            state == STATE::RELEASED;
        }
    }
    STATE get_state() const {return state;}
    void set_led(bool enable) const {
        digitalWrite(PIN_LED, enable ? 1 : 0);
    }
private:
    simpletimer timer;
    STATE state{STATE::RELEASED};
    int prev{-1};
    static constexpr uint8_t PIN_SW{16}, PIN_LED{17};
};

class power_controller {
public:
    void init() {
        heartbeat_timer.start();
        pinMode(PIN, OUTPUT);
    }
    void poll() {
        auto elapsed_ms{heartbeat_timer.read_ms()};
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
                digitalWrite(PIN, 1);
        } else {
            digitalWrite(PIN, 0);
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
    simpletimer heartbeat_timer;
    bool last_enable{false};
    static constexpr uint8_t PIN{13};
};

class charger {
public:
    void init() {
        Serial1.begin(4800, SERIAL_8N1);
        pinMode(PIN_HB_LED, OUTPUT);
        digitalWrite(PIN_HB_LED, 0);
        sw.init();
        power.init();
        timer.start();
        heartbeat_led_timer.start();
    }
    void poll() {
        sw.poll();
        power.poll();
        auto sw_state{sw.get_state()};
        if (sw_state == manual_switch::STATE::PUSHED)
            power.set_enable(true);
        else if (sw_state == manual_switch::STATE::LONG_PUSHED)
            power.set_enable(false);
        sw.set_led(power.get_enable());
        while (Serial1.available()) {
            if (timer.read_ms() > 1000)
                msg.reset();
            timer.reset();
            int c{Serial1.read()};
            if (msg.decode(c)) {
                uint8_t param[3];
                uint8_t command{msg.get_command(param)};
                handle_command(command, param);
            }
        }
        if (heartbeat_led_timer.read_ms() > 1000) {
            heartbeat_led_timer.reset();
            heartbeat_led = !heartbeat_led;
            digitalWrite(PIN_HB_LED, heartbeat_led ? 1 : 0);
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
    manual_switch sw;
    power_controller power;
    serial_message msg;
    simpletimer timer, heartbeat_led_timer;
    bool heartbeat_led{false};
    static constexpr uint8_t PIN_HB_LED{0};
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
