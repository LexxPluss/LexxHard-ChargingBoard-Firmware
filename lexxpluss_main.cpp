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
        digitalWrite(PIN_LED, 0);
    }
    void poll() {
        STATE prev_state{state};
        int now{digitalRead(PIN_SW)};
        if (prev != now) {
            Serial.print("power_switch change to ");
            Serial.println(now);
            prev = now;
            timer.reset();
            timer.start();
        } else if (now == 0) {
            auto elapsed_ms{timer.read_ms()};
            if (elapsed_ms > 5000)
                state = STATE::LONG_PUSHED;
            else if (elapsed_ms > 500)
                state = STATE::PUSHED;
        } else if (now == 1) {
            state = STATE::RELEASED;
        }
        if (prev_state != state) {
            switch (state) {
            case STATE::RELEASED:    Serial.println("switch state change to RELEASED"); break;
            case STATE::PUSHED:      Serial.println("switch state change to PUSHED"); break;
            case STATE::LONG_PUSHED: Serial.println("switch state change to LONG PUSHED"); break;
            }
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
        pinMode(PIN_AC, OUTPUT);
        digitalWrite(PIN_AC, 0);
        pinMode(PIN_FB, OUTPUT);
        digitalWrite(PIN_FB, 1);
    }
    void poll() {
        poll_temperature();
        if (mode == MODE::AUTO) {
            auto elapsed_ms{heartbeat_timer.read_ms()};
            if (elapsed_ms > 10000)
                set_auto_enable(false);
            if (is_connector_overheat())
                set_auto_enable(false);
        }
    }
    void ping() {
        heartbeat_timer.reset();
    }
    void set_auto_enable(bool enable) {
        if (mode != MODE::MANUAL) {
            if (enable && !is_connector_overheat()) {
                digitalWrite(PIN_AC, 1);
                digitalWrite(PIN_FB, 1);
                last_enable = true;
                mode = MODE::AUTO;
            } else {
                digitalWrite(PIN_AC, 0);
                digitalWrite(PIN_FB, 1);
                last_enable = false;
                mode = MODE::STOP;
            }
        }
    }
    void set_manual_enable(bool enable) {
        if (enable) {
            digitalWrite(PIN_AC, 1);
            digitalWrite(PIN_FB, 0);
            last_enable = true;
            mode = MODE::MANUAL;
        } else {
            digitalWrite(PIN_AC, 0);
            digitalWrite(PIN_FB, 1);
            last_enable = false;
            mode = MODE::STOP;
        }
    }
    bool get_auto_enable() const {
        return mode == MODE::AUTO && last_enable;
    }
    bool get_manual_enable() const {
        return mode == MODE::MANUAL && last_enable;
    }
private:
    void poll_temperature() {
        temperature[0] = get_temperature(A0);
        temperature[1] = get_temperature(A1);
    }
    int get_temperature(int pin) const {
        int value{analogRead(pin)};
        if (value <= 0)
            value = 1;
        else if (value >= 1024)
            value = 1023;
        // see https://lexxpluss.esa.io/posts/459
        float adc_voltage{value * 5.0f / 1024.0f};
        static constexpr float R0{3300.0f}, B{3970.0f}, T0{373.0f}, Rpu{10000.0f};
        float R{Rpu * adc_voltage / (5.0f - adc_voltage)};
        float T{1.0f / (logf(R / R0) / B + 1.0f / T0)};
        return static_cast<int>(T - 273.0f);
    }
    bool is_connector_overheat() const {
        return temperature[0] > 80 || temperature[1] > 80;
    }
    enum class MODE {
        STOP, AUTO, MANUAL
    } mode{MODE::STOP};
    simpletimer heartbeat_timer;
    int temperature[2]{0, 0};
    bool last_enable{false};
    static constexpr uint8_t PIN_AC{13}, PIN_FB{14};
};

class charger {
public:
    void init() {
        Serial.begin(115200, SERIAL_8N1);
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
            power.set_manual_enable(true);
        else if (sw_state == manual_switch::STATE::LONG_PUSHED)
            power.set_manual_enable(false);
        sw.set_led(power.get_manual_enable());
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
        power.set_auto_enable(param[1] != 0);
        uint8_t buf[8], send_param[3]{param[0], power.get_auto_enable()};
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
