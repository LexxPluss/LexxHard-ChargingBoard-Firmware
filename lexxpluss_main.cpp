/*
 * Copyright (c) 2022, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>
#include <FastLED.h>
#include "serial_message.hpp"
#include "simpletimer.hpp"
#include "lexxpluss_main.hpp"

namespace {

class led_controller {
public:
    void init() {
        FastLED.addLeds<WS2812B, SPI_DATA, GRB>(led, NUM_LEDS);
    }
    void poll() {
        if (charging) {
            if (level < 0)
                fill_breath();
            else
                fill_charging(level);
        } else {
            fill(CRGB{CRGB::Black});
        }
        FastLED.show();
        ++counter;
    }
    void set_charging(bool enable, int32_t level = -1) {
        if (!this->charging && enable)
            counter = 0;
        this->charging = enable;
        this->level = level;
    }
private:
    void fill(const CRGB &color) {
        for (auto &i : led)
            i = color;
    }
    void fill_charging(int32_t level) {
        static constexpr uint32_t thres{60};
        uint32_t tail;
        if (counter >= thres * 2)
            counter = 0;
        if (counter >= thres)
            tail = NUM_LEDS;
        else
            tail = NUM_LEDS * counter / thres;
        static const CRGB color{CRGB::OrangeRed}, black{CRGB::Black};
        uint32_t n{NUM_LEDS * level / 100U};
        if (n > tail)
            n = tail;
        for (uint32_t i{0}; i < NUM_LEDS; ++i)
            led[NUM_LEDS - 1 - i] = i < n ? color : black;
    }
    void fill_breath() {
        static constexpr uint32_t thres{60};
        if (counter >= thres * 2)
            counter = 0;
        uint32_t percent;
        if (counter < thres)
            percent = counter * 100 / thres;
        else
            percent = (thres * 2 - counter) * 100 / thres;
        CRGB color{CRGB::OrangeRed};
        for (auto &i : color.raw)
            i = i * percent / 100;
        fill(color);
    }
    static constexpr uint32_t NUM_LEDS{45};
    CRGB led[NUM_LEDS];
    uint32_t counter{0};
    int32_t level{0};
    bool charging{false};
};


class fan_controller {
public:
    void init() const{
        pinMode(PIN_FAN, OUTPUT);
        analogWrite(PIN_FAN, 0);
    }
    void poll() {
        if(charging != prev)
        {
            if (charging) {
                analogWrite(PIN_FAN, 255);
            } else {
                analogWrite(PIN_FAN, 0);
            }       
        }
        prev = charging;
    }
    void set_charging(bool enable) {
        this->charging = enable;
    }

private:
    bool charging{false};
    static constexpr uint8_t PIN_FAN{4};
    bool prev{false};
};

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

class power_terminal_individual {
public:
    power_terminal_individual(int pin) : pin(pin) {}
    void poll() {
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
        temperature = T - 273.0f;
    }
    bool is_overheat() const {
        return temperature > 80;
    }
private:
    int pin, temperature{0};
};

class power_terminal {
public:
    void poll() {
        terminal[0].poll();
        terminal[1].poll();
    }
    bool is_overheat() const {
        return terminal[0].is_overheat() || terminal[1].is_overheat();
    }
private:
    power_terminal_individual terminal[2]{A0, A1};
};

enum class CHARGING_MODE {
    AUTO, MANUAL
};

class relay_controller {
public:
    void init() {
        pinMode(PIN_AC, OUTPUT);
        digitalWrite(PIN_AC, 0);
        pinMode(PIN_DC, OUTPUT);
        digitalWrite(PIN_DC, 0);
    }
    void set_enable(bool enable, CHARGING_MODE mode = CHARGING_MODE::MANUAL) {
        this->enable = enable;
        this->mode = mode;
        if (enable) {
            if (mode == CHARGING_MODE::AUTO) {
                digitalWrite(PIN_AC, 1);
                digitalWrite(PIN_DC, 1);
            } else {
                digitalWrite(PIN_AC, 1);
                digitalWrite(PIN_DC, 0);
            }
        } else {
            digitalWrite(PIN_AC, 0);
            digitalWrite(PIN_DC, 0);
        }
    }
    bool is_auto_mode() const {
        return enable && mode == CHARGING_MODE::AUTO;
    }
    bool is_manual_mode() const {
        return enable && mode == CHARGING_MODE::MANUAL;
    }
private:
    CHARGING_MODE mode{CHARGING_MODE::MANUAL};
    bool enable{false};
    static constexpr uint8_t PIN_AC{13}, PIN_DC{14};
};

class power_controller {
public:
    void init() {
        led.init();
        relay.init();
        fan.init();
        heartbeat_timer.start();
    }
    void poll() {
        led.poll();
        terminal.poll();
        fan.poll();
        if (relay.is_auto_mode()) {
            auto elapsed_ms{heartbeat_timer.read_ms()};
            if (elapsed_ms > 10000) {
                Serial.println("heartbeat timeout, stop charging.");
                set_auto_enable(false);
            }
            if (terminal.is_overheat()) {
                Serial.println("terminal overheat, stop charging.");
                set_auto_enable(false);
            }
        }
        if (relay.is_manual_mode()) {
            auto elapsed_ms{manual_charging_timer.read_ms()};
            if (elapsed_ms > 7200000) {
                Serial.println("manual charging timeout, stop charging.");
                set_manual_enable(false);
            }
        }
    }
    void ping() {
        heartbeat_timer.reset();
    }
    void set_auto_enable(bool enable, int32_t level = -1) {
        if (!relay.is_manual_mode()) {
            if (enable && !terminal.is_overheat()) {
                relay.set_enable(true, CHARGING_MODE::AUTO);
                led.set_charging(true, level);
                fan.set_charging(true);
            } else {
                relay.set_enable(false);
                led.set_charging(false);
                fan.set_charging(false);
            }
        }
    }
    void set_manual_enable(bool enable) {
        relay.set_enable(enable);
        led.set_charging(enable);
        fan.set_charging(enable);
        if (enable) {
            manual_charging_timer.reset();
            manual_charging_timer.start();
        } else {
            manual_charging_timer.stop();
            manual_charging_timer.reset();
        }
    }
    bool get_auto_enable() const {
        return relay.is_auto_mode();
    }
    bool get_manual_enable() const {
        return relay.is_manual_mode();
    }
private:
    led_controller led;
    relay_controller relay;
    fan_controller fan;
    power_terminal terminal;
    simpletimer heartbeat_timer, manual_charging_timer;
};

class charging_board {
public:
    void init() {
        Serial.begin(115200, SERIAL_8N1);
        Serial1.begin(4800, SERIAL_8N1);
        pinMode(PIN_HB_LED, OUTPUT);
        digitalWrite(PIN_HB_LED, 0);
        sw.init();
        power.init();
        irda_timer.start();
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
            if (irda_timer.read_ms() > 1000)
                msg.reset();
            irda_timer.reset();
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
    void handle_command(uint8_t command, uint8_t (&param)[3]) {
        switch (command) {
        case serial_message::HEARTBEAT:
            heartbeat(param);
            break;
        default:
            break;
        }
    }
    void heartbeat(const uint8_t (&param)[3]) {
        power.set_auto_enable(param[1] != 0, param[2]);
        uint8_t buf[8], send_param[3]{param[0], power.get_auto_enable()};
        serial_message::compose(buf, serial_message::HEARTBEAT, send_param);
        Serial1.write(buf, sizeof buf);
        power.ping();
    }
    manual_switch sw;
    power_controller power;
    serial_message msg;
    simpletimer irda_timer, heartbeat_led_timer;
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
