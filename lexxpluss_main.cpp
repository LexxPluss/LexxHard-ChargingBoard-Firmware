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
    const CRGB led_color_status_default = CRGB(12, 64, 8);
    const CRGB led_color_status_default_dim = CRGB(6, 25, 3);
    const CRGB led_color_auto_charging = CRGB(32, 128, 0);
    const CRGB led_color_manual_charging = CRGB::OrangeRed;
    const CRGB led_color_manual_charging_dim = CRGB(64, 15, 0);
    const CRGB led_color_status_manual_charge_timeout = CRGB::HotPink;
    const CRGB led_color_status_heartbeat_timeout = CRGB::Blue;
    const CRGB led_color_status_terminal_overheat = CRGB::Red;

    enum failure_status {
        default_state = 0,
        manual_charge_timeout,
        heartbeat_timeout,
        terminal_overheat
    };

    void init() {
        FastLED.addLeds<WS2812B, SPI_DATA, GRB>(led, NUM_LEDS);
    }
    void poll() {
        if (charging) {
            if (level < 0)
                //always fill_breath during the idle state of manual charege
                fill(led_blink_crossfade(led_color_manual_charging_dim, led_color_manual_charging, 3500, 10, 40, 40, 0));
            else {
                fill_charging(level);
                last_autocharge_time_millis = millis();
            }
        } else {
            int32_t t = (int32_t)millis() - (int32_t)last_autocharge_time_millis;
            if( 0 <= t && t < 2000 ) {
                // dim default LED color when immediately after from auto charge because of using same color.
                if (t < 500){
                    fill(CRGB::Black);
                }
                else {
                    CRGB col1_dim;
                    CRGB col2_dim;
                    for (int i = 0 ; i < 3 ; i ++){
                        col1_dim.raw[i] = col1.raw[i] * (t - 500) / 1500 ;
                        col2_dim.raw[i] = col2.raw[i] * (t - 500) / 1500 ;
                    }
                    fill(led_blink_crossfade(col1_dim, col2_dim, 5000, 10, 40, 40, 0));
                }
            }
            else {
                fill(led_blink_crossfade(col1, col2, 5000, 10, 40, 40, 0));
            }
        }
        FastLED.show();
    }
    void set_charging(bool enable, int32_t level = -1) {
        this->charging = enable;
        this->level = level;
    }

    void set_led_status(failure_status s) {
        if (s == manual_charge_timeout) {
            col1 = led_color_status_default_dim;
            col2 = led_color_status_default;
        }
        else if (s == heartbeat_timeout) {
            col1 = led_color_status_default_dim;
            col2 = led_color_status_default;
        }
        else if (s == terminal_overheat) {
            col1 = CRGB::Red;
            col2 = CRGB::Red;
        }
        else { //default
            col1 = led_color_status_default_dim;
            col2 = led_color_status_default;
        }
    }

private:
    void fill(const CRGB &color) {
        for (auto &i : led)
            i = color;
    }
    
    void fill_charging(int32_t level) {
        uint32_t n{(NUM_LEDS - 3) * level / 100U}; // The number of LED to express current battery 

        //blinking tip
        static constexpr uint8_t blinking_tip_length{3}; // The number of LED to blink
        for (uint32_t i{0}; i < NUM_LEDS; ++i) {
            if( i < n )
                led[NUM_LEDS - 1 - i] = led_color_auto_charging;
            else if (i < n + blinking_tip_length){
                led[NUM_LEDS - 1 - i] = led_blink_crossfade(CRGB::Black, led_color_auto_charging, 1500, 40, 20, 20, 0);
            }
            else {
                led[NUM_LEDS - 1 - i] = CRGB::Black;
            }
        }
    }
    CRGB led_blink_crossfade(CRGB color1, CRGB color2, int32_t period_ms, int32_t lit_percent, int32_t raising_percent, int32_t falling_percent, int32_t offset_ms){
        /* 
         * This is the function for changing the color of LED strip gradually. The returned color blended both color1 and color2 by calculated ratio which changing by time. 
         * If we set black to color1 and other color to color2, the return color will be blinked. 
         * The return value is the CRGB color
         * period : the cycle time for each blink [counts]
         * duty_percent : The time ratio of fully lighting vs. cycle time[0 - 100 %]
         * raising_percent : The time ratio of raising raising slope vs. cycle time[0 - 100 %]
         * falling_percent : The time ratio of raising falling slope vs. cycle time[0 - 100 %]
         * offset_ms
         * (This function can't be controlled the origin state of the color, its fully depends on the time elapsed from start up)
        */
        uint32_t elapsed_ms{millis() % period_ms};
  
        // calc color ratio
        uint32_t color_ratio_percent{0};
        uint32_t time_ratio_permill{elapsed_ms * 1000U / period_ms};
        if( time_ratio_permill < raising_percent * 10U ) {  // raising (brightening) phase
            color_ratio_percent = time_ratio_permill * 10U / raising_percent;
        }
        else if ( time_ratio_permill < (raising_percent + lit_percent)*10U ){ // lit phase
            color_ratio_percent = 100U;
        }
        else if ( time_ratio_permill < (raising_percent + lit_percent + falling_percent)*10U ){ // falling (dimming) phase
            color_ratio_percent = ( (raising_percent + lit_percent + falling_percent ) * 10 - time_ratio_permill) * 10U / falling_percent;
        }
        else {
            color_ratio_percent = 0U;
        }
        // blend the colors
        CRGB ret{CRGB::Black};
        for (int i = 0; i < 3; i ++) { // 0:R, 1:G, 2:B
            ret.raw[i] = ((int32_t)color2.raw[i] - (int32_t)color1.raw[i]) * (int32_t)color_ratio_percent / 100 + color1.raw[i];
        }
        return ret;
    }

    
    static constexpr uint32_t NUM_LEDS{45};
    CRGB led[NUM_LEDS];
    CRGB status_color{led_color_status_default};
    CRGB col1{led_color_status_default_dim}, col2{led_color_status_default};
    int32_t level{0};
    int32_t dim_led_color_percent{0};
    unsigned long last_autocharge_time_millis{0};
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
                led.set_led_status(led_controller::failure_status::heartbeat_timeout);
            }
            if (terminal.is_overheat()) {
                Serial.println("terminal overheat, stop charging.");
                set_auto_enable(false);
                led.set_led_status(led_controller::failure_status::terminal_overheat);
            }
        }
        if (relay.is_manual_mode()) {
            auto elapsed_ms{manual_charging_timer.read_ms()};
            if (elapsed_ms > 7200000) {
                Serial.println("manual charging timeout, stop charging.");
                set_manual_enable(false);
                led.set_led_status(led_controller::failure_status::manual_charge_timeout);
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
                led.set_led_status(led_controller::failure_status::default_state);  //back to default color when enabled
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
            led.set_led_status(led_controller::failure_status::default_state);  //back to default color when enabled
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
        if (sw_state == manual_switch::STATE::PUSHED) {
            power.set_manual_enable(true);
            Serial.println("Manual Charge");
        }
        else if (sw_state == manual_switch::STATE::LONG_PUSHED) {
            power.set_manual_enable(false);
            Serial.println("Auto Charge");
        }
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
