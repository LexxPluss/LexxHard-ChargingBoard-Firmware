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

// class led_controller {
// public:
//     void init() {
//         FastLED.addLeds<WS2812B, SPI_DATA, GRB>(led, NUM_LEDS);
//     }
//     void poll() {
//         if (charging) {
//             if (level < 0)
//                 fill_breath();
//             else
//                 fill_charging(level);
//         } else {
//             fill(CRGB{CRGB::Black});
//         }
//         FastLED.show();
//         ++counter;
//     }
//     void set_charging(bool enable, int32_t level = -1) {
//         if (!this->charging && enable)
//             counter = 0;
//         this->charging = enable;
//         this->level = level;
//     }
// private:
//     void fill(const CRGB &color) {
//         for (auto &i : led)
//             i = color;
//     }
//     void fill_charging(int32_t level) {
//         static constexpr uint32_t thres{60};
//         uint32_t tail;
//         if (counter >= thres * 2)
//             counter = 0;
//         if (counter >= thres)
//             tail = NUM_LEDS;
//         else
//             tail = NUM_LEDS * counter / thres;
//         static const CRGB color{CRGB::OrangeRed}, black{CRGB::Black};
//         uint32_t n{NUM_LEDS * level / 100U};
//         if (n > tail)
//             n = tail;
//         for (uint32_t i{0}; i < NUM_LEDS; ++i)
//             led[NUM_LEDS - 1 - i] = i < n ? color : black;
//     }
//     void fill_breath() {
//         static constexpr uint32_t thres{60};
//         if (counter >= thres * 2)
//             counter = 0;
//         uint32_t percent;
//         if (counter < thres)
//             percent = counter * 100 / thres;
//         else
//             percent = (thres * 2 - counter) * 100 / thres;
//         CRGB color{CRGB::OrangeRed};
//         for (auto &i : color.raw)
//             i = i * percent / 100;
//         fill(color);
//     }
//     static constexpr uint32_t NUM_LEDS{45};
//     CRGB led[NUM_LEDS];
//     uint32_t counter{0};
//     int32_t level{0};
//     bool charging{false};
// };


class fan_controller {
public:
    void init() const{
        pinMode(PIN_FAN, OUTPUT);
        analogWrite(PIN_FAN, 255);
    }
    void poll() {
        if(charging != prev)
        {
            if (charging) {
                analogWrite(PIN_FAN, 255);
            } else {
                analogWrite(PIN_FAN, 255);
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

// class indicator {
// public : 
//     void init(int pin){
//         pinMode(pin, OUTPUT);
//         digitalWrite(PIN_LED, 0);
//     }
//     void poll() {
        
//     }
// }

class manual_switch {
public:
    enum class STATE {
        RELEASED, PUSHED, LONG_PUSHED
    };
    manual_switch( int sw, int led ) : PIN_SW(sw) , PIN_LED(led) {}
    void init() const {
        pinMode(PIN_SW, INPUT);
        pinMode(PIN_LED, OUTPUT);
        digitalWrite(PIN_LED, 0);
    }
    void poll() {
        STATE prev_state{state};
        int now{digitalRead(PIN_SW)};
        int sw{0};
        if (prev != now) {
            Serial.print("[switch ");
            Serial.print(PIN_SW);
            Serial.print("] (non-filtered) change to ");
            Serial.println(now);
            prev = now;
            chattering_timer.reset();
            chattering_timer.start();
        }
        auto chattering_elapsed_ms{chattering_timer.read_ms()};
        if (chattering_elapsed_ms > 500) { // no change in past 500 ms
            //timer.reset();
            //timer.start();
            now_filtered_sw = now;
        } else {
//            Serial.print("[switch ");
//            Serial.print(PIN_SW);
//            Serial.print("] manual switch chatterling timer = ");
//            Serial.println(chattering_elapsed_ms);
        }
        //int now_filtered_sw{filtered_sw};
        if (prev_filtered_sw != now_filtered_sw)
        {
            timer.reset();
            timer.start();
            prev_filtered_sw = now_filtered_sw;
            Serial.print("[switch ");
            Serial.print(PIN_SW);
            Serial.print("] (FILTERED value) change to ");
            Serial.println(now_filtered_sw);
        } else {
            auto elapsed_ms{timer.read_ms()};
            if (now_filtered_sw == 1) {
                if (elapsed_ms > 10000) {
                    state = STATE::LONG_PUSHED;
                }else if (elapsed_ms > 3000){
                    state = STATE::PUSHED;
//                    Serial.print("SHORT");
//                    Serial.println(elapsed_ms);
                }
            } else if (now_filtered_sw == 0) {
                state = STATE::RELEASED;
//                Serial.print("RELEASE");
//                Serial.println(elapsed_ms);
            } 
            if (prev_state != state) {
                Serial.print("[switch ");
                Serial.print(PIN_SW);
                Serial.print("]");
                switch (state) {
                case STATE::RELEASED:    Serial.println(" pin switch state change to RELEASED"); break;
                case STATE::PUSHED:      Serial.println(" pin switch state change to PUSHED"); break;
                case STATE::LONG_PUSHED: Serial.println(" pin switch state change to LONG PUSHED"); break;
                }
            }
        }
    }
    STATE get_state() const {return state;}
    int get_raw_read() const {return digitalRead(PIN_SW);}
    void set_led(bool enable) const {
        digitalWrite(PIN_LED, enable ? 1 : 0);
    }
private:
    simpletimer timer, chattering_timer;
    STATE state{STATE::RELEASED};
    int now_filtered_sw{0};
    int prev_filtered_sw{0};
    int prev{-1};
    int PIN_SW, PIN_LED;
    //static constexpr uint8_t PIN_SW{16}, PIN_LED{17};
};

// class power_terminal_individual {
// public:
//     power_terminal_individual(int pin) : pin(pin) {}
//     void poll() {
//         int value{analogRead(pin)};
//         if (value <= 0)
//             value = 1;
//         else if (value >= 1024)
//             value = 1023;
//         // see https://lexxpluss.esa.io/posts/459
//         float adc_voltage{value * 5.0f / 1024.0f};
//         static constexpr float R0{3300.0f}, B{3970.0f}, T0{373.0f}, Rpu{10000.0f};
//         float R{Rpu * adc_voltage / (5.0f - adc_voltage)};
//         float T{1.0f / (logf(R / R0) / B + 1.0f / T0)};
//         temperature = T - 273.0f;
//     }
//     bool is_overheat() const {
//         return temperature > 80;
//     }
// private:
//     int pin, temperature{0};
// };

// class power_terminal {
// public:
//     void poll() {
//         terminal[0].poll();
//         terminal[1].poll();
//     }
//     bool is_overheat() const {
//         return terminal[0].is_overheat() || terminal[1].is_overheat();
//     }
// private:
//     power_terminal_individual terminal[2]{A0, A1};
// };

// enum class CHARGING_MODE {
//     AUTO, MANUAL
// };

class relay_controller {
public:
    //relay_controller(int pin) : PIN_AC(pin) {}
    void init(int relay) {
        PIN_AC = relay;
        pinMode(PIN_AC, OUTPUT);
        digitalWrite(PIN_AC, 0);
        // pinMode(PIN_DC, OUTPUT);
        // digitalWrite(PIN_DC, 0);
        Serial.print("PIN=");
        Serial.println(PIN_AC);
    }
    void set_enable(bool enable/*, CHARGING_MODE mode = CHARGING_MODE::MANUAL*/) {
        this->enable = enable;
        // this->mode = mode;
        if (enable) {
            // if (mode == CHARGING_MODE::AUTO) {
            //     digitalWrite(PIN_AC, 1);
            //     digitalWrite(PIN_DC, 1);
            // } else {
                digitalWrite(PIN_AC, 1);
                // digitalWrite(PIN_DC, 0);
            // }
        } else {
            digitalWrite(PIN_AC, 0);
            // digitalWrite(PIN_DC, 0);
        }
    }
    // bool is_auto_mode() const {
    //     return enable && mode == CHARGING_MODE::AUTO;
    // }
    bool is_manual_mode() const {
        return enable /*&& mode == CHARGING_MODE::MANUAL*/;
    }
private:
    // CHARGING_MODE mode{CHARGING_MODE::MANUAL};
    bool enable{false};
    // static constexpr uint8_t PIN_AC{13}, PIN_DC{14};
    int PIN_AC;
};

class current_sensor {
public:
    void init( int pin ) {
        pin_cs = pin;
    }
    void poll()
    {
        int cur_adc = analogRead(pin_cs);
        // Current ratio : 3000:1, Load res : 680 ohm -> 0.2244 [V/A]
        // A/D value 0-1023, A/D max voltage : 5[V] -> 204.6[/V]
        // 1/(204.6*0.2244)*1000 = 21.78 -> 22[mA/-]
        // However, by the experiment result is 32.
        cur_mA = cur_adc * 32;
        // Low Pass Filter 
        cur_filter_mA = (uint32_t)(cur_filter_mA * 7 + cur_mA) >> 3;

        if (millis() > __last + 1000)
            {
              __last = millis();
              Serial.print("Current Sensor pin");
              Serial.print(pin_cs);
              Serial.print(" is ");
              Serial.print(cur_adc);
              Serial.print(" (");
              Serial.print(cur_mA);
              Serial.print(" , ");
              Serial.print(cur_filter_mA);
              Serial.println(")");
            }
    }

    bool is_charging() const {
        return (cur_filter_mA >= 1000) ? true : false;
    }
    uint32_t get_current_mA(){
        return cur_filter_mA;
    }

private:
    uint16_t cur_mA{0};
    uint32_t cur_filter_mA{0};
    int pin_cs;
    uint32_t __last = 0;
};


class power_controller {
public:
    //power_controller( int relay, int current ) : pin_relay(relay, current) {}
    void init(int pin_relay, int pin_current, int timeout_h ) {
        //led.init();
        relay.init(pin_relay);
        current.init(pin_current);
        //fan.init();
        // heartbeat_timer.start();
        timeout_ms = timeout_h * 60 * 60 * 1000;
    }
    void poll() {
        // led.poll();
        // terminal.poll();
        //fan.poll();
        current.poll();
        // if (relay.is_auto_mode()) {
        //     auto elapsed_ms{heartbeat_timer.read_ms()};
        //     if (elapsed_ms > 10000) {
        //         Serial.println("heartbeat timeout, stop charging.");
        //         set_auto_enable(false);
        //     }
        //     if (terminal.is_overheat()) {
        //         Serial.println("terminal overheat, stop charging.");
        //         set_auto_enable(false);
        //     }
        // }
        if (relay.is_manual_mode()) {
            int32_t elapsed_ms{manual_charging_timer.read_ms()};
            if (elapsed_ms > timeout_ms) {
                set_manual_enable(false);
                Serial.print("Stop Charging due to time out ");
                Serial.print(elapsed_ms);
                Serial.print(" / ");
                Serial.println(timeout_ms);
            }
             if (elapsed_ms > 15000 && !(current.is_charging()) ) {
                 set_manual_enable(false);
                 Serial.println("Stop Charging due to lower limit of current (1A AC)");
             }
            // XXXXX should enable
        }
    }
    // void ping() {
    //     heartbeat_timer.reset();
    // }
    // void set_auto_enable(bool enable, int32_t level = -1) {
    //     if (!relay.is_manual_mode()) {
    //         if (enable && !terminal.is_overheat()) {
    //             relay.set_enable(true, CHARGING_MODE::AUTO);
    //             led.set_charging(true, level);
    //             fan.set_charging(true);
    //         } else {
    //             relay.set_enable(false);
    //             led.set_charging(false);
    //             fan.set_charging(false);
    //         }
    //     }
    // }
    void set_manual_enable(bool enable) {
        relay.set_enable(enable); // XXXXX この1行があるとおかしくなる
         // led.set_charging(enable);
         // fan.set_charging(enable);
        if (enable) {
            manual_charging_timer.reset();
            manual_charging_timer.start();
        } else {
            manual_charging_timer.stop();
            manual_charging_timer.reset();
        }
    }
    // bool get_auto_enable() const {
    //     return relay.is_auto_mode();
    // }
    bool get_manual_enable() const {
        return relay.is_manual_mode();
    }
    uint32_t get_current_mA() {
        return current.get_current_mA();
    }
private:
    // led_controller led;
    //relay_controller relay{pin_relay};
    relay_controller relay;
    // fan_controller fan;
    // power_terminal terminal;
    simpletimer /*heartbeat_timer,*/ manual_charging_timer;
    current_sensor current;
    //int pin_relay;
    int32_t timeout_ms;
};

class charging_board {
public:
    void init() {
        Serial.begin(115200, SERIAL_8N1);
        // Serial1.begin(4800, SERIAL_8N1);
        pinMode(PIN_HB_LED, OUTPUT);
        pinMode(PIN_INDICATOR_24V, OUTPUT); //LED front panel 24V blue
        pinMode(PIN_INDICATOR_48V, OUTPUT); //LED front panel 24V blue
        digitalWrite(PIN_HB_LED, 0);
        digitalWrite(PIN_INDICATOR_24V,0);
        digitalWrite(PIN_INDICATOR_48V,0);
        sw_24V.init();
        sw_48V.init();
        power_24V.init(13, 3, 2);
        power_48V.init(14, 4, 4);
        //irda_timer.start();
        heartbeat_led_timer.start();
        indicator_timer_24V.start();
        indicator_timer_48V.start();
        fan.init();
    }
    void poll() {
        sw_24V.poll();
        sw_48V.poll();
        power_24V.poll();
        power_48V.poll();
        fan.poll();
         auto sw_state_24V{sw_24V.get_state()}; // get stateするだけなら大丈夫
         if (sw_state_24V == manual_switch::STATE::PUSHED)
             power_24V.set_manual_enable(true);
         else if (sw_state_24V == manual_switch::STATE::RELEASED)
             power_24V.set_manual_enable(false); 
         auto sw_state_48V{sw_48V.get_state()};
         if (sw_state_48V == manual_switch::STATE::PUSHED)
             power_48V.set_manual_enable(true);
         else if (sw_state_48V == manual_switch::STATE::RELEASED)
             power_48V.set_manual_enable(false);
         sw_24V.set_led(power_24V.get_manual_enable());
         sw_48V.set_led(power_48V.get_manual_enable());

        if(power_24V.get_manual_enable() || power_48V.get_manual_enable()) {
            fan.set_charging(true);
        } else {
            fan.set_charging(false);
        }
        // while (Serial1.available()) {
        //     if (irda_timer.read_ms() > 1000)
        //         msg.reset();
        //     irda_timer.reset();
        //     int c{Serial1.read()};
        //     if (msg.decode(c)) {
        //         uint8_t param[3];
        //         uint8_t command{msg.get_command(param)};
        //         handle_command(command, param);
        //     }
        // }
        if (heartbeat_led_timer.read_ms() > 1000) {
            heartbeat_led_timer.reset();
            heartbeat_led = !heartbeat_led;
            digitalWrite(PIN_HB_LED, heartbeat_led ? 1 : 0);
            //digitalWrite(A6, heartbeat_led ? 1 : 0);
            //digitalWrite(A5, heartbeat_led ? 1 : 0);
            
        }
        if (power_24V.get_manual_enable()){
            if(power_24V.get_current_mA() < 500) { // Blue when breaker is off
                digitalWrite(PIN_INDICATOR_24V, 0);
            }
            else if(indicator_timer_24V.read_ms() > power_24V.get_current_mA()/8){ //Blink when charging between yellow/white
                indicator_timer_24V.reset();
                digitalWrite(PIN_INDICATOR_24V, 1-digitalRead(PIN_INDICATOR_24V)); 
            }
        } else { //Blink when waiting between blue/none
            if( sw_24V.get_raw_read() != 1 ) {
                if(indicator_timer_24V.read_ms() < 4900){
                    digitalWrite(PIN_INDICATOR_24V,1);
                }else if(indicator_timer_24V.read_ms() < 5000){
                    digitalWrite(PIN_INDICATOR_24V,0);
                }else {
                    digitalWrite(PIN_INDICATOR_24V,1);
                    indicator_timer_24V.reset();
                }
            }else{
                if(indicator_timer_24V.read_ms() < 250){
                    digitalWrite(PIN_INDICATOR_24V,0);
                }else if(indicator_timer_24V.read_ms() < 500){
                    digitalWrite(PIN_INDICATOR_24V,1);
                }else {
                    digitalWrite(PIN_INDICATOR_24V,0);
                    indicator_timer_24V.reset();
                }              
            }
        }
        if (power_48V.get_manual_enable()){
            if(power_48V.get_current_mA() < 500) { // Blue when breaker is off
                digitalWrite(PIN_INDICATOR_48V, 0);
            }
            else if(indicator_timer_48V.read_ms() > power_48V.get_current_mA()/8){ //Blink when charging between yellow/white
                indicator_timer_48V.reset();
                digitalWrite(PIN_INDICATOR_48V, 1-digitalRead(PIN_INDICATOR_48V));
            }
        } else { //Blink when waiting between blue/none
            if( sw_48V.get_raw_read() != 1 ) {
                if(indicator_timer_48V.read_ms() < 4900){
                    digitalWrite(PIN_INDICATOR_48V,1);
                }else if(indicator_timer_48V.read_ms() < 5000){
                    digitalWrite(PIN_INDICATOR_48V,0);
                }else {
                    digitalWrite(PIN_INDICATOR_48V,1);
                    indicator_timer_48V.reset();
                }
            }else{
                if(indicator_timer_48V.read_ms() < 250){
                    digitalWrite(PIN_INDICATOR_48V,0);
                }else if(indicator_timer_48V.read_ms() < 500){
                    digitalWrite(PIN_INDICATOR_48V,1);
                }else {
                    digitalWrite(PIN_INDICATOR_48V,0);
                    indicator_timer_48V.reset();
                }      
            }
        }
        // if (power_48V.get_manual_enable()){
        //     if(indicator_timer.read_ms() < power_48V.get_current_mA()/3 || indicator_timer.read_ms() > 5000){
        //         digitalWrite(PIN_INDICATOR_48V,1);
        //     }else{
        //         digitalWrite(PIN_INDICATOR_48V,0);
        //     }        } else {
        //     if(indicator_timer.read_ms() < 4900 || indicator_timer.read_ms() > 5000){
        //         digitalWrite(PIN_INDICATOR_48V,1);
        //     }else{
        //         digitalWrite(PIN_INDICATOR_48V,0);
        //     }
        // }
        // if (indicator_timer.read_ms() > 5000){
        //     indicator_timer.reset();
        // }

        delay(30);
    }
private:
    // void handle_command(uint8_t command, uint8_t (&param)[3]) {
    //     switch (command) {
    //     case serial_message::HEARTBEAT:
    //         heartbeat(param);
    //         break;
    //     default:
    //         break;
    //     }
    // }
    // void heartbeat(const uint8_t (&param)[3]) {
    //     power.set_auto_enable(param[1] != 0, param[2]);
    //     uint8_t buf[8], send_param[3]{param[0], power.get_auto_enable()};
    //     serial_message::compose(buf, serial_message::HEARTBEAT, send_param);
    //     Serial1.write(buf, sizeof buf);
    //     power.ping();
    // }{
    manual_switch sw_24V{12, 1}, sw_48V{15, 2};
    // power_controller power_24V{13}, power_48V{14};
    power_controller power_24V, power_48V;
    serial_message msg;
    simpletimer /*irda_timer,*/ heartbeat_led_timer, indicator_timer_24V, indicator_timer_48V;
    bool heartbeat_led{false};
    static constexpr uint8_t PIN_HB_LED{0}, PIN_INDICATOR_24V{A6}, PIN_INDICATOR_48V{A5};
    fan_controller fan;
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
