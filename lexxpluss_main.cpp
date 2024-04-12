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

 /* 
 For special custom charger
 It have below features same as normal
 * Charger
 * AC Relay (for turning on Charger)

 But it have 2 of those (charger x2).

 It have below features different from normal
 * Manual Charging Button
   * The charger have no buttons. but it have charging detection wire on charging connector.
 * Manual Charging LED
   * The charger have LEDs. It can blink 3 colors by 2 pins (Blue / Yellow / White(Blut+Yellow))
 * Current Sensor
   * New features. It can detect charger input current.
 * FAN
   * No any changes from normal. The charger have only 1 FAN. So We have to control by the status of 2 chargers.

But it have 2 of those.

 It not have below features unlike as normal
 * LED strip light
 * Auto Charge Connector
 * Thermistor
 * IrDA
 * DC Relay
 
 */

#include <Arduino.h>
#include <FastLED.h>
#include "serial_message.hpp"
#include "simpletimer.hpp"
#include "lexxpluss_main.hpp"

namespace {


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
                /* 
                [BUG] : By PCB circuit bug, the fan revolute always.
                When turning off the circuit it revolute by circuit leakage current.
                This cause the circuit failure (Break).
                So, This code don't control fan, it always ON.
                */
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


/*
For spectial custom charger:
The charger is turning on when connected the robots.
For that the connector have connection detect pins.
The pins connect to our PCB as manual switchs.
So, this software implement the connection detect by diverting the manual_swtich()
The different is belows:
* It is noisy due to the wiring length and circuit configuration. So it newly features filter.
* The charger have 2 circuits and connection detects. So the constant pin number is deleted and constructor have new arguments.
*/
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
        if (chattering_elapsed_ms > 500) { 
            now_filtered_sw = now;
        }
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
                }else if (elapsed_ms > 100){
                    state = STATE::PUSHED;
                }
            } else if (now_filtered_sw == 0) {
                state = STATE::RELEASED;
            } 
            if (prev_state != state) {
                Serial.print("[switch ");
                Serial.print(PIN_SW);
                Serial.print("] pin ");
                switch (state) {
                case STATE::RELEASED:    Serial.println("switch state change to RELEASED"); break;
                case STATE::PUSHED:      Serial.println("switch state change to PUSHED"); break;
                case STATE::LONG_PUSHED: Serial.println("switch state change to LONG PUSHED"); break;
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
};


/*
For special custom charger
* LexxPluss Normal charger have 1 AC and DC relays each.
  * AC relay for turning on charger power
  * DC relay for turning on Auto charge connector
* This special charger have 2 AC relays.
  * AC relay for turning on 24V charger power
  * AC relay for turning on 48V charger power
* Those are using same as normal AC relay.
* So, this code diverted normal AC relay code deleted about DC and Auto Charge functions.
* Also, this class constructed 2 instances.
* So, it deleted const pin numbers and add arguments on init() function.
*/
class relay_controller {
public:
    void init(int relay) {
        PIN_AC = relay;
        pinMode(PIN_AC, OUTPUT);
        digitalWrite(PIN_AC, 0);
        Serial.print("PIN=");
        Serial.println(PIN_AC);
    }
    void set_enable(bool enable) {
        this->enable = enable;
        if (enable) {
                digitalWrite(PIN_AC, 1);
        } else {
            digitalWrite(PIN_AC, 0);
        }
    }
    bool is_manual_mode() const {
        return enable ;
    }
private:
    bool enable{false};
    int PIN_AC;
};

/*
Completely new feature
*/
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

/*
For special custom charger
* It not have these features
  * LED strip
  * Auto Charge Terminal
  * IrDA
    * IrDA heatbeat signal
So, these functions are deleted.
And the charger have 1 fan remainly.
But this power_controller class are constructed 2 instance for 2 internal chargers (24V/48V).
The fan needs to control with the information of 2 power_controller classes.
So, the this code moved the fan control function to higher layer.
Moreover the charging time is different for each charger.
24V system is same as normal.
48V system is 2x long for charging because the charging electric current is lower.
So, the time out changed that can changed by init() argument for each charger.
*/
class power_controller {
public:
    void init(int pin_relay, int pin_current, int timeout_h ) {
        relay.init(pin_relay);
        current.init(pin_current);
        timeout_ms = (uint32_t)timeout_h * 60 * 60 * 1000;
    }
    void poll() {
        current.poll();
        if (relay.is_manual_mode()) {
            uint32_t elapsed_ms{manual_charging_timer.read_ms()};
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
        }
    }
    void set_manual_enable(bool enable) {
        relay.set_enable(enable);
        if (enable) {
            manual_charging_timer.reset();
            manual_charging_timer.start();
        } else {
            manual_charging_timer.stop();
            manual_charging_timer.reset();
        }
    }
    bool get_manual_enable() const {
        return relay.is_manual_mode();
    }
    uint32_t get_current_mA() {
        return current.get_current_mA();
    }
private:
    relay_controller relay;
    simpletimer manual_charging_timer;
    current_sensor current;
    uint32_t timeout_ms;
};

/*
For speacial custom charger.
* Serial1 is IrDA so deleted.
*/
class charging_board {
public:
    void init() {
        Serial.begin(115200, SERIAL_8N1);
        pinMode(PIN_HB_LED, OUTPUT);
        pinMode(PIN_INDICATOR_24V, OUTPUT); //LED front panel 24V blue
        pinMode(PIN_INDICATOR_48V, OUTPUT); //LED front panel 48V blue
        digitalWrite(PIN_HB_LED, 0);
        digitalWrite(PIN_INDICATOR_24V,0);
        digitalWrite(PIN_INDICATOR_48V,0);
        sw_24V.init();
        sw_48V.init();
        power_24V.init(13, 3, 2);
        power_48V.init(14, 4, 4);
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
         auto sw_state_24V{sw_24V.get_state()};
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
        if (heartbeat_led_timer.read_ms() > 1000) {
            heartbeat_led_timer.reset();
            heartbeat_led = !heartbeat_led;
            digitalWrite(PIN_HB_LED, heartbeat_led ? 1 : 0);
        }

        // LED indicator
        /* 
        For special custom charger
        it have LEDs that have 3 colors and 2 pins (Blue / Yellow / White(Blue+Yellow))
        This code is implemented as different role for 2 pins as belows.
        * Yellow : Simple light it show charger power status
          * When the charger contol relay is ON when it lit.
        * Blue : General Indicator
          * It blink for showing status of this charger
          * When not charging (Yellow is not lit)
            * Long lit and short not lit when not charging (Blue_____ _____ _____) : Waiting (heartbeat)
            * Short lit and short not lit when not charging (Blue__  __  __  __) : Connected
          * When charging (Yellow is lit)
            * Blink fastly when charging (White/Yellow_-_-_-_-_) : Charging (Blink speed is changed by charging current)
            * OFF when charging(Yellow__________) : No current (Abnormal status)
        On this code, these LEDs are implemented by different place.
        * Yellow: manual_switch class as manual switch LED.
        * Blue : Below
        */
        if (power_24V.get_manual_enable()){ //When charging
            if(power_24V.get_current_mA() < 300) { // Yellow no blink when breaker is off
                digitalWrite(PIN_INDICATOR_24V, 0);
            }
            else if(indicator_timer_24V.read_ms() > power_24V.get_current_mA()/8){ //Blink when charging between yellow/white
                indicator_timer_24V.reset();
                digitalWrite(PIN_INDICATOR_24V, 1-digitalRead(PIN_INDICATOR_24V)); 
            }
        } else { //Blink when waiting between blue/none
            if( sw_24V.get_raw_read() != 1 ) { //When not detected connection
                if(indicator_timer_24V.read_ms() < 4900){
                    digitalWrite(PIN_INDICATOR_24V,1);
                }else if(indicator_timer_24V.read_ms() < 5000){
                    digitalWrite(PIN_INDICATOR_24V,0);
                }else {
                    digitalWrite(PIN_INDICATOR_24V,1);
                    indicator_timer_24V.reset();
                }
            }else{ //When detected connection (but not started the charge)
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
        if (power_48V.get_manual_enable()){ //When charging
            if(power_48V.get_current_mA() < 300) { // Yellow no blink when breaker is off
                digitalWrite(PIN_INDICATOR_48V, 0);
            }
            else if(indicator_timer_48V.read_ms() > power_48V.get_current_mA()/8){ //Blink when charging between yellow/white
                indicator_timer_48V.reset();
                digitalWrite(PIN_INDICATOR_48V, 1-digitalRead(PIN_INDICATOR_48V));
            }
        } else { //Blink when waiting between blue/none
            if( sw_48V.get_raw_read() != 1 ) { //When not detected connection
                if(indicator_timer_48V.read_ms() < 4900){
                    digitalWrite(PIN_INDICATOR_48V,1);
                }else if(indicator_timer_48V.read_ms() < 5000){
                    digitalWrite(PIN_INDICATOR_48V,0);
                }else {
                    digitalWrite(PIN_INDICATOR_48V,1);
                    indicator_timer_48V.reset();
                }
            }else{ //When detected connection (but not started the charge)
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

        delay(30);
    }
private:
    manual_switch sw_24V{12, 1}, sw_48V{15, 2};
    power_controller power_24V, power_48V;
    serial_message msg;
    simpletimer heartbeat_led_timer, indicator_timer_24V, indicator_timer_48V;
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
