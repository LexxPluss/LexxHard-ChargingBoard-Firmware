#pragma once

#include <Arduino.h>

class simpletimer {
public:
    simpletimer() {reset();}
    void start() {
        if (!is_running) {
            is_running = true;
            started_ms = millis();
        }
    }
    void stop() {
        elapsed_ms += millis();
        is_running = false;
    }
    void reset(unsigned long now = millis()) {
        started_ms = now;
        elapsed_ms = 0;
    }
    unsigned long read_ms() const {return elapsed_ms + get_running_time();}
private:
    unsigned long get_running_time() const {
        return is_running ? (millis() - started_ms) : 0;
    }
    unsigned long started_ms = 0, elapsed_ms = 0;
    bool is_running = false;
};

// vim: expandtab shiftwidth=4:
