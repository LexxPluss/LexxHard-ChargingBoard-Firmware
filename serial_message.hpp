#pragma once

#include "simpletimer.hpp"

class serial_message {
public:
    void init() {
        timer.start();
    }
    bool decode(uint8_t data) {
        if (timer.read_ms() > 1000)
            reset();
        timer.reset();
        return decode_single(data);
    }
    uint8_t get_command(uint8_t param[3]) const {
        param[0] = request.detail.param[0];
        param[1] = request.detail.param[1];
        param[2] = request.detail.param[2];
        return request.detail.command;
    }
    static void compose(uint8_t buf[8], uint8_t command, uint8_t *param) {
        buf[0] = 'L';
        buf[1] = 'P';
        buf[2] = command;
        if (param != nullptr) {
            buf[3] = param[0];
            buf[4] = param[1];
            buf[5] = param[2];
        } else {
            buf[3] = buf[4] = buf[5] = 0;
        }
        Arduino_CRC32 crc32;
        uint32_t crc32_calc = crc32.calc(buf, 6);
        buf[6] = crc32_calc;
        buf[7] = crc32_calc >> 8;
    }
    static constexpr uint8_t HEARTBEAT = 0x01;
    static constexpr uint8_t POWERON = 0x02;
    static constexpr uint8_t POWEROFF = 0x03;
private:
    void reset() {state = STATE::HEAD0;}
    bool decode_single(uint8_t data) {
        bool result{false};
        switch (state) {
        case STATE::HEAD0:
            request.raw[0] = data;
            if (data == 'L')
                state = STATE::HEAD1;
            break;
        case STATE::HEAD1:
            request.raw[1] = data;
            if (data == 'P') {
                state = STATE::DATA;
                data_count = 2;
            } else {
                reset();
            }
            break;
        case STATE::DATA:
            request.raw[data_count] = data;
            if (++data_count >= sizeof request.raw) {
                reset();
                Arduino_CRC32 crc32;
                uint32_t crc32_calc = crc32.calc(request.raw, 6);
                if (((crc32_calc >> 0) & 0xff) == request.detail.sum[0] &&
                    ((crc32_calc >> 8) & 0xff) == request.detail.sum[1])
                    result = true;
            }
            break;
        }
        return result;
    }
    simpletimer timer;
    union {
        uint8_t raw[8];
        struct {
            uint8_t head[2];
            uint8_t command;
            uint8_t param[3];
            uint8_t sum[2];
        } detail;
    } request;
    uint32_t data_count{0};
    enum class STATE {
        HEAD0, HEAD1, DATA
    } state{STATE::HEAD0};
};

// vim: expandtab shiftwidth=4:
