#ifndef _MY_SERIAL_HPP_
#define _MY_SERIAL_HPP_

#include "serial.hpp"

#include "example_interfaces/msg/u_int64.hpp"
#include "example_interfaces/msg/int64.hpp"

namespace my_serial {
    const uint64_t FRAME_HEAD = 0xAA;
    const uint64_t FRAME_TAIL = 0xBB;

    struct Head {
        uint64_t SOF = FRAME_HEAD;    // 帧头
        uint64_t length = 0;    // 数据长度
        uint64_t cmd_id = 0x00; // 命令字
    };

    struct password_send_t {
        int64_t password1 = 0; // 密码片段1
        int64_t password2 = 0; // 密码片段2
    };

    struct password_receive_t {
        int64_t password = 0; // 密码 
    };

    struct Tail {
        uint64_t crc16 = FRAME_TAIL; // 校验
    };

    class MySerial
        :public serial::Serial<Head, Tail>
    {
    public:
        MySerial(const std::string& port_path, speed_t baud_rate, bool is_big_endian = true)
            :serial::Serial<Head, Tail>(port_path, baud_rate, is_big_endian)
        {
            set_head_check([](const Head& head) {return head.SOF == FRAME_HEAD;});
            set_tail_check([](const Tail& tail) {return tail.crc16 == FRAME_TAIL;});
            set_get_cmd_id([](const Head& head) {return head.cmd_id;});
            set_get_length([](const Head& head) {return head.length;});
            set_set_length([](Head& head, uint64_t length) { head.length = length;});
            set_set_cmd_id([](Head& head, uint64_t cmd_id) {head.cmd_id = cmd_id;});
        }
    public:
    private:
    };
} // ^^ namespace my_serial

#endif // ^^ !_MY_SERIAL_HPP_