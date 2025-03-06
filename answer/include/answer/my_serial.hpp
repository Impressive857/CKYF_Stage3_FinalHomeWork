#ifndef _MY_SERIAL_HPP_
#define _MY_SERIAL_HPP_

#include "../serialPro/serialPro.h"

#include "example_interfaces/msg/u_int64.hpp"
#include "example_interfaces/msg/int64.hpp"

namespace my_serial {
    constexpr uint64_t FRAME_HEAD = 0xAA;
    constexpr uint64_t FRAME_TAIL = 0xBB;
    constexpr uint64_t CMD_WRITE = 0x00;
    constexpr uint64_t CMD_READ = 0x01;

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
        :public sp::serialPro<Head, Tail>
    {
    public:
        MySerial(const std::string& port_path, int baud_rate)
            :sp::serialPro<Head, Tail>(port_path, baud_rate)
        {
            registerChecker([](const Head& head)->int {return head.SOF != FRAME_HEAD;});
            registerChecker([](const Tail& tail, const uint8_t*, const int&)->int {return tail.crc16 != FRAME_TAIL;});
            setGetId([](const Head& head)->int {return head.cmd_id;});
            setGetLength([](const Head& head)->int {return static_cast<int>(head.length);});
        }
        
        template<typename _Ty>
        bool write(const _Ty& data) {
            return sp::serialPro<Head, Tail>::write({ FRAME_HEAD,sizeof(data),CMD_WRITE }, data);
        }
    public:
    private:
    };
} // ^^ namespace my_serial

#endif // ^^ !_MY_SERIAL_HPP_