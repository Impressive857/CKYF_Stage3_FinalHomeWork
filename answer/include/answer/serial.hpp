#ifndef _SERIAL_HPP_
#define _SERIAL_HPP_

#include <fcntl.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <functional>
#include <type_traits>
#include <cstring>

namespace serial {

    const uint64_t CMD_WRITE = 0x00;
    const uint64_t CMD_READ = 0x01;

    template<typename _Head, typename _Tail>
    class Serial {
    public:
        Serial() = delete;
        Serial(const std::string& port_path, speed_t baud_rate, bool is_big_endian = true)
            :m_port_path(port_path), m_baud_rate(baud_rate), m_is_big_endian(is_big_endian)
        {
            m_fd = open(m_port_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (m_fd < 0) {
                std::cerr << "fail to open serial port " << m_port_path << "\n";
                m_is_ok = false;
                m_is_open = false;
            }
            else {
                m_is_open = true;
                fcntl(m_fd, F_SETFL, 0);
                if (!set_port_config(std::bind(&serial::Serial<_Head, _Tail>::default_port_config, this, std::placeholders::_1))) {
                    std::cerr << "fail to init port " << m_port_path << " 's config\n";
                    m_is_open = false;
                    close(m_fd);
                }
                else {
                    m_is_ok = true;
                }
            }
        }
        ~Serial() {
            if (m_is_open) {
                close(m_fd);
            }
        }
    public:
        /// @brief 向串口写数据
        /// @param data 要写入的数据
        /// @return 是否写入成功
        bool write_to_port(const void* data, size_t length) {
            if (!m_is_open) {
                std::cerr << "fail to write, port is not open!\n";
                return false;
            }
            if (!m_is_ok) {
                std::cerr << "fail to write, port is not ok!\n";
                return false;
            }
            if (nullptr == data) {
                std::cerr << "data ptr is nullptr!\n";
                return false;
            }
            // 写入长度为结构体大小加上数据长度
            size_t write_length = sizeof(_Head) + sizeof(_Tail) + length;

            m_set_length_fn(m_head, length);
            m_set_cmd_id_fn(m_head, CMD_WRITE);

            char* buffer = new char[write_length];

            std::memcpy(buffer, &m_head, sizeof(m_head));

            std::memcpy(buffer + sizeof(m_head), data, length);

            std::memcpy(buffer + sizeof(m_head) + length, &m_tail, sizeof(m_tail));

            ssize_t bytes_written = write(m_fd, buffer, write_length);

            delete buffer;

            if (bytes_written < 0) {
                std::cerr << "fail to write!\n";
                return false;
            }
            return bytes_written == static_cast<ssize_t>(write_length);
        }
        /// @brief 从串口读取数据
        /// @param data 读取的数据
        /// @param length 读取的长度
        /// @return 是否读取成功
        /// @warning 传入的data指针是引用，获取数据失败时为nullptr
        /// @warning 会在内部根据数据大小分配内存，会覆盖传入的data，最好传入nullptr
        /// @warning 获取完数据后内存未释放，用完数据记得释放内存
        bool read_from_port(void*& data, size_t& data_length) {
            if (!m_is_open) {
                std::cerr << "fail to read, port is not open!\n";
                return false;
            }
            if (!m_is_ok) {
                std::cerr << "fail to read, port is not ok!\n";
                return false;
            }
            if (nullptr != data) {
                delete data;
                data = nullptr;
            }

            char head_buffer[sizeof(m_head)];
            ssize_t head_read_length = read(m_fd, head_buffer, sizeof(m_head));
            if (sizeof(m_head) != head_read_length) {
                std::cerr << "fail to read head!";
                return false;
            }

            std::memcpy(&m_head, head_buffer, sizeof(m_head));

            if (!m_head_check_fn(m_head)) {
                std::cerr << "head check is not accessed!";
                return false;
            }

            data_length = m_get_length_fn(m_head);

            data = new char[data_length];

            ssize_t data_read_length = read(m_fd, data, data_length);
            if (data_length != data_read_length) {
                std::cerr << "fail to read data!";

                // 读取数据失败要及时释放内存
                delete data;
                data = nullptr;

                data_length = 0;

                return false;
            }

            char tail_buffer[sizeof(m_tail)];
            ssize_t tail_read_length = read(m_fd, tail_buffer, sizeof(m_tail));
            if (sizeof(m_tail) != tail_read_length) {
                std::cerr << "fail to read tail!";

                // 读取数据失败要及时释放内存
                delete data;
                data = nullptr;

                data_length = 0;

                return false;
            }

            if (!m_tail_check_fn(m_tail)) {
                std::cerr << "tail check is not accessed!";

                // 读取数据失败要及时释放内存
                delete data;
                data = nullptr;

                data_length = 0;

                return false;
            }

            return true;
        }
        /// @brief 检查串口是否正常
        bool is_ok() const {
            return m_is_ok;
        }
        /// @brief 检查串口是否打开
        bool is_open() const {
            return m_is_open;
        }
        /// @brief 设置波特率
        /// @param baud_rate 要设置的波特率
        void set_baud_rate(speed_t baud_rate) {
            m_baud_rate = baud_rate;
            cfsetispeed(&m_options, m_baud_rate);
            cfsetospeed(&m_options, m_baud_rate);
        }
        /// @brief 获取当前串口波特率
        /// @return 当前串口波特率
        speed_t baud_rate() const {
            return m_baud_rate;
        }
        /// @brief 设置串口属性
        /// @param option_fn 串口属性函数
        /// @return 是否设置成功
        bool set_port_config(std::function<void(struct termios&)> option_fn) {
            if (tcgetattr(m_fd, &m_options) != 0) {
                std::cerr << "Failed to get serial port attributes\n";
                return false;
            }

            option_fn(m_options);

            // 设置波特率
            set_baud_rate(m_baud_rate);

            if (tcsetattr(m_fd, TCSANOW, &m_options) != 0) {
                std::cerr << "Failed to set serial port attributes\n";
                return false;
            }

            return true;
        }
        /// @brief 获取默认串口属性函数
        /// @return 默认串口属性函数
        std::function<void(struct termios&)> get_default_port_config_fn() {
            return std::bind(&serial::Serial<_Head, _Tail>::default_port_config, this, std::placeholders::_1);
        }
        /// @brief 设置头校验函数
        /// @param check_fn 头校验函数
        void set_head_check(std::function<bool(const _Head&)> check_fn) {
            m_head_check_fn = check_fn;
        }
        /// @brief 设置尾校验函数
        /// @param check_fn 尾校验函数
        void set_tail_check(std::function<bool(const _Tail&)> check_fn) {
            m_tail_check_fn = check_fn;
        }
        /// @brief 设置获取数据长度函数
        /// @param get_fn 获取数据长度函数
        void set_get_length(std::function<size_t(const _Head&)> get_fn) {
            m_get_length_fn = get_fn;
        }
        /// @brief 设置获取命令字函数
        /// @param get_fn 获取命令字函数
        void set_get_cmd_id(std::function<int(const _Head&)> get_fn) {
            m_get_cmd_id_fn = get_fn;
        }
        /// @brief 设置设置数据长度函数
        /// @param get_fn 设置数据长度函数
        void set_set_length(std::function<void(_Head&, size_t length)> set_fn){
            m_set_length_fn = set_fn;
        }
        /// @brief 设置设置命令字函数
        /// @param get_fn 设置命令字函数
        void set_set_cmd_id(std::function<void(_Head&, int cmd_id)> set_fn){
            m_set_cmd_id_fn = set_fn;
        }
    private:
        /// @brief 默认串口属性函数
        void default_port_config(struct termios& options) {
            // 设置数据位为 8 位
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;

            // 设置无校验位
            options.c_cflag &= ~PARENB;

            // 设置 1 位停止位
            options.c_cflag &= ~CSTOPB;

            // 启用接收
            options.c_cflag |= CREAD | CLOCAL;

            // 禁用硬件流控制
            options.c_cflag &= ~CRTSCTS;

            // 禁用软件流控制
            options.c_iflag &= ~(IXON | IXOFF | IXANY);

            // 设置原始输入模式
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

            // 设置原始输出模式
            options.c_oflag &= ~OPOST;

            // 设置最小字符数和超时时间
            options.c_cc[VMIN] = 1;
            options.c_cc[VTIME] = 0;
        }
    private:
        std::string m_port_path;                                    // 串口路径
        speed_t m_baud_rate;                                        // 波特率
        bool m_is_big_endian;                                       // 是否是大端字节序
        int m_fd;                                                   // 串口唯一标识
        bool m_is_ok;                                               // 标识串口是否正常
        bool m_is_open;                                             // 标识串口是否打开
        struct termios m_options;                                   // 串口选项
        _Head m_head;                                               // 帧头
        _Tail m_tail;                                               // 帧尾
        std::function<bool(const _Head&)> m_head_check_fn;          // 头校验函数
        std::function<bool(const _Tail&)> m_tail_check_fn;          // 尾校验函数
        std::function<size_t(const _Head&)> m_get_length_fn;        // 获得数据长度函数
        std::function<int(const _Head&)> m_get_cmd_id_fn;           // 获取命令字函数
        std::function<void(_Head&, size_t length)> m_set_length_fn; // 设置数据长度函数
        std::function<void(_Head&, int cmd_id)> m_set_cmd_id_fn;    // 设置命令字函数
    };

} // ^^ namespace serial

#endif // ^^ !_SERIAL_HPP_