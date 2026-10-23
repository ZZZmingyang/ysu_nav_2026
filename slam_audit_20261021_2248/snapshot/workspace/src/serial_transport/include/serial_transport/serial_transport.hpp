#ifndef SERIAL_TRANSPORT_HPP_
#define SERIAL_TRANSPORT_HPP_

#include "serial_transport/struct_typedef.h"
#include <asio.hpp>
#include <thread>
#include <atomic>
#include <span>
#include <deque>



namespace serial_transport
{

//Serial配置
struct Serial_Config
{
    std::string port_name_;
    uint32_t baud_rate_;
    uint32_t character_size_;
    asio::serial_port_base::parity::type parity_;
    asio::serial_port_base::stop_bits::type stop_bits_;
    asio::serial_port_base::flow_control::type flow_control_;
};


class SerialTransport
{
    public:

        SerialTransport();
        ~SerialTransport();

        bool start(Serial_Config config,std::function<void(std::span<const uint8_t>)> receive_callback);
        void stop();
        void async_write(std::span<const uint8_t> frame);

    private:

        //======================================================
        // TX
        //======================================================
        void enqueue_write(std::vector<uint8_t> frame);
        void start_async_write();

        //======================================================
        // RX
        //======================================================
        void start_async_read();
        void async_read_callback(const std::error_code & ec,std::size_t bytes_transferred);

        //======================================================
        // 读写失败处理函数
        //======================================================
        void handle_serial_error(const std::error_code & ec);

        //======================================================
        // 串口重连机制
        //======================================================
        void open_serial();
        void schedule_reconnect();

        //======================================================
        // Asio
        //======================================================
        asio::io_context io_context_;
        asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
        asio::serial_port serial_port_;
        std::jthread asio_thread_;

        //重连机制
        asio::steady_timer reconnect_timer_{io_context_};
        std::atomic_bool started_{false};
        bool stopping_{false};
        bool reconnect_pending_{false};

        //串口配置
        Serial_Config config_;

        //RX
        std::array<uint8_t, 1024> rx_buffer_{};
        std::function<void(std::span<const uint8_t>)> receive_callback_;

        //TX
        std::deque<std::vector<uint8_t>> send_queue_;
        std::shared_ptr<std::vector<uint8_t>> active_write_; //防止send_queue_.clear()造成的buffer 悬空风险
};

}  // namespace serial_transport

#endif  // SERIAL_TRANSPORT_HPP_