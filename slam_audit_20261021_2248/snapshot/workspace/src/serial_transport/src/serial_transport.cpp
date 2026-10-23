#include "serial_transport/serial_transport.hpp"
#include <cstdio>
#include <utility>

using namespace std::chrono_literals;

namespace serial_transport
{

//======================================================
// 构造与析构
//======================================================

SerialTransport::SerialTransport()
    :   io_context_(),
        work_guard_(asio::make_work_guard(io_context_)),
        serial_port_(io_context_)
{
    std::printf("SerialTransport已经被创建!\n");
}

SerialTransport::~SerialTransport()
{
    stop();
}

//======================================================
// Public API
//======================================================
bool SerialTransport::start(Serial_Config config,std::function<void(std::span<const uint8_t>)> receive_callback)
{
    if (started_)
    {
        std::printf("Serial已经启动,无需重复启动\n");
        return false;
    }

    //早点标记，防止影响后面post的任务
    started_ = true;
    stopping_ = false;
    reconnect_pending_ = false;

    config_ = std::move(config);

    //设置接收回调函数
    receive_callback_ = std::move(receive_callback);
    std::printf("接收回调已设置完成！\n");

    //投递初始化任务，但此时不会执行，因为没有任何线程跑run()。
    asio::post(io_context_,
    [this]()
          {
              open_serial();
          });

    //这种短任务没必要用std::bind了，直接用lambda最直观
    asio_thread_ = std::jthread(
            [this]()->void
            {
                std::printf("Asio线程启动\n");
                io_context_.run();
                std::printf("Asio线程退出\n");
            });

    return started_;
}

void SerialTransport::stop()
{
    std::printf("准备关闭Serial\n");

    if (!started_)
    {
        std::printf("Serial并未启动,无需停止\n");
        return;
    }

    // 不直接在main主线程操作serial_port_，防止跨线程操作serial_port_
    // 而是把关闭任务交给Asio线程
    asio::post(io_context_,
        [this]()->void
              {
                stopping_ = true;

                std::error_code ec;

                // 取消重连
                reconnect_timer_.cancel(ec);

                if (serial_port_.is_open())
                {
                  //取消当前这个串口上正在挂起的异步操作
                  serial_port_.cancel(ec);
                  //关闭这个串口设备
                  serial_port_.close(ec);
                }
                send_queue_.clear();
                active_write_.reset();

                // 允许io_context在处理完剩余任务之后退出，但这一条不一定比上面cancel()更慢，上面只是投递了任务
                work_guard_.reset();

              });
    
    //让线程执行join()，但改用jthread后，这两条也可以被取消了
    if (asio_thread_.joinable())
    {
        asio_thread_.join();
    }

    started_ = false;
    std::printf("Serial关闭完成\n");
}

void SerialTransport::async_write(std::span<const uint8_t> frame)
{
    //--------------------------------------------------
    // 重点：
    // 主线程不直接操作串口。
    // 只把“我要发送什么”
    // post给io_context。
    //--------------------------------------------------

    if (!started_)
    {
        std::printf("Serial未启动,无法发送数据\n");
        return;
    }

    // span 不拥有数据，所以必须在当前线程立即复制

    asio::post(
        io_context_,
        [this, frame_copy = std::vector<uint8_t> (frame.begin(), frame.end())]() mutable
                {
                    enqueue_write(std::move(frame_copy));
                });
}

//======================================================
// TX
//======================================================
void SerialTransport::enqueue_write(std::vector<uint8_t> frame)
{
    // 注意：
    // 这个函数只会从Asio线程中调用。

    if (serial_port_.is_open() == false)
    {
        return;
    }

    send_queue_.push_back(std::move(frame));

    // 如果之前没有正在发送的数据
    // 那么启动第一次发送。
    // 如果已经在发送，则只需要排队。
    if (!active_write_) 
    {
        start_async_write();
    }
}

void SerialTransport::start_async_write()
{
    // 已经有一个 write 在进行
    if (active_write_)
    {
        return;
    }

    // 没有待发送的数据
    if (send_queue_.empty())
    {
        return;
    }

    // 从等待队列取出一帧
    active_write_ =std::make_shared<std::vector<uint8_t>>(std::move(send_queue_.front()));

    //取出完毕后，把取出去的frame开除队列
    send_queue_.pop_front();

    // 再复制一份 shared_ptr 给 handler
    auto frame = active_write_;

    asio::async_write(serial_port_,asio::buffer(*frame),
    [this,frame](const std::error_code & ec,std::size_t bytes_transferred)->void
    {
        // 当前 write 已经结束
        active_write_.reset();

        if (ec)
        {
          handle_serial_error(ec);
          return;
        }

        //默认不打印，想检查问题的时候取消注释
        // std::printf("[Asio] 发送完成: %zu bytes\n",bytes_transferred);
        (void)bytes_transferred;

        // 如果队列中还有数据，继续发送下一帧（不过这个在开头判断过了，这里可以优化，但为了代码可读性，没优化掉）
        if (!send_queue_.empty())
        {
          start_async_write();
        }
    });
}

//======================================================
// RX
//======================================================
void SerialTransport::start_async_read()
{
    // 注意：
    // 这个函数只会从Asio线程中调用。

    if (serial_port_.is_open() == false)
    {
        return;
    }

    serial_port_.async_read_some(asio::buffer(rx_buffer_),
      std::bind(&SerialTransport::async_read_callback,this,std::placeholders::_1,std::placeholders::_2));
}

void SerialTransport::async_read_callback(const std::error_code & ec,std::size_t bytes_transferred)
{
    if(ec)
    {
        handle_serial_error(ec);
        return;
    }

    //默认不打印，想检查问题的时候取消注释
    // std::printf("[Asio] 收到 %zu bytes\n",bytes_transferred);

    if (receive_callback_)
    {
        receive_callback_(
            std::span<const uint8_t>(rx_buffer_.data(),bytes_transferred)
        );
    }

    // 重新注册下一次异步接收
    start_async_read();
}

//======================================================
// 读写失败处理函数
//======================================================
void SerialTransport::handle_serial_error(const std::error_code & ec)
{
    if (stopping_ || ec == asio::error::operation_aborted)
    {
        return;
    }
    std::printf("串口通信异常: %s",ec.message().c_str());
    std::error_code ignore_ec;

    if (serial_port_.is_open())
    {
        serial_port_.cancel(ignore_ec);
        serial_port_.close(ignore_ec);
    }
    // 当前简单策略：
    // 断线后历史发送数据全部丢弃
    send_queue_.clear();
    active_write_.reset();

    schedule_reconnect();
}

//======================================================
// 串口重连机制
//======================================================
void SerialTransport::open_serial()
{
    if (stopping_)
    {
      return;
    }

    std::error_code ec;

    //如果现在是开着的，说明是重连，所以先关闭串口
    if (serial_port_.is_open())
    {
        serial_port_.close(ec);
    }

    //打开串口
    serial_port_.open(config_.port_name_, ec);

    if (ec)
    {
        std::printf("串口打开失败:%s\n",ec.message().c_str());

        schedule_reconnect();

        return;

    }

    try
    {
        serial_port_.set_option(asio::serial_port_base::baud_rate(config_.baud_rate_));
        serial_port_.set_option(asio::serial_port_base::character_size(config_.character_size_));
        serial_port_.set_option(asio::serial_port_base::parity(config_.parity_));
        serial_port_.set_option(asio::serial_port_base::stop_bits(config_.stop_bits_));
        serial_port_.set_option(asio::serial_port_base::flow_control(config_.flow_control_));

        std::printf("串口打开成功: %s, baud=%u\n",config_.port_name_.c_str(),config_.baud_rate_);
    }
    catch (const std::system_error & e)
    {
        std::printf("串口配置失败: %s\n",e.what());

        std::error_code ignore_ec;
        serial_port_.close(ignore_ec);

        schedule_reconnect();
        return;
    }

    start_async_read();
}

void SerialTransport::schedule_reconnect()
{
    if (stopping_ || reconnect_pending_)
    {
        return;
    }
    reconnect_pending_ = true;

    reconnect_timer_.expires_after(1s);
    reconnect_timer_.async_wait([this](const std::error_code & ec)->void
    {

        reconnect_pending_ = false;

        if (ec == asio::error::operation_aborted)
        {
          return;
        }
        if (ec)
        {
          std::printf("重连定时器错误: %s\n",ec.message().c_str());return;
        }

        std::printf("尝试重新连接串口...\n");
        open_serial();
    });
}


}  // namespace serial_transport