#include "rclcpp/rclcpp.hpp"
#include "serial_transport/serial_transport.hpp"
#include <chrono>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <wire_protocol/protocol.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <functional>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <limits>

// pitch 软限位
static constexpr fp32 PITCH_MIN = -0.56f;
static constexpr fp32 PITCH_MAX =  0.56f;

// 限制速度
constexpr fp64 MAX_LINEAR_SPEED = 0.20;   // m/s，平移合速度
constexpr fp64 MAX_ANGULAR_SPEED = 0.20;  // rad/s，底盘角速度


using namespace std::chrono_literals;

// 按关节名读取角度，不要求 pitch、yaw 的排列顺序
static bool read_gimbal_position(
    const sensor_msgs::msg::JointState &msg,
    fp64 &pitch_position,
    fp64 &yaw_position)
{
    if (msg.name.size() != msg.position.size() ||
        msg.position.size() < 2)
    {
        return false;
    }

    bool found_pitch = false;
    bool found_yaw = false;

    for (std::size_t i = 0; i < msg.name.size(); ++i)
    {
        if (msg.name[i] == "gimbal_pitch_joint")
        {
            // 同一个目标关节不能重复出现
            if (found_pitch)
            {
                return false;
            }

            pitch_position = msg.position[i];
            found_pitch = true;
        }
        else if (msg.name[i] == "gimbal_yaw_joint")
        {
            if (found_yaw)
            {
                return false;
            }

            yaw_position = msg.position[i];
            found_yaw = true;
        }
    }

    return found_pitch && found_yaw &&
           std::isfinite(pitch_position) &&
           std::isfinite(yaw_position);
}



class Serial_Node: public rclcpp::Node
{
  public:
    Serial_Node()
      : Node("serial_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"serial_node启动!");

      //==================================================
      // ROS2 参数
      //==================================================

      //声明并设置默认参数
      this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
      this->declare_parameter<int>("baud_rate", 115200);

      //参数读取
      serial_config_.port_name_ = this->get_parameter("port").as_string();
      serial_config_.baud_rate_ = this->get_parameter("baud_rate").as_int();
      //硬参数
      serial_config_.character_size_ = 8;
      serial_config_.parity_ = asio::serial_port_base::parity::none;
      serial_config_.stop_bits_ = asio::serial_port_base::stop_bits::one;
      serial_config_.flow_control_ = asio::serial_port_base::flow_control::none;

      //串口包协议回调函数设置
      if(!protocol_.set_unpack_callback(0x11, &Serial_Node::handle_joint_state, this))
      {
        RCLCPP_ERROR(this->get_logger(), "gimbal_joint_state 协议回调注册失败");
        return;
      }

      // 必须在串口开启前开启发布者，不然pub_对象可能会是空指针的时候就被串口回调使用
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
      imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("serial/imu", 10);

      //回调注册完毕后，再开启串口与异步接收
      serial_driver_.start(serial_config_, std::bind(&Serial_Node::serial_receive_callback,this,std::placeholders::_1));

      // 在串口开启前在后都可以
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Serial_Node::cmd_vel_sub_callback,this,std::placeholders::_1));
      joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("cmd_gimbal_joint", 10, std::bind(&Serial_Node::joint_state_sub_callback,this,std::placeholders::_1));

      // 创建两个定时器模拟两个 topic（必须在串口开启后，尽量）
      //模拟/cmd_vel这种高频消息
      timer1_ = this->create_wall_timer(10ms,std::bind(&Serial_Node::timer1_callback,this));
      //模拟/set_mode这种低频消息
      timer2_ = this->create_wall_timer(500ms,std::bind(&Serial_Node::timer2_callback,this));

    }

  private:
    void handle_joint_state(fp32 abs_roll,fp32 abs_pitch,fp32 abs_yaw,fp32 rel_pitch,fp32 rel_yaw)
    {
      //先搞时间戳
      const auto stamp = this->now();

      sensor_msgs::msg::JointState msg_jointstate;
      sensor_msgs::msg::Imu msg_imu;

      // 北极熊 IMU 的参考 frame
      msg_imu.header.frame_id = "gimbal_pitch_odom";

      //优先级必须很高
      msg_jointstate.header.stamp = stamp;
      msg_imu.header.stamp = stamp;

      msg_jointstate.name = {
          "gimbal_pitch_joint",      // 小pitch
          "gimbal_yaw_joint",        //小yaw
          "gimbal_pitch_odom_joint",   //fixed
          "gimbal_yaw_odom_joint"    //大yaw
      };

      msg_jointstate.position = {
                static_cast<fp64>(rel_pitch),  // 唯一机械 pitch
                static_cast<fp64>(rel_yaw),    // 唯一机械 yaw
                0.0,                          // 固定层
                0.0                           // 固定层
      };

      tf2::Quaternion q;

      q.setRPY(static_cast<fp64>(abs_roll), static_cast<fp64>(abs_pitch), static_cast<fp64>(abs_yaw));

      msg_imu.orientation.x = q.x();
      msg_imu.orientation.y = q.y();
      msg_imu.orientation.z = q.z();
      msg_imu.orientation.w = q.w();

      joint_state_pub_->publish(msg_jointstate);
      imu_pub_->publish(msg_imu);

      RCLCPP_DEBUG(this->get_logger(),"[RX gimbal_joint_state] pitch=%.3f yaw=%.3f",rel_pitch,rel_yaw);
      RCLCPP_DEBUG(this->get_logger(),"[RX imu] roll=%.3f pitch=%.3f yaw=%.3f",abs_roll,abs_pitch,abs_yaw);
    }

    void serial_receive_callback(std::span<const uint8_t> msg)
    {
      //解包
      protocol_.feed(msg);
    }

    void cmd_vel_sub_callback(const geometry_msgs::msg::Twist &msg)
    {

      fp64 vx = msg.linear.x;
      fp64 vy = msg.linear.y;
      fp64 wz = msg.angular.z;

      // 无效输入：清零，并标记为无效
      if (!std::isfinite(vx) || !std::isfinite(vy) ||
          !std::isfinite(wz)) 
      {
        cmd_vel_field = {};
        received_cmd_vel_ = false;
        return;
      }

      // 同比缩小两个分量，保持平移方向
      const fp64 speed = std::hypot(vx, vy);
      if (speed > MAX_LINEAR_SPEED) 
      {
        const fp64 scale = MAX_LINEAR_SPEED / speed;
        vx *= scale;
        vy *= scale;
      }

      // 限制角速度
      wz = std::clamp(wz, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

      // 相对于云台的线速度、线速度和角速度，单位都是 m/s 和 rad/s 分别
      cmd_vel_field.vx = static_cast<fp32>(vx);
      cmd_vel_field.vy = static_cast<fp32>(vy);
      cmd_vel_field.wz = static_cast<fp32>(wz);  //含自旋的角速度wz_spin了，再加上wz_nav2,这里的是wz_final

      last_cmd_vel_time_ = std::chrono::steady_clock::now();
      received_cmd_vel_ = true;
    }

    void joint_state_sub_callback(const sensor_msgs::msg::JointState &msg)
    {
      fp64 pitch_position = 0.0;
      fp64 yaw_position = 0.0;

      if (!read_gimbal_position(msg, pitch_position, yaw_position) ||
        std::abs(yaw_position) > std::numeric_limits<fp32>::max())
      {
        received_cmd_gimbal_ = false;
        return;
      }

      // pitch 在串口发送前再次限幅
      const fp32 pitch = static_cast<fp32>(std::clamp(
        pitch_position, static_cast<fp64>(PITCH_MIN), static_cast<fp64>(PITCH_MAX)));

      constexpr fp64 PI = 3.14159265358979323846;
      constexpr fp64 TWO_PI = 2.0 * PI;

      // fmod 对 2π 取余，结果与输入同号，范围为 (-2π, 2π)；随后调整到 [-π, π)
      fp64 yaw_single = std::fmod(yaw_position, TWO_PI);

      // 归一化 yaw 到 [-π, π)
      if (yaw_single >= PI)
      {
        yaw_single -= TWO_PI;
      }
      else if (yaw_single < -PI)
      {
        yaw_single += TWO_PI;
      }

      const fp32 yaw = static_cast<fp32>(yaw_single);

      if (!std::isfinite(pitch) || !std::isfinite(yaw))
      {
        received_cmd_gimbal_ = false;
        return;
      }

      cmd_gimbal_joint_field.rel_pitch = pitch;
      cmd_gimbal_joint_field.rel_yaw = yaw;

      last_cmd_gimbal_time_ = std::chrono::steady_clock::now();
      received_cmd_gimbal_ = true;
    }

    void timer1_callback()
    {
      // 断流处理
      std::chrono::steady_clock::time_point steady_now = std::chrono::steady_clock::now();

      bool cmd_vel_fresh =
        received_cmd_vel_ &&
        (steady_now - last_cmd_vel_time_ <= CMD_VEL_TIMEOUT);

      uint8_t cmd_vel_fresh_byte = static_cast<uint8_t>(cmd_vel_fresh);

      bool cmd_gimbal_fresh =
        received_cmd_gimbal_ &&
        (steady_now - last_cmd_gimbal_time_ <= CMD_GIMBAL_TIMEOUT);

      uint8_t cmd_gimbal_fresh_byte = static_cast<uint8_t>(cmd_gimbal_fresh);

      // 清空无效数据
      if (!cmd_vel_fresh) 
      {
        cmd_vel_field = {};
      }

      // 打包数据

      /*
      | 字段 | 当前应约定的含义 | 单位 |
      |---|---|---|
      | `vx` | **底盘平动速度**，沿 `gimbal_yaw` 的 X 轴，向前为正 | m/s |
      | `vy` | **底盘平动速度**，沿 `gimbal_yaw` 的 Y 轴，向左为正 | m/s |
      | `wz` | 底盘绕竖直 Z 轴的目标角速度，俯视逆时针为正 | rad/s |
      | `rel_pitch` | pitch 关节相对于机械零位的目标位置角 | rad |
      | `rel_yaw` | yaw 关节相对于机械零位的目标位置角，即云台相对底盘的目标偏角 | rad |
      */

      auto frame = protocol_.pack(
        0x01, 
        cmd_vel_fresh_byte,
        cmd_vel_field.vx,
        cmd_vel_field.vy,
        cmd_vel_field.wz,
        cmd_gimbal_fresh_byte,
        cmd_gimbal_joint_field.rel_pitch,
        cmd_gimbal_joint_field.rel_yaw
      );

      //异步发送数据
      serial_driver_.async_write(frame);
    }

    void timer2_callback()
    {
      // 模拟不断变化的模式命令
      ++mode_;

      if (mode_ > 3)
      {
        mode_ = 0;
      }

      uint32_t seq = ++event_count_;

      RCLCPP_DEBUG(this->get_logger(),"[set_mode] seq = %u,mode=%d",seq,mode_);

      auto frame = protocol_.pack(0x02, seq,mode_);

      //异步发送数据
      serial_driver_.async_write(frame);
    }

    //ROS
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    //注意，protocol_一定要比serial_driver_早。
    // protocol_ 先构造、后析构；串口析构时先 stop() 并等待接收线程退出。
    wire_protocol::CallbackProtocol protocol_;

    //Serial
    serial_transport::Serial_Config serial_config_;
    serial_transport::SerialTransport serial_driver_;

    // 断流处理

    /*
    | 情况 | `received_cmd_gimbal_` | `cmd_gimbal_fresh` |
    |---|---:|---:|
    | 从未收到目标 | false | false |
    | 收到过，且未超时 | true | true |
    | 收到过，但已超时 | true | false |
    */

    //作用是记录是否收到过cmd_vel和gimbal_joint_state，
    //如果没有这个，则刚开始200ms内没有收到任何数据的时候就被判定是有效数据，并被定时器发出去
    bool received_cmd_vel_{false};
    bool received_cmd_gimbal_{false};

    // 这里一定要用steady_clock,因为ros的wall_clock会因为系统时间变化而改变，而steady_clock是单调时钟不会随系统时间变化
    std::chrono::steady_clock::time_point last_cmd_vel_time_{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point last_cmd_gimbal_time_{std::chrono::steady_clock::now()};

    const std::chrono::milliseconds CMD_VEL_TIMEOUT = 200ms;
    const std::chrono::milliseconds CMD_GIMBAL_TIMEOUT = 200ms;

    //cmd_vel数据
    struct
    {
      fp32 vx;
      fp32 vy;
      fp32 wz;
    }cmd_vel_field{0.0,0.0,0.0};

    //云台数据
    struct
    {
      fp32 rel_pitch{0.0f};
      fp32 rel_yaw{0.0f};
    } cmd_gimbal_joint_field;

    //模拟数据
    int32_t mode_{0};
    uint32_t event_count_{0};

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  auto node_ = std::make_shared<Serial_Node>();
  rclcpp::spin(node_);

  rclcpp::shutdown();
  return 0;
}
