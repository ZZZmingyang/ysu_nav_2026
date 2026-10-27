#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

typedef float fp32;

using namespace std::chrono_literals;

class Gimbal_Scan_Node: public rclcpp::Node
{
  public:
    Gimbal_Scan_Node():Node("gimbal_scan_server")
    {
        RCLCPP_INFO(this->get_logger(),"云台扫描节点已启动，等待云台反馈!");

        joint_state_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "serial/gimbal_joint_state",
                10,
                std::bind(&Gimbal_Scan_Node::joint_state_sub_callback,this,std::placeholders::_1)
            );

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint",10);

        // 在创建定时器之前初始化时间
        last_update_ = this->now();

        timer1_ = this->create_wall_timer(10ms,std::bind(&Gimbal_Scan_Node::timer1_callback,this));
    }

  private:

    // 停止推进目标角度，保留已有目标位置
    void stop_input()
    {
        cmd_gimbal_joint_field.yaw_vel = 0.0f;
        cmd_gimbal_joint_field.pitch_vel = 0.0f;
    }

    // 订阅编码器角度
    void joint_state_sub_callback(const sensor_msgs::msg::JointState &msg)
    {
        // 按当前 serial_comm 的反馈顺序：
        // position[0]：pitch，position[1]：yaw
        if (msg.name.size() != msg.position.size() || msg.position.size() < 2)
        {
            return;
        }

        if (msg.name[0] != "gimbal_pitch_joint" || msg.name[1] != "gimbal_yaw_joint")
        {
            return;
        }

        if (!std::isfinite(msg.position[0]) || !std::isfinite(msg.position[1]))
        {
            return;
        }

        // 只在第一次有效反馈时初始化目标角度
        // 后续反馈不覆盖正在积分的目标角度
        if (!received_joint_state_)
        {
            cmd_gimbal_joint_field.pitch = msg.position[0];
            cmd_gimbal_joint_field.yaw = msg.position[1];

            // 先用固定速度演示，后面再改成扫描任务参数
            cmd_gimbal_joint_field.yaw_vel = 0.3f;
            cmd_gimbal_joint_field.pitch_vel = 0.0f;

            last_update_ = this->now();

            RCLCPP_INFO(this->get_logger(),"开始扫描，pitch=%.3f，yaw=%.3f",cmd_gimbal_joint_field.pitch,cmd_gimbal_joint_field.yaw);
        }

        last_joint_state_time_ = std::chrono::steady_clock::now();
        received_joint_state_ = true;
    }

    void timer1_callback()
    {
        auto now = this->now();

        fp32 dt = (now - last_update_).seconds();
        last_update_ = now;

        // 尚未得到有效反馈时，等待初始化
        if (!received_joint_state_)
        {
            return;
        }

        // 断流检查
        std::chrono::steady_clock::time_point steady_now = std::chrono::steady_clock::now();

        bool joint_state_fresh =
            received_joint_state_ &&
            (steady_now - last_joint_state_time_ <=
                JOINT_STATE_TIMEOUT);

        if (!joint_state_fresh)
        {
            stop_input();

            if (!scan_fault_)
            {
                scan_fault_ = true;

                RCLCPP_WARN(this->get_logger(),"云台反馈超时，停止推进扫描目标!");
            }
        }

        // 以下角度积分逻辑与 joy 保持一致
        // 暂停时 dt 为零；时间回跳或异常跨越时跳过本次积分
        if (dt > 0.0 && dt <= 0.1)
        {
            cmd_gimbal_joint_field.yaw +=
                cmd_gimbal_joint_field.yaw_vel * dt;

            cmd_gimbal_joint_field.pitch +=
                cmd_gimbal_joint_field.pitch_vel * dt;
        }

        auto joint_state_msg = sensor_msgs::msg::JointState();

        joint_state_msg.header.stamp = now;
        joint_state_msg.header.frame_id = "gimbal_pitch_odom_joint";

        joint_state_msg.name.push_back("gimbal_pitch_joint");
        joint_state_msg.name.push_back("gimbal_yaw_joint");

        joint_state_msg.position.push_back(cmd_gimbal_joint_field.pitch);

        joint_state_msg.position.push_back(cmd_gimbal_joint_field.yaw);

        joint_state_pub_->publish(joint_state_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    rclcpp::TimerBase::SharedPtr timer1_;

    // 目标角度、角速度结构
    struct
    {
        fp32 yaw{0.0f};
        fp32 pitch{0.0f};
        fp32 yaw_vel{0.0f};
        fp32 pitch_vel{0.0f};
    } cmd_gimbal_joint_field;

    rclcpp::Time last_update_;

    // 云台反馈断流处理
    bool received_joint_state_{false};
    bool scan_fault_{false};

    std::chrono::steady_clock::time_point last_joint_state_time_{std::chrono::steady_clock::now()};

    const std::chrono::milliseconds JOINT_STATE_TIMEOUT = 200ms;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);

    rclcpp::spin(std::make_shared<Gimbal_Scan_Node>());

    rclcpp::shutdown();
    return 0;
}