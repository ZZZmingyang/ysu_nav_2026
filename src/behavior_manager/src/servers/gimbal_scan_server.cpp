#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "behavior_manager/struct_typedef.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

// pitch 软限位；yaw 支持多圈，不限制位置角度
static constexpr fp32 PITCH_MIN = -0.56f;
static constexpr fp32 PITCH_MAX =  0.56f;

// 软件速度上限，不代表机械极限
static constexpr fp32 MAX_YAW_SPEED = 2.5f;
static constexpr fp32 MAX_PITCH_SPEED = 1.0f;

// 云台扫描速度 单位：rad/s
static constexpr fp32 YAW_SCAN_SPEED = 1.25f;
static constexpr fp32 PITCH_SCAN_SPEED = 0.5f;

using namespace std::chrono_literals;

// 按关节名读取角度，不要求 pitch、yaw 的排列顺序
// 仿真里他也做了这种处理，但是你从仿真里读取关节角度却定死了先 yaw 后 pitch,所以这里读角度要做自适应读取角度
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
        fp64 pitch_position = 0.0;
        fp64 yaw_position = 0.0;

        if (!read_gimbal_position(msg, pitch_position, yaw_position))
        {
            return;
        }

        // 只在第一次有效反馈时初始化目标角度
        // 后续反馈不覆盖正在积分的目标角度
        if (!received_joint_state_)
        {
            // 初始角度超出软限位时等待处理，避免直接跳到限位位置
            // yaw 保留多圈值，但必须能够表示为当前使用的 fp32 类型
            // std::numeric_limits<fp32>::max() 是获取 fp32 类型的最大表示值
            if (pitch_position < PITCH_MIN || pitch_position > PITCH_MAX ||
                std::abs(yaw_position) > std::numeric_limits<fp32>::max())
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "初始云台角度超出允许范围，等待有效反馈!");
                return;
            }

            cmd_gimbal_joint_field.pitch = pitch_position;
            cmd_gimbal_joint_field.yaw = yaw_position;

            // 先用固定速度演示，后面再改成扫描任务参数
            cmd_gimbal_joint_field.yaw_vel = YAW_SCAN_SPEED;
            cmd_gimbal_joint_field.pitch_vel =
                cmd_gimbal_joint_field.pitch >= PITCH_MAX ? //初始位置低着头，扫描时上移
                -PITCH_SCAN_SPEED : PITCH_SCAN_SPEED;

            last_update_ = this->now();

            RCLCPP_INFO(this->get_logger(),"开始扫描，pitch=%.3f，yaw=%.3f",cmd_gimbal_joint_field.pitch,cmd_gimbal_joint_field.yaw);
        }

        last_joint_state_time_ = std::chrono::steady_clock::now();
        received_joint_state_ = true;
    }

    void timer1_callback()
    {
        auto now = this->now();

        // 积分时间间隔
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

        // 非有限目标或速度不参与积分，也不发送给下位机
        if (!std::isfinite(cmd_gimbal_joint_field.yaw) ||
            !std::isfinite(cmd_gimbal_joint_field.pitch) ||
            !std::isfinite(cmd_gimbal_joint_field.yaw_vel) ||
            !std::isfinite(cmd_gimbal_joint_field.pitch_vel))
        {
            stop_input();
            scan_fault_ = true;
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "云台目标或速度无效，停止发布扫描命令!");
            return;
        }

        // 角度积分
        // 暂停时 dt 为零；时间回跳或异常跨越时跳过本次积分
        // 故障后保持停止，避免边界换向重新启动 pitch
        if (!scan_fault_ && dt > 0.0 && dt <= 0.1)
        {
            cmd_gimbal_joint_field.yaw_vel = std::clamp(
                cmd_gimbal_joint_field.yaw_vel,
                -MAX_YAW_SPEED,
                MAX_YAW_SPEED
            );

            cmd_gimbal_joint_field.pitch_vel = std::clamp(
                cmd_gimbal_joint_field.pitch_vel,
                -MAX_PITCH_SPEED,
                MAX_PITCH_SPEED
            );

            // 多圈Yaw
            cmd_gimbal_joint_field.yaw +=
                cmd_gimbal_joint_field.yaw_vel * dt;

            cmd_gimbal_joint_field.pitch +=
                cmd_gimbal_joint_field.pitch_vel * dt;

            // pitch 到达软限位后往返，yaw 继续同向转圈
            if (cmd_gimbal_joint_field.pitch_vel > 0.0f &&
                cmd_gimbal_joint_field.pitch >= PITCH_MAX)
            {
                cmd_gimbal_joint_field.pitch = PITCH_MAX;
                cmd_gimbal_joint_field.pitch_vel =
                    -cmd_gimbal_joint_field.pitch_vel;
            }
            else if (cmd_gimbal_joint_field.pitch_vel < 0.0f &&
                     cmd_gimbal_joint_field.pitch <= PITCH_MIN)
            {
                cmd_gimbal_joint_field.pitch = PITCH_MIN;
                cmd_gimbal_joint_field.pitch_vel =
                    -cmd_gimbal_joint_field.pitch_vel;
            }
        }

        if (!std::isfinite(cmd_gimbal_joint_field.yaw) ||
            !std::isfinite(cmd_gimbal_joint_field.pitch))
        {
            stop_input();
            scan_fault_ = true;
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "云台积分目标无效，停止发布扫描命令!");
            return;
        }

        // 在发布前执行最终 pitch 软限幅
        cmd_gimbal_joint_field.pitch = std::clamp(
            cmd_gimbal_joint_field.pitch,
            PITCH_MIN,
            PITCH_MAX
        );

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