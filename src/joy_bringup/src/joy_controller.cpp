#include "joy_bringup/joy_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <sensor_msgs/msg/detail/joy__struct.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>

// pitch 软限位；yaw 支持多圈，不限制位置角度
static constexpr fp32 PITCH_MIN = -0.56f;
static constexpr fp32 PITCH_MAX =  0.56f;

// 软件速度上限，不代表机械极限
static constexpr fp32 MAX_YAW_SPEED = 2.5f;
static constexpr fp32 MAX_PITCH_SPEED = 1.0f;

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


class Joy_Node: public rclcpp::Node
{
  public:
    Joy_Node():Node("joy_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(),"joy2cmdvel节点已启动!");
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Joy_Node::joy_sub_callback,this,std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10, std::bind(&Joy_Node::joint_state_sub_callback,this,std::placeholders::_1));  


        // 构造函数中，创建定时器之前
        last_update_ = this->now();
        timer1_ = this->create_wall_timer(10ms, std::bind(&Joy_Node::timer1_callback,this));
    }
  private:

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

            last_update_ = this->now();

            RCLCPP_INFO(this->get_logger(),"云台初始角度，pitch=%.3f，yaw=%.3f",cmd_gimbal_joint_field.pitch,cmd_gimbal_joint_field.yaw);
        }

        last_joint_state_time_ = std::chrono::steady_clock::now();
        received_joint_state_ = true;
    }


    // 停止运动输入，但保留云台目标角度
    void stop_input()
    {
        cmd_vel_field.vx = 0.0f;
        cmd_vel_field.vy = 0.0f;
        cmd_vel_field.wz = 0.0f;

        cmd_gimbal_joint_field.yaw_vel = 0.0f;
        cmd_gimbal_joint_field.pitch_vel = 0.0f;
    }


    void joy_sub_callback(const sensor_msgs::msg::Joy &msg)
    {

        /*
        vx > 0  → 向前
        vx < 0  → 向后

        vy > 0  → 向左
        vy < 0  → 向右

        wz > 0  → 逆时针旋转（从车顶往下看）
        wz < 0  → 顺时针旋转
        */

        // 使用了 axes[6]，所以至少需要七根轴
        if (msg.axes.size() < 7) 
        {
            stop_input();
            received_joy_ = false;
            return;
        }

        cmd_vel_field.vx = msg.axes[1]; //左摇杆朝前是1.0
        cmd_vel_field.vy = msg.axes[0]; //左摇杆朝左是1.0
        cmd_vel_field.wz = msg.axes[6] * 0.1; //十字键朝左是1.0，旋转角度是1.0 * M_PI
        cmd_gimbal_joint_field.yaw_vel = msg.axes[3] * 2.5; //右摇杆朝左是1.0 * 2.5
        cmd_gimbal_joint_field.pitch_vel = msg.axes[4] * 1.0; //右摇杆朝前是1.0 * 1.0

        last_joy_time_ = std::chrono::steady_clock::now();
        received_joy_ = true;
    }

    void timer1_callback()
    {
        auto now = this->now();

        // 积分时间间隔
        fp32 dt = (now - last_update_).seconds();
        last_update_ = now;

        // 首帧有效反馈前只等待，不发布零角度目标，也不锁定超时故障
        if (!received_joint_state_)
        {
            stop_input();
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
            return;
        }

        // 检查输入是否断流：这里使用真实经过的时间
        std::chrono::steady_clock::time_point steady_now = std::chrono::steady_clock::now();

        bool joy_fresh =
            received_joy_ &&
            (steady_now - last_joy_time_ <= JOY_TIMEOUT);

        if (!joy_fresh)
        {
            stop_input();
        }

        bool joint_state_fresh =
            received_joint_state_ &&
            (steady_now - last_joint_state_time_ <= JOINT_STATE_TIMEOUT);

        if (!joint_state_fresh)
        {
            stop_input();

            if (!scan_fault_)
            {
                scan_fault_ = true;

                RCLCPP_WARN(this->get_logger(),"云台反馈超时，停止运动输入!");
            }
        }

        // 故障锁定后持续清零，避免新手柄消息重新写入非零速度
        if (scan_fault_)
        {
            stop_input();
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
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
            return;
        }

        // 暂停时 dt 为零；时间回跳或异常跨越时跳过这次积分，Yaw支持多圈，不限制位置角度
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
        }

        if (!std::isfinite(cmd_gimbal_joint_field.yaw) ||
            !std::isfinite(cmd_gimbal_joint_field.pitch))
        {
            stop_input();
            scan_fault_ = true;
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "云台积分目标无效，停止发布扫描命令!");
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
            return;
        }

        // 在发布前执行最终 pitch 软限幅
        cmd_gimbal_joint_field.pitch = std::clamp(
            cmd_gimbal_joint_field.pitch,
            PITCH_MIN,
            PITCH_MAX
        );

        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        auto joint_state_msg = sensor_msgs::msg::JointState();

        cmd_vel_msg.linear.x = cmd_vel_field.vx;
        cmd_vel_msg.linear.y = cmd_vel_field.vy;
        cmd_vel_msg.linear.z = 0.0f;

        cmd_vel_msg.angular.x = 0.0f;
        cmd_vel_msg.angular.y = 0.0f;
        cmd_vel_msg.angular.z = cmd_vel_field.wz;

        joint_state_msg.header.stamp = now;
        joint_state_msg.header.frame_id = "gimbal_pitch_odom_joint"; //貌似没啥用

        joint_state_msg.name.push_back("gimbal_pitch_joint");
        joint_state_msg.name.push_back("gimbal_yaw_joint");

        joint_state_msg.position.push_back(cmd_gimbal_joint_field.pitch);
        joint_state_msg.position.push_back(cmd_gimbal_joint_field.yaw);

        joint_state_pub_->publish(joint_state_msg);

        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer1_;

    struct
    {
        fp32 vx{0.0f};
        fp32 vy{0.0f};
        fp32 wz{0.0f};
    } cmd_vel_field;

    struct
    {
        fp32 yaw{0.0f};
        fp32 pitch{0.0f};
        fp32 yaw_vel{0.0f};
        fp32 pitch_vel{0.0f};
    } cmd_gimbal_joint_field;

    rclcpp::Time last_update_;

    // 手柄断流处理
    bool received_joy_{false};
    std::chrono::steady_clock::time_point last_joy_time_{std::chrono::steady_clock::now()};
    const std::chrono::milliseconds JOY_TIMEOUT = 200ms;

    // 云台反馈断流处理
    bool received_joint_state_{false};
    bool scan_fault_{false};
    std::chrono::steady_clock::time_point last_joint_state_time_{std::chrono::steady_clock::now()};
    const std::chrono::milliseconds JOINT_STATE_TIMEOUT = 200ms;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Joy_Node>());

  rclcpp::shutdown();
  return 0;
}