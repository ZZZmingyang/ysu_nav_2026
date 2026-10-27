#include "joy_bringup/joy_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/joy__struct.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>

//已知问题待修复：yaw、pitch 目标初始值都是零，需要改成根据当前云台角度计算。

// 先用保守的软限位，避免碰到模型的机械限位
static constexpr fp32 PITCH_MIN = -0.56f;
static constexpr fp32 PITCH_MAX =  0.56f;

using namespace std::chrono_literals;

class Joy_Node: public rclcpp::Node
{
  public:
    Joy_Node():Node("joy_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(),"joy2cmdvel节点已启动!");
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Joy_Node::joy_sub_callback,this,std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint", 10);

        // 构造函数中，创建定时器之前
        last_update_ = this->now();
        timer1_ = this->create_wall_timer(10ms, std::bind(&Joy_Node::timer1_callback,this));
    }
  private:
  
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

        fp32 dt = (now - last_update_).seconds();
        last_update_ = now;

        // 检查输入是否断流：这里使用真实经过的时间
        std::chrono::steady_clock::time_point steady_now = std::chrono::steady_clock::now();

        bool joy_fresh =
            received_joy_ &&
            (steady_now - last_joy_time_ <= JOY_TIMEOUT);

        if (!joy_fresh)
        {
            stop_input();
        }

        // 暂停时 dt 为零；时间回跳或异常跨越时跳过这次积分，Yaw支持多圈，不限制位置角度
        if (dt > 0.0 && dt <= 0.1) 
        {
            cmd_gimbal_joint_field.yaw +=
                cmd_gimbal_joint_field.yaw_vel * dt;

            cmd_gimbal_joint_field.pitch +=
                cmd_gimbal_joint_field.pitch_vel * dt;
        }

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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Joy_Node>());

  rclcpp::shutdown();
  return 0;
}