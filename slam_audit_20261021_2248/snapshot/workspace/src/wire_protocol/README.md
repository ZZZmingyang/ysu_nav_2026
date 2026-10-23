# wire_protocol

协议核心是纯 C++20，不引用 ROS 头文件；在 ROS 2 工作区中使用
`ament_cmake` 打包。推荐用 `CallbackProtocol`：创建对象后注册命令处理函数，
接收端只需调用 `feed()`，发送端调用 `pack()`。

## 固定容量回调 API

```cpp
#include <wire_protocol/protocol.hpp>

class Serial_Node : public rclcpp::Node
{
  public:
    Serial_Node() : Node("serial_node_cpp")
    {
        // 先注册，随后再启动串口接收。
        const bool vel_ok = protocol_.set_unpack_callback(
            0x01, &Serial_Node::handle_cmd_vel, this);
        const bool mode_ok = protocol_.set_unpack_callback(
            0x02, &Serial_Node::handle_mode, this);
        if (!vel_ok || !mode_ok)
        {
            // 命令重复或回调表已满，由上层处理初始化错误。
        }
    }

  private:
    void handle_cmd_vel(std::uint32_t seq, float vx, float vy, float wz);
    void handle_mode(std::uint32_t seq, std::int32_t mode);

    void serial_receive_callback(std::span<const std::uint8_t> bytes)
    {
        protocol_.feed(bytes);
    }

    wire_protocol::CallbackProtocol protocol_;
};

// 发送时，返回拥有字节的定长 std::array。
auto tx = protocol_.pack(0x01, seq, vx, vy, wz);
serial_driver.async_write(tx); // 当前 PC 端 SerialTransport 会复制发送数据

// 串口断开、超时或重新连接时丢弃残留半帧；注册的回调仍保留。
protocol_.reset();
```

handler 的参数类型就是该命令的 payload schema。`feed()` 处理帧头、长度、CRC、
帧尾、半包和粘包，再根据命令自动解包并调用 handler；业务层不需要接触
Parser、Frame、payload 或字段计数。没有注册的命令及长度不符合 schema 的帧会被忽略。
handler 必须返回 `void`，字段参数按值传递且类型明确。

还支持普通函数和小型捕获 lambda：

```cpp
protocol_.set_unpack_callback(0x03, handle_status);
protocol_.set_unpack_callback(0x04,
    [this](std::uint32_t seq, float voltage)
    {
        handle_voltage(seq, voltage);
    });
```

`CallbackProtocol` 内有 8 个固定槽位，每槽最多存储 32 字节的回调绑定对象。
重复命令、槽位已满或传入空函数／对象指针时，
`set_unpack_callback()` 返回 `false`。
为保证注册过程也不申请堆内存，只接受可平凡复制、移动和销毁的回调；
`[this]`、小型数值捕获及成员函数绑定可以使用，捕获 `std::string`、`std::vector`
等非平凡对象会在编译期拒绝。绑定对象超出 32 字节或对齐要求也在编译期拒绝。
回调本身若执行堆分配操作，不属于协议库能保证的范围。

每路串口使用独立、长期存在的 `CallbackProtocol`。应在开始接收前完成注册；
不要在 `feed()` 期间重新注册，也不要从多个线程或中断并发访问同一实例。
注册成员函数时，被绑定的对象在协议使用期间必须保持有效。
`CallbackProtocol` 不可复制或移动，避免回调对象指针悬空。

## 编译期 dispatch API

原有的 `make_protocol(dispatch(...))` 保留，适合希望回调在构造时完全确定的场景：

```cpp
auto protocol = wire_protocol::make_protocol(
    wire_protocol::dispatch(0x01, handle_cmd_vel),
    wire_protocol::dispatch(0x02, handle_mode));

auto tx = protocol.pack(0x01, seq, vx, vy, wz);
protocol.feed(bytes);
protocol.reset();
```

这个 API 将 handler 类型保存在 `Protocol<...>` 模板参数中，无固定回调容量，
但作为类成员时需要显式写出类型；`CallbackProtocol` 免去了这部分声明。
两种 API 共用同一套打包、解包、帧解析和 CRC 代码。
泛型 lambda（`auto` 参数）、`std::bind`、引用参数、指针参数和结构体自动序列化均不支持。

## 线格式

```text
A5 5A | LEN(1) | CMD(1) | DATA(LEN) | CRC16(2) | FF
```

CRC16 初值 `0xFFFF`、反射多项式 `0xA001`；**覆盖 LEN + CMD + DATA**，
不包含帧头、CRC 本身和帧尾。CRC 发送时高字节在前。此覆盖范围与第一版
“只覆盖 DATA”不同，两端需要同时升级；旧版完整帧不能直接与本版互通。
它也不是 Modbus RTU：Modbus 的帧结构与 CRC 字节发送顺序不同。

字段按类型分组，依次为 `bool`、8 位整数、16 位整数、32 位整数、`float32`；
同组内保持参数顺序（包括展开固定数组的顺序），不是全局参数顺序。
`bool` 每 8 个占 1 字节，第一个放在 bit 0。整数和浮点位模式大端。
有符号与无符号整数在同一宽度组。当前支持：

```text
bool
int8_t / uint8_t
int16_t / uint16_t
int32_t / uint32_t
float
std::array<上述类型, N>
```

`double`、64 位整数、枚举、字符串、动态数组等在编译期拒绝；浮点常量请写成 `1.0f`。
发送 DATA 最多 100 字节，超限编译失败；接收解析器的内部缓存也按 100 字节
DATA 配置。线上不携带字段类型信息：长度相同但 schema 不同的错误无法自动识别，
发送端与接收端必须约定每条命令的字段类型和各组内顺序。

解析器按字节流工作，支持半包、粘包、前导噪声和 CRC／帧尾错误后的重新同步。
若损坏的 LEN 仍在合法范围内，解析器可能暂时等待更多字节；连接重建或接收超时
应调用 `reset()`。CRC 是传输错误检测，不提供身份认证。

## STM32 与构建

协议由 `include/wire_protocol/protocol.hpp` 和 `src/protocol.cpp` 组成，
不使用动态 `new`、`std::function`、`std::vector`、异常、RTTI 或虚函数。
注册时使用定位 `new` 在对象内部的固定槽位构造回调，不会申请堆内存。
CRC、帧解析、字节读写和 bool 位打包实现在 cpp；字段长度与类型推导、
handler 签名推导和静态 dispatch 是模板，保留在 hpp 中供调用方实例化。
`std::array` 和 `std::tuple` 均为对象内固定存储；`std::span` 只借用接收数据。
STM32 工具链需提供 C++20 的 `std::span`、`std::bit_cast`、concepts 和 `std::invoke`。
通常在主循环／任务中喂入 DMA 接收片段；如直接把 `pack()` 返回数组交给 UART DMA，
数组必须活到 DMA 发送完成。

在 ROS 2 工作区根目录执行：

```sh
colcon build --packages-select wire_protocol
```

其他 ROS 2 包在 `package.xml` 中声明 `<depend>wire_protocol</depend>`，
在 CMake 中使用 `find_package(wire_protocol REQUIRED)` 和
`ament_target_dependencies(目标名 wire_protocol)`。STM32 工程需要同时加入
`include/wire_protocol/protocol.hpp` 和 `src/protocol.cpp`，不需要 ROS 或 ament。
