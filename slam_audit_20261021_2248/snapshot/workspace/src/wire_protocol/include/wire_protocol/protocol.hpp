#ifndef WIRE_PROTOCOL_PROTOCOL_HPP_
#define WIRE_PROTOCOL_PROTOCOL_HPP_

#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <new>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>

namespace wire_protocol
{

inline constexpr std::size_t kFrameOverhead = 7;
inline constexpr std::size_t kMaxPayloadSize = 100;

namespace detail
{

//======================================================
// 字段类型与线格式
//======================================================

template <typename T>
inline constexpr bool scalar = std::is_same_v<T, bool> ||
                               std::is_same_v<T, std::int8_t> ||
                               std::is_same_v<T, std::uint8_t> ||
                               std::is_same_v<T, std::int16_t> ||
                               std::is_same_v<T, std::uint16_t> ||
                               std::is_same_v<T, std::int32_t> ||
                               std::is_same_v<T, std::uint32_t> ||
                               std::is_same_v<T, float>;

template <typename T>
struct Field
{
    using Value = T;
    static constexpr bool supported = scalar<T>;
    static constexpr std::size_t count = 1;
};

template <typename T, std::size_t N>
struct Field<std::array<T, N>>
{
    using Value = T;
    static constexpr bool supported = scalar<T>;
    static constexpr std::size_t count = N;
};

template <typename T>
using Traits = Field<std::remove_cvref_t<T>>;

template <typename T>
constexpr int group() noexcept
{
    using Value = typename Traits<T>::Value;
    if constexpr (std::is_same_v<Value, bool>)
    {
        return 0;
    }
    else if constexpr (std::is_same_v<Value, float>)
    {
        return 4;
    }
    else if constexpr (sizeof(Value) == 1)
    {
        return 1;
    }
    else if constexpr (sizeof(Value) == 2)
    {
        return 2;
    }
    else
    {
        return 3;
    }
}

template <int Group, typename... T>
constexpr std::size_t count() noexcept
{
    return (std::size_t{0} + ... + (group<T>() == Group ? Traits<T>::count : 0));
}

template <typename... T>
inline constexpr std::size_t payload_size =
    (count<0, T...>() + 7) / 8 + count<1, T...>() +
    2 * count<2, T...>() + 4 * count<3, T...>() + 4 * count<4, T...>();

template <int Group, typename T, typename Function>
void visit(T& field, Function&& function) noexcept
{
    if constexpr (group<T>() == Group)
    {
        if constexpr (scalar<std::remove_cvref_t<T>>)
        {
            function(field);
        }
        else
        {
            for (auto& value : field)
            {
                function(value);
            }
        }
    }
}

std::uint16_t crc16(std::span<const std::uint8_t> bytes) noexcept;

// 固定字节读写在 protocol.cpp；模板只负责把 C++ 字段映射到位模式。
class PayloadWriter
{
  public:
    PayloadWriter(std::span<std::uint8_t> payload, std::size_t bool_count) noexcept;

    void write_bool(bool value) noexcept;
    void write_u8(std::uint8_t value) noexcept;
    void write_u16(std::uint16_t value) noexcept;
    void write_u32(std::uint32_t value) noexcept;

  private:
    std::span<std::uint8_t> payload_;
    std::size_t bool_index_{};
    std::size_t cursor_{};
};

class PayloadReader
{
  public:
    PayloadReader(std::span<const std::uint8_t> payload,
                  std::size_t bool_count) noexcept;

    bool read_bool() noexcept;
    std::uint8_t read_u8() noexcept;
    std::uint16_t read_u16() noexcept;
    std::uint32_t read_u32() noexcept;

  private:
    std::span<const std::uint8_t> payload_;
    std::size_t bool_index_{};
    std::size_t cursor_{};
};

void finish_frame(std::span<std::uint8_t> output,
                  std::uint8_t command) noexcept;

} // namespace detail

template <typename T>
concept WireField = detail::Traits<T>::supported;

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "wire_protocol requires IEEE 754 float32");

namespace detail
{

//======================================================
// 模板连接层：字段布局在编译期确定
//======================================================

template <typename T>
void write_field(PayloadWriter& writer, T value) noexcept
{
    if constexpr (std::is_same_v<T, float>)
    {
        writer.write_u32(std::bit_cast<std::uint32_t>(value));
    }
    else if constexpr (sizeof(T) == 1)
    {
        writer.write_u8(std::bit_cast<std::uint8_t>(value));
    }
    else if constexpr (sizeof(T) == 2)
    {
        writer.write_u16(std::bit_cast<std::uint16_t>(value));
    }
    else
    {
        writer.write_u32(std::bit_cast<std::uint32_t>(value));
    }
}

template <typename T>
void read_field(PayloadReader& reader, T& value) noexcept
{
    if constexpr (std::is_same_v<T, float>)
    {
        value = std::bit_cast<float>(reader.read_u32());
    }
    else if constexpr (sizeof(T) == 1)
    {
        value = std::bit_cast<T>(reader.read_u8());
    }
    else if constexpr (sizeof(T) == 2)
    {
        value = std::bit_cast<T>(reader.read_u16());
    }
    else
    {
        value = std::bit_cast<T>(reader.read_u32());
    }
}

template <WireField... T>
[[nodiscard]] auto pack(std::uint8_t command, const T&... fields) noexcept
{
    constexpr auto size = payload_size<T...>;
    static_assert(size <= kMaxPayloadSize, "payload exceeds 100 bytes");

    std::array<std::uint8_t, size + kFrameOverhead> output{};
    PayloadWriter writer({output.data() + 4, size}, count<0, T...>());
    [[maybe_unused]] const auto write_bool = [&](bool value) noexcept
    {
        writer.write_bool(value);
    };
    [[maybe_unused]] const auto write_value = [&](auto value) noexcept
    {
        write_field(writer, value);
    };
    (visit<0>(fields, write_bool), ...);
    (visit<1>(fields, write_value), ...);
    (visit<2>(fields, write_value), ...);
    (visit<3>(fields, write_value), ...);
    (visit<4>(fields, write_value), ...);
    finish_frame(output, command);
    return output;
}

// payload 仅在 Parser 成功解析后、consume() 前有效。
struct FrameView
{
    std::uint8_t command{};
    std::span<const std::uint8_t> payload{};

    template <WireField... T>
        requires ((!std::is_const_v<T> && !std::is_volatile_v<T>) && ...)
    [[nodiscard]] bool unpack(T&... fields) const noexcept
    {
        constexpr auto size = payload_size<T...>;
        static_assert(size <= 255, "payload exceeds one-byte length");
        if (payload.size() != size)
        {
            return false;
        }

        PayloadReader reader(payload, count<0, T...>());
        [[maybe_unused]] const auto read_bool = [&](bool& value) noexcept
        {
            value = reader.read_bool();
        };
        [[maybe_unused]] const auto read_value = [&](auto& value) noexcept
        {
            read_field(reader, value);
        };
        (visit<0>(fields, read_bool), ...);
        (visit<1>(fields, read_value), ...);
        (visit<2>(fields, read_value), ...);
        (visit<3>(fields, read_value), ...);
        (visit<4>(fields, read_value), ...);
        return true;
    }
};

// 每路串口一个解析器；字节扫描与缓存管理实现在 protocol.cpp。
class Parser
{
  public:
    void reset() noexcept;
    void append(std::uint8_t byte) noexcept;
    [[nodiscard]] bool next(FrameView& frame) noexcept;
    void consume() noexcept;

  private:
    void discard(std::size_t count) noexcept;

    std::array<std::uint8_t, kMaxPayloadSize + kFrameOverhead> buffer_{};
    std::size_t size_{};
};

//======================================================
// handler 签名提取：普通函数、成员函数、明确签名的 functor
//======================================================

template <typename>
inline constexpr bool unsupported_callable = false;

template <typename Callable, typename = void>
struct CallableTraits
{
    static_assert(unsupported_callable<Callable>,
                  "dispatch requires a non-generic callable with explicit parameter types");
};

template <typename Result, typename... Args>
struct CallableTraits<Result (*)(Args...), void>
{
    using Return = Result;
    using Arguments = std::tuple<Args...>;
};

template <typename Result, typename... Args>
struct CallableTraits<Result (*)(Args...) noexcept, void>
    : CallableTraits<Result (*)(Args...)>
{
};

#define WIRE_PROTOCOL_MEMBER_TRAITS(QUALIFIERS)                              \
    template <typename Result, typename Class, typename... Args>              \
    struct CallableTraits<Result (Class::*)(Args...) QUALIFIERS, void>         \
    {                                                                           \
        using Return = Result;                                                   \
        using Arguments = std::tuple<Args...>;                                    \
    }

WIRE_PROTOCOL_MEMBER_TRAITS();
WIRE_PROTOCOL_MEMBER_TRAITS(const);
WIRE_PROTOCOL_MEMBER_TRAITS(noexcept);
WIRE_PROTOCOL_MEMBER_TRAITS(const noexcept);

#undef WIRE_PROTOCOL_MEMBER_TRAITS

template <typename Callable>
struct CallableTraits<Callable,
                      std::void_t<decltype(&Callable::operator())>>
    : CallableTraits<decltype(&Callable::operator())>
{
};

template <typename Method, typename Object>
struct MemberHandler
{
    Method method;
    Object* object;

    template <typename... Args>
    void operator()(Args&&... args)
    {
        std::invoke(method, *object, std::forward<Args>(args)...);
    }
};

template <typename Method, typename Object>
struct CallableTraits<MemberHandler<Method, Object>, void> : CallableTraits<Method>
{
};

template <typename Tuple>
struct ValidArguments;

template <typename... Args>
struct ValidArguments<std::tuple<Args...>>
    : std::bool_constant<(... &&
        (std::is_same_v<Args, std::remove_cvref_t<Args>> && WireField<Args>))>
{
};

} // namespace detail

//======================================================
// 命令绑定：handler 参数即 payload schema
//======================================================

template <typename Callable>
class Dispatch
{
    using Signature = detail::CallableTraits<Callable>;
    using Arguments = typename Signature::Arguments;

    static_assert(std::is_same_v<typename Signature::Return, void>,
                  "dispatch handler must return void");
    static_assert(detail::ValidArguments<Arguments>::value,
                  "dispatch parameters must be supported wire fields passed by value");

  public:
    explicit Dispatch(std::uint8_t command, Callable handler)
        : command_(command), handler_(std::move(handler))
    {
    }

    [[nodiscard]] std::uint8_t command() const noexcept
    {
        return command_;
    }

    void handle(const detail::FrameView& frame)
    {
        handle_with_arguments(frame, std::type_identity<Arguments>{});
    }

  private:
    template <typename... Args>
    void handle_with_arguments(
        const detail::FrameView& frame,
        std::type_identity<std::tuple<Args...>>)
    {
        std::tuple<Args...> values{};
        const bool decoded = std::apply(
            [&](auto&... value) { return frame.unpack(value...); }, values);
        if (decoded)
        {
            std::apply(
                [&](auto&... value) { std::invoke(handler_, value...); }, values);
        }
    }

    std::uint8_t command_{};
    Callable handler_;
};

template <typename Callable>
[[nodiscard]] auto dispatch(std::uint8_t command, Callable&& handler)
{
    using Stored = std::decay_t<Callable>;
    return Dispatch<Stored>{command, std::forward<Callable>(handler)};
}

template <typename Method, typename Object>
    requires std::is_member_function_pointer_v<Method>
[[nodiscard]] auto dispatch(std::uint8_t command, Method method, Object* object)
{
    using Stored = detail::MemberHandler<Method, Object>;
    return Dispatch<Stored>{command, Stored{method, object}};
}

//======================================================
// 高层 API：发送、接收、重置
//======================================================

template <typename... Dispatches>
class Protocol
{
  public:
    explicit Protocol(Dispatches... dispatches)
        : dispatches_(std::move(dispatches)...)
    {
    }

    template <WireField... T>
    [[nodiscard]] auto pack(std::uint8_t command, const T&... fields) const noexcept
    {
        return detail::pack(command, fields...);
    }

    void feed(std::span<const std::uint8_t> bytes)
    {
        for (const auto byte : bytes)
        {
            parser_.append(byte);
            detail::FrameView frame{};
            while (parser_.next(frame))
            {
                dispatch_frame(frame);
                parser_.consume();
            }
        }
    }

    void reset() noexcept
    {
        parser_.reset();
    }

  private:
    void dispatch_frame(const detail::FrameView& frame)
    {
        bool handled = false;
        const auto visit = [&](auto& entry)
        {
            if (!handled && entry.command() == frame.command)
            {
                handled = true;
                entry.handle(frame);
            }
        };
        std::apply([&](auto&... entries) { (visit(entries), ...); }, dispatches_);
    }

    std::tuple<Dispatches...> dispatches_;
    detail::Parser parser_;
};

template <typename... Dispatches>
Protocol(Dispatches...) -> Protocol<Dispatches...>;

template <typename... Dispatches>
[[nodiscard]] auto make_protocol(Dispatches&&... dispatches)
{
    return Protocol<std::decay_t<Dispatches>...>{
        std::forward<Dispatches>(dispatches)...};
}

//======================================================
// 固定容量回调协议：构造后注册，不使用动态内存
//======================================================

class CallbackProtocol
{
  public:
    static constexpr std::size_t kMaxCallbacks = 8;
    static constexpr std::size_t kCallbackStorageSize = 32;

    CallbackProtocol() = default;
    CallbackProtocol(const CallbackProtocol&) = delete;
    CallbackProtocol& operator=(const CallbackProtocol&) = delete;
    CallbackProtocol(CallbackProtocol&&) = delete;
    CallbackProtocol& operator=(CallbackProtocol&&) = delete;

    // 返回 false 表示命令重复、回调表已满或传入空函数/对象指针。
    // 在 feed() 开始前完成注册。
    template <typename Callable>
    [[nodiscard]] bool set_unpack_callback(std::uint8_t command,
                                           Callable&& callback) noexcept
    {
        using Stored = std::decay_t<Callable>;
        using Binding = Dispatch<Stored>;
        static_assert(std::is_trivially_copy_constructible_v<Stored> &&
                      std::is_trivially_move_constructible_v<Stored> &&
                      std::is_trivially_destructible_v<Stored>,
                      "callback must be trivially copyable and destructible");
        static_assert(sizeof(Binding) <= kCallbackStorageSize,
                      "callback exceeds fixed inline storage");
        static_assert(alignof(Binding) <= alignof(std::max_align_t),
                      "callback alignment exceeds inline storage");
        static_assert(std::is_trivially_destructible_v<Binding>,
                      "callback binding must be trivially destructible");

        if constexpr (std::is_pointer_v<std::remove_reference_t<Callable>>)
        {
            if (callback == nullptr)
            {
                return false;
            }
        }

        auto* entry = free_entry(command);
        if (entry == nullptr)
        {
            return false;
        }
        ::new (static_cast<void*>(entry->storage.data()))
            Binding(command, std::forward<Callable>(callback));
        entry->command = command;
        entry->invoke = [](void* data, const detail::FrameView& frame)
        {
            auto* binding = std::launder(reinterpret_cast<Binding*>(data));
            binding->handle(frame);
        };
        return true;
    }

    template <typename Method, typename Object>
        requires std::is_member_function_pointer_v<Method>
    [[nodiscard]] bool set_unpack_callback(std::uint8_t command,
                                           Method method,
                                           Object* object) noexcept
    {
        if (method == nullptr || object == nullptr)
        {
            return false;
        }
        using Stored = detail::MemberHandler<Method, Object>;
        return set_unpack_callback(command, Stored{method, object});
    }

    template <WireField... T>
    [[nodiscard]] auto pack(std::uint8_t command, const T&... fields) const noexcept
    {
        return detail::pack(command, fields...);
    }

    void feed(std::span<const std::uint8_t> bytes);
    void reset() noexcept;

  private:
    struct Entry
    {
        alignas(std::max_align_t)
        std::array<std::byte, kCallbackStorageSize> storage{};
        void (*invoke)(void*, const detail::FrameView&){};
        std::uint8_t command{};
    };

    [[nodiscard]] Entry* free_entry(std::uint8_t command) noexcept;
    void dispatch_frame(const detail::FrameView& frame);

    std::array<Entry, kMaxCallbacks> entries_{};
    detail::Parser parser_;
};

} // namespace wire_protocol

#endif // WIRE_PROTOCOL_PROTOCOL_HPP_
