#include "wire_protocol/protocol.hpp"

#include <cstring>

namespace wire_protocol::detail
{

//======================================================
// CRC16：初值 0xFFFF，反射多项式 0xA001
//======================================================

std::uint16_t crc16(std::span<const std::uint8_t> bytes) noexcept
{
    std::uint16_t crc = 0xFFFF;
    for (const auto byte : bytes)
    {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = static_cast<std::uint16_t>(
                (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0));
        }
    }
    return crc;
}

//======================================================
// Payload 编码：bool 按位打包，其余字段按大端字节序
//======================================================

PayloadWriter::PayloadWriter(std::span<std::uint8_t> payload,
                             std::size_t bool_count) noexcept
    : payload_(payload), cursor_((bool_count + 7) / 8)
{
}

void PayloadWriter::write_bool(bool value) noexcept
{
    if (value)
    {
        payload_[bool_index_ / 8] |=
            static_cast<std::uint8_t>(1U << (bool_index_ % 8));
    }
    ++bool_index_;
}

void PayloadWriter::write_u8(std::uint8_t value) noexcept
{
    payload_[cursor_++] = value;
}

void PayloadWriter::write_u16(std::uint16_t value) noexcept
{
    write_u8(static_cast<std::uint8_t>(value >> 8));
    write_u8(static_cast<std::uint8_t>(value));
}

void PayloadWriter::write_u32(std::uint32_t value) noexcept
{
    write_u8(static_cast<std::uint8_t>(value >> 24));
    write_u8(static_cast<std::uint8_t>(value >> 16));
    write_u8(static_cast<std::uint8_t>(value >> 8));
    write_u8(static_cast<std::uint8_t>(value));
}

PayloadReader::PayloadReader(std::span<const std::uint8_t> payload,
                             std::size_t bool_count) noexcept
    : payload_(payload), cursor_((bool_count + 7) / 8)
{
}

bool PayloadReader::read_bool() noexcept
{
    const auto value =
        (payload_[bool_index_ / 8] & (1U << (bool_index_ % 8))) != 0;
    ++bool_index_;
    return value;
}

std::uint8_t PayloadReader::read_u8() noexcept
{
    return payload_[cursor_++];
}

std::uint16_t PayloadReader::read_u16() noexcept
{
    const auto high = static_cast<std::uint16_t>(read_u8());
    return static_cast<std::uint16_t>((high << 8) | read_u8());
}

std::uint32_t PayloadReader::read_u32() noexcept
{
    const auto high = static_cast<std::uint32_t>(read_u16());
    return (high << 16) | read_u16();
}

void finish_frame(std::span<std::uint8_t> output,
                  std::uint8_t command) noexcept
{
    const auto length = output.size() - kFrameOverhead;
    output[0] = 0xA5;
    output[1] = 0x5A;
    output[2] = static_cast<std::uint8_t>(length);
    output[3] = command;

    // CRC 覆盖 LEN + CMD + DATA。
    const auto crc = crc16(output.subspan(2, length + 2));
    output[length + 4] = static_cast<std::uint8_t>(crc >> 8);
    output[length + 5] = static_cast<std::uint8_t>(crc);
    output[length + 6] = 0xFF;
}

//======================================================
// 串口字节流解析：半包、粘包与坏帧重新同步
//======================================================

void Parser::reset() noexcept
{
    size_ = 0;
}

void Parser::discard(std::size_t count) noexcept
{
    size_ -= count;
    if (size_ != 0)
    {
        std::memmove(buffer_.data(), buffer_.data() + count, size_);
    }
}

void Parser::append(std::uint8_t byte) noexcept
{
    if (size_ == buffer_.size())
    {
        discard(1);
    }
    buffer_[size_++] = byte;
}

bool Parser::next(FrameView& frame) noexcept
{
    while (size_ != 0)
    {
        if (buffer_[0] != 0xA5)
        {
            discard(1);
            continue;
        }
        if (size_ < 2)
        {
            return false;
        }
        if (buffer_[1] != 0x5A)
        {
            discard(1);
            continue;
        }
        if (size_ < 3)
        {
            return false;
        }

        const std::size_t length = buffer_[2];
        if (length > kMaxPayloadSize)
        {
            discard(1);
            continue;
        }
        const auto total = length + kFrameOverhead;
        if (size_ < total)
        {
            return false;
        }

        const auto received_crc = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(buffer_[length + 4]) << 8) |
            buffer_[length + 5]);
        const auto expected_crc = crc16({buffer_.data() + 2, length + 2});
        if (buffer_[total - 1] != 0xFF || received_crc != expected_crc)
        {
            discard(1);
            continue;
        }

        frame.command = buffer_[3];
        frame.payload = {buffer_.data() + 4, length};
        return true;
    }
    return false;
}

void Parser::consume() noexcept
{
    discard(static_cast<std::size_t>(buffer_[2]) + kFrameOverhead);
}

} // namespace wire_protocol::detail

namespace wire_protocol
{

//======================================================
// 固定容量回调表与字节流分发
//======================================================

CallbackProtocol::Entry* CallbackProtocol::free_entry(
    std::uint8_t command) noexcept
{
    Entry* available = nullptr;
    for (auto& entry : entries_)
    {
        if (entry.invoke != nullptr && entry.command == command)
        {
            return nullptr;
        }
        if (entry.invoke == nullptr && available == nullptr)
        {
            available = &entry;
        }
    }
    return available;
}

void CallbackProtocol::dispatch_frame(const detail::FrameView& frame)
{
    for (auto& entry : entries_)
    {
        if (entry.invoke != nullptr && entry.command == frame.command)
        {
            entry.invoke(static_cast<void*>(entry.storage.data()), frame);
            return;
        }
    }
}

void CallbackProtocol::feed(std::span<const std::uint8_t> bytes)
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

void CallbackProtocol::reset() noexcept
{
    parser_.reset();
}

} // namespace wire_protocol
