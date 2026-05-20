/**
 * @file PressureInsoleCanReader.cpp
 * @brief CAN byte-stream decoder for the 18-zone pressure insole.
 */

#include "PressureInsoleCanReader.h"

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Config.h"

namespace
{
    bool elapsed_us(uint32_t now_us, uint32_t start_us, uint32_t duration_us)
    {
        return (uint32_t)(now_us - start_us) >= duration_us;
    }
}

PressureInsoleCanReader::PressureInsoleCanReader()
{
}

bool PressureInsoleCanReader::read_frame(InsoleRawFrame& frame, bool& is_left, uint32_t now_us)
{
    if (_try_extract_packet(_left_stream, 1, frame, is_left, now_us))
    {
        return true;
    }

    if (_try_extract_packet(_right_stream, 2, frame, is_left, now_us))
    {
        return true;
    }

    CAN* can = CAN::getInstance();
    CAN_message_t msg;
    uint8_t frames_read = 0;
    while ((frames_read < MAX_CAN_FRAMES_PER_CALL) &&
           can->read_matching(msg, PressureInsoleCanReader::_is_pressure_can_frame, this))
    {
        frames_read++;
        _feed_can_payload(msg, now_us);
        StreamState* stream = _stream_for_can_id(msg.id);
        if (stream != NULL &&
            _try_extract_packet(*stream, _expected_foot_id_for_can_id(msg.id), frame, is_left, now_us))
        {
            return true;
        }
    }

    return false;
}

bool PressureInsoleCanReader::_is_pressure_can_frame(const CAN_message_t& msg, void* context)
{
    (void)context;

    if (msg.flags.extended || msg.len == 0)
    {
        return false;
    }

    return msg.id == insole_can_config::LEFT_PRESSURE_CAN_ID ||
           msg.id == insole_can_config::RIGHT_PRESSURE_CAN_ID;
}

uint8_t PressureInsoleCanReader::_expected_foot_id_for_can_id(uint32_t can_id)
{
    if (can_id == insole_can_config::LEFT_PRESSURE_CAN_ID)
    {
        return 1;
    }

    if (can_id == insole_can_config::RIGHT_PRESSURE_CAN_ID)
    {
        return 2;
    }

    return 0;
}

bool PressureInsoleCanReader::_feed_can_payload(const CAN_message_t& msg, uint32_t now_us)
{
    StreamState* stream = _stream_for_can_id(msg.id);
    if (stream == NULL)
    {
        return false;
    }

    if (stream->buffer_len > 0 &&
        stream->last_fragment_us != 0 &&
        elapsed_us(now_us, stream->last_fragment_us, insole_can_config::FRAGMENT_TIMEOUT_US))
    {
        _reset_stream(*stream, insole_fault_flags::timeout | insole_fault_flags::invalid_frame);
    }

    stream->last_fragment_us = now_us;

    for (uint8_t i = 0; i < msg.len && i < 8; i++)
    {
        if (stream->buffer_len >= MAX_BUFFER_LEN)
        {
            _drop_bytes(*stream, 1);
            stream->pending_fault_flags |= insole_fault_flags::invalid_frame;
        }
        stream->buffer[stream->buffer_len++] = msg.buf[i];
    }

    return true;
}

bool PressureInsoleCanReader::_try_extract_packet(
    StreamState& stream,
    uint8_t expected_foot_id,
    InsoleRawFrame& frame,
    bool& is_left,
    uint32_t now_us)
{
    while (stream.buffer_len > 0)
    {
        uint8_t header_index = 0;
        while (header_index < stream.buffer_len && stream.buffer[header_index] != PACKET_HEADER)
        {
            header_index++;
        }

        if (header_index >= stream.buffer_len)
        {
            _drop_bytes(stream, stream.buffer_len);
            stream.pending_fault_flags |= insole_fault_flags::invalid_frame;
            return false;
        }

        if (header_index > 0)
        {
            _drop_bytes(stream, header_index);
            stream.pending_fault_flags |= insole_fault_flags::invalid_frame;
        }

        if (stream.buffer_len < PACKET_LEN)
        {
            return false;
        }

        const uint8_t foot_id = stream.buffer[1];
        if (foot_id != expected_foot_id)
        {
            _drop_bytes(stream, 1);
            stream.pending_fault_flags |= insole_fault_flags::invalid_frame;
            continue;
        }

        if (!_checksum_ok(stream, 0))
        {
            _drop_bytes(stream, 1);
            stream.pending_fault_flags |= insole_fault_flags::invalid_frame;
            continue;
        }

        frame.reset();
        frame.timestamp_us = now_us;
        frame.valid = true;
        frame.fault_flags = stream.pending_fault_flags;
        for (uint8_t zone = 0; zone < insole_defs::zone_count; zone++)
        {
            const uint8_t hi = stream.buffer[2 + zone * 2];
            const uint8_t lo = stream.buffer[3 + zone * 2];
            frame.zones_g[zone] = ((uint16_t)hi << 8) | (uint16_t)lo;
        }

        is_left = (expected_foot_id == 1);
        stream.pending_fault_flags = 0;
        _drop_bytes(stream, PACKET_LEN);
        return true;
    }

    return false;
}

bool PressureInsoleCanReader::_checksum_ok(const StreamState& stream, uint8_t start_index) const
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < PACKET_LEN - 1; i++)
    {
        checksum += stream.buffer[start_index + i];
    }

    return checksum == stream.buffer[start_index + PACKET_LEN - 1];
}

void PressureInsoleCanReader::_drop_bytes(StreamState& stream, uint8_t count)
{
    if (count >= stream.buffer_len)
    {
        stream.buffer_len = 0;
        return;
    }

    for (uint8_t i = count; i < stream.buffer_len; i++)
    {
        stream.buffer[i - count] = stream.buffer[i];
    }
    stream.buffer_len -= count;
}

void PressureInsoleCanReader::_reset_stream(StreamState& stream, uint32_t fault_flags)
{
    stream.buffer_len = 0;
    stream.pending_fault_flags |= fault_flags;
}

PressureInsoleCanReader::StreamState* PressureInsoleCanReader::_stream_for_can_id(uint32_t can_id)
{
    if (can_id == insole_can_config::LEFT_PRESSURE_CAN_ID)
    {
        return &_left_stream;
    }

    if (can_id == insole_can_config::RIGHT_PRESSURE_CAN_ID)
    {
        return &_right_stream;
    }

    return NULL;
}

#endif
