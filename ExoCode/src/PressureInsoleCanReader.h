/**
 * @file PressureInsoleCanReader.h
 * @brief Reconstructs 18-zone pressure-insole TTL packets from CAN fragments.
 */

#ifndef PressureInsoleCanReader_h
#define PressureInsoleCanReader_h

#include "Arduino.h"

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "CAN.h"
#include "SideData.h"
#include <stdint.h>

class PressureInsoleCanReader
{
    public:
        PressureInsoleCanReader();

        /**
         * @brief Attempts to read one decoded pressure-insole frame.
         *
         * Units and conventions:
         * - timestamp_us uses micros().
         * - frame.zones_g is grams, positive compressive normal load.
         * - CAN ID 0x601 is the left-foot stream and CAN ID 0x602 is the right-foot stream.
         * - The packet foot_id is still cross-checked against the dedicated CAN stream.
         * - Returns false when no complete 39-byte packet is available.
         */
        bool read_frame(InsoleRawFrame& frame, bool& is_left, uint32_t now_us);

    private:
        static const uint8_t PACKET_LEN = 39;
        static const uint8_t PACKET_HEADER = 0xAA;
        static const uint8_t MAX_BUFFER_LEN = 78;
        static const uint8_t MAX_CAN_FRAMES_PER_CALL = 8;

        struct StreamState
        {
            uint8_t buffer[MAX_BUFFER_LEN];
            uint8_t buffer_len;
            uint32_t last_fragment_us;
            uint32_t pending_fault_flags;

            StreamState()
            : buffer{0}
            , buffer_len(0)
            , last_fragment_us(0)
            , pending_fault_flags(0)
            {
            }
        };

        static bool _is_pressure_can_frame(const CAN_message_t& msg, void* context);
        static uint8_t _expected_foot_id_for_can_id(uint32_t can_id);

        bool _feed_can_payload(const CAN_message_t& msg, uint32_t now_us);
        bool _try_extract_packet(StreamState& stream, uint8_t expected_foot_id, InsoleRawFrame& frame, bool& is_left, uint32_t now_us);
        bool _checksum_ok(const StreamState& stream, uint8_t start_index) const;
        void _drop_bytes(StreamState& stream, uint8_t count);
        void _reset_stream(StreamState& stream, uint32_t fault_flags);
        StreamState* _stream_for_can_id(uint32_t can_id);

        StreamState _left_stream;
        StreamState _right_stream;
};

#endif
#endif
