/**
 * @file MotionTelemetry.h
 * @brief Binary pressure-insole motion telemetry frame for the Python GUI.
 */

#ifndef MotionTelemetry_h
#define MotionTelemetry_h

#include "Arduino.h"
#include "ExoData.h"

#include <stdint.h>

namespace motion_telemetry
{
    static const uint8_t MAGIC[4] = {'M', 'O', 'T', 'N'};
    static const uint8_t VERSION = 1;

    namespace validity_flags
    {
        static const uint8_t pressure_valid = 1U << 0;
        static const uint8_t cop_valid = 1U << 1;
        static const uint8_t tau_proxy_valid = 1U << 2;
        static const uint8_t intent_unload_request = 1U << 3;
        static const uint8_t insole_heel_strike = 1U << 4;
        static const uint8_t insole_toe_off = 1U << 5;
    }

#pragma pack(push, 1)
    struct Frame
    {
        uint8_t magic[4];
        uint8_t version;
        uint8_t side;
        uint16_t frame_len;
        uint32_t timestamp_us;
        uint32_t sample_age_us;
        uint32_t raw_fault_flags;
        uint32_t hl_fault_flags;
        uint16_t zones_g[insole_defs::zone_count];
        uint8_t validity;
        uint8_t gait_state;
        uint8_t motion_state;
        uint8_t motion_intent;
        uint32_t insole_heel_strike_timestamp_us;
        uint32_t insole_toe_off_timestamp_us;
        float p_total;
        float p_fore;
        float p_arch;
        float p_heel;
        float p_medial;
        float p_lateral;
        float fz_n;
        float cop_x_mm;
        float cop_y_mm;
        float cop_ap_norm;
        float cop_ml_norm;
        float cop_ap_dot_norm_s;
        float cop_ml_dot_norm_s;
        float stance_phase;
        float tau_proxy_nm;
        float intent_confidence;
    };
#pragma pack(pop)

    class Publisher
    {
        public:
            Publisher();
            void update(ExoData* data, uint32_t now_us);

        private:
            void _maybe_send_side(const SideData& side_data, bool is_left, uint32_t now_us);
            void _build_frame(const SideData& side_data, bool is_left, uint32_t now_us, Frame& frame) const;

            uint32_t _last_sent_output_timestamp_us[2];
            uint32_t _last_send_us[2];
    };
}

#endif
