/**
 * @file PressureInsoleHighLevel.h
 * @brief Raw pressure-insole to DPJMC high-level FSM interface.
 */

#ifndef PressureInsoleHighLevel_h
#define PressureInsoleHighLevel_h

#include "Arduino.h"
#include "SideData.h"

class ExoData;

/**
 * @brief Processes each side's 18-zone pressure insole raw frame into SideData::dpjmc_hl.
 *
 * The module is intentionally outside the DJPMC controller so it can observe both feet for
 * bilateral coordination while preserving the legacy heel/toe FSR gait path.
 */
class PressureInsoleHighLevel
{
    public:
        PressureInsoleHighLevel();

        /**
         * @brief Copies a newly decoded raw pressure-insole frame into the high-level processor.
         *
         * @param is_left true for left side, false for right side.
         * @param frame raw frame in grams with timestamp_us from micros().
         */
        void set_raw_frame(bool is_left, const InsoleRawFrame& frame);

        /**
         * @brief Updates both sides and writes the latest high-level values to ExoData::left/right_side.dpjmc_hl.
         *
         * @param data shared Exo data.
         * @param now_us current micros() timestamp.
         */
        void update(ExoData* data, uint32_t now_us);

    private:
        class FootProcessor
        {
            public:
                FootProcessor(bool is_left = false);

                void set_raw_frame(const InsoleRawFrame& frame);
                void update(SideData& side_data, uint32_t now_us);
                bool is_fresh_swing(uint32_t now_us) const;

            private:
                void _reset_internal_state();
                void _clear_event_pulses();
                void _apply_event_state(DPJMCHighLevelInput& out) const;
                void _set_invalid_output(DPJMCHighLevelInput& out, uint32_t timestamp_us, uint32_t fault_flags);
                void _process_new_frame(SideData& side_data, uint32_t now_us);
                void _update_contact_state(float total_g, float forefoot_g, float arch_g, float heel_g, uint32_t frame_time_us);
                void _update_zero_refresh(const InsoleRawFrame& raw, float total_g, uint32_t frame_time_us, uint32_t& fault_flags);
                void _write_valid_output(SideData& side_data, uint32_t fault_flags, uint32_t frame_time_us);
                InsoleGaitState _classify_gait(float forefoot_g, float heel_g, float cop_ap_norm, float stance_phase) const;
                InsoleMotionState _classify_motion_state(const DPJMCHighLevelInput& out, uint32_t frame_time_us) const;
                InsoleMotionIntent _classify_intent(float cop_ml_norm, float& confidence) const;
                float _zone_ml_norm(uint8_t zone_index) const;
                float _calc_tau_proxy_nm(const SideData& side_data, float fz_n, float cop_ap_norm) const;

                bool _is_left;
                bool _has_frame;
                bool _filters_initialized;
                bool _confirmed_contact;
                bool _candidate_contact;
                bool _heel_strike_pulse;
                bool _toe_off_pulse;
                uint8_t _candidate_frames;

                InsoleRawFrame _raw;
                uint32_t _last_processed_timestamp_us;
                uint32_t _stance_start_us;
                uint32_t _swing_start_us;
                uint32_t _last_heel_strike_us;
                uint32_t _last_toe_off_us;
                uint32_t _expected_stance_us;
                uint32_t _no_load_start_us;
                uint32_t _last_zero_refresh_us;

                float _offset_g[insole_defs::zone_count];
                float _prev_raw_corrected_g[insole_defs::zone_count];
                float _prev2_raw_corrected_g[insole_defs::zone_count];
                float _filtered_g[insole_defs::zone_count];
                float _last_cop_ap_norm;
                float _last_cop_ml_norm;
                uint32_t _last_cop_timestamp_us;
                bool _last_cop_valid;
        };

        FootProcessor _left;
        FootProcessor _right;
        uint32_t _both_swing_start_us;
};

#endif
