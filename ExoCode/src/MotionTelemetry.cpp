/**
 * @file MotionTelemetry.cpp
 * @brief Binary pressure-insole motion telemetry publisher.
 */

#include "MotionTelemetry.h"

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include "Config.h"
#include "UARTHandler.h"
#include "uart_commands.h"

namespace
{
    bool elapsed_us(uint32_t now_us, uint32_t start_us, uint32_t duration_us)
    {
        return (uint32_t)(now_us - start_us) >= duration_us;
    }
}

static_assert(sizeof(motion_telemetry::Frame) <= UART_MSG_T_MAX_RAW_DATA_LEN, "Motion frame exceeds UART raw payload capacity");

motion_telemetry::Publisher::Publisher()
{
    _last_sent_output_timestamp_us[0] = 0;
    _last_sent_output_timestamp_us[1] = 0;
    _last_send_us[0] = 0;
    _last_send_us[1] = 0;
}

void motion_telemetry::Publisher::update(ExoData* data, uint32_t now_us)
{
    if (data == nullptr || !data->motion_telemetry_enabled)
    {
        return;
    }

    _maybe_send_side(data->left_side, true, now_us);
    _maybe_send_side(data->right_side, false, now_us);
}

void motion_telemetry::Publisher::_maybe_send_side(const SideData& side_data, bool is_left, uint32_t now_us)
{
    if (side_data.latest_valid_insole_raw.timestamp_us == 0)
    {
        return;
    }

    const uint8_t index = is_left ? 0 : 1;
    const uint32_t output_timestamp_us = side_data.dpjmc_hl.timestamp_us;
    const bool has_new_output = output_timestamp_us != _last_sent_output_timestamp_us[index];
    const bool heartbeat_due = (_last_send_us[index] == 0) ||
        elapsed_us(now_us, _last_send_us[index], insole_high_level::EXPECTED_PACKET_PERIOD_US);
    if (!has_new_output && !heartbeat_due)
    {
        return;
    }

    Frame frame = {};
    _build_frame(side_data, is_left, now_us, frame);

    UARTHandler* uart_handler = UARTHandler::get_instance();
    uart_handler->UART_raw_msg(
        UART_command_names::update_motion_telemetry,
        0,
        reinterpret_cast<const uint8_t*>(&frame),
        (uint8_t)sizeof(Frame)
    );

    _last_sent_output_timestamp_us[index] = output_timestamp_us;
    _last_send_us[index] = now_us;
}

void motion_telemetry::Publisher::_build_frame(const SideData& side_data, bool is_left, uint32_t now_us, Frame& frame) const
{
    const InsoleRawFrame& raw = side_data.latest_valid_insole_raw;
    const DPJMCHighLevelInput& hl = side_data.dpjmc_hl;

    frame.magic[0] = MAGIC[0];
    frame.magic[1] = MAGIC[1];
    frame.magic[2] = MAGIC[2];
    frame.magic[3] = MAGIC[3];
    frame.version = VERSION;
    frame.side = is_left ? 1 : 2;
    frame.frame_len = (uint16_t)sizeof(Frame);
    frame.timestamp_us = hl.timestamp_us;
    frame.sample_age_us = (hl.timestamp_us == 0) ? 0xFFFFFFFFUL : (uint32_t)(now_us - hl.timestamp_us);
    frame.raw_fault_flags = raw.fault_flags;
    frame.hl_fault_flags = hl.fault_flags;

    for (uint8_t i = 0; i < insole_defs::zone_count; i++)
    {
        frame.zones_g[i] = raw.zones_g[i];
    }

    if (hl.pressure_valid)
    {
        frame.validity |= validity_flags::pressure_valid;
    }
    if (hl.cop_valid)
    {
        frame.validity |= validity_flags::cop_valid;
    }
    if (hl.tau_proxy_valid)
    {
        frame.validity |= validity_flags::tau_proxy_valid;
    }
    if (hl.intent_unload_request)
    {
        frame.validity |= validity_flags::intent_unload_request;
    }
    if (hl.insole_heel_strike)
    {
        frame.validity |= validity_flags::insole_heel_strike;
    }
    if (hl.insole_toe_off)
    {
        frame.validity |= validity_flags::insole_toe_off;
    }

    frame.gait_state = hl.gait_state;
    frame.motion_state = hl.motion_state;
    frame.motion_intent = hl.motion_intent;
    frame.insole_heel_strike_timestamp_us = hl.insole_heel_strike_timestamp_us;
    frame.insole_toe_off_timestamp_us = hl.insole_toe_off_timestamp_us;
    frame.p_total = hl.p_total;
    frame.p_fore = hl.p_fore;
    frame.p_arch = hl.p_arch;
    frame.p_heel = hl.p_heel;
    frame.p_medial = hl.p_medial;
    frame.p_lateral = hl.p_lateral;
    frame.fz_n = hl.fz_n;
    frame.cop_x_mm = hl.cop_x_mm;
    frame.cop_y_mm = hl.cop_y_mm;
    frame.cop_ap_norm = hl.cop_ap_norm;
    frame.cop_ml_norm = hl.cop_ml_norm;
    frame.cop_ap_dot_norm_s = hl.cop_ap_dot_norm_s;
    frame.cop_ml_dot_norm_s = hl.cop_ml_dot_norm_s;
    frame.stance_phase = hl.stance_phase;
    frame.tau_proxy_nm = hl.tau_proxy_nm;
    frame.intent_confidence = hl.intent_confidence;
}

#endif
