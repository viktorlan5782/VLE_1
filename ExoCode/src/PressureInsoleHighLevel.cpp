/**
 * @file PressureInsoleHighLevel.cpp
 * @brief Raw pressure-insole preprocessing, COP, gait FSM, and DPJMC high-level outputs.
 */

#include "PressureInsoleHighLevel.h"

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Config.h"
#include "ControllerData.h"
#include "ExoData.h"
#include "ParseIni.h"

#include <math.h>

namespace
{
    // Zone centers are derived from Documentation/Insole/insole_geometry_parameter.m.
    // Raw zones_g[0] maps to manual point 1, zones_g[17] maps to manual point 18.
    const float zone_center_x_mm[insole_defs::zone_count] =
    {
        12.08f, 6.90f, 40.68f, 40.68f, 32.80f, 22.49f,
        30.72f, 23.28f, 64.27f, 64.27f, 50.93f, 48.67f,
        49.97f, 38.71f, 68.00f, 71.34f, 68.70f, 59.46f
    };

    const float zone_center_y_mm[insole_defs::zone_count] =
    {
        188.14f, 227.50f, 49.82f, 84.70f, 119.14f, 153.56f,
        188.14f, 229.97f, 49.82f, 84.70f, 119.14f, 153.56f,
        188.14f, 229.97f, 119.14f, 153.56f, 188.14f, 225.20f
    };

    const float foot_y_min_mm = 36.68f;
    const float foot_y_max_mm = 252.20f;
    const float foot_center_x_mm = 39.805f;
    const float foot_half_width_mm = 39.805f;

    float clamp_float(float value, float min_value, float max_value)
    {
        if (value < min_value)
        {
            return min_value;
        }
        if (value > max_value)
        {
            return max_value;
        }
        return value;
    }

    bool finite_float(float value)
    {
        return value == value && fabsf(value) < 1000000.0f;
    }

    bool elapsed_us(uint32_t now_us, uint32_t start_us, uint32_t duration_us)
    {
        return (uint32_t)(now_us - start_us) >= duration_us;
    }

    float median3(float a, float b, float c)
    {
        if ((a <= b && b <= c) || (c <= b && b <= a))
        {
            return b;
        }
        if ((b <= a && a <= c) || (c <= a && a <= b))
        {
            return a;
        }
        return c;
    }

    float ewma(float previous, float sample, float alpha)
    {
        return previous + alpha * (sample - previous);
    }

    float zone_ap_norm(uint8_t zone_index)
    {
        const float ap = (zone_center_y_mm[zone_index] - foot_y_min_mm) / (foot_y_max_mm - foot_y_min_mm);
        return clamp_float(ap, 0.0f, 1.0f);
    }
}

PressureInsoleHighLevel::PressureInsoleHighLevel()
: _left(true)
, _right(false)
, _both_swing_start_us(0)
{
}

void PressureInsoleHighLevel::set_raw_frame(bool is_left, const InsoleRawFrame& frame)
{
    if (is_left)
    {
        _left.set_raw_frame(frame);
    }
    else
    {
        _right.set_raw_frame(frame);
    }
}

void PressureInsoleHighLevel::update(ExoData* data, uint32_t now_us)
{
    if (data == nullptr)
    {
        return;
    }

    _left.update(data->left_side, now_us);
    _right.update(data->right_side, now_us);

    data->left_side.dpjmc_hl.fault_flags &= ~insole_fault_flags::bilateral_inconsistency;
    data->right_side.dpjmc_hl.fault_flags &= ~insole_fault_flags::bilateral_inconsistency;

    const bool both_swing = _left.is_fresh_swing(now_us) && _right.is_fresh_swing(now_us);
    if (both_swing)
    {
        if (_both_swing_start_us == 0)
        {
            _both_swing_start_us = now_us;
        }

        if (elapsed_us(now_us, _both_swing_start_us, insole_high_level::BILATERAL_SWING_FAULT_US))
        {
            data->left_side.dpjmc_hl.fault_flags |= insole_fault_flags::bilateral_inconsistency;
            data->right_side.dpjmc_hl.fault_flags |= insole_fault_flags::bilateral_inconsistency;
            data->left_side.dpjmc_hl.motion_state = static_cast<uint8_t>(InsoleMotionState::Unload);
            data->right_side.dpjmc_hl.motion_state = static_cast<uint8_t>(InsoleMotionState::Unload);
            data->left_side.dpjmc_hl.motion_intent = static_cast<uint8_t>(InsoleMotionIntent::Unload);
            data->right_side.dpjmc_hl.motion_intent = static_cast<uint8_t>(InsoleMotionIntent::Unload);
            data->left_side.dpjmc_hl.intent_unload_request = true;
            data->right_side.dpjmc_hl.intent_unload_request = true;
        }
    }
    else
    {
        _both_swing_start_us = 0;
    }
}

PressureInsoleHighLevel::FootProcessor::FootProcessor(bool is_left)
: _is_left(is_left)
{
    _reset_internal_state();
}

void PressureInsoleHighLevel::FootProcessor::set_raw_frame(const InsoleRawFrame& frame)
{
    _raw = frame;
    _has_frame = true;
}

void PressureInsoleHighLevel::FootProcessor::update(SideData& side_data, uint32_t now_us)
{
    _clear_event_pulses();
    _apply_event_state(side_data.dpjmc_hl);

    if (!_has_frame)
    {
        _set_invalid_output(side_data.dpjmc_hl, 0, insole_fault_flags::invalid_frame | insole_fault_flags::timeout);
        return;
    }

    if (!_raw.valid || _raw.timestamp_us == 0)
    {
        _set_invalid_output(side_data.dpjmc_hl, _raw.timestamp_us, _raw.fault_flags | insole_fault_flags::invalid_frame);
        return;
    }

    if (elapsed_us(now_us, _raw.timestamp_us, insole_high_level::FRAME_TIMEOUT_US))
    {
        _set_invalid_output(side_data.dpjmc_hl, _raw.timestamp_us, _raw.fault_flags | insole_fault_flags::timeout);
        return;
    }

    if (_raw.timestamp_us == _last_processed_timestamp_us)
    {
        return;
    }

    _process_new_frame(side_data, now_us);
}

bool PressureInsoleHighLevel::FootProcessor::is_fresh_swing(uint32_t now_us) const
{
    if (!_has_frame || !_raw.valid || _raw.timestamp_us == 0)
    {
        return false;
    }

    if (elapsed_us(now_us, _raw.timestamp_us, insole_high_level::FRAME_TIMEOUT_US))
    {
        return false;
    }

    return !_confirmed_contact;
}

void PressureInsoleHighLevel::FootProcessor::_reset_internal_state()
{
    _has_frame = false;
    _filters_initialized = false;
    _confirmed_contact = false;
    _candidate_contact = false;
    _heel_strike_pulse = false;
    _toe_off_pulse = false;
    _candidate_frames = 0;
    _last_processed_timestamp_us = 0;
    _stance_start_us = 0;
    _swing_start_us = 0;
    _last_heel_strike_us = 0;
    _last_toe_off_us = 0;
    _expected_stance_us = insole_high_level::EXPECTED_STANCE_US;
    _no_load_start_us = 0;
    _last_zero_refresh_us = 0;
    _last_cop_ap_norm = 0.0f;
    _last_cop_ml_norm = 0.0f;
    _last_cop_timestamp_us = 0;
    _last_cop_valid = false;
    _raw.reset();

    for (uint8_t i = 0; i < insole_defs::zone_count; i++)
    {
        _offset_g[i] = 0.0f;
        _prev_raw_corrected_g[i] = 0.0f;
        _prev2_raw_corrected_g[i] = 0.0f;
        _filtered_g[i] = 0.0f;
    }
}

void PressureInsoleHighLevel::FootProcessor::_clear_event_pulses()
{
    _heel_strike_pulse = false;
    _toe_off_pulse = false;
}

void PressureInsoleHighLevel::FootProcessor::_apply_event_state(DPJMCHighLevelInput& out) const
{
    out.insole_heel_strike = _heel_strike_pulse;
    out.insole_toe_off = _toe_off_pulse;
    out.insole_heel_strike_timestamp_us = _last_heel_strike_us;
    out.insole_toe_off_timestamp_us = _last_toe_off_us;
}

void PressureInsoleHighLevel::FootProcessor::_set_invalid_output(DPJMCHighLevelInput& out, uint32_t timestamp_us, uint32_t fault_flags)
{
    out.reset();
    out.timestamp_us = timestamp_us;
    out.pressure_valid = false;
    out.cop_valid = false;
    out.gait_state = static_cast<uint8_t>(InsoleGaitState::Invalid);
    out.motion_state = static_cast<uint8_t>(InsoleMotionState::Unknown);
    out.motion_intent = static_cast<uint8_t>(InsoleMotionIntent::Unload);
    out.intent_confidence = 1.0f;
    out.intent_unload_request = true;
    out.fault_flags = fault_flags;
    _apply_event_state(out);
}

void PressureInsoleHighLevel::FootProcessor::_process_new_frame(SideData& side_data, uint32_t now_us)
{
    (void)now_us;

    uint32_t fault_flags = _raw.fault_flags;
    _last_processed_timestamp_us = _raw.timestamp_us;
    side_data.latest_valid_insole_raw = _raw;

    float corrected_g[insole_defs::zone_count];
    float total_g = 0.0f;
    float forefoot_g = 0.0f;
    float arch_g = 0.0f;
    float heel_g = 0.0f;
    float medial_g = 0.0f;
    float lateral_g = 0.0f;

    if (_last_zero_refresh_us == 0)
    {
        _last_zero_refresh_us = _raw.timestamp_us;
    }

    for (uint8_t i = 0; i < insole_defs::zone_count; i++)
    {
        float raw_g = (float)_raw.zones_g[i];
        raw_g = clamp_float(raw_g, insole_high_level::ZONE_MIN_G, insole_high_level::ZONE_MAX_G);
        if (raw_g >= insole_high_level::SATURATION_THRESHOLD_G)
        {
            fault_flags |= insole_fault_flags::saturation;
        }

        corrected_g[i] = raw_g - _offset_g[i];
        if (corrected_g[i] < 0.0f)
        {
            corrected_g[i] = 0.0f;
        }

        if (!_filters_initialized)
        {
            _filtered_g[i] = corrected_g[i];
            _prev_raw_corrected_g[i] = corrected_g[i];
            _prev2_raw_corrected_g[i] = corrected_g[i];
        }
        else
        {
            float sample_g = median3(corrected_g[i], _prev_raw_corrected_g[i], _prev2_raw_corrected_g[i]);
            if (fabsf(corrected_g[i] - sample_g) > insole_high_level::SPIKE_REJECTION_DELTA_G)
            {
                sample_g = median3(sample_g, _prev_raw_corrected_g[i], _prev2_raw_corrected_g[i]);
            }
            _filtered_g[i] = ewma(_filtered_g[i], sample_g, insole_high_level::FILTER_EWMA_ALPHA);
            _prev2_raw_corrected_g[i] = _prev_raw_corrected_g[i];
            _prev_raw_corrected_g[i] = corrected_g[i];
        }

        total_g += _filtered_g[i];
        if (i <= 5)
        {
            forefoot_g += _filtered_g[i];
        }
        else if (i <= 11)
        {
            arch_g += _filtered_g[i];
        }
        else
        {
            heel_g += _filtered_g[i];
        }

        if (_zone_ml_norm(i) >= 0.0f)
        {
            lateral_g += _filtered_g[i];
        }
        else
        {
            medial_g += _filtered_g[i];
        }
    }

    _filters_initialized = true;

    _update_contact_state(total_g, forefoot_g, arch_g, heel_g, _raw.timestamp_us);
    _update_zero_refresh(_raw, total_g, _raw.timestamp_us, fault_flags);
    _write_valid_output(side_data, fault_flags, _raw.timestamp_us);

    side_data.dpjmc_hl.p_total = _confirmed_contact ? total_g : 0.0f;
    side_data.dpjmc_hl.p_fore = _confirmed_contact ? forefoot_g : 0.0f;
    side_data.dpjmc_hl.p_arch = _confirmed_contact ? arch_g : 0.0f;
    side_data.dpjmc_hl.p_heel = _confirmed_contact ? heel_g : 0.0f;
    side_data.dpjmc_hl.p_medial = _confirmed_contact ? medial_g : 0.0f;
    side_data.dpjmc_hl.p_lateral = _confirmed_contact ? lateral_g : 0.0f;
    side_data.dpjmc_hl.fz_n = _confirmed_contact ? (total_g * 0.00980665f) : 0.0f;
    side_data.dpjmc_hl.motion_state = static_cast<uint8_t>(_classify_motion_state(side_data.dpjmc_hl, _raw.timestamp_us));
}

void PressureInsoleHighLevel::FootProcessor::_update_contact_state(float total_g, float forefoot_g, float arch_g, float heel_g, uint32_t frame_time_us)
{
    uint8_t loaded_regions = 0;
    if (forefoot_g > insole_high_level::FOREFOOT_THRESHOLD_G)
    {
        loaded_regions++;
    }
    if (arch_g > insole_high_level::ARCH_THRESHOLD_G)
    {
        loaded_regions++;
    }
    if (heel_g > insole_high_level::HEEL_THRESHOLD_G)
    {
        loaded_regions++;
    }

    bool proposed_contact = _confirmed_contact;
    if (total_g > (insole_high_level::TOTAL_STANCE_THRESHOLD_G * insole_high_level::STANCE_ON_FACTOR) && loaded_regions > 0)
    {
        proposed_contact = true;
    }
    else if (total_g < (insole_high_level::TOTAL_STANCE_THRESHOLD_G * insole_high_level::SWING_OFF_FACTOR) || loaded_regions == 0)
    {
        proposed_contact = false;
    }

    if (proposed_contact != _candidate_contact)
    {
        _candidate_contact = proposed_contact;
        _candidate_frames = 1;
    }
    else if (_candidate_frames < 255)
    {
        _candidate_frames++;
    }

    if (_candidate_contact != _confirmed_contact && _candidate_frames >= insole_high_level::MIN_STATE_FRAMES)
    {
        if (_candidate_contact)
        {
            _confirmed_contact = true;
            _stance_start_us = frame_time_us;
            _heel_strike_pulse = true;
            _last_heel_strike_us = frame_time_us;
        }
        else
        {
            if (_stance_start_us != 0)
            {
                const uint32_t measured_stance_us = frame_time_us - _stance_start_us;
                if (measured_stance_us > 200000 && measured_stance_us < 2000000)
                {
                    _expected_stance_us = (uint32_t)(0.8f * (float)_expected_stance_us + 0.2f * (float)measured_stance_us);
                }
            }
            _confirmed_contact = false;
            _swing_start_us = frame_time_us;
            _toe_off_pulse = true;
            _last_toe_off_us = frame_time_us;
        }
    }
}

void PressureInsoleHighLevel::FootProcessor::_update_zero_refresh(const InsoleRawFrame& raw, float total_g, uint32_t frame_time_us, uint32_t& fault_flags)
{
    if (_confirmed_contact)
    {
        _no_load_start_us = 0;
    }
    else if (total_g < insole_high_level::NO_LOAD_THRESHOLD_G)
    {
        if (_no_load_start_us == 0)
        {
            _no_load_start_us = frame_time_us;
        }

        if (elapsed_us(frame_time_us, _no_load_start_us, insole_high_level::ZERO_REFRESH_HOLD_US))
        {
            for (uint8_t i = 0; i < insole_defs::zone_count; i++)
            {
                _offset_g[i] = ewma(_offset_g[i], (float)raw.zones_g[i], insole_high_level::ZERO_REFRESH_EWMA_ALPHA);
            }
            _last_zero_refresh_us = frame_time_us;
        }
    }
    else
    {
        _no_load_start_us = 0;
    }

    if (_last_zero_refresh_us != 0 && elapsed_us(frame_time_us, _last_zero_refresh_us, insole_high_level::ZERO_REFRESH_STALE_US))
    {
        fault_flags |= insole_fault_flags::zero_refresh_stale;
    }
}

void PressureInsoleHighLevel::FootProcessor::_write_valid_output(SideData& side_data, uint32_t fault_flags, uint32_t frame_time_us)
{
    DPJMCHighLevelInput& out = side_data.dpjmc_hl;
    out.reset();
    out.timestamp_us = frame_time_us;
    out.pressure_valid = true;
    out.fault_flags = fault_flags;

    float total_g = 0.0f;
    float cop_ap_num = 0.0f;
    float cop_ml_num = 0.0f;
    float cop_x_num = 0.0f;
    float cop_y_num = 0.0f;
    float forefoot_g = 0.0f;
    float heel_g = 0.0f;

    for (uint8_t i = 0; i < insole_defs::zone_count; i++)
    {
        const float zone_g = _filtered_g[i];
        total_g += zone_g;
        cop_ap_num += zone_g * zone_ap_norm(i);
        cop_ml_num += zone_g * _zone_ml_norm(i);
        cop_x_num += zone_g * zone_center_x_mm[i];
        cop_y_num += zone_g * zone_center_y_mm[i];

        if (i <= 5)
        {
            forefoot_g += zone_g;
        }
        else if (i >= 12)
        {
            heel_g += zone_g;
        }
    }

    const bool has_cop = _confirmed_contact && total_g >= insole_high_level::COP_MIN_TOTAL_G;
    if (!has_cop)
    {
        out.cop_valid = false;
        out.cop_x_mm = 0.0f;
        out.cop_y_mm = 0.0f;
        out.cop_ap_norm = 0.0f;
        out.cop_ml_norm = 0.0f;
        out.cop_ap_dot_norm_s = 0.0f;
        out.cop_ml_dot_norm_s = 0.0f;
        out.tau_proxy_nm = 0.0f;
        out.tau_proxy_valid = false;
        out.stance_phase = 0.0f;
        out.gait_state = static_cast<uint8_t>(InsoleGaitState::Swing);
        out.motion_intent = static_cast<uint8_t>(InsoleMotionIntent::Unload);
        out.intent_confidence = 1.0f;
        out.intent_unload_request = true;
        out.fault_flags |= insole_fault_flags::low_pressure | insole_fault_flags::cop_invalid;
        _apply_event_state(out);
        _last_cop_valid = false;
        return;
    }

    const float cop_x_mm = cop_x_num / total_g;
    const float cop_y_mm = cop_y_num / total_g;
    const float cop_ap_norm = clamp_float(cop_ap_num / total_g, 0.0f, 1.0f);
    const float cop_ml_norm = clamp_float(cop_ml_num / total_g, -1.0f, 1.0f);
    const float fz_n = total_g * 0.00980665f;
    float cop_ap_dot_norm_s = 0.0f;
    float cop_ml_dot_norm_s = 0.0f;

    if (_last_cop_valid && _last_cop_timestamp_us != frame_time_us)
    {
        const float dt_s = (float)(frame_time_us - _last_cop_timestamp_us) * 0.000001f;
        if (dt_s > 0.0f)
        {
            cop_ap_dot_norm_s = (cop_ap_norm - _last_cop_ap_norm) / dt_s;
            cop_ml_dot_norm_s = (cop_ml_norm - _last_cop_ml_norm) / dt_s;
        }
    }

    _last_cop_ap_norm = cop_ap_norm;
    _last_cop_ml_norm = cop_ml_norm;
    _last_cop_timestamp_us = frame_time_us;
    _last_cop_valid = true;

    float stance_phase = 0.0f;
    if (_stance_start_us != 0 && _expected_stance_us > 0)
    {
        stance_phase = clamp_float((float)(frame_time_us - _stance_start_us) / (float)_expected_stance_us, 0.0f, 1.0f);
    }

    float intent_confidence = 0.0f;
    const InsoleMotionIntent intent = _classify_intent(cop_ml_norm, intent_confidence);

    out.cop_valid = true;
    out.cop_x_mm = cop_x_mm;
    out.cop_y_mm = cop_y_mm;
    out.cop_ap_norm = cop_ap_norm;
    out.cop_ml_norm = cop_ml_norm;
    out.cop_ap_dot_norm_s = cop_ap_dot_norm_s;
    out.cop_ml_dot_norm_s = cop_ml_dot_norm_s;
    out.stance_phase = stance_phase;
    out.gait_state = static_cast<uint8_t>(_classify_gait(forefoot_g, heel_g, cop_ap_norm, stance_phase));
    out.motion_intent = static_cast<uint8_t>(intent);
    out.intent_confidence = intent_confidence;
    out.intent_unload_request = false;
    out.tau_proxy_nm = _calc_tau_proxy_nm(side_data, fz_n, cop_ap_norm);
    out.tau_proxy_valid = true;
    _apply_event_state(out);
}

InsoleGaitState PressureInsoleHighLevel::FootProcessor::_classify_gait(float forefoot_g, float heel_g, float cop_ap_norm, float stance_phase) const
{
    (void)forefoot_g;
    (void)heel_g;

    if (!_confirmed_contact)
    {
        return InsoleGaitState::Swing;
    }

    if (cop_ap_norm < 0.25f)
    {
        if (stance_phase < 0.12f)
        {
            return InsoleGaitState::InitialContact;
        }
        return InsoleGaitState::LoadingResponse;
    }

    if (cop_ap_norm < 0.40f)
    {
        return InsoleGaitState::LoadingResponse;
    }

    if (cop_ap_norm < 0.65f)
    {
        return InsoleGaitState::MidStance;
    }

    if (stance_phase < 0.85f)
    {
        return InsoleGaitState::TerminalStance;
    }

    return InsoleGaitState::PreSwing;
}

InsoleMotionState PressureInsoleHighLevel::FootProcessor::_classify_motion_state(const DPJMCHighLevelInput& out, uint32_t frame_time_us) const
{
    if (!out.pressure_valid)
    {
        return InsoleMotionState::Unknown;
    }

    if (!_confirmed_contact || out.intent_unload_request)
    {
        return InsoleMotionState::Unload;
    }

    const bool recent_heel_strike = _last_heel_strike_us != 0
        && (uint32_t)(frame_time_us - _last_heel_strike_us) <= insole_high_level::MOTION_STATE_RECENT_EVENT_US;
    const bool recent_toe_off = _last_toe_off_us != 0
        && (uint32_t)(frame_time_us - _last_toe_off_us) <= insole_high_level::MOTION_STATE_RECENT_EVENT_US;
    if (recent_heel_strike && recent_toe_off)
    {
        return InsoleMotionState::Walking;
    }

    return InsoleMotionState::Static;
}

InsoleMotionIntent PressureInsoleHighLevel::FootProcessor::_classify_intent(float cop_ml_norm, float& confidence) const
{
    const float threshold = insole_high_level::ML_DEVIATION_THRESHOLD_NORM;
    const float abs_ml = fabsf(cop_ml_norm);

    if (abs_ml <= threshold)
    {
        confidence = 1.0f - clamp_float(abs_ml / threshold, 0.0f, 1.0f);
        return InsoleMotionIntent::Normal;
    }

    confidence = clamp_float(abs_ml / (threshold * 2.0f), 0.0f, 1.0f);
    if (cop_ml_norm > 0.0f)
    {
        return InsoleMotionIntent::LateralDeviation;
    }

    return InsoleMotionIntent::MedialDeviation;
}

float PressureInsoleHighLevel::FootProcessor::_zone_ml_norm(uint8_t zone_index) const
{
    float ml_norm = (zone_center_x_mm[zone_index] - foot_center_x_mm) / foot_half_width_mm;
    ml_norm = clamp_float(ml_norm, -1.0f, 1.0f);
    if (_is_left)
    {
        ml_norm = -ml_norm;
    }
    return ml_norm;
}

float PressureInsoleHighLevel::FootProcessor::_calc_tau_proxy_nm(const SideData& side_data, float fz_n, float cop_ap_norm) const
{
    float ankle_x_ap_norm = insole_high_level::DEFAULT_ANKLE_X_AP_NORM;
    float tau_proxy_scale = insole_high_level::DEFAULT_TAU_PROXY_SCALE;

    const ControllerData& ankle_controller = side_data.ankle.controller;
    if (ankle_controller.controller == static_cast<uint8_t>(config_defs::ankle_controllers::dpjmc))
    {
        const float param_ankle_x = ankle_controller.parameters[controller_defs::dpjmc::ankle_x_ap_norm_idx];
        const float param_scale = ankle_controller.parameters[controller_defs::dpjmc::tau_proxy_scale_nm_idx];

        if (finite_float(param_ankle_x) && param_ankle_x > 0.0f && param_ankle_x < 1.0f)
        {
            ankle_x_ap_norm = param_ankle_x;
        }

        if (finite_float(param_scale) && param_scale > 0.0f)
        {
            tau_proxy_scale = param_scale;
        }
    }

    const float tau_proxy_nm = (cop_ap_norm - ankle_x_ap_norm) * insole_high_level::FOOT_LENGTH_M * fz_n * tau_proxy_scale;
    return clamp_float(tau_proxy_nm, -insole_high_level::TAU_PROXY_LIMIT_NM, insole_high_level::TAU_PROXY_LIMIT_NM);
}

#endif
