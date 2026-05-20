/**
 * @file SideData.h
 *
 * @brief Declares a class used to store data for side to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef SideData_h
#define SideData_h

#include "Arduino.h"

#include "JointData.h"
#include "ParseIni.h"
#include "Board.h"
#include "InclinationDetector.h"

#include <stdint.h>

//Forward declaration
class ExoData;

namespace insole_defs
{
    static const uint8_t zone_count = 18;
}

enum class InsoleGaitState : uint8_t
{
    Invalid = 0,
    Swing = 1,
    InitialContact = 2,
    LoadingResponse = 3,
    MidStance = 4,
    TerminalStance = 5,
    PreSwing = 6
};

enum class InsoleMotionIntent : uint8_t
{
    Normal = 0,
    MedialDeviation = 1,
    LateralDeviation = 2,
    Unload = 3,
    Unknown = 255
};

enum class InsoleMotionState : uint8_t
{
    Unload = 0,
    Static = 1,
    Walking = 2,
    Unknown = 255
};

namespace insole_fault_flags
{
    static const uint32_t invalid_frame = 1UL << 0;
    static const uint32_t timeout = 1UL << 1;
    static const uint32_t low_pressure = 1UL << 2;
    static const uint32_t saturation = 1UL << 3;
    static const uint32_t cop_invalid = 1UL << 4;
    static const uint32_t zero_refresh_stale = 1UL << 5;
    static const uint32_t bilateral_inconsistency = 1UL << 6;
}

/**
 * @brief Raw 18-zone pressure-insole packet for one side.
 *
 * Units and conventions:
 * - timestamp_us is in microseconds from micros().
 * - zones_g uses grams, positive for compressive normal load.
 * - valid=false marks parser/transport failure before high-level processing.
 */
struct InsoleRawFrame
{
        uint32_t timestamp_us;
        uint16_t zones_g[insole_defs::zone_count];
        bool valid;
        uint32_t fault_flags;

        void reset()
        {
            timestamp_us = 0;
            valid = false;
            fault_flags = 0;
            for (uint8_t i = 0; i < insole_defs::zone_count; i++)
            {
                zones_g[i] = 0;
            }
        }
};

/**
 * @brief DPJMC high-level input interface.
 *
 * Units and conventions:
 * - timestamp_us is in microseconds from micros().
 * - pressure values use grams, positive for compressive normal load.
 * - cop_x_mm / cop_y_mm use the physical insole plane in millimeters.
 * - normalized COP values use foot coordinates: AP 0=heel, 1=toe; ML 0=centerline, +lateral.
 * - COP velocity values are normalized foot coordinates per second.
 * - fz_n is vertical-load proxy in newtons.
 * - tau_proxy_nm is a pressure-only plantarflexion-positive biological moment proxy.
 */
struct DPJMCHighLevelInput
{
        bool pressure_valid;
        bool cop_valid;
        uint32_t timestamp_us;
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
        bool tau_proxy_valid;
        uint8_t gait_state;
        uint8_t motion_state;
        uint8_t motion_intent;
        float intent_confidence;
        bool intent_unload_request;
        bool insole_heel_strike;
        bool insole_toe_off;
        uint32_t insole_heel_strike_timestamp_us;
        uint32_t insole_toe_off_timestamp_us;
        uint32_t fault_flags;

        void reset()
        {
            pressure_valid = false;
            cop_valid = false;
            timestamp_us = 0;
            p_total = 0.0f;
            p_fore = 0.0f;
            p_arch = 0.0f;
            p_heel = 0.0f;
            p_medial = 0.0f;
            p_lateral = 0.0f;
            fz_n = 0.0f;
            cop_x_mm = 0.0f;
            cop_y_mm = 0.0f;
            cop_ap_norm = 0.0f;
            cop_ml_norm = 0.0f;
            cop_ap_dot_norm_s = 0.0f;
            cop_ml_dot_norm_s = 0.0f;
            stance_phase = 0.0f;
            tau_proxy_nm = 0.0f;
            tau_proxy_valid = false;
            gait_state = static_cast<uint8_t>(InsoleGaitState::Invalid);
            motion_state = static_cast<uint8_t>(InsoleMotionState::Unknown);
            motion_intent = static_cast<uint8_t>(InsoleMotionIntent::Unknown);
            intent_confidence = 0.0f;
            intent_unload_request = false;
            insole_heel_strike = false;
            insole_toe_off = false;
            insole_heel_strike_timestamp_us = 0;
            insole_toe_off_timestamp_us = 0;
            fault_flags = 0;
        }
};

/**
 * @brief class to store information related to the side.
 * 
 */
class SideData {
	   
    public:
        SideData(bool is_left, uint8_t* config_to_send);
        
        /**
         * @brief Reconfigures the side data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        JointData hip;      /**< Data for the hip joint */
        JointData knee;     /**< Data for the knee joint */
        JointData ankle;    /**< Data for the ankle joint */
        JointData elbow;    /**< Data for the elbow joint */
        JointData arm_1;    /**< Data for the arm 1 joint */
        JointData arm_2;    /**< Data for the arm 2 joint */
        
        float percent_gait;             /**< Estimate of the percent gait based on heel strike */
        float expected_step_duration;   /**< Estimate of how long the next step will take based on the most recent step times */

        float percent_stance;           /**< Estimate of the percent stance based on heel strike and toe off */
        float expected_stance_duration; /**< Estimate of how long the next stance will take based on the most recent stance times */

        float percent_swing;            /**< Estimate of the percent swing based on toe off and heel strike */
        float expected_swing_duration;  /**< Estimate of how long the next swing will take based on the most recent swing times */
        
        float heel_fsr;                 /**< Calibrated FSR reading for the heel */
        float heel_fsr_upper_threshold; /**< Upper threshold for the heel */
        float heel_fsr_lower_threshold; /**< Lower threshold for the heel */
        float toe_fsr;                  /**< Calibrated FSR reading for the toe */
        float toe_fsr_upper_threshold;  /**< Upper threshold for the toe */
        float toe_fsr_lower_threshold;  /**< Lower threshold for the toe */
        InsoleRawFrame latest_valid_insole_raw; /**< Latest checksum-valid raw 18-zone frame for this side. */
        DPJMCHighLevelInput dpjmc_hl;    /**< Pressure/COP/motion-intent input for DPJMC. */
        
        bool ground_strike;             /**< Trigger when we go from swing to one FSR making contact. */
        bool toe_strike;                /**< Trigger when we detect toe strike after the last detcted toe off */
        bool toe_off;                   /**< Trigger when we go from toe FSR making contact to swing. */
        bool toe_on;                    /**< Trigger when we go from toe FSR not making contact to making contact */
        bool heel_stance;               /**< High when the heel FSR is in ground contact */
        bool toe_stance;                /**< High when the toe FSR is in ground contact */
        bool prev_heel_stance;          /**< High when the heel FSR was in ground contact on the previous iteration */
        bool prev_toe_stance;           /**< High when the toe FSR was in ground contact on the previous iteration */
        
        bool is_left;                               /**< 1 if the side is on the left, 0 otherwise */
        bool is_used;                               /**< 1 if the side is used, 0 otherwise */
        bool do_calibration_toe_fsr;                /**< Flag for if the toe calibration should be done */
        bool do_calibration_refinement_toe_fsr;     /**< Flag for if the toe calibration refinement should be done */
        bool do_calibration_heel_fsr;               /**< Flag for if the heel calibration should be done */
        bool do_calibration_refinement_heel_fsr;    /**< Flag for if the heel calibration refinement should be done */

        float ankle_angle_at_ground_strike;         /**< Estimated angle of the ankle when at ground strike */
        float expected_duration_window_upper_coeff; /**< Factor to multiply by the expected duration to get the upper limit of the window to determine if a ground strike is considered a new step. */
        float expected_duration_window_lower_coeff; /**< Factor to multiply by the expected duration to get the lower limit of the window to determine if a ground strike is considered a new step. */

        Inclination inclination;        /**< Data for inclination */
};

#endif
