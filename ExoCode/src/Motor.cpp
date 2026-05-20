/*
 * P. Stegall Jan. 2022
*/

#include "Arduino.h" 

#include "Motor.h"
#include "CAN.h"
#include "Config.h"
#include "ErrorManager.h"
#include "error_codes.h"
#include "Logger.h"
#include "ErrorReporter.h"
#include "error_codes.h"
#include <string.h>
#include <math.h>
//#define MOTOR_DEBUG           //Uncomment if you want to print debug statments to the serial monitor

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 


_Motor::_Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin)
{
    _id = id;
    _is_left = ((uint8_t)this->_id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    _data = exo_data;
    _enable_pin = enable_pin;
    _prev_motor_enabled = false; 
    _prev_on_state = false;
    
    #ifdef MOTOR_DEBUG
        logger::print("_Motor::_Motor : _enable_pin = ");
        logger::print(_enable_pin);
        logger::print("\n");
    #endif
    
    pinMode(_enable_pin, OUTPUT);
    
    //Set _motor_data to point to the data specific to this motor.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.hip.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.hip.motor);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.knee.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.knee.motor);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.ankle.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.ankle.motor);
            }
            break;
        case (uint8_t)config_defs::joint_id::elbow:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.elbow.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.elbow.motor);
            }
            break;
        case (uint8_t)config_defs::joint_id::arm_1:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.arm_1.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.arm_1.motor);
            }
            break;
        case (uint8_t)config_defs::joint_id::arm_2:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.arm_2.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.arm_2.motor);
            }
            break;
    }

    #ifdef MOTOR_DEBUG
        logger::println("_Motor::_Motor : Leaving Constructor");
    #endif

};

bool _Motor::get_is_left() 
{
    return _is_left;
};

config_defs::joint_id _Motor::get_id()
{
    return _id;
};

/*
 * Constructor for the CAN Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the CAN (e.g. TMotor) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model (e.g. AK60), which may have specific torque constants etc.
 */
_CANMotor::_CANMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin) //Constructor: type is the motor type
: _Motor(id, exo_data, enable_pin)
{
    _KP_MIN = 0.0f;
    _KP_MAX = 500.0f;
    _KD_MIN = 0.0f;
    _KD_MAX = 5.0f;
    _P_MAX = 12.5f;

    JointData* j_data = exo_data->get_joint_with(static_cast<uint8_t>(id));
    j_data->motor.kt = this->get_Kt();

    _enable_response = false;

    #ifdef MOTOR_DEBUG
        logger::println("_CANMotor::_CANMotor : Leaving Constructor");
    #endif
};

void _CANMotor::transaction(float torque)
{
    //Send data and read response 
    send_data(torque);
    read_data();
    check_response();
};

void _CANMotor::read_data()
{
    if (_motor_data->enabled)
    {
        CAN* can = can->getInstance();
        int direction_modifier = _motor_data->flip_direction ? -1 : 1;

        CAN_message_t msg = can->read();

        // Determine if the motor type is AK60v3 (extended, new format) or old AK (standard, old format)
		// Background: AK60v3 motors employ a different communication protocol and are handled differently in this context.
        bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);
    
        // When the motor type is AK60v3
        if (is_ak60v3) {
            if (msg.len == 0 || !msg.flags.extended) {
                return;
            }
            // AK60v3: NEW message format
            if ((msg.id & 0xFF) == uint8_t(_motor_data->id))
            {
                uint32_t p_int = (msg.buf[0] << 8) | msg.buf[1];
                uint32_t v_int = (msg.buf[2] << 8) | msg.buf[3];
                uint32_t i_int = (msg.buf[4] << 8) | msg.buf[5];
                _motor_data->p = direction_modifier * _uint_to_float(p_int, -_P_MAX, _P_MAX, 16);
                _motor_data->v = direction_modifier * _uint_to_float(v_int, -_V_MAX, _V_MAX, 12);
                _motor_data->i = direction_modifier * _uint_to_float(i_int, -_I_MAX, _I_MAX, 12);
                #ifdef MOTOR_DEBUG
                    logger::print("_CANMotor::read_data():Got data-");
                    logger::print("ID:" + String(uint32_t(_motor_data->id)) + ",");
                    logger::print("P:"+String(_motor_data->p) + ",V:" + String(_motor_data->v) + ",I:" + String(_motor_data->i));
                    logger::print("\n");
                #endif
                _motor_data->timeout_count = 0;
            }
        }
		//When the motor type is a CAN motor other than the AK60v3.
		else {
            if (msg.len == 0 || msg.flags.extended) {
                return;
            }
            // Old AK: OLD message format
            if (msg.buf[0] == uint32_t(_motor_data->id))
            {
                uint32_t p_int = (msg.buf[1] << 8) | msg.buf[2];
                uint32_t v_int = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
                uint32_t i_int = ((msg.buf[4] & 0xF) << 8) | msg.buf[5];
                _motor_data->p = direction_modifier * _uint_to_float(p_int, -_P_MAX, _P_MAX, 16);
                _motor_data->v = direction_modifier * _uint_to_float(v_int, -_V_MAX, _V_MAX, 12);
                _motor_data->i = direction_modifier * _uint_to_float(i_int, -_I_MAX, _I_MAX, 12);
                #ifdef MOTOR_DEBUG
                    logger::print("_CANMotor::read_data():Got data-");
                    logger::print("ID:" + String(uint32_t(_motor_data->id)) + ",");
                    logger::print("P:"+String(_motor_data->p) + ",V:" + String(_motor_data->v) + ",I:" + String(_motor_data->i));
                    logger::print("\n");
                #endif
                _motor_data->timeout_count = 0;
            }
        }
    }
    return;
};

void _CANMotor::send_data(float torque)
{
    #ifdef MOTOR_DEBUG
        logger::print("Sending data: ");
        logger::print(uint32_t(_motor_data->id));
        logger::print("\n");
    #endif

    int direction_modifier = _motor_data->flip_direction ? -1 : 1;

    _motor_data->t_ff = torque;
    const float current = torque / get_Kt();

    float p_sat = constrain(direction_modifier * _motor_data->p_des, -_P_MAX, _P_MAX);
    float v_sat = constrain(direction_modifier * _motor_data->v_des, -_V_MAX, _V_MAX);
    float kp_sat = constrain(_motor_data->kp, _KP_MIN, _KP_MAX);
    float kd_sat = constrain(_motor_data->kd, _KD_MIN, _KD_MAX);
    float i_sat = constrain(direction_modifier * current, -_I_MAX, _I_MAX);
    _motor_data->last_command = i_sat;
    uint32_t p_int = _float_to_uint(p_sat, -_P_MAX, _P_MAX, 16);
    uint32_t v_int = _float_to_uint(v_sat, -_V_MAX, _V_MAX, 12);
    uint32_t kp_int = _float_to_uint(kp_sat, _KP_MIN, _KP_MAX, 12);
    uint32_t kd_int = _float_to_uint(kd_sat, _KD_MIN, _KD_MAX, 12);
    uint32_t i_int = _float_to_uint(i_sat, -_I_MAX, _I_MAX, 12);

    CAN_message_t msg;
    
    // Determine if this is an AK60v3 (extended, new format) or old AK (standard, old format)
    bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

    if (is_ak60v3) {
        // AK60v3: Extended CAN, NEW format
        msg.flags.extended = 1;
        msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;
        // NEW format
        msg.buf[0] = kp_int >> 4;
        msg.buf[1] = ((kp_int&0xF) << 4) | (kd_int >> 8);
        msg.buf[2] = (kd_int & 0xFF);
        msg.buf[3] = p_int >> 8;
        msg.buf[4] = p_int & 0xFF;
        msg.buf[5] = v_int >> 4;
        msg.buf[6] = ((v_int & 0xF) << 4) | (i_int >> 8);
        msg.buf[7] = i_int & 0xFF;
    } else {
        // Old AK: Standard CAN, OLD format
        msg.flags.extended = 0;
        msg.id = (uint32_t)_motor_data->id;
        // OLD format
        msg.buf[0] = p_int >> 8;
        msg.buf[1] = p_int & 0xFF;
        msg.buf[2] = v_int >> 4;
        msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
        msg.buf[4] = kp_int & 0xFF;
        msg.buf[5] = kd_int >> 4;
        msg.buf[6] = ((kd_int & 0xF) << 4) | (i_int >> 8);
        msg.buf[7] = i_int & 0xFF;
    }
    logger::print("_CANMotor::send_data::t_sat:: ");
    logger::print(torque);
    logger::print("\n");

    CAN* can = can->getInstance();

    if (_motor_data->enabled)
    {
        //Set data in motor
        can->send(msg);
        _prev_motor_enabled = true;
    }
    else if (_prev_motor_enabled)
    {
        // Motor was just disabled - send one final zero-torque command
        // This is critical for AK60v3 which will otherwise hold the last command
        uint32_t zero_p_int = _float_to_uint(0, -_P_MAX, _P_MAX, 16);
        uint32_t zero_v_int = _float_to_uint(0, -_V_MAX, _V_MAX, 12);
        uint32_t zero_kp_int = _float_to_uint(0, _KP_MIN, _KP_MAX, 12);
        uint32_t zero_kd_int = _float_to_uint(0, _KD_MIN, _KD_MAX, 12);
        uint32_t zero_i_int = _float_to_uint(0, -_I_MAX, _I_MAX, 12);

        if (is_ak60v3) {
            msg.buf[0] = zero_kp_int >> 4;
            msg.buf[1] = ((zero_kp_int&0xF) << 4) | (zero_kd_int >> 8);
            msg.buf[2] = (zero_kd_int & 0xFF);
            msg.buf[3] = zero_p_int >> 8;
            msg.buf[4] = zero_p_int & 0xFF;
            msg.buf[5] = zero_v_int >> 4;
            msg.buf[6] = ((zero_v_int & 0xF) << 4) | (zero_i_int >> 8);
            msg.buf[7] = zero_i_int & 0xFF;
        } else {
            msg.buf[0] = zero_p_int >> 8;
            msg.buf[1] = zero_p_int & 0xFF;
            msg.buf[2] = zero_v_int >> 4;
            msg.buf[3] = ((zero_v_int & 0xF) << 4) | (zero_kp_int >> 8);
            msg.buf[4] = zero_kp_int & 0xFF;
            msg.buf[5] = zero_kd_int >> 4;
            msg.buf[6] = ((zero_kd_int & 0xF) << 4) | (zero_i_int >> 8);
            msg.buf[7] = zero_i_int & 0xFF;
        }

        can->send(msg);
        _prev_motor_enabled = false;
    }
    return;
};

void _CANMotor::check_response()
{
    //Only run if the motor is supposed to be enabled
    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) ||
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);

    if (_data->user_paused || !active_trial || _data->estop || _error)
    {
        return;
    }

    //Measured current variance should be non-zero
    _measured_current.push(_motor_data->i);

    if (_measured_current.size() > _current_queue_size)
    {
        _measured_current.pop();
        auto pop_vals = utils::online_std_dev(_measured_current);

        // Only attempt to re-enable if the motor is currently disabled
        // Low variance during constant torque is normal and shouldn't trigger re-enable
        if (pop_vals.second < _variance_threshold && !_motor_data->enabled)
        {
            _motor_data->enabled = true;
            enable(true);
        }

    }
};

void _CANMotor::on_off()
{
    if (_data->estop || _error)
    {
        _motor_data->is_on = false;

        // logger::print("_CANMotor::on_off(bool is_on) : E-stop pulled - ");
        // logger::print(uint32_t(_motor_data->id));
        // logger::print("\n");
    }

    if (_prev_on_state != _motor_data->is_on) //If was here to save time, can be removed if making problems, or add overide
    {
        if (_motor_data->is_on)
        {
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_on_state);

            // logger::print("_CANMotor::on_off(bool is_on) : Power on- ");
            // logger::print(uint32_t(_motor_data->id));
            // logger::print("\n");
        }
        else 
        {
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_off_state);

            // logger::print("_CANMotor::on_off(bool is_on) : Power off- ");
            // logger::print(uint32_t(_motor_data->id));
            // logger::print("\n");
        }
    }
    _prev_on_state = _motor_data->is_on;

    #ifdef HEADLESS
        delay(2000);    //Two second delay between motor's turning on and enabeling, we've run into some issues with enabling while in headless mode if this delay is not present. 
    #endif

};

bool _CANMotor::enable()
{
    return enable(false);
};

bool _CANMotor::enable(bool overide)
{
    #ifdef MOTOR_DEBUG
        //  logger::print(_prev_motor_enabled);
        //  logger::print("\t");
        //  logger::print(_motor_data->enabled);
        //  logger::print("\t");
        //  logger::print(_motor_data->is_on);
        //  logger::print("\n");
    #endif

    //Only change the state and send messages if the enabled state has changed.
    if ((_prev_motor_enabled != _motor_data->enabled) || overide || !_enable_response)
    {
        CAN_message_t msg;

        // Determine if this is an AK60v3 (extended format) or old AK (standard format)
        bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

        // Initialize message format fields
        msg.len = 8;
        if (is_ak60v3) {
            msg.flags.extended = 1;
            msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;
        } else {
            msg.flags.extended = 0;
            msg.id = (uint32_t)_motor_data->id;
        }

        msg.buf[0] = 0xFF;
        msg.buf[1] = 0xFF;
        msg.buf[2] = 0xFF;
        msg.buf[3] = 0xFF;
        msg.buf[4] = 0xFF;
        msg.buf[5] = 0xFF;
        msg.buf[6] = 0xFF;

        if (_motor_data->enabled && !_error && !_data->estop)
        {
            msg.buf[7] = 0xFC;
        }
        else
        {
            _enable_response = false;
            msg.buf[7] = 0xFD;
        }

        CAN* can = can->getInstance();
        can->send(msg);
        delayMicroseconds(500);
        read_data();

        if (_motor_data->timeout_count == 0)
        {
            _enable_response = true;
        }
    }

    _prev_motor_enabled = _motor_data->enabled;
    return _enable_response;
};

void _CANMotor::zero()
{
    CAN_message_t msg;

    // Determine if this is an AK60v3 (extended format) or old AK (standard format)
    bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

    // Initialize message format fields
    msg.len = 8;
    if (is_ak60v3) {
        msg.flags.extended = 1;
        msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;
    } else {
        msg.flags.extended = 0;
        msg.id = uint32_t(_motor_data->id);
    }

    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFE;
    CAN* can = can->getInstance();
    can->send(msg);

    read_data();
};

float _CANMotor::get_Kt()
{
    return _Kt;
};

void _CANMotor::set_error()
{
    _error = true;
};

void _CANMotor::set_Kt(float Kt)
{
    _Kt = Kt;
};

void _CANMotor::_handle_read_failure()
{
    // Commented out for AK60v3 integration. 
    //logger::println("CAN Motor - Handle Read Failure", LogLevel::Error);
    //_motor_data->timeout_count++;
};

float _CANMotor::_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if (bits == 12) {
      pgg = (unsigned int) ((x-offset)*4095.0/span); 
    }
    if (bits == 16) {
      pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
};

float _CANMotor::_uint_to_float(unsigned int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float pgg = 0;
    if (bits == 12) {
      pgg = ((float)x_int)*span/4095.0 + offset;
    }
    if (bits == 16) {
      pgg = ((float)x_int)*span/65535.0 + offset;
    }
    return pgg;
};

//**************************************
/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60::AK60(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 22.0f;
    _V_MAX = 41.87f;
    
    float kt = 0.068 * 6;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK60::AK60 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60v1_1::AK60v1_1(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 13.5f;
    _V_MAX = 23.04f;

    float kt = 0.1725 * 6; //We set KT to 0.1725 * 6 whcih differs from the manufacturer's stated KT, that's because they are wrong (This has been validated mulitple ways). We only have validated for this version as we use open loop at the hip with these, other motors are used with closed loop and thus are corrected in real-time. We recommend validating these KTs if using for open loop. 
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK60v1_1::AK60v1_1 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK80::AK80(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 24.0f;
    _V_MAX = 25.65f;

    float kt = 0.091 * 9;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK80::AK80 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK70::AK70(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 23.2f;
    _V_MAX = 15.5f;
    
    float kt = 0.13 * 10;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK70::AK70 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60v3::AK60v3(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 10.3f;
    _V_MAX = 48.0f;

    float kt = 0.420*6;//corrected values
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK60v3::AK60v3 : Leaving Constructor");
#endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK45_36::AK45_36(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 6.5f;
    _V_MAX = 5.44f;

    float kt = 0.127f;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK45_36::AK45_36 : Leaving Constructor");
#endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK45_10::AK45_10(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 6.5f;
    _V_MAX = 18.85f;

    float kt = 0.127f;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK45_10::AK45_10 : Leaving Constructor");
#endif
};


namespace pda_cmd
{
    const uint8_t SYSTEM = 0x08;
    const uint8_t ADAPTIVE_POSITION = 0x0B;
    const uint8_t PRESET = 0x0C;
    const uint8_t MOTION_AID = 0x0D;
    const uint8_t SET_POSITION_TRACK = 0x19;
    const uint8_t SET_POSITION_TRAPEZOID = 0x1A;
    const uint8_t SET_POSITION_FF = 0x1B;
    const uint8_t SET_SPEED = 0x1C;
    const uint8_t SET_TORQUE = 0x1D;
    const uint8_t READ_PROPERTY = 0x1E;
    const uint8_t WRITE_PROPERTY = 0x1F;

    const uint16_t INPUT_MODE_PASSTHROUGH = 1;
    const uint16_t INPUT_MODE_VEL_RAMP = 2;
    const uint16_t INPUT_MODE_TORQUE_RAMP = 6;
}

static const PdaModelSpec PDA08_SPEC = {
    pda_config::PDA08_RATED_TORQUE_NM,
    pda_config::PDA08_PEAK_TORQUE_NM,
    pda_config::PDA08_RATED_SPEED_RPM,
    pda_config::PDA08_RATED_CURRENT_A,
    pda_config::PDA08_STALL_CURRENT_A,
    pda_config::PDA08_ROTOR_INERTIA_GCM2
};

static const PdaModelSpec PDA01_SPEC = {
    pda_config::PDA01_RATED_TORQUE_NM,
    pda_config::PDA01_PEAK_TORQUE_NM,
    pda_config::PDA01_RATED_SPEED_RPM,
    pda_config::PDA01_RATED_CURRENT_A,
    pda_config::PDA01_STALL_CURRENT_A,
    pda_config::PDA01_ROTOR_INERTIA_GCM2
};

PdaMotor::PdaMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin, const PdaModelSpec& spec)
: _Motor(id, exo_data, enable_pin)
, _spec(spec)
{
    _pda_id = _motor_data->pda_id;
    _enable_response = false;
    _feedback_configured = false;
    _detect_scan_id = 1;
    _last_detect_probe_us = 0;
    _last_feedback_config_us = 0;
    _last_pda_debug_tx_us = 0;
    _last_pda_debug_rx_us = 0;
    _last_pda_debug_status_us = 0;

    _Kt = 1.0f;
    _motor_data->kt = _Kt;
    _motor_data->pda_id = _pda_id;
    _motor_data->pda_rated_torque_nm = _spec.rated_torque_nm;
    _motor_data->pda_peak_torque_nm = _spec.peak_torque_nm;
    _motor_data->pda_rated_speed_rpm = _spec.rated_speed_rpm;
    _motor_data->pda_rated_current_a = _spec.rated_current_a;
    _motor_data->pda_stall_current_a = _spec.stall_current_a;
    _motor_data->pda_rotor_inertia_gcm2 = _spec.rotor_inertia_gcm2;

    if (_motor_data->pda_torque_limit_nm <= 0.0f || _motor_data->pda_torque_limit_nm > _spec.rated_torque_nm)
    {
        _motor_data->pda_torque_limit_nm = _spec.rated_torque_nm;
    }

    if (_motor_data->pda_speed_limit_rpm <= 0.0f || _motor_data->pda_speed_limit_rpm > _spec.rated_speed_rpm)
    {
        _motor_data->pda_speed_limit_rpm = _spec.rated_speed_rpm;
    }

    const bool can_autodetect_invalid_id = pda_config::AUTO_DETECT_CAN_ID && (_pda_id == 0);
    if ((!_has_valid_pda_id() && !can_autodetect_invalid_id) ||
        (_has_valid_pda_id() && !_pda_id_is_unique()))
    {
        _error = true;
        _motor_data->enabled = false;
        _enable_response = false;
    }

    #ifdef MOTOR_DEBUG
        logger::print("PdaMotor::PdaMotor : PDA id ");
        logger::println(_pda_id);
    #endif
}

uint16_t PdaMotor::_frame_id(uint8_t cmd_id) const
{
    return _frame_id_for(_pda_id, cmd_id);
}

uint16_t PdaMotor::_frame_id_for(uint8_t pda_id, uint8_t cmd_id) const
{
    return ((uint16_t)pda_id << 5) + cmd_id;
}

bool PdaMotor::_has_valid_pda_id() const
{
    return (_pda_id >= 1) && (_pda_id <= 63);
}

bool PdaMotor::_is_valid_pda_command_id(uint8_t pda_id) const
{
    return pda_id <= 63;
}

bool PdaMotor::_is_pda_motor_type(uint8_t motor_type) const
{
    return (motor_type == (uint8_t)config_defs::motor::PDA08) ||
           (motor_type == (uint8_t)config_defs::motor::PDA01);
}

bool PdaMotor::_pda_id_is_unique() const
{
    JointData* joints[] = {
        &_data->left_side.hip,
        &_data->left_side.knee,
        &_data->left_side.ankle,
        &_data->left_side.elbow,
        &_data->left_side.arm_1,
        &_data->left_side.arm_2,
        &_data->right_side.hip,
        &_data->right_side.knee,
        &_data->right_side.ankle,
        &_data->right_side.elbow,
        &_data->right_side.arm_1,
        &_data->right_side.arm_2
    };

    for (uint8_t idx = 0; idx < 12; idx++)
    {
        if ((&joints[idx]->motor != _motor_data) &&
            joints[idx]->is_used &&
            _is_pda_motor_type(joints[idx]->motor.motor_type) &&
            (joints[idx]->motor.pda_id == _pda_id))
        {
            return false;
        }
    }
    return true;
}

bool PdaMotor::_can_autodetect_this_motor() const
{
    uint8_t pda_motor_count = 0;
    JointData* joints[] = {
        &_data->left_side.hip,
        &_data->left_side.knee,
        &_data->left_side.ankle,
        &_data->left_side.elbow,
        &_data->left_side.arm_1,
        &_data->left_side.arm_2,
        &_data->right_side.hip,
        &_data->right_side.knee,
        &_data->right_side.ankle,
        &_data->right_side.elbow,
        &_data->right_side.arm_1,
        &_data->right_side.arm_2
    };

    for (uint8_t idx = 0; idx < 12; idx++)
    {
        if (joints[idx]->is_used && _is_pda_motor_type(joints[idx]->motor.motor_type))
        {
            pda_motor_count++;
        }
    }

    return pda_motor_count == 1;
}

void PdaMotor::_encode_float32_le(float val, uint8_t* data)
{
    memcpy(data, &val, sizeof(float));
}

void PdaMotor::_encode_uint32_le(uint32_t val, uint8_t* data)
{
    data[0] = (uint8_t)(val);
    data[1] = (uint8_t)(val >> 8);
    data[2] = (uint8_t)(val >> 16);
    data[3] = (uint8_t)(val >> 24);
}

void PdaMotor::_encode_uint16_le(uint16_t val, uint8_t* data)
{
    data[0] = (uint8_t)(val);
    data[1] = (uint8_t)(val >> 8);
}

void PdaMotor::_encode_int16_le(int16_t val, uint8_t* data)
{
    data[0] = (uint8_t)(val);
    data[1] = (uint8_t)(val >> 8);
}

float PdaMotor::_decode_float32_le(const uint8_t* data)
{
    float val = 0.0f;
    memcpy(&val, data, sizeof(float));
    return val;
}

int16_t PdaMotor::_decode_int16_le(const uint8_t* data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

int16_t PdaMotor::_scale_to_int16(float value, float scale) const
{
    int32_t scaled = (int32_t)(value / scale);
    if (scaled > 32767)
    {
        return 32767;
    }
    if (scaled < -32768)
    {
        return -32768;
    }
    return (int16_t)scaled;
}

uint16_t PdaMotor::_scale_to_uint16(float value, float scale) const
{
    int32_t scaled = (int32_t)(value / scale);
    if (scaled > 65535)
    {
        return 65535;
    }
    if (scaled < 0)
    {
        return 0;
    }
    return (uint16_t)scaled;
}

float PdaMotor::_apply_torque_limit(float torque_nm) const
{
    return constrain(torque_nm, -_motor_data->pda_torque_limit_nm, _motor_data->pda_torque_limit_nm);
}

float PdaMotor::_apply_speed_limit(float speed_rpm) const
{
    return constrain(speed_rpm, -_motor_data->pda_speed_limit_rpm, _motor_data->pda_speed_limit_rpm);
}

void PdaMotor::transaction(float torque)
{
    read_data();
    _run_can_id_autodetect();
    _handle_timeout();
    send_data(torque);
}

void PdaMotor::read_data()
{
    CAN* can = CAN::getInstance();
    CAN_message_t msg;

    uint8_t frames_read = 0;
    while ((frames_read < 8) && can->read_matching(msg, PdaMotor::_is_pda_can_frame, this))
    {
        const bool decoded_feedback = _decode_any_pda_feedback(msg);
        _debug_print_rx(msg, decoded_feedback);
        frames_read++;
    }
}

bool PdaMotor::_is_pda_can_frame(const CAN_message_t& msg, void* context)
{
    PdaMotor* self = static_cast<PdaMotor*>(context);
    if (self == NULL || msg.flags.extended || msg.len < 8)
    {
        return false;
    }

    const uint8_t pda_id = (uint8_t)((msg.id & 0x07E0) >> 5);
    if (pda_id < 1 || pda_id > 63)
    {
        return false;
    }

    if (self->_get_pda_motor_data_by_pda_id(pda_id) != NULL)
    {
        return true;
    }

    return pda_config::AUTO_DETECT_CAN_ID && self->_can_autodetect_this_motor();
}

bool PdaMotor::_is_feedback_frame(const CAN_message_t& msg) const
{
    if (msg.flags.extended || msg.len < 8)
    {
        return false;
    }

    const uint8_t pda_id = (uint8_t)((msg.id & 0x07E0) >> 5);
    return pda_id == _pda_id;
}

void PdaMotor::_decode_feedback(const CAN_message_t& msg)
{
    _decode_feedback(_motor_data, msg);
}

void PdaMotor::_decode_feedback(MotorData* motor_data, const CAN_message_t& msg)
{
    if (motor_data == NULL)
    {
        return;
    }

    const int direction_modifier = motor_data->flip_direction ? -1 : 1;
    const float angle_deg = direction_modifier * _decode_float32_le(&msg.buf[0]);
    const float speed_rpm = direction_modifier * (_decode_int16_le(&msg.buf[4]) * 0.01f);
    const float torque_nm = direction_modifier * (_decode_int16_le(&msg.buf[6]) * 0.01f);

    motor_data->pda_angle_deg = angle_deg;
    motor_data->pda_speed_rpm = speed_rpm;
    motor_data->pda_torque_nm = torque_nm;

    motor_data->p = angle_deg * PI / 180.0f;
    motor_data->v = speed_rpm * 2.0f * PI / 60.0f;
    motor_data->i = torque_nm;

    motor_data->pda_last_feedback_us = micros();
    motor_data->pda_feedback_valid = true;
    motor_data->timeout_count = 0;
}

bool PdaMotor::_decode_any_pda_feedback(const CAN_message_t& msg)
{
    if (msg.flags.extended || msg.len < 8)
    {
        return false;
    }

    const uint8_t pda_id = (uint8_t)((msg.id & 0x07E0) >> 5);
    const uint8_t cmd_id = (uint8_t)(msg.id & 0x001F);
    const uint16_t property_address = (uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8);
    const uint16_t property_type = (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
    const bool is_known_property_response =
        (cmd_id == pda_cmd::READ_PROPERTY) &&
        (property_type <= 3) &&
        ((property_address == 22001) ||
         (property_address == 30003) ||
         (property_address == 31001) ||
         (property_address == 31002));
    const bool is_property_frame = (cmd_id == pda_cmd::WRITE_PROPERTY) || is_known_property_response;
    MotorData* target_motor_data = _get_pda_motor_data_by_pda_id(pda_id);
    if (target_motor_data == NULL)
    {
        if (pda_config::AUTO_DETECT_CAN_ID &&
            _can_autodetect_this_motor() &&
            _motor_data->enabled &&
            _motor_data->is_on &&
            !_data->estop &&
            !_error &&
            (pda_id >= 1) &&
            (pda_id <= 63))
        {
            _adopt_pda_id(pda_id);

            if (is_property_frame)
            {
                return true;
            }

            _decode_feedback(_motor_data, msg);
            return true;
        }

        return false;
    }

    if (target_motor_data == _motor_data && _pda_id != pda_id)
    {
        _adopt_pda_id(pda_id);
    }

    if (!is_property_frame)
    {
        _decode_feedback(target_motor_data, msg);
    }
    return true;
}

MotorData* PdaMotor::_get_pda_motor_data_by_pda_id(uint8_t pda_id) const
{
    JointData* joints[] = {
        &_data->left_side.hip,
        &_data->left_side.knee,
        &_data->left_side.ankle,
        &_data->left_side.elbow,
        &_data->left_side.arm_1,
        &_data->left_side.arm_2,
        &_data->right_side.hip,
        &_data->right_side.knee,
        &_data->right_side.ankle,
        &_data->right_side.elbow,
        &_data->right_side.arm_1,
        &_data->right_side.arm_2
    };

    for (uint8_t idx = 0; idx < 12; idx++)
    {
        if (joints[idx]->is_used &&
            _is_pda_motor_type(joints[idx]->motor.motor_type) &&
            (joints[idx]->motor.pda_id == pda_id))
        {
            return &joints[idx]->motor;
        }
    }
    return NULL;
}

void PdaMotor::send_data(float torque)
{
    send_torque_direct_nm(torque);
}

bool PdaMotor::_can_send_motion_command()
{
    if (!_motor_data->enabled ||
        !_motor_data->is_on ||
        _data->estop ||
        _error ||
        !_motor_data->pda_feedback_valid ||
        (_motor_data->timeout_count > 0))
    {
        _send_zero_torque();
        return false;
    }
    return true;
}

void PdaMotor::_send_command(uint8_t cmd_id, const uint8_t* data)
{
    _send_command_to_id(_pda_id, cmd_id, data);
}

void PdaMotor::_send_command_to_id(uint8_t pda_id, uint8_t cmd_id, const uint8_t* data)
{
    if (!_is_valid_pda_command_id(pda_id))
    {
        return;
    }

    CAN_message_t msg;
    msg.flags.extended = 0;
    msg.id = _frame_id_for(pda_id, cmd_id);
    msg.len = 8;

    for (uint8_t idx = 0; idx < 8; idx++)
    {
        msg.buf[idx] = data[idx];
    }

    CAN* can = CAN::getInstance();
    can->send(msg);
    _debug_print_tx(pda_id, cmd_id, data, "TX");
}

void PdaMotor::_send_torque_command(float torque_nm, uint16_t input_mode, int16_t ramp_rate)
{
    _send_torque_command_to_id(_pda_id, torque_nm, input_mode, ramp_rate);
}

void PdaMotor::_send_torque_command_to_id(uint8_t pda_id, float torque_nm, uint16_t input_mode, int16_t ramp_rate)
{
    uint8_t data[8];
    _encode_float32_le(torque_nm, &data[0]);
    _encode_int16_le(ramp_rate, &data[4]);
    _encode_uint16_le(input_mode, &data[6]);
    _send_command_to_id(pda_id, pda_cmd::SET_TORQUE, data);
}

void PdaMotor::_send_zero_torque()
{
    _send_torque_command(0.0f, pda_cmd::INPUT_MODE_PASSTHROUGH, 0);
}

bool PdaMotor::send_torque_direct_nm(float torque_nm)
{
    const int direction_modifier = _motor_data->flip_direction ? -1 : 1;
    const float motor_torque_nm = _apply_torque_limit(direction_modifier * torque_nm);
    _motor_data->t_ff = motor_torque_nm;
    _motor_data->last_command = motor_torque_nm;

    if (!_can_send_motion_command())
    {
        return false;
    }

    _send_torque_command(motor_torque_nm, pda_cmd::INPUT_MODE_PASSTHROUGH, 0);
    return true;
}

bool PdaMotor::send_torque_ramp_nm_s(float torque_nm, float ramp_nm_s)
{
    const int direction_modifier = _motor_data->flip_direction ? -1 : 1;
    const float motor_torque_nm = _apply_torque_limit(direction_modifier * torque_nm);
    const int16_t ramp_rate = _scale_to_int16(fabs(ramp_nm_s), 0.01f);
    _motor_data->t_ff = motor_torque_nm;
    _motor_data->last_command = motor_torque_nm;

    if (!_can_send_motion_command())
    {
        return false;
    }

    _send_torque_command(motor_torque_nm, pda_cmd::INPUT_MODE_TORQUE_RAMP, ramp_rate);
    return true;
}

bool PdaMotor::send_speed_rpm(float speed_rpm, float param, uint8_t mode)
{
    const int direction_modifier = _motor_data->flip_direction ? -1 : 1;
    const float motor_speed_rpm = _apply_speed_limit(direction_modifier * speed_rpm);
    uint8_t data[8];
    _encode_float32_le(motor_speed_rpm, &data[0]);

    if (mode == 0)
    {
        const float torque_ff_nm = _apply_torque_limit(direction_modifier * param);
        _encode_int16_le(_scale_to_int16(torque_ff_nm, 0.01f), &data[4]);
        _encode_uint16_le(pda_cmd::INPUT_MODE_PASSTHROUGH, &data[6]);
    }
    else
    {
        _encode_int16_le(_scale_to_int16(fabs(param), 0.01f), &data[4]);
        _encode_uint16_le(pda_cmd::INPUT_MODE_VEL_RAMP, &data[6]);
    }

    if (!_can_send_motion_command())
    {
        return false;
    }

    _send_command(pda_cmd::SET_SPEED, data);
    return true;
}

bool PdaMotor::send_position_deg(float angle_deg, float speed_rpm, float param, uint8_t mode)
{
    const int direction_modifier = _motor_data->flip_direction ? -1 : 1;
    const float motor_angle_deg = direction_modifier * angle_deg;
    uint8_t data[8];
    _encode_float32_le(motor_angle_deg, &data[0]);

    if (mode == 0)
    {
        const float speed_limit_rpm = fabs(_apply_speed_limit(speed_rpm));
        const float width_hz = constrain(fabs(param), 0.0f, 300.0f);
        _encode_int16_le(_scale_to_int16(speed_limit_rpm, 0.01f), &data[4]);
        _encode_int16_le(_scale_to_int16(width_hz, 0.01f), &data[6]);
        if (!_can_send_motion_command()) { return false; }
        _send_command(pda_cmd::SET_POSITION_TRACK, data);
        return true;
    }

    if (mode == 1)
    {
        const float speed_limit_rpm = fabs(_apply_speed_limit(speed_rpm));
        const float accel_rpm_s = fabs(param);
        if (speed_limit_rpm <= 0.0f || accel_rpm_s <= 0.0f)
        {
            return false;
        }
        _encode_int16_le(_scale_to_int16(speed_limit_rpm, 0.01f), &data[4]);
        _encode_int16_le(_scale_to_int16(accel_rpm_s, 0.01f), &data[6]);
        if (!_can_send_motion_command()) { return false; }
        _send_command(pda_cmd::SET_POSITION_TRAPEZOID, data);
        return true;
    }

    const float motor_speed_ff_rpm = _apply_speed_limit(direction_modifier * speed_rpm);
    const float motor_torque_ff_nm = _apply_torque_limit(direction_modifier * param);
    _encode_int16_le(_scale_to_int16(motor_speed_ff_rpm, 0.01f), &data[4]);
    _encode_int16_le(_scale_to_int16(motor_torque_ff_nm, 0.01f), &data[6]);
    if (!_can_send_motion_command()) { return false; }
    _send_command(pda_cmd::SET_POSITION_FF, data);
    return true;
}

bool PdaMotor::send_adaptive_position_deg(float angle_deg, float speed_rpm, float torque_limit_nm)
{
    const int direction_modifier = _motor_data->flip_direction ? -1 : 1;
    const float motor_angle_deg = direction_modifier * angle_deg;
    const float speed_limit_rpm = fabs(_apply_speed_limit(speed_rpm));
    const float torque_limit = fabs(_apply_torque_limit(torque_limit_nm));

    uint8_t data[8];
    _encode_float32_le(motor_angle_deg, &data[0]);
    _encode_int16_le(_scale_to_int16(speed_limit_rpm, 0.01f), &data[4]);
    _encode_int16_le(_scale_to_int16(torque_limit, 0.01f), &data[6]);

    if (!_can_send_motion_command())
    {
        return false;
    }

    _send_command(pda_cmd::ADAPTIVE_POSITION, data);
    return true;
}

bool PdaMotor::send_impedance_deg_rpm_nm(float angle_deg, float speed_rpm, float tff_nm, float kp_nm_per_deg, float kd_nm_per_rpm, uint8_t mode)
{
    float kp = fabs(kp_nm_per_deg);
    float kd = fabs(kd_nm_per_rpm);
    kp = constrain(kp, 0.0f, 20.0f);
    kd = constrain(kd, 0.0f, 20.0f);

    float angle_set_deg = angle_deg;
    if (mode == 1)
    {
        if (kp <= 0.0f)
        {
            return false;
        }
        angle_set_deg = (-kd * speed_rpm - tff_nm) / kp + angle_deg;
    }

    if (!send_position_deg(angle_set_deg, speed_rpm, tff_nm, 2))
    {
        return false;
    }

    uint8_t data[8];
    _encode_uint32_le(0x15, &data[0]);
    _encode_int16_le(_scale_to_int16(kp, 0.001f), &data[4]);
    _encode_int16_le(_scale_to_int16(kd, 0.001f), &data[6]);
    _send_command(pda_cmd::SYSTEM, data);
    return true;
}

bool PdaMotor::send_motion_aid(float angle_deg, float speed_rpm, float angle_err_deg, float speed_err_rpm, float torque_nm)
{
    if (angle_deg < -300.0f || angle_deg > 300.0f)
    {
        return false;
    }

    const int direction_modifier = _motor_data->flip_direction ? -1 : 1;
    const float motor_angle_deg = direction_modifier * angle_deg;
    const float motor_torque_nm = _apply_torque_limit(direction_modifier * torque_nm);
    const float speed_limit_rpm = fabs(_apply_speed_limit(speed_rpm));
    const float angle_err = fabs(angle_err_deg);
    const float speed_err = fabs(speed_err_rpm);

    uint8_t data[8];
    _encode_int16_le(_scale_to_int16(motor_angle_deg, 0.01f), &data[0]);
    _encode_uint16_le(_scale_to_uint16(angle_err, 0.01f), &data[2]);
    _encode_uint16_le(_scale_to_uint16(speed_err, 0.01f), &data[4]);
    _encode_int16_le(_scale_to_int16(motor_torque_nm, 0.01f), &data[6]);

    if (!_can_send_motion_command())
    {
        return false;
    }

    _send_command(pda_cmd::MOTION_AID, data);

    uint8_t preset_data[8];
    _encode_float32_le(speed_limit_rpm, &preset_data[0]);
    _encode_int16_le(0, &preset_data[4]);
    _encode_int16_le(0, &preset_data[6]);
    _send_command(pda_cmd::PRESET, preset_data);

    uint8_t system_data[8];
    _encode_uint32_le(0x20, &system_data[0]);
    _encode_uint16_le(0, &system_data[4]);
    _encode_uint16_le(0, &system_data[6]);
    _send_command(pda_cmd::SYSTEM, system_data);
    return true;
}

void PdaMotor::on_off()
{
    if (_data->estop || _error)
    {
        _motor_data->is_on = false;
    }

    if (_prev_on_state != _motor_data->is_on)
    {
        if (_motor_data->is_on)
        {
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_on_state);
        }
        else
        {
            _send_zero_torque();
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_off_state);
        }
    }

    _prev_on_state = _motor_data->is_on;
}

bool PdaMotor::enable()
{
    return enable(false);
}

bool PdaMotor::enable(bool overide)
{
    if (_data->estop || _error)
    {
        _motor_data->enabled = false;
    }

    const bool can_enable_motion = _motor_data->enabled && _motor_data->is_on && !_error && !_data->estop;
    const bool enable_state_changed = (_prev_motor_enabled != _motor_data->enabled);

    if (can_enable_motion)
    {
        if (enable_state_changed || overide || !_feedback_configured)
        {
            _motor_data->pda_last_feedback_us = micros();
            _motor_data->pda_feedback_valid = false;
            _motor_data->timeout_count = 0;
            _configure_feedback_for_current_id();
            _enable_response = true;
        }
    }
    else
    {
        if (enable_state_changed || overide || _feedback_configured)
        {
            _send_zero_torque();
            _send_write_property_u32(30003, 3, 1);
        }

        _feedback_configured = false;
        _enable_response = false;
    }

    _prev_motor_enabled = _motor_data->enabled;
    return _enable_response;
}

void PdaMotor::_send_write_property_u32(uint16_t param_address, uint16_t param_type, uint32_t value)
{
    _send_write_property_u32_to_id(_pda_id, param_address, param_type, value);
}

void PdaMotor::_send_write_property_u32_to_id(uint8_t pda_id, uint16_t param_address, uint16_t param_type, uint32_t value)
{
    uint8_t data[8];
    _encode_uint16_le(param_address, &data[0]);
    _encode_uint16_le(param_type, &data[2]);
    _encode_uint32_le(value, &data[4]);
    _send_command_to_id(pda_id, pda_cmd::WRITE_PROPERTY, data);
}

void PdaMotor::_send_read_property_u32(uint8_t pda_id, uint16_t param_address, uint16_t param_type)
{
    uint8_t data[8];
    _encode_uint16_le(param_address, &data[0]);
    _encode_uint16_le(param_type, &data[2]);
    _encode_uint32_le(0, &data[4]);
    _send_command_to_id(pda_id, pda_cmd::READ_PROPERTY, data);
}

void PdaMotor::_configure_feedback_for_current_id()
{
    if (!_has_valid_pda_id())
    {
        return;
    }

    _last_feedback_config_us = micros();
    _debug_print_status("PDA_CONFIGURE_FEEDBACK");
    _send_write_property_u32(31002, 3, pda_config::FEEDBACK_PERIOD_MS);
    _send_write_property_u32(22001, 3, 1);
    _send_write_property_u32(30003, 3, 2);
    _send_read_property_u32(_pda_id, 31001, 3);
    _feedback_configured = true;
}

void PdaMotor::_run_can_id_autodetect()
{
    if (_has_valid_pda_id() ||
        !pda_config::AUTO_DETECT_CAN_ID ||
        !_can_autodetect_this_motor() ||
        _motor_data->pda_feedback_valid ||
        !_motor_data->enabled ||
        !_motor_data->is_on ||
        _data->estop ||
        _error)
    {
        return;
    }

    const uint32_t now_us = micros();
    if ((now_us - _last_detect_probe_us) < pda_config::DETECT_PROBE_PERIOD_US)
    {
        return;
    }

    _last_detect_probe_us = now_us;
    _send_read_property_u32(_detect_scan_id, 31001, 3);
    _detect_scan_id++;
    if (_detect_scan_id > 63)
    {
        _detect_scan_id = 1;
    }
}

void PdaMotor::_adopt_pda_id(uint8_t pda_id)
{
    if (pda_id < 1 || pda_id > 63)
    {
        return;
    }

    if (_pda_id != pda_id)
    {
        _pda_id = pda_id;
        _motor_data->pda_id = pda_id;

        #ifdef SIMPLE_DEBUG
            Serial.print("PDA_CAN_ID_DETECTED=");
            Serial.println(_pda_id);
        #endif
    }

    _error = false;
    _enable_response = true;
    _motor_data->timeout_count = 0;
    _motor_data->pda_last_feedback_us = micros();
    _configure_feedback_for_current_id();
}

bool PdaMotor::_debug_should_print(uint32_t* last_print_us, uint32_t interval_us)
{
    const uint32_t now_us = micros();
    if (*last_print_us == 0)
    {
        *last_print_us = now_us;
        return true;
    }

    if ((now_us - *last_print_us) < interval_us)
    {
        return false;
    }

    *last_print_us = now_us;
    return true;
}

void PdaMotor::_debug_print_tx(uint8_t pda_id, uint8_t cmd_id, const uint8_t* data, const char* label)
{
#ifdef PDA_DEBUG
    const bool always_print = (cmd_id == pda_cmd::READ_PROPERTY) || (cmd_id == pda_cmd::WRITE_PROPERTY);
    if (!always_print && !_debug_should_print(&_last_pda_debug_tx_us, pda_config::DEBUG_PRINT_PERIOD_US))
    {
        return;
    }

    Serial.print("PDA_");
    Serial.print(label);
    Serial.print(" id=");
    Serial.print(pda_id);
    Serial.print(" arb=0x");
    Serial.print(_frame_id_for(pda_id, cmd_id), HEX);
    Serial.print(" cmd=0x");
    Serial.print(cmd_id, HEX);
    Serial.print(" data=");
    for (uint8_t idx = 0; idx < 8; idx++)
    {
        if (data[idx] < 16)
        {
            Serial.print('0');
        }
        Serial.print(data[idx], HEX);
        if (idx < 7)
        {
            Serial.print(' ');
        }
    }

    if (cmd_id == pda_cmd::SET_TORQUE)
    {
        Serial.print(" torqueNm=");
        Serial.print(_decode_float32_le(&data[0]), 3);
    }

    Serial.print(" feedback=");
    Serial.print(_motor_data->pda_feedback_valid ? 1 : 0);
    Serial.print(" timeout=");
    Serial.println(_motor_data->timeout_count);
#else
    (void)pda_id;
    (void)cmd_id;
    (void)data;
    (void)label;
#endif
}

void PdaMotor::_debug_print_rx(const CAN_message_t& msg, bool decoded_feedback)
{
#ifdef PDA_DEBUG
    const uint8_t cmd_id = (uint8_t)(msg.id & 0x001F);
    const bool always_print = (cmd_id == pda_cmd::READ_PROPERTY) || (cmd_id == pda_cmd::WRITE_PROPERTY);
    if (!always_print && !_debug_should_print(&_last_pda_debug_rx_us, pda_config::DEBUG_PRINT_PERIOD_US))
    {
        return;
    }

    Serial.print("PDA_RX arb=0x");
    Serial.print(msg.id, HEX);
    Serial.print(" id=");
    Serial.print((uint8_t)((msg.id & 0x07E0) >> 5));
    Serial.print(" cmd=0x");
    Serial.print(cmd_id, HEX);
    Serial.print(" len=");
    Serial.print(msg.len);
    Serial.print(" feedback=");
    Serial.print(decoded_feedback ? 1 : 0);
    Serial.print(" valid=");
    Serial.print(_motor_data->pda_feedback_valid ? 1 : 0);
    Serial.print(" pdaAngleDeg=");
    Serial.print(_motor_data->pda_angle_deg, 2);
    Serial.print(" pdaSpeedRpm=");
    Serial.print(_motor_data->pda_speed_rpm, 2);
    Serial.print(" pdaTorqueNm=");
    Serial.println(_motor_data->pda_torque_nm, 3);
#else
    (void)msg;
    (void)decoded_feedback;
#endif
}

void PdaMotor::_debug_print_status(const char* label)
{
#ifdef PDA_DEBUG
    if (!_debug_should_print(&_last_pda_debug_status_us, pda_config::DEBUG_PRINT_PERIOD_US))
    {
        return;
    }

    Serial.print(label);
    Serial.print(" id=");
    Serial.print(_pda_id);
    Serial.print(" enabled=");
    Serial.print(_motor_data->enabled ? 1 : 0);
    Serial.print(" isOn=");
    Serial.print(_motor_data->is_on ? 1 : 0);
    Serial.print(" feedback=");
    Serial.print(_motor_data->pda_feedback_valid ? 1 : 0);
    Serial.print(" timeout=");
    Serial.println(_motor_data->timeout_count);
#else
    (void)label;
#endif
}

void PdaMotor::zero()
{
    _send_zero_torque();

    uint8_t data[8];
    _encode_uint32_le(0x23, &data[0]);
    _encode_uint16_le(0, &data[4]);
    _encode_uint16_le(0, &data[6]);
    _send_command(pda_cmd::SYSTEM, data);
}

void PdaMotor::_handle_timeout()
{
    if (!_motor_data->enabled || _error)
    {
        return;
    }

    const uint32_t now_us = micros();
    const bool timed_out = (now_us - _motor_data->pda_last_feedback_us) > pda_config::FEEDBACK_TIMEOUT_US;

    if (timed_out)
    {
        if (pda_config::AUTO_DETECT_CAN_ID && !_motor_data->pda_feedback_valid && _can_autodetect_this_motor())
        {
            _send_zero_torque();
            _motor_data->timeout_count = 0;
            _motor_data->pda_last_feedback_us = now_us;
            return;
        }

        _motor_data->timeout_count++;
        _send_zero_torque();

        if (!_motor_data->pda_feedback_valid &&
            _has_valid_pda_id() &&
            ((now_us - _last_feedback_config_us) >= pda_config::CONFIG_RETRY_PERIOD_US))
        {
            _configure_feedback_for_current_id();
        }

        if (_motor_data->pda_feedback_valid && (_motor_data->timeout_count >= _motor_data->timeout_count_max))
        {
            _error = true;
            _motor_data->enabled = false;
        }
    }
}

float PdaMotor::get_Kt()
{
    return _Kt;
}

void PdaMotor::set_error()
{
    _error = true;
    _send_zero_torque();
}

Pda08Motor::Pda08Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin)
: PdaMotor(id, exo_data, enable_pin, PDA08_SPEC)
{
}

Pda01Motor::Pda01Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin)
: PdaMotor(id, exo_data, enable_pin, PDA01_SPEC)
{
}

/*
 * Constructor for the PWM (Maxon) Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the PWM (e.g. Maxon) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model, which may have specific torque constants etc.
 */
MaxonMotor::MaxonMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin) //Constructor: type is the motor type
: _Motor(id, exo_data, enable_pin)
{
    JointData* j_data = exo_data->get_joint_with(static_cast<uint8_t>(id));
	
    #ifdef MOTOR_DEBUG
        logger::println("MaxonMotor::MaxonMotor: Leaving Constructor");
    #endif
};

void MaxonMotor::transaction(float torque)
{
    //Send data
    send_data(torque);

    //Only enable the motor when it is an active trial 
    master_switch();

	if (_motor_data->enabled)
	{
		maxon_manager(true); //Monitors for and corrects motor resetting error if the system is operational.
	}
	else
	{
		maxon_manager(false);   //Reset the motor error detection function, in case user pauses device in middle of error event
	}

	// Serial.print("\nRight leg MaxonMotor::transaction(float torque)  |  torque = ");
	// Serial.print(torque);
};

bool MaxonMotor::enable()
{
    return true;    //This function is currently bypassed for this motor at the moment.
};

bool MaxonMotor::enable(bool overide)
{	
	//Only change the state and send messages if the enabled state (used as a master switch for this motor) has changed.
    if ((_prev_motor_enabled != _motor_data->enabled) || overide)
    {
		if (_motor_data->enabled)   //_motor_data->enabled is controlled by the GUI
		{
            //Enable motor
			digitalWrite(_enable_pin,HIGH);         //Relocate in the future
		}

		_enable_response = true;
	}

	if (!overide)                   //When enable(false), send the disable motor command, set the analogWrite resolution, and send 50% PWM command
    {
		_enable_response = false;
		
        //Disable motor, the message after this shouldn't matter as the power is cut, and the send() doesn't send a message if not enabled.
		digitalWrite(_enable_pin,LOW);
		analogWrite(_ctrl_right_pin,_pwm_neutral_val);
		analogWrite(_ctrl_left_pin,_pwm_neutral_val);
    }
	
	if (!_motor_data->enabled)   //_motor_data->enabled is controlled by the GUI
		{
            //Disable motor
			digitalWrite(_enable_pin,LOW);         //Relocate in the future
		}

	_prev_motor_enabled = _motor_data->enabled;

    return _enable_response;
	
    #ifdef MOTOR_DEBUG
        logger::print(_prev_motor_enabled);
        logger::print("\t");
        logger::print(_motor_data->enabled);
        logger::print("\t");
        logger::print(_motor_data->is_on);
        logger::print("\n");
    #endif
};

void MaxonMotor::send_data(float torque) //Always send motor command regardless of the motor "enable" status
{
    #ifdef MOTOR_DEBUG
        logger::print("Sending data: ");
        logger::print(uint32_t(_motor_data->id));
        logger::print("\n");
    #endif
	
	int direction_modifier = _motor_data->flip_direction ? -1 : 1; 

	_motor_data->t_ff = torque;
    _motor_data->last_command = torque;
	
	uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) ||
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);
   
	if (_data->user_paused || !active_trial || _data->estop)        //Ignores the exo error handler for the moment
    {
        analogWrite(_ctrl_left_pin,_pwm_neutral_val);   //Set 50% PWM (0 current)
		analogWrite(_ctrl_right_pin,_pwm_neutral_val);	//Set 50% PWM (0 current)
    }
    else
    {
		//Constrain the motor pwm command
		uint16_t post_fuse_torque = max(_pwm_l_bound,_pwm_neutral_val+(direction_modifier*torque));    //Set the lowest allowed PWM command
		post_fuse_torque = min(_pwm_u_bound,post_fuse_torque);                              //Set the highest allowed PWM command
		analogWrite((_motor_data->is_left? _ctrl_left_pin : _ctrl_right_pin),post_fuse_torque);	//Send the motor command to the motor driver
	}
};

void MaxonMotor::master_switch()
{
   //Only run if the motor is supposed to be enabled
    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || 
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);

	if (_data->user_paused || !active_trial || _data->estop)
    {
		pinMode(_err_left_pin, INPUT_PULLUP);
		pinMode(_err_right_pin, INPUT_PULLUP);
		pinMode(_current_left_pin,INPUT);
		pinMode(_current_right_pin,INPUT);
		analogWriteResolution(12);
		analogWriteFrequency(_ctrl_left_pin, 5000);
		analogWriteFrequency(_ctrl_right_pin, 5000);
		
		//_motor_data->enabled = false;
        enable(false);
    }
	else
    {
		//_motor_data->enabled = true;
        enable(true);
	}
};

//Our implementation of the Maxon motor including the ec motor and the Escon 50_8 Motor Controller would occasionally cause 50_8 to enter error mode, with "Over current" being one of the errors.
//To address this issue, we have developed a solution contained in maxon_manager() below. 
void MaxonMotor::maxon_manager(bool manager_active)
{
    //Initialize variables when switch is set to false, run the error detection and rest code when switch is set to true. 
    if (!manager_active)
    {
		//Reset Maxon motor reset utilities
        do_scan4maxon_err_left = true;       
        maxon_counter_active_left = false;
		do_scan4maxon_err_right = true;       
        maxon_counter_active_right = false;
    }
    else
    {
		unsigned long maxon_reset_current_t = millis();
        
		//Scan for left motor error
		if ((do_scan4maxon_err_left) && (!digitalRead(_err_left_pin)))
		{
			do_scan4maxon_err_left = false;          
			maxon_counter_active_left = true;
			zen_millis_left = maxon_reset_current_t;
		}

		//Left motor reset
		if (maxon_counter_active_left) 
		{
			//Two iterations after maxon_counter_actie = true, de-enable motor
			if (maxon_reset_current_t - zen_millis_left >= 2)
			{
				enable(false);
			}

			//Ten iterations after maxon_counter_actie = true, re-enable motor
			if (maxon_reset_current_t - zen_millis_left >= 10)
			{
				enable(true);
			}
			
			//Thirty iterations after maxon_counter_actie = true, start scanning for error again
			if (maxon_reset_current_t - zen_millis_left >= 30)
			{
				do_scan4maxon_err_left = true;
				maxon_counter_active_left = false;                                   
				_motor_data->maxon_plotting_scalar = -1 * _motor_data->maxon_plotting_scalar;
			}
		}

		//Scan for right motor error
		if ((do_scan4maxon_err_right) && (!digitalRead(_err_right_pin)))
		{
			do_scan4maxon_err_right = false;          
			maxon_counter_active_right = true;
			zen_millis_right = maxon_reset_current_t;
		}
		
		//Right motor reset
		if (maxon_counter_active_right) 
		{
			//Two iterations after maxon_counter_actie = true, de-enable motor
			if (maxon_reset_current_t - zen_millis_right >= 2)
			{
				enable(false);
			}

			//Ten iterations after maxon_counter_actie = true, re-enable motor
			if (maxon_reset_current_t - zen_millis_right >= 10)
			{
				enable(true);
			}
			
			//Thirty iterations after maxon_counter_actie = true, start scanning for error again
			if (maxon_reset_current_t - zen_millis_right >= 30)
			{
				do_scan4maxon_err_right = true;
				maxon_counter_active_right = false;                                   
				_motor_data->maxon_plotting_scalar = -1 * _motor_data->maxon_plotting_scalar;
			}
		}
    }
};


#endif
