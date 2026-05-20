/**
 * @file Motor.h
 *
 * @brief Declares a class used to interface with motors
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/

#ifndef Motor_h
#define Motor_h

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include "Arduino.h"

#include "ExoData.h"
#include "ParseIni.h"
#include "Board.h"
#include "CAN.h"
#include "Utilities.h"

#include <stdint.h>

/**
 * @brief Abstract class to define the interface for all motors.
 * All controllers must have a:
 * void read_data()
 * void send_data(float torque)
 * void transaction(float torque)
 * void on_off()
 * bool enable()
 * bool enable(bool overide)
 * void zero()
 * bool get_is_left()
 * config_defs::joint_id get_id()
 */
class _Motor
{
	public:
		_Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        virtual ~_Motor(){};
		
        //Pure virtual functions, these will have to be defined for each one.
        
        /**
         * @brief Reads motor data from each motor used on that side and stores the values
         */
        virtual void read_data() = 0; 

        /**
         * @brief Sends the new motor command to the motor.
         * 
         * @param motor torque command in Nm
         */
		virtual void send_data(float torque) = 0;  
		
        /**
         * @brief Sends the new motor command to the motor and reads the current state of the motor.
         * 
         * @param motor torque command in Nm
         */
        virtual void transaction(float torque) = 0;
		
        /**
         * @brief Powers on or off the motors depending on the is_on value in motor data 
         */
        virtual void on_off() = 0;  
        
        /**
         * @brief Enables or disables the motors depending on the state stored in the corresponding enabled state in motor data.
         * Only sends commands if the state has changes in the motor data.
         */
        virtual bool enable() = 0;  
        
        /**
         * @brief Same as enable but will resend commands if override is true, regardless of what the state of the system is.
         */
        virtual bool enable(bool overide) = 0;  
        
        /**
         * @brief Set position to zero
         */
        virtual void zero() = 0;  
        
        /**
         * @brief Lets you know if it is a left or right side.
         * 
         * @return 1 if the motor is on the left side, 0 otherwise
         */
        virtual bool get_is_left();  // 
        
        /**
         * @brief Returns the motor id, same as the joint id
         *
         * @return the motor id
         */
        virtual config_defs::joint_id get_id();

        virtual float get_Kt() = 0;                 /**< Torque constant of the motor, at the motor output. [Nm/A] */

        virtual void set_error() = 0;               /**< Sets the error flag for the motor. */
		
	protected:
        config_defs::joint_id _id;                  /**< Motor ID */
		bool _is_left;
        ExoData* _data;
		MotorData* _motor_data;
        int _enable_pin;
        bool _prev_motor_enabled; 
        bool _prev_on_state;
        bool _error = false;
        float _Kt;                                  /**< Torque constant of the motor, at the motor output. [Nm/A] */  
};

/**
 * @brief A motor that does nothing
 */
class NullMotor : public _Motor
{
    public:
    NullMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin):_Motor(id, exo_data, enable_pin) {};
    void read_data() {};
    void send_data(float torque) {};
    void transaction(float torque) {};
    void on_off() {};
    bool enable() {return true;};
    bool enable(bool overide) {return true;};
    void zero() {};
    float get_Kt() {return 0.0;};
    void set_error() {};
};

/**
 * @brief Class for Maxon EC motor
 */
class MaxonMotor : public _Motor
{
    public:
    MaxonMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
    void transaction(float torque);
	void read_data() {};
    void send_data(float torque);
    void on_off() {};
    bool enable();
    bool enable(bool overide);
    void zero() {};
    float get_Kt() {return 0.0;};
    void set_error() {};                        //Not yet implemented for this motor type
	void master_switch();
	void maxon_manager(bool manager_active);    /**< Quickly and automatically reset the Maxon motor in case of the driver board reporting an error. */
	
	protected:
	bool _enable_response;                      /**< True if the motor responded to an enable command */
	bool do_scan4maxon_err_left = true;              /**< Part of the Maxon motor driver error reporting utilities: A switch to enable or disable error detection */
	bool maxon_counter_active_left = false;          /**< Part of the Maxon motor driver error reporting utilities: A switch for the error detection counter */
	unsigned long zen_millis_left;                   /**< Part of the Maxon motor driver error reporting utilities: A timer for the motor reset function */
	bool do_scan4maxon_err_right = true;              /**< Part of the Maxon motor driver error reporting utilities: A switch to enable or disable error detection */
	bool maxon_counter_active_right = false;          /**< Part of the Maxon motor driver error reporting utilities: A switch for the error detection counter */
	unsigned long zen_millis_right;                   /**< Part of the Maxon motor driver error reporting utilities: A timer for the motor reset function */
	const int _ctrl_left_pin = logic_micro_pins::maxon_ctrl_left_pin;	/**< Teensy pin to transmit left Maxon motor pwm signals */
	const int _ctrl_right_pin = logic_micro_pins::maxon_ctrl_right_pin;	/**< Teensy pin to transmit right Maxon motor pwm signals */
	const int _err_left_pin = logic_micro_pins::maxon_err_left_pin;	/**< Teensy pin to receive left Maxon motor driver errors */
	const int _err_right_pin = logic_micro_pins::maxon_err_right_pin;	/**< Teensy pin to receive right Maxon motor driver errors */
	const int _current_left_pin = logic_micro_pins::maxon_current_left_pin;	/**< Teensy pin to receive left Maxon motor current data */
	const int _current_right_pin = logic_micro_pins::maxon_current_right_pin;	/**< Teensy pin to receive right Maxon motor current data */
	const int _pwm_neutral_val = logic_micro_pins::maxon_pwm_neutral_val;	/**< Neutral pwm command for Maxon motor drivers */
	const int _pwm_u_bound = logic_micro_pins::maxon_pwm_u_bound;	/**< Upper bound of pwm command for Maxon motor drivers */
	const int _pwm_l_bound = logic_micro_pins::maxon_pwm_l_bound;	/**< Lower bound of pwm command for Maxon motor drivers */
};


/**
 * @brief This will define some of the common communication used by all the CAN motors and should be inherited by all of them.
 */
class _CANMotor : public _Motor
{
    public:
        _CANMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        virtual ~_CANMotor(){};
        void transaction(float torque);
        void read_data();
        void send_data(float torque);
        void on_off();
        bool enable();
        bool enable(bool overide);
        void zero();
        float get_Kt();
        void check_response();
        void set_error();
        
    protected:

        void set_Kt(float Kt);
        
        /**
         * @brief Packs a float into the uint format needed to be sent to the motor.
         *
         * @param Float to be packed
         * @param Lower limit of the range of x values, used for scaling
         * @param Upper limit of the range of x values, used for scaling
         * @param Number of bits to pack the value into, 12 or 16
         *
         * @return Should return a uint that has been scaled to a position between x_min and x_max.  Currently returns a float, but it seems to work.
         */
        float _float_to_uint(float x, float x_min, float x_max, int bits);
        
        /**
         * @brief Unpacks a unsigned int format from the motor into a float.
         *
         * @param Unsigned int to be unpacked
         * @param Lower limit of the range of x values, used for scaling
         * @param Upper limit of the range of x values, used for scaling
         * @param Number of bits to pack the value into, 12 or 16
         *
         * @return unpacked float value 
         */
        float _uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
        
        /**
         * @brief Detects timeouts in case of a read failure.
         *
         */
        void _handle_read_failure();
        
        float _KP_MIN;                              /**< Lower limit of the P gain for the motor */
        float _KP_MAX;                              /**< Upper limit of the P gain for the motor */
        float _KD_MIN;                              /**< Lower limit of the D gain for the motor */
        float _KD_MAX;                              /**< Upper limit of the D gain for the motor */
        float _P_MAX;                               /**< Max angle of the motor */
        float _I_MAX;                               /**< Max current of the motor */
        float _V_MAX;                               /**< Max velocity of the motor */
        bool _enable_response;                      /**< True if the motor responded to an enable command */
        const uint32_t _timeout = 500;              /**< Time to wait for response from the motor in micro-seconds */

        std::queue<float> _measured_current;        /**< Queue of the measured current values */
        const int _current_queue_size = 25;         /**< Size of the queue of measured current values */
        const float _variance_threshold = 0.01;     /**< Threshold for the variance of the measured current values */
};

/**
 * @brief Class for AK60 V1.0 motor
 */
class AK60 : public _CANMotor
{
    public:
        AK60(config_defs::joint_id id, ExoData* exo_data, int enable_pin); //Constructor: type is the motor type
		~AK60(){};
};

/**
 * @brief Class for AK60 V1.1 motor - Takes Current for Input
 */
class AK60v1_1 : public _CANMotor
{
    public:
        AK60v1_1(config_defs::joint_id id, ExoData* exo_data, int enable_pin); //Constructor: type is the motor type
		~AK60v1_1(){};
};

/**
 * @brief Class for AK80 V1.0 motor
 */
class AK80 : public _CANMotor
{
    public:
        AK80(config_defs::joint_id id, ExoData* exo_data, int enable_pin); //Constructor: type is the motor type
		~AK80(){};   
};

/**
 * @brief Class for AK70 V1.0 motor
 */
class AK70 : public _CANMotor
{
    public:
        AK70(config_defs::joint_id id, ExoData* exo_data, int enable_pin); //Constructor: type is the motor type
        ~AK70(){};
};

/**
* @brief Class for AK60v3 motor
*/
class AK60v3 : public _CANMotor
{
  	public:
          AK60v3(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // Constructor: type is the motor type
          ~AK60v3(){};
};

/**
* @brief Class for AK45-36 motor
*/
class AK45_36 : public _CANMotor
{
  	public:
          AK45_36(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // Constructor: type is the motor type
          ~AK45_36(){};
};

/**
* @brief Class for AK45-10 motor
*/
class AK45_10 : public _CANMotor
{
  	public:
          AK45_10(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // Constructor: type is the motor type
          ~AK45_10(){};
};

struct PdaModelSpec
{
    float rated_torque_nm;
    float peak_torque_nm;
    float rated_speed_rpm;
    float rated_current_a;
    float stall_current_a;
    float rotor_inertia_gcm2;
};

/**
* @brief Base class for Dr. Empower PDA-series motors.
*
* PDA motors use the CAN physical layer, but their protocol is not the AK MIT
* packet used by _CANMotor. Keep this parallel to _CANMotor so AK enable, zero,
* and fixed-point packet assumptions do not leak into PDA commands.
*/
class PdaMotor : public _Motor
{
    public:
        PdaMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin, const PdaModelSpec& spec);
        ~PdaMotor(){};
        void transaction(float torque);
        void read_data();
        void send_data(float torque);
        void on_off();
        bool enable();
        bool enable(bool overide);
        void zero();
        float get_Kt();
        void set_error();

        bool send_torque_direct_nm(float torque_nm);
        bool send_torque_ramp_nm_s(float torque_nm, float ramp_nm_s);
        bool send_speed_rpm(float speed_rpm, float param, uint8_t mode);
        bool send_position_deg(float angle_deg, float speed_rpm, float param, uint8_t mode);
        bool send_adaptive_position_deg(float angle_deg, float speed_rpm, float torque_limit_nm);
        bool send_impedance_deg_rpm_nm(float angle_deg, float speed_rpm, float tff_nm, float kp_nm_per_deg, float kd_nm_per_rpm, uint8_t mode);
        bool send_motion_aid(float angle_deg, float speed_rpm, float angle_err_deg, float speed_err_rpm, float torque_nm);

    protected:
        uint16_t _frame_id(uint8_t cmd_id) const;
        uint16_t _frame_id_for(uint8_t pda_id, uint8_t cmd_id) const;
        bool _is_feedback_frame(const CAN_message_t& msg) const;
        void _decode_feedback(const CAN_message_t& msg);
        void _decode_feedback(MotorData* motor_data, const CAN_message_t& msg);
        bool _decode_any_pda_feedback(const CAN_message_t& msg);
        MotorData* _get_pda_motor_data_by_pda_id(uint8_t pda_id) const;
        void _send_command(uint8_t cmd_id, const uint8_t* data);
        void _send_command_to_id(uint8_t pda_id, uint8_t cmd_id, const uint8_t* data);
        void _send_torque_command(float torque_nm, uint16_t input_mode, int16_t ramp_rate);
        void _send_torque_command_to_id(uint8_t pda_id, float torque_nm, uint16_t input_mode, int16_t ramp_rate);
        void _send_zero_torque();
        void _send_write_property_u32(uint16_t param_address, uint16_t param_type, uint32_t value);
        void _send_write_property_u32_to_id(uint8_t pda_id, uint16_t param_address, uint16_t param_type, uint32_t value);
        void _send_read_property_u32(uint8_t pda_id, uint16_t param_address, uint16_t param_type);
        void _configure_feedback_for_current_id();
        void _run_can_id_autodetect();
        void _adopt_pda_id(uint8_t pda_id);
        void _handle_timeout();
        static bool _is_pda_can_frame(const CAN_message_t& msg, void* context);
        void _debug_print_tx(uint8_t pda_id, uint8_t cmd_id, const uint8_t* data, const char* label);
        void _debug_print_rx(const CAN_message_t& msg, bool decoded_feedback);
        void _debug_print_status(const char* label);
        bool _debug_should_print(uint32_t* last_print_us, uint32_t interval_us);
        bool _can_send_motion_command();
        bool _has_valid_pda_id() const;
        bool _is_valid_pda_command_id(uint8_t pda_id) const;
        bool _can_autodetect_this_motor() const;
        bool _pda_id_is_unique() const;
        bool _is_pda_motor_type(uint8_t motor_type) const;
        float _apply_torque_limit(float torque_nm) const;
        float _apply_speed_limit(float speed_rpm) const;
        int16_t _scale_to_int16(float value, float scale) const;
        uint16_t _scale_to_uint16(float value, float scale) const;
        void _encode_float32_le(float val, uint8_t* data);
        void _encode_uint32_le(uint32_t val, uint8_t* data);
        void _encode_uint16_le(uint16_t val, uint8_t* data);
        void _encode_int16_le(int16_t val, uint8_t* data);
        float _decode_float32_le(const uint8_t* data);
        int16_t _decode_int16_le(const uint8_t* data);

        PdaModelSpec _spec;
        uint8_t _pda_id;
        bool _enable_response;
        bool _feedback_configured;
        uint8_t _detect_scan_id;
        uint32_t _last_detect_probe_us;
        uint32_t _last_feedback_config_us;
        uint32_t _last_pda_debug_tx_us;
        uint32_t _last_pda_debug_rx_us;
        uint32_t _last_pda_debug_status_us;
};

class Pda08Motor : public PdaMotor
{
    public:
        Pda08Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        ~Pda08Motor(){};
};

class Pda01Motor : public PdaMotor
{
    public:
        Pda01Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        ~Pda01Motor(){};
};

#endif
#endif
