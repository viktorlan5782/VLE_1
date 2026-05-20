/**
 * @file Config.h
 * @author Chancelor Cuddeback
 * @brief Configuration variables for the codebase.
 * @date 2023-07-18
 * 
 */

#ifndef Config_h
#define Config_h 

#include "Arduino.h"
#include "LogLevels.h"
	#define SIMPLE_DEBUG //Uncomment to enter SIMPLE_DEBUG mode. The exoskeleton’s operating status will be printed to the Serial Monitor.
    #define PDA_DEBUG //Comment out after PDA CAN bring-up to silence throttled PDA TX/RX Serial debug.
    #define TEENSY_LOADER_SAFE_BOOT_DELAY_MS 2000 // Set to 0 after bring-up if Teensy Loader no longer disables Auto Mode.
    #define FIRMWARE_VERSION 0_1_0

    #define AK_Board_V0_1 1
    #define AK_Board_V0_3 2
    #define AK_Board_V0_4 3
    #define AK_Board_V0_5_1 4
	#define OpenExo_Board_V0_6_Maxon 5

    //Update the define below to match the board version being used
    #define BOARD_VERSION AK_Board_V0_5_1 //Update this define to match the board version being used

	#define BATTERY_SENSOR 0 //Set it to 0 to disable, 219 to use INA219, 260 to use INA260, 3 to use the OpenExo Board 0.5.1 Mark 3's onboard voltage divider
	#define CRITICAL_BATT_VAL 18 //In volts. Battery voltage below this will trigger the low battery warning in the GUI.
	#define RESISTOR_1 46700 //Set it to the measured resistance of R1 on the OpenExo Board 0.5.1 Mark 3, and update the volt_sense pin mapping in Board.h
	#define RESISTOR_2 4670 //Set it to the measured resistance of R2 on the OpenExo Board 0.5.1 Mark 3, and update the volt_sense pin mapping in Board.h
    #define REAL_TIME_I2C 1
    #define LOOP_FREQ_HZ 500
    #define LOOP_TIME_TOLERANCE 0.1 
    
    #define USE_SPEED_CHECK 0
	#define USE_ANGLE_SENSORS 1

    //MACRO magic to convert a define to a string
    #define VAL(str) #str
    #define TOSTRING(str) VAL(str)

    namespace logging
    {
        const LogLevel level = LogLevel::Release; //Release or Debug (Note: Enter Debug to have Logger print to serial monitor)
        const int baud_rate = 115200;
    }
    
    namespace sync_time
    {
        const unsigned int NUM_START_STOP_BLINKS = 1;                                   //The number of times to have the LED on during the start stop sequence
        const unsigned int SYNC_HALF_PERIOD_US = 125000;                                //Half blink period in micro seconds
        const unsigned int SYNC_START_STOP_HALF_PERIOD_US = 4 * SYNC_HALF_PERIOD_US;    //Half blink period for the begining and end of the sequence. This is usually longer so it is easy to identify.
    }

    namespace fsr_config
    {
        const float FSR_UPPER_THRESHOLD = 0.25;
        const float FSR_LOWER_THRESHOLD = 0.15;
        const float SCHMITT_DELTA = (FSR_UPPER_THRESHOLD - FSR_LOWER_THRESHOLD)/2;
    }

    namespace insole_high_level
    {
        const uint8_t ZONE_COUNT = 18;

        // Timing: pressure insole raw packet stream is expected at 20 Hz while the firmware loop runs at 500 Hz.
        const uint32_t EXPECTED_PACKET_PERIOD_US = 50000;
        const uint32_t FRAME_TIMEOUT_US = 150000;
        const uint32_t ZERO_REFRESH_HOLD_US = 250000;
        const uint32_t ZERO_REFRESH_STALE_US = 10000000;
        const uint32_t EXPECTED_STANCE_US = 650000;
        const uint32_t BILATERAL_SWING_FAULT_US = 300000;
        const uint32_t MOTION_STATE_RECENT_EVENT_US = 2000000;

        // Pressure units are grams. Positive values mean compressive normal load.
        const float ZONE_MIN_G = 0.0f;
        const float ZONE_MAX_G = 65535.0f;
        const float SATURATION_THRESHOLD_G = 65000.0f;
        const float NO_LOAD_THRESHOLD_G = 200.0f;
        const float COP_MIN_TOTAL_G = 200.0f;
        const float TOTAL_STANCE_THRESHOLD_G = 2000.0f;
        const float FOREFOOT_THRESHOLD_G = 600.0f;
        const float ARCH_THRESHOLD_G = 400.0f;
        const float HEEL_THRESHOLD_G = 800.0f;
        const float STANCE_ON_FACTOR = 1.05f;
        const float SWING_OFF_FACTOR = 0.95f;
        const uint8_t MIN_STATE_FRAMES = 3;

        // Filter and zero-refresh parameters. The method is deterministic median spike rejection + EWMA.
        const float FILTER_EWMA_ALPHA = 0.35f;
        const float ZERO_REFRESH_EWMA_ALPHA = 0.01f;
        const float SPIKE_REJECTION_DELTA_G = 3000.0f;

        // COP and pressure-only ankle biological moment proxy.
        const float FOOT_LENGTH_M = 0.21089f;
        const float DEFAULT_ANKLE_X_AP_NORM = 0.18f;
        const float DEFAULT_TAU_PROXY_SCALE = 1.0f;
        const float TAU_PROXY_LIMIT_NM = 200.0f;
        const float ML_DEVIATION_THRESHOLD_NORM = 0.15f;
    }

    namespace insole_can_config
    {
        // Pressure-insole CAN IDs are intentionally far from the motor-ID range (0x000-0x00A)
        // so sensor traffic stays easy to isolate during bring-up and bus debugging.
        const uint32_t LEFT_PRESSURE_CAN_ID = 0x601;   // Standard CAN ID carrying left-foot TTL fragments.
        const uint32_t RIGHT_PRESSURE_CAN_ID = 0x602;  // Standard CAN ID carrying right-foot TTL fragments.
        const uint32_t FRAGMENT_TIMEOUT_US = 30000;    // us, max gap inside one 39-byte pressure packet.
    }
	
	namespace angle_sensor
	{
		const float ANGLE_UPPER_THRESHOLD = 0.9;
		const float ANGLE_LOWER_THRESHOLD = 0.1;
		const float ROM_LEFT = 103.2f;
		const float ROM_RIGHT = 91.4f; //In degrees
	}

    namespace analog
    {
        const float RESOLUTION = 12;    //The resolution of the analog to digital converter
        const float COUNTS = 4096;      //The number of counts the ADC can have
    }
    
    namespace torque_calibration
    {
        const float AI_CNT_TO_V = 3.3 / 4096;   //Conversion from count to voltage
        const float TRQ_V_TO_NM = 53.70;        //Conversion from voltage to Nm (Negative do to mismatch in torque sensor and motor torque directions) S12:(Left) = 39.8, S05 (Right) = 44.6; (These will be sensor specific).
    }

    namespace pda_config
    {
        const uint32_t CAN_BAUD = 1000000;              // bit/s
        const float PDA08_RATED_TORQUE_NM = 8.0f;       // Nm
        const float PDA08_PEAK_TORQUE_NM = 24.0f;       // Nm
        const float PDA08_RATED_SPEED_RPM = 200.0f;     // rpm
        const float PDA08_RATED_CURRENT_A = 5.0f;       // A
        const float PDA08_STALL_CURRENT_A = 15.5f;      // A
        const float PDA08_ROTOR_INERTIA_GCM2 = 0.0f;    // g*cm^2, not specified in the PDA08 files found
        const float PDA01_RATED_TORQUE_NM = 1.0f;       // Nm
        const float PDA01_PEAK_TORQUE_NM = 3.0f;        // Nm
        const float PDA01_RATED_SPEED_RPM = 524.0f;     // rpm
        const float PDA01_RATED_CURRENT_A = 2.0f;       // A
        const float PDA01_STALL_CURRENT_A = 5.0f;       // A
        const float PDA01_ROTOR_INERTIA_GCM2 = 4.92f;   // g*cm^2
        const uint32_t FEEDBACK_PERIOD_MS = 10;         // ms, 100 Hz for phase-1 single-joint tests
        const uint32_t FEEDBACK_TIMEOUT_US = 30000;     // us, 3 missed 10 ms feedback periods
        const bool AUTO_DETECT_CAN_ID = false;          // Fixed-ID bench mode. Set true only when config ID is unknown.
        const uint32_t DETECT_PROBE_PERIOD_US = 2000;   // us, one CAN ID probe per 500 Hz control loop
        const uint32_t CONFIG_RETRY_PERIOD_US = 250000; // us, retry feedback enable while no feedback has arrived
        const uint32_t DEBUG_PRINT_PERIOD_US = 250000;  // us, throttle PDA TX/RX debug output
    }

    namespace BLE_times
    {
        const float _status_msg_delay = 2000000;    //Microseconds
        const float _real_time_msg_delay = 9000;    //Microseconds (~111 Hz target, expect ~100 Hz actual)
        const float _update_delay = 1000;           //Microseconds
        const float _poll_timeout = 4;              //Milliseconds
    }
    
    //Update this namespace for future exo updates to display correct information on app
    namespace exo_info
    {
        const String FirmwareVersion = String(TOSTRING(FIRMWARE_VERSION));  //String to add to firmware char
        const String PCBVersion = String(TOSTRING(BOARD_VERSION));          //String to add to pcb char
        const String DeviceName = String("NULL");                            //String to add to device char, if you would like the system to set it use "NULL"
    }

    namespace UART_times
    {
        const float UPDATE_PERIOD = 1000;       //Microseconds, time between updating data over uart
        const float COMS_MCU_TIMEOUT = 5000;    //Microseconds
        const float CONT_MCU_TIMEOUT = 1000;    //Microseconds
        const float CONFIG_TIMEOUT = 8000;      //Milliseconds
    }

#endif
