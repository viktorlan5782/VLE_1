/**
 * @file Exo.h
 *
 * @brief Declares exo class that all the other components will live in. 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef Exo_h
#define Exo_h

//Arduino compiles everything in the src folder even if not included so it causes an error for the nano if this is not included
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Arduino.h"

#include "Side.h"
#include <stdint.h>
#include "ParseIni.h"
#include "Board.h"
#include "Utilities.h"
#include "SyncLed.h"
#include "StatusLed.h"
#include "StatusDefs.h"
#include "Config.h"
#include "PressureInsoleCanReader.h"
#include "PressureInsoleHighLevel.h"
#include "MotionTelemetry.h"

class Exo
{
    public:
		Exo(ExoData* exo_data); //Constructor: uses initializer list for the Side objects.
		
        /**
         * @brief Reads motor data from each motor used on that side and stores the values
         * 
         * @return true if the code ran, ie tiiming was satisfied
         * @return false 
         */
        bool run();  

        /**
         * @brief Injects one decoded pressure-insole raw frame for high-level DPJMC processing.
         *
         * This does not alter the legacy heel/toe FSR gait path.
         */
        void set_pressure_insole_frame(bool is_left, const InsoleRawFrame& frame);
		
        ExoData *data;      /**< Pointer to ExoData that is getting updated by the coms mcu so they share format.*/
        Side left_side;     /**< Left side object that contains all the joints and sensors for that side */
        Side right_side;    /**< Right side object that contains all the joints and sensors for that side */
        PressureInsoleCanReader _insole_can_reader; /**< CAN byte-stream decoder for pressure-insole raw frames. */
        PressureInsoleHighLevel _insole_hl; /**< Raw pressure-insole high-level FSM for DPJMC. */
        motion_telemetry::Publisher _motion_telemetry; /**< Page-scoped binary motion telemetry publisher. */
        
        #ifdef USE_SPEED_CHECK
            utils::SpeedCheck speed_check; /**< Used to check the speed of the loop without needing prints */
        #endif
        
        SyncLed sync_led;       /**< Used to syncronize data with a motion capture system */
        StatusLed status_led;   /**< Used to display the system status */
			
	private:
		
};
#endif

#endif
