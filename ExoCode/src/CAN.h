/**
 * @file CAN.h
 * @author Chancelor Cuddeback
 * @brief Uses the FlexCan library to send and receive CAN messages.
 * @date 2023-07-18
 * 
 */

#ifndef CAN_H
#define CAN_H

#include "Logger.h"
#include "Arduino.h"

 //Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "FlexCAN_T4.h"
#if defined(ARDUINO_TEENSY36)
    static FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#elif defined(ARDUINO_TEENSY41)
    static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
#endif

typedef bool (*CanFramePredicate)(const CAN_message_t& msg, void* context);

/**
 * @brief CAN class for sending and receiving CAN messages. Singleton
 * 
 */
class CAN 
{
    public:
        /**
         * @brief Get the Singleton object
         * 
         * @return CAN* 
         */
        static CAN* getInstance()
        {
            static CAN* instance = new CAN;
            return instance;
        }

        /**
         * @brief Send a CAN message
         * 
         * @param msg CAN_message_t to send
         */
        void send(CAN_message_t msg)
        {
            if(!Can0.write(msg)) 
            {
                logger::println("Error Sending" + String(msg.id), LogLevel::Error);
            }
        }

        /**
         * @brief Read a CAN message
         * 
         * @return CAN_message_t 
         */
        CAN_message_t read()
        {
            CAN_message_t msg;
            Can0.read(msg);
            return msg;
        }

        /**
         * @brief Try to read a CAN message without hiding whether a frame was received.
         *
         * @param msg CAN_message_t updated when a frame is available.
         * @return true if a frame was read, false otherwise.
         */
        bool read(CAN_message_t& msg)
        {
            if (_stash_count > 0)
            {
                msg = _stashed[0];
                _remove_stashed(0);
                return true;
            }

            return Can0.read(msg);
        }

        /**
         * @brief Read the next matching frame and preserve nonmatching frames.
         *
         * This lets multiple CAN consumers share one bus without dropping each
         * other's traffic, e.g. PDA motor feedback and pressure-insole packets.
         */
        bool read_matching(CAN_message_t& msg, CanFramePredicate predicate, void* context, uint8_t max_scan = 16)
        {
            if (predicate == NULL)
            {
                return read(msg);
            }

            for (uint8_t i = 0; i < _stash_count; i++)
            {
                if (predicate(_stashed[i], context))
                {
                    msg = _stashed[i];
                    _remove_stashed(i);
                    return true;
                }
            }

            CAN_message_t candidate;
            uint8_t scanned = 0;
            while ((scanned < max_scan) && Can0.read(candidate))
            {
                scanned++;
                if (predicate(candidate, context))
                {
                    msg = candidate;
                    return true;
                }
                _stash(candidate);
            }

            return false;
        }

    private:
        static const uint8_t RX_STASH_SIZE = 32;
        CAN_message_t _stashed[RX_STASH_SIZE];
        uint8_t _stash_count;
        uint32_t _stash_overflows;

        void _stash(const CAN_message_t& msg)
        {
            if (_stash_count < RX_STASH_SIZE)
            {
                _stashed[_stash_count++] = msg;
                return;
            }

            _stash_overflows++;
            _remove_stashed(0);
            _stashed[_stash_count++] = msg;
        }

        void _remove_stashed(uint8_t index)
        {
            if (index >= _stash_count)
            {
                return;
            }

            for (uint8_t i = index; i + 1 < _stash_count; i++)
            {
                _stashed[i] = _stashed[i + 1];
            }
            _stash_count--;
        }

        /**
         * @brief Construct a new CAN object and initialize the CAN bus. This is private 
         * because this is a singleton.
         * 
         */
        CAN()
        : _stash_count(0)
        , _stash_overflows(0)
        {   
            Can0.begin();
            Can0.setBaudRate(1000000);
        }
};

#endif
#endif
