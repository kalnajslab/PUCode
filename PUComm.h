/*
 *  PUComm.h
 *  Author:  Lars Kalnajs
 *  Created: September 2019
 *  
 *  This file declares an Arduino library (C++ class) that implements the communication
 *  between the PIB and the PU. The class inherits its protocol from the SerialComm
 *  class.
 */

#ifndef PUComm_H
#define PUComm_H

#include "SerialComm.h"

enum PUMessages_t : uint8_t {
    PU_NO_MESSAGE = 0,

    // PIB -> PU (no params)
    PU_SEND_STATUS, //1
    PU_SEND_PROFILE_RECORD, //2
    PU_SEND_TSEN_RECORD, //3
    PU_RESET, //4

    // PIB -> PU (with params)
    PU_SET_HEATERS, //5
    PU_GO_LOWPOWER, //6
    PU_GO_IDLE,  //7
    PU_GO_WARMUP, //8
    PU_GO_PREPROFILE, //9
    PU_GO_PROFILE, //10
    PU_UPDATE_GPS, //11
    PU_LORA_STATUS,
    PU_LORA_TM,

    // PU -> PIB (no params)
    PU_IS_DOCKED, //12
    PU_NO_MORE_RECORDS, //13
    PU_PROFILE_RECORD,  // 14 binary transfer
    PU_TSEN_RECORD, // 15 binary transfer

    // PU -> PIB (with params)
    PU_STATUS, //16
    PU_ERROR //17
};


class PUComm : public SerialComm {
public:
    PUComm(Stream * serial_port);
    ~PUComm() { };

    // PIB -> PU (with params) -----------------------
    bool TX_SetHeaters(float Heater1T, float Heater2T); //Set the heater temperature (parameter not state)
    bool RX_SetHeaters(float * Heater1T, float * Heater2T);

    bool TX_LowPower(float survivalT); //go low power, heater set to survival_temp
    bool RX_LowPower(float * survivalT);

    bool TX_Idle(int32_t TSENTMRate); //idle, TSEN TM packet ready every TMRate seconds
    bool RX_Idle(int32_t * TSENTMRate); //idle, TSEN TM packet ready every TMRate seconds

    bool TX_WarmUp(float FLASH_T, float Heater_1_T, float Heater_2_T, int8_t FLASH_power, int8_t TSEN_power);
    bool RX_WarmUp(float * FLASH_T, float * Heater_1_T, float * Heater_2_T, int8_t * FLASH_power, int8_t * TSEN_power);

    bool TX_PreProfile(int32_t preTime, int32_t TM_period, int32_t data_rate, int8_t TSEN_power, int8_t ROPC_power, int8_t FLASH_power);
    bool RX_PreProfile(int32_t * preTime, int32_t * TM_period, int32_t * data_rate, int8_t * TSEN_power, int8_t * ROPC_power, int8_t * FLASH_power);

    bool TX_Profile(int32_t t_down, int32_t t_dwell, int32_t t_up, int32_t rate_profile, int32_t rate_dwell, int8_t TSEN_power, int8_t ROPC_power, int8_t FLASH_power, int8_t LoRa_TM);
    bool RX_Profile(int32_t * t_down, int32_t * t_dwell, int32_t * t_up, int32_t * rate_profile, int32_t * rate_dwell, int8_t * TSEN_power, int8_t * ROPC_power, int8_t * FLASH_power, int8_t * LoRa_TM);

    bool TX_UpdateGPS(uint32_t ZephyrGPSTime, float ZephyrGPSlat, float ZephyrGPSlon, uint16_t ZephyrGPSAlt);
    bool RX_UpdateGPS(uint32_t * ZephyrGPSTime, float * ZephyrGPSlat, float * ZephyrGPSlon, uint16_t * ZephyrGPSAlt);

    bool TX_PULoRaStatus(uint16_t LoRaTXStatus);
    bool RX_PULoRaStatus(uint16_t * LoRaTXStatus);

    bool TX_PULoRaTM(uint8_t LoRaTXTM);
    bool RX_PULoRaTM(uint8_t * LoRaTXTM);

    // PU -> PIB (with params) -----------------------

    bool TX_Status(uint32_t PUTime, float VBattery, float ICharge, float Therm1T, float Therm2T, uint8_t HeaterStat);
    bool RX_Status(uint32_t * PUTime, float * VBattery, float * ICharge, float * Therm1T, float * Therm2T, uint8_t * HeaterStat);

    bool TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);
};

#endif /* PUComm_H */
