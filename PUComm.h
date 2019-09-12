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

enum MCBMessages_t : uint8_t {
    PU_NO_MESSAGE = 0,

    // PIB -> PU (no params)
    PU_SEND_STATUS,
    PU_SEND_PROFILE_RECORD,
    PE_SEND_TSEN_RECORD,

    // PIB -> PU (with params)
    PU_SET_HEATERS,
    PU_GO_LOWPOWER,
    PU_GO_IDLE,
    PU_GO_WARMUP,
    PU_GO_PREPROFILE,
    PU_GO_PROFILE,
    PU_UPDATE_GPS,

    // PU -> PIB (no params)
    PU_IS_DOCKED,
    PU_NO_MORE_RECORDS,
    PU_PROFILE_RECORD,  // binary transfer
    PU_TSEN_RECORD, // binary transfer

    // PU -> PIB (with params)
    PU_STATUS,
    PU_ERROR
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

    bool TX_Idle(int TSENTMRate); //idle, TSEN TM packet ready every TMRate seconds
    bool RX_Idle(int * TSENTMRate); //idle, TSEN TM packet ready every TMRate seconds

    bool TX_WarmUp(float FLASH_T, float Heater_1_T, float Heater_2_T, int FLASH_power, int TSEN_power);
    bool RX_WarmUp(float * FLASH_T, floa * Heater_1_T, float * Heater_2_T, int * FLASH_power, int * TSEN_power);

    bool TX_PreProfile(int preTime, int TM_period, int data_rate, int TSEN_power, int ROPC_power, int FLASH_power);
    bool RX_PreProfile(int * preTime, int * TM_period, int * data_rate, int * TSEN_power, int * ROPC_power, int * FLASH_power);

    bool TX_Profile(int t_down, int t_dwell, int t_up, int rate_profile, int rate_dwell, int TSEN_power, int ROPC_power, int FLASH_power);
    bool RX_Profile(int * t_down, int * t_dwell, int * t_up, int * rate_profile, int * rate_dwell, int * TSEN_power, int * ROPC_power, int * FLASH_power);

    bool TX_UpdateGPS(int ZephyrGPSTime, double ZephyrGPSlat, double ZephyrGPSlon, int ZephyrGPSAlt);
    bool RX_UpdateGPS(int * ZephyrGPSTime, double * ZephyrGPSlat, double * ZephyrGPSlon, int * ZephyrGPSAlt);

    // PU -> PIB (with params) -----------------------

    bool TX_SendStatus(int PUTime, float VBattery, float ICharge, float Therm1T, float Therm2T);
    bool RX_SendStatus(int * PUTime, float * VBattery, float * ICharge, float * Therm1T, float * Therm2T);

    bool TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);
};

#endif /* PUComm_H */