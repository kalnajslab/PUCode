/*
 *  PUComm.cpp
 *  Author:  Lars Kalnajs
 *  Created: September 2019
 *  
 *  This file implements an Arduino library (C++ class) that implements the communication
 *  between the PIB and PU. The class inherits its protocol from the SerialComm
 *  class.
 */

#include "PUComm.h"

PUComm::PUComm(Stream * serial_port)
    : SerialComm(serial_port)
{
}

// PIB -> PU (with params) ---------------------------

bool PUComm::TX_SetHeaters(float Heater1T, float Heater2T)
{
    if (!Add_float(Heater1T)) return false;
    if (!Add_float(Heater2T)) return false;
   
    TX_ASCII(PU_SET_HEATERS);

    return true;
}

bool PUComm::RX_SetHeaters(float * Heater1T, float * Heater2T)
{
    float temp1, temp2;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
   
    *Heater1T = temp1;
    *Heater2T = temp2;

    Serial.println(temp1);

    return true;
}

bool PUComm::TX_LowPower(float survivalT)
{
    if (!Add_float(survivalT)) return false;
   
    TX_ASCII(PU_GO_LOWPOWER);

    return true;
}

bool PUComm::RX_LowPower(float * survivalT)
{
    float temp1;

    if (!Get_float(&temp1)) return false;
   
    *survivalT = temp1;

    return true;
}

bool PUComm::TX_Idle(int32_t TSENTMRate)
{
    if (!Add_int32(TSENTMRate)) return false;
   
    TX_ASCII(PU_GO_IDLE);

    return true;
}

bool PUComm::RX_Idle(int32_t * TSENTMRate)
{
    int32_t temp1;

    if (!Get_int32(&temp1)) return false;
   
    *TSENTMRate = temp1;

    return true;
}

bool PUComm::TX_WarmUp(float FLASH_T, float Heater_1_T, float Heater_2_T, int8_t FLASH_power, int8_t TSEN_power)
{
    if (!Add_float(FLASH_T)) return false;
    if (!Add_float(Heater_1_T)) return false;
    if (!Add_float(Heater_2_T)) return false;
    if (!Add_int8(FLASH_power)) return false;
    if (!Add_int8(TSEN_power)) return false;
   
    TX_ASCII(PU_GO_WARMUP);

    return true;
}

bool PUComm::RX_WarmUp(float * FLASH_T, float * Heater_1_T, float * Heater_2_T, int8_t * FLASH_power, int8_t * TSEN_power)
{
    float temp1, temp2, temp3;
    int8_t temp4, temp5;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_int8(&temp4)) return false;
    if (!Get_int8(&temp5)) return false;

    *FLASH_T = temp1;
    *Heater_1_T = temp2;
    *Heater_2_T = temp3;
    *FLASH_power = temp4;
    *TSEN_power = temp5;

    return true;
}

bool PUComm::TX_PreProfile(int32_t preTime, int32_t TM_period, int32_t data_rate, int8_t TSEN_power, int8_t ROPC_power, int8_t FLASH_power)
{
    if (!Add_int32(preTime)) return false;
    if (!Add_int32(TM_period)) return false;
    if (!Add_int32(data_rate)) return false;
    if (!Add_int8(TSEN_power)) return false;
    if (!Add_int8(ROPC_power)) return false;
    if (!Add_int8(FLASH_power)) return false;
   
    TX_ASCII(PU_GO_PREPROFILE);

    return true;
}

bool PUComm::RX_PreProfile(int32_t * preTime, int32_t * TM_period, int32_t * data_rate, int8_t * TSEN_power, int8_t * ROPC_power, int8_t * FLASH_power)
{
    int32_t temp1, temp2, temp3;
    int8_t  temp4, temp5, temp6;

    if (!Get_int32(&temp1)) return false;
    if (!Get_int32(&temp2)) return false;
    if (!Get_int32(&temp3)) return false;
    if (!Get_int8(&temp4)) return false;
    if (!Get_int8(&temp5)) return false;
    if (!Get_int8(&temp6)) return false;

    *preTime = temp1;
    *TM_period = temp2;
    *data_rate = temp3;
    *TSEN_power = temp4;
    *ROPC_power = temp5;
    *FLASH_power = temp6;

    return true;
}

bool PUComm::TX_Profile(int32_t t_down, int32_t t_dwell, int32_t t_up, int32_t rate_profile, int32_t rate_dwell, int8_t TSEN_power, int8_t ROPC_power, int8_t FLASH_power)
{
    if (!Add_uint16(t_down)) return false;
    if (!Add_uint16(t_dwell)) return false;
    if (!Add_uint16(t_up)) return false;
    if (!Add_uint16(rate_profile)) return false;
    if (!Add_uint16(rate_dwell)) return false;
    if (!Add_uint8(TSEN_power)) return false;
    if (!Add_uint8(ROPC_power)) return false;
    if (!Add_uint8(FLASH_power)) return false;
   
    TX_ASCII(PU_GO_PROFILE);

    return true;
}

bool PUComm::RX_Profile(int32_t * t_down, int32_t * t_dwell, int32_t * t_up, int32_t * rate_profile, int32_t * rate_dwell, int8_t * TSEN_power, int8_t * ROPC_power, int8_t * FLASH_power)
{
    int32_t temp1, temp2, temp3, temp4, temp5;
    int8_t temp6, temp7, temp8;

    if (!Get_int32(&temp1)) return false;
    if (!Get_int32(&temp2)) return false;
    if (!Get_int32(&temp3)) return false;
    if (!Get_int32(&temp4)) return false;
    if (!Get_int32(&temp5)) return false;
    if (!Get_int8(&temp6)) return false;
    if (!Get_int8(&temp7)) return false;
    if (!Get_int8(&temp8)) return false;

    *t_down = temp1;
    *t_dwell = temp2;
    *t_up = temp3;
    *rate_profile = temp4;
    *rate_dwell = temp5;
    *TSEN_power = temp6;
    *ROPC_power = temp7;
    *FLASH_power = temp8;

    return true;
}

bool PUComm::TX_UpdateGPS(uint32_t ZephyrGPSTime, float ZephyrGPSlat, float ZephyrGPSlon, uint16_t ZephyrGPSAlt)
{
    if (!Add_uint32(ZephyrGPSTime)) return false;
    if (!Add_float(ZephyrGPSlat)) return false;
    if (!Add_float(ZephyrGPSlon)) return false;
    if (!Add_uint16(ZephyrGPSAlt)) return false;
    
    TX_ASCII(PU_UPDATE_GPS);

    return true;
}

bool PUComm::RX_UpdateGPS(uint32_t * ZephyrGPSTime, float * ZephyrGPSlat, float * ZephyrGPSlon, uint16_t * ZephyrGPSAlt)
{
    uint32_t temp1;
    uint16_t temp4;
    float temp2, temp3;

    if (!Get_uint32(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_uint16(&temp4)) return false;
   
    *ZephyrGPSTime= temp1;
    *ZephyrGPSlat = temp2;
    *ZephyrGPSlon = temp3;
    *ZephyrGPSAlt = temp4;

    return true;
}

bool PUComm::TX_Status(uint32_t PUTime, float VBattery, float ICharge, float Therm1T, float Therm2T)
{
    if (!Add_uint32(PUTime)) return false;
    if (!Add_float(VBattery)) return false;
    if (!Add_float(ICharge)) return false;
    if (!Add_float(Therm1T)) return false;
    if (!Add_float(Therm2T)) return false;
   
    TX_ASCII(PU_STATUS);

    return true;
}

bool PUComm::RX_Status(uint32_t * PUTime, float * VBattery, float * ICharge, float * Therm1T, float * Therm2T)
{
    uint32_t temp1;
    float temp2, temp3, temp4, temp5;

    if (!Get_uint32(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_float(&temp4)) return false;
    if (!Get_float(&temp5)) return false;

    *PUTime = temp1;
    *VBattery = temp2;
    *ICharge = temp3;
    *Therm1T = temp4;
    *Therm2T = temp5;

    return true;
}

// -- PU to PIB error string

bool PUComm::TX_Error(const char * error)
{
    if (Add_string(error)) return false;

    TX_ASCII(PU_ERROR);

    return true;
}

bool PUComm::RX_Error(char * error, uint8_t buffer_size)
{
    return Get_string(error, buffer_size);
}


