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

bool PUComm::TX_SetHeaters(float * Heater1T, float * Heater2T)
{
    float temp1, temp2;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
   
    *Heater1T = temp1;
    *Heater2T = temp2;

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

bool PUComm::TX_Idle(int TSENTMRate)
{
    if (!Add_int(TSENTMRate)) return false;
   
    TX_ASCII(PU_GO_IDLE);

    return true;
}

bool PUComm::RX_Idle(int * TSENTMRate)
{
    int temp1;

    if (!Get_int(&temp1)) return false;
   
    *survivalT = temp1;

    return true;
}

bool PUComm::TX_WarmUp(float FLASH_T, float Heater_1_T, float Heater_2_T, int FLASH_power, int TSEN_power)
{
    if (!Add_float(FLASH_T)) return false;
    if (!Add_float(Heater_1_T)) return false;
    if (!Add_float(Heater_2_T)) return false;
    if (!Add_int(FLASH_power)) return false;
    if (!Add_int(TSEN_power)) return false;
   
    TX_ASCII(PU_GO_WARMUP);

    return true;
}

bool PUComm::RX_WarmUp(float * FLASH_T, float * Heater_1_T, float * Heater_2_T, int * FLASH_power, int * TSEN_power)
{
    float temp1, temp2, temp3;
    int temp4, temp5;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_int(&temp4)) return false;
    if (!Get_int(&temp5)) return false;

    *FLASH_T = temp1;
    *Heater_1_T = temp2;
    *Heater_2_T = temp3;
    *FLASH_power = temp4;
    *TSEN_power = temp5;

    return true;
}

bool PUComm::TX_PreProfile(int preTime, int TM_period, int data_rate, int TSEN_power, int ROPC_power, int FLASH_power)
{
    if (!Add_int(preTime)) return false;
    if (!Add_int(TM_period)) return false;
    if (!Add_int(data_rate)) return false;
    if (!Add_int(TSEN_power)) return false;
    if (!Add_int(ROPC_power)) return false;
    if (!Add_int(FLASH_power)) return false;
   
    TX_ASCII(PU_GO_PREPROFILE);

    return true;
}

bool PUComm::RX_PreProfile(int * preTime, int * TM_period, int * data_rate, int * TSEN_power, int * ROPC_power, int * FLASH_power)
{
    int temp1, temp2, temp3, temp4, temp5, temp6;

    if (!Get_int(&temp1)) return false;
    if (!Get_int(&temp2)) return false;
    if (!Get_int(&temp3)) return false;
    if (!Get_int(&temp4)) return false;
    if (!Get_int(&temp5)) return false;
    if (!Get_int(&temp6)) return false;

    *preTime = temp1;
    *TM_period = temp2;
    *data_rate = temp3;
    *TSEN_power = temp4;
    *ROPC_power = temp5;
    *FLASH_power = temp6;

    return true;
}

bool PUComm::TX_Profile(int t_down, int t_dwell, int t_up, int rate_profile, int rate_dwell, int TSEN_power, int ROPC_power, int FLASH_power)
{
    if (!Add_int(t_down)) return false;
    if (!Add_int(t_dwell)) return false;
    if (!Add_int(t_up)) return false;
    if (!Add_int(rate_profile)) return false;
    if (!Add_int(rate_dwell)) return false;
    if (!Add_int(TSEN_power)) return false;
    if (!Add_int(ROPC_power)) return false;
    if (!Add_int(FLASH_power)) return false;
   
    TX_ASCII(PU_GO_PROFILE);

    return true;
}

bool PUComm::RX_Profile(int * t_down, int * t_dwell, int * t_up, int * rate_profile, int * rate_dwell, int * TSEN_power, int * ROPC_power, int * FLASH_power)
{
    int temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8;

    if (!Get_int(&temp1)) return false;
    if (!Get_int(&temp2)) return false;
    if (!Get_int(&temp3)) return false;
    if (!Get_int(&temp4)) return false;
    if (!Get_int(&temp5)) return false;
    if (!Get_int(&temp6)) return false;
    if (!Get_int(&temp7)) return false;
    if (!Get_int(&temp8)) return false;

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

bool PUComm::TX_UpdateGPS(int ZephyrGPSTime, double ZephyrGPSlat, double ZephyrGPSlon, int ZephyrGPSAlt)
{
    if (!Add_int(ZephyrGPSTime)) return false;
    if (!Add_float(ZephyrGPSlat)) return false;
    if (!Add_float(ZephyrGPSlon)) return false;
    if (!Add_int(ZephyrGPSAlt)) return false;
    
    TX_ASCII(PU_UPDATE_GPS);

    return true;
}

bool PUComm::RX_UpdateGPS(int * ZephyrGPSTime, double * ZephyrGPSlat, double * ZephyrGPSlon, int * ZephyrGPSAlt)
{
    int temp1, temp4;
    float temp2, temp3;

    if (!Get_int(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_int(&temp4)) return false;
   
    *ZephyrGPSTime= temp1;
    *ZephyrGPSlat = temp2;
    *ZephyrGPSlon = temp3;
    *ZephyrGPSAlt = temp4;

    return true;
}

bool PUComm::TX_SendStatus(int PUTime, float VBattery, float ICharge, float Therm1T, float Therm2T)
{
    if (!Add_int(PUTime)) return false;
    if (!Add_float(VBattery)) return false;
    if (!Add_float(ICharge)) return false;
    if (!Add_float(Therm1T)) return false;
    if (!Add_float(Therm2T)) return false;
   
    TX_ASCII(PU_STATUS);

    return true;
}

bool PUComm::RX_SendStatus(int * PUTime, float * VBattery, float * ICharge, float * Therm1T, float * Therm2T)
{
    int temp1;
    float temp2, temp3, temp4, temp5;

    if (!Get_int(&temp1)) return false;
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


