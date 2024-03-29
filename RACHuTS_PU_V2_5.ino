/*
 * RACHuTS Profiling Unit Operating Code
 * V2.4
 * Lars Kalnajs August 2019
 * 
 */

#include "PULibrary1.h"
#include <TinyGPS++.h>
#include <math.h>
#include <Adafruit_SleepyDog.h>
#include <TimeLib.h>
#include <SerialComm.h>
#include <PUComm.h>
//SD Library
#include <SdFat.h>
#include <SdFatConfig.h>
//LoRa Library from: https://github.com/sandeepmistry/arduino-LoRa
#include <LoRa.h>


//Teensy 3.6 specific SD card config
#define USE_SDIO 1

#define DEBUG_SERIAL Serial
#define TM_BUFFER_LENGTH 5000
#define TSEN_BUFFER_LENGTH 1800
#define TM_RECORD_LENGTH 15   //Length of the TM Record in uint16 - ie twice this in bytes (150KB)
#define TSEN_RECORD_LENGTH 5  //Length of the TSEN Record in uint16 - ie twice this in bytes (18KB)
#define PUCOMM_ACK_TIMEOUT 6000 //want this to be longer than the NAK timeout on PIB

// LoRa Configuration
#define BAND 868E6
#define SS_PIN 43
#define RESET_PIN 50
#define INTERUPT_PIN 49

#define BANDWIDTH 250E3
#define SF 10
#define RF_POWER 17

PULibrary1 PU(13);
TinyGPSPlus gps;
//PUComm pucomm(&Serial);  //For ground test through USB
PUComm pucomm(&Serial1);
SdFatSdio SD;

/* Global enum to store the current/next mode */
enum PUModes_t {
    LOWPOWER,
    IDLE,
    WARMUP,
    PREPROFILE,
    PROFILE
};

PUModes_t Mode = IDLE;

/* PU Configurable parameters */
int32_t TSENDataRate = 5;  //TSEN polling rate in seconds, update from 1 to 5 in V2.5
int32_t TSENTMRate = 950; //TSEN TM sending rate in # records to send
float Heater1Setpoint = 0.0;
float Heater2Setpoint = -20.0;
float DeadBand = .5;
int8_t chargerStatus = HIGH;
int32_t PreProfilePeriod = 120; //How long to take data docked before a profile
float FLASH_MinT = -25.0;  //Minimum operating temp or FLASH
int32_t PreProfileTMPeriod = 10; //number of records per TM in PreProfile
int32_t PreProfileDataRate = 10; //Seconds between data records in PreProfile
int32_t ProfileDownTime = 2000; //Seconds to collect data for on descent
int32_t ProfileUpTime = 2000; //Seconds to collect data for on ascent
int32_t ProfileDwellTime = 1800; //Seconds to collect data for at bottom dwell
int32_t ProfileMovingDataRate = 1; //Seconds per record while actively Profiling
int32_t ProfileDwellDataRate = 10; //Seconds per record while at bottom dwell

uint16_t LoRaStatusRate = 30;  //(seconds) the rate at which to send PU status strings when idle
uint8_t LoRaTxTM = true; //Switch to send profile data via LoRa
long LoRaStatTime = 0l; 

int TMRecordsToSend = 200; //The maximum number of TM records to send in one transaction 200*15*2 = 6kb
int TSENRecordsToSend = 750; //The maximum number of TSEN records to send in one transaction 750 * 5 *2 = 7.5kb
int TMRecordOffset = 0;  //pointer to where we are in the TM Record array for sending to PIB
int TSENRecordOffset = 0; //pointer to where we are in the TSEN Record array for sending to PIB

/* Profile Data Buffer */
uint16_t ProfileData[TM_BUFFER_LENGTH][TM_RECORD_LENGTH] = {0};  //150 KB
time_t GPSStartTime;  //Unix time_t of starttime of profile from PU clock
float GPSStartLat;  // Initial Latitude decimal degrees (float32) = 1m
float GPSStartLon; //Initial Longitude decimal degrees (float32) = 1m
uint16_t ZephyrAlt; //Alt from Zephyr in case PU GPS doesn't have lock
float ZephyrLat; //Lat from Zephyr in case PU GPS doesn't have lock
float ZephyrLon; //Lon from Zephyr in case PU GPS doesn't have lock
uint32_t ZephyrGPSTime; //GPS time from Zephyr
uint8_t bytes[sizeof(float)]; //byte array for converting floats to uint16_t
uint8_t TSEN_P1_bytes[4];
uint8_t TSEN_P2_bytes[4];
uint8_t HK_enum = 0;  //Enumeration index for HK data in TM packet
int TMRecordIndx = 0;

/* TSEN Globals */
uint16_t TSENindx = 0;
uint16_t TSENValues[4];
uint16_t TSENData[TSEN_BUFFER_LENGTH][TSEN_RECORD_LENGTH] = {0};// 18 KBytes 
uint32_t TSENDataStartTime = 0;

/*FLASH Variables, main data 16 bit unsigned ints*/
uint16_t FLASH_time;
uint16_t FLASH_fluor;
uint16_t FLASH_bkg;
float FLASH_intT;
float FLASH_pmtV;
float FLASH_lampI;
float FLASH_lampV;
float FLASH_lampT;
float FLASH_V;
float FLASH_T;


/*Flash reduced variables for debug */
float FLASH_float_T;
float FLASH_float_lampT;

/* OPC Variables */
uint16_t OPC_Time;
uint16_t OPC_300 = 0;
uint16_t OPC_500 = 0;
uint16_t OPC_700 = 0;
uint16_t OPC_1000 = 0;
uint16_t OPC_2000 = 0;
uint16_t OPC_3000 = 0;
uint16_t OPC_5000 = 0;
uint16_t OPC_10000 = 0;

/* HK Globals */
float HTR1_Therm;
float HTR2_Therm;
float PUMP_Therm;
float Spare_Therm;

float V_Battery;
float V_3v3;
float V_Input;
float V_Pump;
float I_Tsen;
float I_Flash;
float I_Opc;
float I_Charge;

int8_t FLASH_Power = true;
int8_t TSEN_Power = true;
int8_t ROPC_Power = true;

bool Charge_Status;
bool Heater1Status;
bool Heater2Status;

/* Initialize buffers for instrument data */
 String FLASH_Buff;
 String OPC_Buff;
 String GPS_Buff;
 String TSEN_Buff;
 String GONDOLA_Buff;
 String DEBUG_Buff;
 
 char NMEA_Buff[128];
 //uint8_t bin_rx[128] = {0}; 
 //uint8_t bin_tx[128] = {0};
 //uint16_t bin_tx_length = sizeof(bin_tx);

String TSENFileName;

char LoRa_Buff[255];

void setup() {
 FLASH_Buff.reserve(64);
 OPC_Buff.reserve(32);
 GPS_Buff.reserve(128);
 TSEN_Buff.reserve(32);
 GONDOLA_Buff.reserve(128);
 DEBUG_Buff.reserve(128);

  DEBUG_SERIAL.begin(115200);
  Serial1.begin(115200); //Docking Serial Port;
  delay(1000);

  /*Set Up the watchdog with a 60s timeout */
  int countdownMS = Watchdog.enable(60000);
  Serial.print("[nominal] Enabled the watchdog with max countdown of ");
  Serial.print(countdownMS, DEC);
  Serial.println(" milliseconds!");

  // set the Time library to use Teensy 3.6's RTC to keep time
  //setSyncProvider(getTeensy3Time);

  LoRa.setPins(SS_PIN, RESET_PIN,INTERUPT_PIN);
  LoRa.setSPI(SPI2);
    if (!LoRa.begin(BAND)){
      DEBUG_SERIAL.println("[warning] Starting LoRa failed!");
    }
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setTxPower(RF_POWER);

  DEBUG_SERIAL.print("[nominal] LoRa Radio Setup Complete for: ");
  DEBUG_SERIAL.print(BAND/1000000.0);
  DEBUG_SERIAL.print(" MHz SF: ");
  DEBUG_SERIAL.print(SF);
  DEBUG_SERIAL.print(" BW: ");
  DEBUG_SERIAL.println(BANDWIDTH);
  //LoRa.dumpRegisters(Serial);

  LoRa.beginPacket();
  LoRa.print("ST: Profiling Unit Initialized");
  if (LoRa.endPacket() == 1)
    DEBUG_SERIAL.println("[nominal] LoRa Init TM Sent");
  

  pinMode(PULSE_LED, OUTPUT);  //Initialize heart beat LED
  digitalWrite(PULSE_LED, HIGH);
  digitalWrite(CHARGER_SHUTDOWN, HIGH); // Turn off the charger

  PU.SetUp(); 
  PU.configure_memory_table();
  PU.ConfigureChannels();
  PU.configure_global_parameters();  

  digitalWrite(PUMP_PWR, HIGH); //turn off OPC pump
  digitalWrite(GPS_RESET, HIGH);
  digitalWrite(GPS_EXTINT, LOW);
  
  /* Put MAX3381 in auto power save mode */
  digitalWrite(FORCEOFF_232, HIGH);
  digitalWrite(FORCEON_232, LOW);
  
  /* Setup Serial ports */
  FLASH_SERIAL.begin(9600);
  TSEN_SERIAL.begin(9600);
  OPC_SERIAL.begin(9600);
  GPS_SERIAL.begin(9600);
  
  //pucomm.AssignBinaryRXBuffer(bin_rx, 128);
  //pucomm.AssignBinaryTXBuffer((uint8_t *) bin_tx, bin_tx_length);

  analogReference(EXTERNAL); //Set the reference to the external 3V ref
  analogReadResolution(12); // 1 bit = 0.000733 V
  analogReadAveraging(4); 

  /*Set up the UBLOX - note make sure it will work above 12km */
  byte settingsArray[] = {0x06, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00}; //
  PU.configureUblox(settingsArray); 
  
  if(!SD.begin()){
   Serial.println("[warning] SD card not inserted");
  }

  DEBUG_SERIAL.println("[nominal] Profiler set up complete!");
  //pucomm.TX_Error("Reboot - switching to idle \n");

  Mode = IDLE; //start off in Idle mode

}

void loop() {

switch(Mode){
    case LOWPOWER:
        lowPowerMode();
        break;
    case IDLE:
        idleMode();
        break;
    case WARMUP:
        warmUpMode();
        break;
    case PREPROFILE:
        PreProfile();
        break;
    case PROFILE:
        Profile();
        break;
    default:
        idleMode();
        break;
    }

}

bool lowPowerMode()
{
  /*
   * 1. Low Power Mode:  This is the lowest power mode, all instruments (including TSEN) 
   * are powered off, the Teensy is in low power mode, heaters are in survival mode set at survivalT.   
   * The PU will send a warning to the PIB when the battery voltage gets low (Vb < 11.0V) at 
   * which point the charger should be switched on.  This mode should only be used in an 
   * anomalous situation as it shuts down TSEN and risks a critical low Temperature/Battery.
   * During this mode, the PU sends house keeping once every 10s.
   */

  DEBUG_SERIAL.println("[nominal] Entering lowPowerMode");
  // Turn everything off
   digitalWrite(PUMP_PWR, HIGH); //turn off OPC pump
   digitalWrite(OPC_PWR,  LOW); //turn off OPC
   digitalWrite(FLASH_PWR,  LOW); //turn off FLASH
   digitalWrite(TSEN_PWR, LOW); //turn off TSEN

   //turn off the charger
   digitalWrite(CHARGER_SHUTDOWN, HIGH); 

  
  Heater2Setpoint = Heater1Setpoint - 5.0;  //set the second heater 5 degrees lower as a backup
  while(!checkForSerial())
   {
     Watchdog.reset();
     if (millis()%6000 < 1000)
     {
      scanAnalog();
      readTemps();
      setHeaters();
      //GONDOLA_SERIAL.printf("#67,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d\r\n",millis()/1000, HTR1_Therm, HTR2_Therm, PUMP_Therm, V_Battery, V_Input, I_Charge, Heater1Status, Heater2Status);
      DEBUG_SERIAL.printf("#LOWPOWER,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d\r\n", millis()/1000,HTR1_Therm, HTR2_Therm, PUMP_Therm, V_Battery, V_Input, I_Charge,Heater1Status, Heater2Status);
      delay(1000); //Wait a second to make sure we don't jump back into the loop 
     }
     delay(1);  
   }

}

void idleMode()
{
  /*
   * In this mode all instruments are off except TSEN, which provides a TM every TSENRate seconds.   
   * All heaters are in charge mode (T > 0C).   By default the battery charging circuit is on.
   * This is the standard mode to be used between profiles and during the day.
   */
   DEBUG_SERIAL.println("[nominal] Entering idleMode");
   
   // Turn everything except TSEN off
   digitalWrite(PUMP_PWR, HIGH); //turn off OPC pump
   digitalWrite(OPC_PWR,  LOW); //turn off OPC
   digitalWrite(FLASH_PWR, LOW); //turn off FLASH
   digitalWrite(TSEN_PWR, HIGH); //Set TSEN on over riding prior state

   //turn on the charger
   digitalWrite(CHARGER_SHUTDOWN, LOW); 

   long startTime = millis() + TSENDataRate*1000l; 
   
   

   LoRaStatTime= millis() + LoRaStatusRate*1000l;
   
   while(!checkForSerial())
   {
     Watchdog.reset();
     if (millis() > startTime)
     {
      startTime = millis() + TSENDataRate*1000l;
      scanAnalog();
      readTemps();
      setHeaters();
      TSENindx = getTSEN(TSENindx);
      //pucomm.TX_Status(now(), V_Battery, I_Charge, HTR1_Therm, HTR2_Therm, Heater1Status + Heater2Status*2);
      //GONDOLA_SERIAL.printf("#67,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d\r\n",millis()/1000, HTR1_Therm, HTR2_Therm, PUMP_Therm, V_Battery, V_Input, I_Charge, Heater1Status, Heater2Status,TSENData[1][TSENindx]);
      DEBUG_SERIAL.printf("#IDLE,%ld,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%d,%d,%d,%d,%d,%5.2f\r\n", now(),HTR1_Therm, HTR2_Therm, PUMP_Therm, V_Battery, V_Input, I_Charge,Heater1Status, Heater2Status,TSENData[TSENindx-1][0],TSENData[TSENindx-1][1],TSENData[TSENindx-1][2],TSENData[TSENindx-1][3],TSENData[TSENindx-1][4],gps.satellites.value(),gps.altitude.meters());
      updateRTCfromGPS();  //Update the RTC is necessary
     }
     if ((millis() > LoRaStatTime) && (LoRaStatusRate != 0))
     {
       LoRaStatTime = millis() + LoRaStatusRate*1000l;
       int chars_written = sprintf(LoRa_Buff,"ST,ID,%d,%d,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d",millis()/1000, int(gps.altitude.meters()),gps.location.age(),HTR1_Therm, HTR2_Therm, PUMP_Therm, V_Battery, V_Input, I_Charge, Heater1Status, Heater2Status );
       
       for (int i = 0; i<chars_written; i++)
          DEBUG_SERIAL.print(LoRa_Buff[i]);
       DEBUG_SERIAL.println();
       LoRa.beginPacket();
       LoRa.write(LoRa_Buff,chars_written);
       LoRa.endPacket(true); //send the LoRa packet without blocking
     }
     delay(1);  
   }
      
}

bool warmUpMode()
{
  TSENindx = 0;

  DEBUG_SERIAL.println("[nominal] Entering Warm Up Mode");

  long LoRaStatTime = 0l; 
  if(LoRaStatusRate == 0)
    LoRaStatTime = millis() - 1; //never execute 
  else{
    LoRa.beginPacket();
    LoRa.print("ST: Profiler Entering WarmUp");
    LoRa.endPacket(true); //send the LoRa packet without blocking
    LoRaStatTime= millis() + LoRaStatusRate*1000l; //set startime for LoRa Status
  }

  if (!FLASH_Power) //if FLASH is off set the temperature to be above the setpoint so we don't wait for it.
    FLASH_float_T = 99.9;

  if (FLASH_Power)
    digitalWrite(FLASH_PWR,  HIGH); //turn on FLASH
    
  if (TSEN_Power)
    digitalWrite(TSEN_PWR,  HIGH); //turn on TSEN

  if  (!TSEN_Power)
    digitalWrite(TSEN_PWR,  LOW); //turn off TSEN

  long startTime = millis() + TSENDataRate*1000l; 
  
  while((FLASH_float_T < FLASH_T) || (HTR1_Therm < Heater1Setpoint) || (HTR2_Therm < Heater2Setpoint) )
   {
     
     if(checkForSerial())
        return false;

     Watchdog.reset();
     if (millis() > startTime)
     {
      startTime = millis() + TSENDataRate*1000l;
      scanAnalog();
      readTemps();
      setHeaters();
      TSENindx = getTSEN(TSENindx);
      //GONDOLA_SERIAL.printf("#68,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f\r\n",millis()/1000, HTR1_Therm, HTR2_Therm, V_Battery, V_Input, I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[TSENindx][1], FLASH_float_T, FLASH_float_lampT );
      DEBUG_SERIAL.printf("#WARM_UP,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f\r\n", millis()/1000,HTR1_Therm, HTR2_Therm, V_Battery, V_Input, I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[TSENindx][1], FLASH_float_T, FLASH_float_lampT);
     }
     if ((millis() > LoRaStatTime))
     {
       LoRaStatTime = millis() + LoRaStatusRate*1000l;
       int chars_written = sprintf(LoRa_Buff,"ST,WM,%d,%d,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d",millis()/1000, int(gps.altitude.meters()),gps.location.age(),HTR1_Therm, HTR2_Therm, PUMP_Therm, V_Battery, V_Input, I_Charge, Heater1Status, Heater2Status );
       
       for (int i = 0; i<chars_written; i++)
          DEBUG_SERIAL.print(LoRa_Buff[i]);
       DEBUG_SERIAL.println();
       LoRa.beginPacket();
       LoRa.write(LoRa_Buff,chars_written);
       LoRa.endPacket(true); //send the LoRa packet without blocking
     }
     delay(1);  
   }
  DEBUG_SERIAL.println("[nominal] Warm Up Mode temperature reached");
  Mode = IDLE;
  return true;
}

bool PreProfile() 
{
  TMRecordOffset = 0; //reset the TM Indicies before we start preprofile
  TMRecordIndx = 0; 
  TSENindx = 0;

  DEBUG_SERIAL.println("[nominal] Entering PreProfile");

  if (TSEN_Power)
    digitalWrite(TSEN_PWR,  HIGH); //turn on TSEN
  if  (!TSEN_Power)
    digitalWrite(TSEN_PWR,  LOW); //turn off TSEN
  
  if (ROPC_Power)
  {
    digitalWrite(OPC_PWR,  HIGH); //turn on TSEN
    digitalWrite(PUMP_PWR, LOW); //turn on OPC pump
  } 
  if  (!ROPC_Power)
  {
    digitalWrite(OPC_PWR, LOW); //turn off TSEN
    digitalWrite(PUMP_PWR, HIGH); //turn off OPC pump
  }  
  
  if (FLASH_Power)
    digitalWrite(FLASH_PWR,  HIGH); //turn on FLASH
  if  (!FLASH_Power)
    digitalWrite(FLASH_PWR,  LOW); //turn off FLASH

  uint32_t startTime = millis() + PreProfileDataRate*1000l; 
  uint32_t endTime = millis() + PreProfilePeriod*1000l;
  int dataRecords = 0;
  while(millis() < endTime)
   {
     if(checkForSerial())
        return false;
     
     Watchdog.reset();
     if (millis() > startTime)
     {
      startTime = millis() + PreProfileDataRate*1000l;
      scanAnalog();
      readTemps();
      setHeaters();
      TSENindx = getTSEN(TSENindx);
      //GONDOLA_SERIAL.printf("#69,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f,%d,%d,%d,%d,%5.2f\r\n",millis()/1000, HTR1_Therm, HTR2_Therm, V_Battery, V_Input,I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[1][TSENindx], FLASH_float_T, FLASH_float_lampT,FLASH_fluor, FLASH_bkg, OPC_300, OPC_500, PUMP_Therm );
      TMRecordIndx = SaveRecord(TMRecordIndx); //update data record
      DEBUG_SERIAL.printf(  "#PREPROFILE,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f,%d,%d,%d,%d,%5.2f\r\n",now(), HTR1_Therm, HTR2_Therm, V_Battery, V_Input,I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[1][TSENindx], FLASH_float_T, FLASH_float_lampT,FLASH_fluor, FLASH_bkg, OPC_300, OPC_500, PUMP_Therm );
 
     }
     delay(1);  
   }
  return true;
}


bool Profile() 
{
  if(LoRaStatusRate != 0)
  {
    LoRa.beginPacket();
    LoRa.print("ST: Profiler Starting Down Profile");
    LoRa.endPacket(true); //send the LoRa packet without blocking
  }
  
  TSENindx = 0;
  //GONDOLA_SERIAL.println("[Nominal] Entering Profile");
  DEBUG_SERIAL.println("[nominal] Entering Profile");

   //turn off the charger
   digitalWrite(CHARGER_SHUTDOWN, HIGH); 
  

  if (TSEN_Power)
  {
    DEBUG_SERIAL.println("[profile] turning on TSEN");
    digitalWrite(TSEN_PWR,  HIGH); //turn on TSEN
  }
 // else
 //   digitalWrite(TSEN_PWR,  LOW); //turn off TSEN
  
  if (ROPC_Power)
  {
    DEBUG_SERIAL.println("[profile] turning on ROPC and Pump");
    digitalWrite(OPC_PWR,  HIGH); //turn on TSEN
    digitalWrite(PUMP_PWR, LOW); //turn on OPC pump
  } 
//   else
//   {
//     digitalWrite(OPC_PWR, LOW); //turn off TSEN
//     digitalWrite(PUMP_PWR, HIGH); //turn off OPC pump
//   }  
  
  if (FLASH_Power)
  {
    DEBUG_SERIAL.println("[profile] turning on FLASHB");
    digitalWrite(FLASH_PWR,  HIGH); //turn on FLASH
    delay(1000); //wait for boot. 
    FLASH_SERIAL.write('A'); //put it in auto mode
  }
 // else
 //   digitalWrite(FLASH_PWR,  LOW); //turn off FLASH

  uint32_t startTime = millis() + ProfileMovingDataRate*1000l; 
  
 /* Down Profile */
  TMRecordOffset = 0; //reset the TM Indicies before we start a profile
  TMRecordIndx = 0; 
  TMRecordIndx = SaveRecord(TMRecordIndx); //writes the header (line 0) and increments record index
  int dataRecords = 0; //local counter for how many measurements we have made
  
  while(dataRecords*ProfileMovingDataRate < ProfileDownTime)
   {
     if(checkForSerial())
        return false;

     Watchdog.reset();
     if (millis() > startTime)
     {
      startTime = millis() + ProfileMovingDataRate*1000l;
      scanAnalog();
      readTemps();
      setHeaters();
      DEBUG_SERIAL.printf("#PROFILE_DOWN,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f,%d,%d,%d,%d,%5.2f,%d\r\n",now(), HTR1_Therm, HTR2_Therm, V_Battery, V_Input,I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[TSENindx-1][1], FLASH_float_T, FLASH_float_lampT,FLASH_fluor, FLASH_bkg, OPC_300, OPC_500, PUMP_Therm, TMRecordIndx );
     
      TSENindx = getTSEN(TSENindx);
      TMRecordIndx = SaveRecord(TMRecordIndx); //writes a data record to the TM array
      //GONDOLA_SERIAL.printf("#70,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f,%d,%d,%d,%d,%5.2f\r\n",millis()/1000, HTR1_Therm, HTR2_Therm, V_Battery, V_Input,I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[1][TSENindx], FLASH_float_T, FLASH_float_lampT,FLASH_fluor, FLASH_bkg, OPC_300, OPC_500, PUMP_Therm );
       dataRecords++;
     }
   }

  /*Dwell */
 startTime = millis() + ProfileDwellDataRate*1000l; 
 dataRecords = 0;
 if(LoRaStatusRate != 0)
  {
    LoRa.beginPacket();
    LoRa.print("ST: Profiler Starting Dwell");
    LoRa.endPacket(true); //send the LoRa packet without blocking
  }
  while(dataRecords*ProfileDwellDataRate < ProfileDwellTime)
   {
     if(checkForSerial())
        return false;

     Watchdog.reset();
     if (millis() > startTime)
     {
      startTime = millis() + ProfileDwellDataRate*1000l;
      scanAnalog();
      readTemps();
      setHeaters();
      
      //GONDOLA_SERIAL.printf("#71,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f,%d,%d,%d,%d,%5.2f\r\n",millis()/1000, HTR1_Therm, HTR2_Therm, V_Battery, V_Input,I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[1][TSENindx], FLASH_float_T, FLASH_float_lampT,FLASH_fluor, FLASH_bkg, OPC_300, OPC_500, PUMP_Therm );
      DEBUG_SERIAL.printf("#PROFILE_DWELL,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f,%d,%d,%d,%d,%5.2f,%d\r\n",now(), HTR1_Therm, HTR2_Therm, V_Battery, V_Input,I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[TSENindx-1][1], FLASH_float_T, FLASH_float_lampT,FLASH_fluor, FLASH_bkg, OPC_300, OPC_500, PUMP_Therm,TMRecordIndx );
     
      TSENindx = getTSEN(TSENindx);
      TMRecordIndx = SaveRecord(TMRecordIndx); //writes a data record to the TM array
       dataRecords++;
     }
     delay(1);  
   }

   /*Up Profile */
  startTime = millis() + ProfileMovingDataRate*1000l; 
  dataRecords = 0;
  if(LoRaStatusRate != 0)
  {
    LoRa.beginPacket();
    LoRa.print("ST: Profiler Starting Up Profile");
    LoRa.endPacket(true); //send the LoRa packet without blocking
  }
  while(dataRecords*ProfileMovingDataRate < ProfileUpTime)
   {
     if(checkForSerial())
        return false;
     Watchdog.reset();
     if (millis() > startTime)
     {
      startTime = millis() + ProfileMovingDataRate*1000l;
      scanAnalog();
      readTemps();
      setHeaters();
       DEBUG_SERIAL.printf("#PROFILE_UP,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f,%d,%d,%d,%d,%5.2f,%d\r\n",now(), HTR1_Therm, HTR2_Therm, V_Battery, V_Input,I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[TSENindx-1][1], FLASH_float_T, FLASH_float_lampT,FLASH_fluor, FLASH_bkg, OPC_300, OPC_500, PUMP_Therm,TMRecordIndx );
     
      //GONDOLA_SERIAL.printf("#72,%d,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d,%5.2f,%5.2f,%d,%d,%d,%d,%5.2f\r\n",millis()/1000, HTR1_Therm, HTR2_Therm, V_Battery, V_Input,I_Charge, I_Flash, Heater1Status, Heater2Status,TSENData[TSENindx][1], FLASH_float_T, FLASH_float_lampT,FLASH_fluor, FLASH_bkg, OPC_300, OPC_500, PUMP_Therm );
      TSENindx = getTSEN(TSENindx);
      TMRecordIndx = SaveRecord(TMRecordIndx); //writes a data record to the TM array
      dataRecords++;
     }
   }
 writeProfileFile(TMRecordIndx);
 TSENindx = 0; //reset the TSEN indx so that we start a new TSEN run after a profile.
 Mode = IDLE;  
 return true;
}

int SaveRecord(int dataindx)
{
    if (dataindx >= TM_BUFFER_LENGTH)
        dataindx = 1;
    
    if(dataindx == 0) //first line of array is header line
    {
        GPSStartTime = now(); //get time as time_t from Teensy clock (4 bytes)
        ProfileData[0][0] = (uint16_t) (GPSStartTime);
        ProfileData[0][1] = (uint16_t) (GPSStartTime >> 16);  
        

        GPSStartLat = gps.location.lat();
        float single = (float)GPSStartLat;  //convert double to single float
        *(float*)(bytes) = single;  // convert float to bytes
        ProfileData[0][2] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
        ProfileData[0][3] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)
        
        
        GPSStartLon = gps.location.lng();
        single = (float)GPSStartLon;  //convert double to single float
        *(float*)(bytes) = single;  // convert float to bytes
        ProfileData[0][4] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
        ProfileData[0][5] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)
        
        
        ProfileData[0][6] = (uint16_t) (ZephyrAlt); //Altitude in meters
        
        *(float*)(bytes) = ZephyrLat;  // convert float to bytes
        ProfileData[0][7] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
        ProfileData[0][8] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)
        

        *(float*)(bytes) = ZephyrLon;  // convert float to bytes
        ProfileData[0][9] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
        ProfileData[0][10] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)
        
        ProfileData[0][11] = (uint16_t)(V_3v3*1000);  //Voltage in mV
        ProfileData[0][12] = (uint16_t)(V_Input*1000);  //Voltage in mV
        ProfileData[0][13] = (uint16_t)(I_Charge*1000);  //Current in mA
        ProfileData[0][14] = 0xFFFF;  //Currently unused
    }

    else{
        ProfileData[dataindx][0] = (uint16_t)(now() - GPSStartTime); // Elapsed time in seconds
        ProfileData[dataindx][1] = (int16_t)((gps.location.lat() - GPSStartLat)*50000.0); //difference in lat 0.00005 degrees
        ProfileData[dataindx][2] = (int16_t)((gps.location.lng() - GPSStartLon)*50000.0); //difference in lon 0.00005 degrees
        ProfileData[dataindx][3] = (uint16_t)(gps.altitude.meters());  //Altitude in meters
        ProfileData[dataindx][4] = (uint16_t)(FLASH_fluor);  //Flash-B FLouresence signal
        ProfileData[dataindx][5] = (uint16_t)(FLASH_bkg);  //Flash-B Background signal
        ProfileData[dataindx][6] = (uint16_t)(OPC_300); OPC_300 = 0; //ROPC 300nm Channel
        ProfileData[dataindx][7] = (uint16_t)(TSENData[TSENindx-1][1]);  //TSEN variable 1
        ProfileData[dataindx][8] = (uint16_t)(TSENData[TSENindx-1][2]);  //TSEN variable 2
        ProfileData[dataindx][9] = (uint16_t)(TSENData[TSENindx-1][3]);  //TSEN variable 3
        ProfileData[dataindx][10] = (uint16_t)(TSENData[TSENindx-1][4]);  //TSEN variable 3
        ProfileData[dataindx][11] = (uint16_t)((HK_enum << 8) + Heater2Status*2 + Heater1Status); //Enum index and heater on/off bits
        switch (HK_enum){
            case 0:
                ProfileData[dataindx][12] =(uint16_t)((FLASH_float_T + 273.15) * 100); //Flash internal T in K at 0.01C precision
                ProfileData[dataindx][13] = (uint16_t)(OPC_500); OPC_500 = 0; //ROPC 500nm Channel
                ProfileData[dataindx][14] = (uint16_t)(V_Battery*1000); //Battery V in mV
                break;
            case 1:
                ProfileData[dataindx][12] =(uint16_t)(FLASH_pmtV * 10); //Flash PMT Voltage in mV
                ProfileData[dataindx][13] = (uint16_t)(OPC_700); OPC_700 = 0;//ROPC 700nm Channel
                ProfileData[dataindx][14] = (uint16_t)(V_Pump*1000); //Pump V in mV
                break;
            case 2:
                ProfileData[dataindx][12] = (uint16_t)(FLASH_lampI * 10); //Flash lamp current in mA
                ProfileData[dataindx][13] = (uint16_t)(OPC_1000); OPC_1000 = 0; //ROPC 1000nm Channel
                ProfileData[dataindx][14] = (uint16_t)(I_Tsen*1000); //TSEN Current in mA
                break;
            case 3:
                ProfileData[dataindx][12] = (uint16_t)(FLASH_lampV * 10); //Flash lamp voltage in mV
                ProfileData[dataindx][13] = (uint16_t)(OPC_2000); OPC_2000 = 0;//ROPC 2000nm Channel
                ProfileData[dataindx][14] = (uint16_t)(I_Flash*1000); //FLASH Current in mA
                break;
            case 4:
                ProfileData[dataindx][12] = (uint16_t)((FLASH_float_lampT + 273.15) * 100); //Flash lamp T in K at 0.01C precision
                ProfileData[dataindx][13] = (uint16_t)(OPC_3000); OPC_3000 = 0;//ROPC 3000nm Channel
                ProfileData[dataindx][14] = (uint16_t)(I_Opc*1000); //ROPC Current in mA
                break;
            case 5:
                ProfileData[dataindx][12] = (uint16_t)(FLASH_V * 1000); //Flash Voltage in mV
                ProfileData[dataindx][13] = (uint16_t)(OPC_5000); OPC_5000 = 0; //ROPC 5000nm Channel
                ProfileData[dataindx][14] = (uint16_t)((PUMP_Therm + 273.15)*100); //ROPC Pump T in K at 0.01C precision
                break;
            case 6:
                ProfileData[dataindx][12] = (uint16_t)((FLASH_T + 273.15) * 100); //Flash T in K at 0.01C precision
                ProfileData[dataindx][13] = (uint16_t)(OPC_10000); OPC_10000 = 0; //ROPC 10000nm Channel
                ProfileData[dataindx][14] = (uint16_t)((HTR1_Therm + 273.15)*100); //Heater 1 T in K at 0.01C precision
                break;
            case 7:
                ProfileData[dataindx][12] = (uint16_t) 0; //Currently unused
                ProfileData[dataindx][13] = (uint16_t) 0; //Currently unused
                ProfileData[dataindx][14] = (uint16_t)((HTR2_Therm + 273.15)*100); //Heater 2 T in K at 0.01C precision
                break;
        }
        HK_enum++;
        if (HK_enum > 7)
            HK_enum = 0;

      if ((dataindx%5 == 0) && (LoRaTxTM != 0)) // Every 5th line (except the first line, 0) (150 bytes) send the data via LoRa
      {
        //DEBUG_SERIAL.println("Sending LoRa Profile Packet");
        long lrst = millis();
        
        LoRa.beginPacket();
        LoRa.print("TM");
        LoRa.write((uint8_t *) ProfileData + ((dataindx - 5) * TM_RECORD_LENGTH * 2),5 * TM_RECORD_LENGTH * 2);
        LoRa.endPacket(); //send the LoRa packet without blocking
        Serial.printf("LoRa TM took %ul ms to send\n", millis() - lrst);
      }
    }
    
    
    dataindx++;
    return dataindx;
}

int getTSEN(int dataIndex)
{
  String header = "#";
  char tsen_buff[32];
  char * pEnd;
  uint32_t TSEN_P1 = 0;
  uint32_t TSEN_P2 = 0;
  

  if (dataIndex >= TSEN_BUFFER_LENGTH)
  {
        writeTSENFile(dataIndex);
        dataIndex = 0;
  }
        

  if (dataIndex == 0) //if this is the first sample in the TM packet write the header
    {
        TSENDataStartTime = now();
        TSENData[0][0] = (uint16_t) (TSENDataStartTime);
        TSENData[0][1] = (uint16_t) (TSENDataStartTime >> 16);  
        TSENData[0][2] = (uint16_t) (I_Tsen * 1000); //TSEN Current
        TSENData[0][3] = (uint16_t) ((HTR1_Therm + 273.15)*100);  //Temperature near TSEN
        TSENData[0][4] = (uint16_t)(V_Battery*1000); //Battery V in mV

        dataIndex++;
    }

  
  TSEN_SERIAL.print("*01A?");
  TSEN_SERIAL.write('\r');
  //delay(1); //make sure there is time for TSEN to respond
   
   while (TSEN_SERIAL.available()) {
   char TSEN_Char = TSEN_SERIAL.read();
   if(TSEN_Char == '\r') //got a EOL from TSEN
   {
      if ( TSEN_Buff.startsWith("#")) //String starts with the right char
      {
        
        //DEBUG_SERIAL.println(TSEN_Buff);
        if ( dataIndex > TSEN_BUFFER_LENGTH)
            dataIndex = 0;
    
        TSENData[dataIndex][0] = (uint16_t)(now() - TSENDataStartTime);
        TSEN_Buff.toCharArray(tsen_buff,32);
        TSENData[dataIndex][1] = (uint16_t)strtoul(tsen_buff+1,&pEnd,16); 
        TSEN_P1 = (uint32_t)strtoul(pEnd,&pEnd,16); //three bytes for P1
        TSEN_P2 = (uint32_t)strtoul(pEnd,NULL,16); //three bytes for P2
        
        *(uint32_t*)(TSEN_P1_bytes) = TSEN_P1;  //uint32_t to bytes
        *(uint32_t*)(TSEN_P2_bytes) = TSEN_P2;  //uint32_t to bytes

        TSENData[dataIndex][2] = (uint16_t)((TSEN_P1_bytes[1] << 8) + TSEN_P1_bytes[0]); //two LSB for TSEN P1
        TSENData[dataIndex][3] = (uint16_t)((TSEN_P2_bytes[1] << 8) + TSEN_P2_bytes[0]);
        TSENData[dataIndex][4] = (uint16_t)((TSEN_P1_bytes[2] << 8) + TSEN_P2_bytes[2]);
        //DEBUG_SERIAL.printf("TSEN2: %X, TSEN3: %X, Array2: %X, Array3: %X, Array4: %X\n",TSEN_P1_bytes[2],TSEN_P2_bytes[2], TSENData[dataIndex][2],TSENData[dataIndex][3],TSENData[dataIndex][4] );
        //DEBUG_SERIAL.printf("TSEN: %d,%d,%d,%d\n", TSENData[0][dataIndex],TSENData[1][dataIndex],TSENData[2][dataIndex],TSENData[3][dataIndex]);
        dataIndex++;
      }
    TSEN_Buff = "";
    return dataIndex;   
    
   } else
   {
    TSEN_Buff += TSEN_Char;
   }   
  }

  return dataIndex;
}

bool sendTM(void)
{
 /*
 * The PIB requests TM records
 * The PU sends 250 records from the array
 * If the record is acked by the PIB then increment pointer to next record
 * If the record is nacked then don't increment pointer and wait for next requests
 * Send a message with message id 'no more records'
 */
 pucomm.ack_value = false; //reset the ack_value to false

 // If there are no more records to send, send a 'no more records' msg
 if(TMRecordOffset > TMRecordIndx)
 { 
   DEBUG_SERIAL.println("[nominal] Profile Records requested but none to send, reset pointers");
   pucomm.TX_ASCII(PU_NO_MORE_RECORDS);
   writeProfileFile(TMRecordIndx);
   TMRecordOffset = 0;
   TMRecordIndx = 0;
   return true;
 } 
 //check to make sure we have enough records to send and send a reduced number if near the end
 else if((TMRecordOffset + TMRecordsToSend) > TMRecordIndx )
 { 
    int remainingRecords = TMRecordIndx - TMRecordOffset;
    DEBUG_SERIAL.printf("[nominal]Sending %d Remaining Profile Records, pointer at: %d\n",remainingRecords, ProfileData + TMRecordOffset );
    pucomm.AssignBinaryTXBuffer((uint8_t *) ProfileData + TMRecordOffset * TM_RECORD_LENGTH * 2, remainingRecords * TM_RECORD_LENGTH * 2, remainingRecords * TM_RECORD_LENGTH * 2);
    pucomm.TX_Bin(PU_PROFILE_RECORD);
 } 
 //send the full number (200) records if available
 else{
    DEBUG_SERIAL.printf("[nominal]Sending %d Profile Records, pointer at: %d \n",TMRecordsToSend, ProfileData + TMRecordOffset);
    pucomm.AssignBinaryTXBuffer((uint8_t *) ProfileData + TMRecordOffset * TM_RECORD_LENGTH * 2, TMRecordsToSend * TM_RECORD_LENGTH * 2,TMRecordsToSend * TM_RECORD_LENGTH *2);
    pucomm.TX_Bin(PU_PROFILE_RECORD);
 }

 uint32_t TimeOut = millis() + PUCOMM_ACK_TIMEOUT; //timeout for an ACK
 while(millis() < TimeOut)
    {
      if(checkForSerial())
            return false;

      if(pucomm.ack_value == true)
        {
            
            TMRecordOffset += TMRecordsToSend;
            DEBUG_SERIAL.printf("[nominal] TSEN Data ACK'd, Advance offset to: %d", TMRecordOffset);
            pucomm.ack_value = false; //reset the ack_value to false
            return true;
        }
    }
    
    DEBUG_SERIAL.printf("[warning] profile records not acked, index not advanced\n");
    return false;
}

bool sendTSEN(void)
{
 /*
 * The PIB requests TSEN records and should do this ~ every 15 minutes
 * The PU sends upto 950 records from the array (there should only be 900 to send)
 * If the record is acked by the PIB then increment pointer to next record
 * If the record is nacked then don't increment pointer and wait for next requests
 * Send a message with message id 'no more records'
 */
 pucomm.ack_value = false; //reset the ack_value to false

 // If there are no more records to send, send a 'no more records' msg
 if(TSENRecordOffset > TSENindx)
 { 
   DEBUG_SERIAL.println("[nominal] TSEN Records requested but none to send, reset pointers");
   pucomm.TX_ASCII(PU_NO_MORE_RECORDS);
   writeTSENFile(TSENindx);
   TSENRecordOffset = 0;
   TSENindx = 0;
   return true;
 } 
 //check to make sure we have enough records to send and send a reduced number if near the end
 else if((TSENRecordOffset + TSENRecordsToSend) > TSENindx)
 { 
    int remainingRecords = TSENindx - TSENRecordOffset;
    DEBUG_SERIAL.printf("[nominal]Sending %d Remaining TSEN Records, TSENOffset: %d, TSENindx: %d\n",remainingRecords, TSENRecordOffset, TSENindx);
    pucomm.AssignBinaryTXBuffer((uint8_t *) TSENData + TSENRecordOffset * TSEN_RECORD_LENGTH * 2, remainingRecords * TSEN_RECORD_LENGTH * 2, remainingRecords * TSEN_RECORD_LENGTH * 2);
    pucomm.TX_Bin(PU_TSEN_RECORD);
 } 
 //send the full number (200) records if available
 else{
    DEBUG_SERIAL.printf("[nominal]Sending %d TSEN Records\n",TSENRecordsToSend);
    pucomm.AssignBinaryTXBuffer((uint8_t *) TSENData + TSENRecordOffset * TSEN_RECORD_LENGTH * 2, TSENRecordsToSend * TSEN_RECORD_LENGTH * 2,TSENRecordsToSend * TSEN_RECORD_LENGTH *2);
    pucomm.TX_Bin(PU_TSEN_RECORD);
 }

 uint32_t TimeOut = millis() + PUCOMM_ACK_TIMEOUT; //six second timeout for an ACK
 while(millis() < TimeOut)
    {
      SerComRX();
      delay(200);

      if(pucomm.ack_value == true)
        {
            TSENRecordOffset += TSENRecordsToSend;
            DEBUG_SERIAL.print("[nominal] TSEN Data ACK'd, Advance index to: "); DEBUG_SERIAL.println(TSENRecordOffset);
            pucomm.ack_value = false;
            return true;
        }
    }
    
    DEBUG_SERIAL.printf("[warning] TSEN records not acked, index not advanced\n");
    return false;
}


void readTemps()
{
  HTR1_Therm = PU.MeasureLTC2983(3); //OPC side of PU
  if (HTR1_Therm < -100)
    PU.ResetLTC2983();
  HTR2_Therm = PU.MeasureLTC2983(5); // Main board side of PU
  if (HTR2_Therm < -100)
    PU.ResetLTC2983();
  PUMP_Therm = PU.MeasureLTC2983(7);  // OPC Pump
  if(PUMP_Therm < -100)
    PU.ResetLTC2983();
  //Spare_Therm = PU.MeasureLTC2983(9); 
}

int setHeaters()
{
  /* Thermostat for both heaters, bases on global variables
   *  Heater1Setpoint, Heater2Setpoint and DeadBand
   *  Returns true if either heater is on
   */

  if (HTR1_Therm < -900 && HTR2_Therm < -900) //if both thermistors fail go to 50% duty cycle
  {
      digitalWrite(HEATER1_PWR, !digitalRead(HEATER1_PWR)); 
  }
   
  if (HTR1_Therm > -990) //if heater 1 therm is working, set heater 1
  {
    if(HTR1_Therm < Heater1Setpoint)
    {
        digitalWrite(HEATER1_PWR, HIGH);
        Heater1Status = true;
    }

    if(HTR1_Therm > (Heater1Setpoint + DeadBand))
    {
        digitalWrite(HEATER1_PWR, LOW);
        Heater1Status = false;
    }
  }

  if (HTR2_Therm > -990) //if heater 2 therm is working, set heater 2
  {
    if(HTR2_Therm < Heater2Setpoint)
    {
        digitalWrite(HEATER2_PWR, HIGH);
        Heater2Status = true;
    }

    if(HTR2_Therm > (Heater2Setpoint + DeadBand))
    {
        digitalWrite(HEATER2_PWR, LOW);
        Heater2Status = false;
    }
  }
  if (Heater1Status || Heater2Status)
    return true;
  else
    return false;
}

void scanAnalog()
{
  V_Battery = analogRead(V_BATTERY)* 0.00073 * 5.016;
  V_3v3 = analogRead(V_3V3) * 0.00073 * 2.0;
  V_Input = analogRead(V_IN) * 0.00073 * 8.299;
  V_Pump = analogRead(V_PUMP) * 0.00073 * 5.016;

  I_Charge = analogRead(I_CHARGE) * 0.00073 / 2.262;
  I_Tsen = analogRead(I_TSEN) * 0.00073*1.097  -2.271; 
  I_Flash = analogRead(I_FLASH) * 0.00073*1.0708 - 2.56; 
  I_Opc = analogRead(I_OPC) * 0.00073*1.066 - 2.4638; 
}

bool checkForSerial()
{
  /* Checks for any new bytes on the various serial ports
   *  and processes them when necessary.  If we receive a mode change
   * from the PIB this function returns true and we should exit whatever
   * mode we are in and return to main the assume the new mode
   */


  if(SerComRX())
    return true;

  if(LoRa.parsePacket()) //check for commands via lora
    parseLoRa();

  Watchdog.reset(); 
  
  if (OPC_SERIAL.available()) {
   char OPC_Char = OPC_SERIAL.read();
   if(OPC_Char == '\r')
   {
    DEBUG_SERIAL.print("OPC String: ");
    DEBUG_SERIAL.println(OPC_Buff);
    parseOPC();
    OPC_Buff = "";
   } else
   {
    OPC_Buff += OPC_Char;
   }   
  }

if (FLASH_SERIAL.available()) {
   char FLASH_Char = FLASH_SERIAL.read();
   
   if(FLASH_Char == '\r')
   {
    parseFLASH();
    FLASH_Buff = "";
   } else
   {
    FLASH_Buff += FLASH_Char;
   }   
  }

  if (DEBUG_SERIAL.available()) {
   char DEBUG_Char = DEBUG_SERIAL.read();
   
   if((DEBUG_Char == '\n') || (DEBUG_Char == '\r'))
   {
    parseCommand(DEBUG_Buff);
    DEBUG_Buff = "";
   } else
   {
    DEBUG_Buff += DEBUG_Char;
   }   
  }

  if (GPS_SERIAL.available()) {
    char c = GPS_SERIAL.read();
    gps.encode(c);
    //DEBUG_SERIAL.write(c);
  }

  return false;

}

 bool SerComRX()
 {
     /* Recieves commands from the PIB and parses them.  
     *  If we receive an immediate change mode command then this 
     * returns true, which should cause the mode to return to main
     * and update to the new mode
     */
     int8_t tmp1;
     uint32_t tmp3;

     switch (pucomm.RX()) {
         case ASCII_MESSAGE:
            //Serial.print("Received message: "); Serial.println(pucomm.ascii_rx.msg_id);
            //DEBUG_SERIAL.print("Message Content: "); DEBUG_SERIAL.println(pucomm.ascii_rx.buffer);
            switch (pucomm.ascii_rx.msg_id){
                case PU_SEND_STATUS:
                    tmp3 = now();
                    tmp1 = pucomm.TX_Status(tmp3,V_Battery,I_Charge, HTR1_Therm, HTR2_Therm, Heater1Status + Heater2Status*2);
                    DEBUG_SERIAL.println("[nominal] Received PU_SEND_STATUS");
                    return false;
                case PU_SEND_PROFILE_RECORD:
                    DEBUG_SERIAL.println("[nominal] Received PU_SEND_PROFILE_RECORD");
                    sendTM();
                    return false;
                case PU_SEND_TSEN_RECORD:
                    DEBUG_SERIAL.println("[nominal] Received PU_SEND_TSEN_RECORD");
                    sendTSEN();
                    return false;
                case PU_RESET:
                    pucomm.TX_Ack(PU_RESET,true);
                    DEBUG_SERIAL.println("[warning] Rebooting in 2 seconds");
                    delay(2000);
                    WRITE_RESTART(0x5FA0004);
                    return false;
                case PU_SET_HEATERS:
                    tmp1 = pucomm.RX_SetHeaters(&Heater1Setpoint, &Heater2Setpoint);
                    pucomm.TX_Ack(PU_SET_HEATERS,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.println("[nominal] Received PU_SET_HEATERS");
                        DEBUG_SERIAL.println(Heater1Setpoint); DEBUG_SERIAL.println(Heater2Setpoint);
                    }
                    return false;
                case PU_GO_LOWPOWER:
                    tmp1 = pucomm.RX_LowPower(&Heater1Setpoint);
                    pucomm.TX_Ack(PU_GO_LOWPOWER,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.println("[nominal] Received PU_GO_LOWPOWER");
                        Mode = LOWPOWER;
                        return true;
                    }
                case PU_GO_IDLE:
                    tmp1 = pucomm.RX_Idle(&TSENTMRate); 
                    pucomm.TX_Ack(PU_GO_IDLE,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.println("[nominal] Received PU_GO_IDLE");
                        Mode = IDLE;
                        return true;
                    }
                case PU_GO_WARMUP:
                    tmp1 = pucomm.RX_WarmUp(&FLASH_MinT, &Heater1Setpoint, &Heater2Setpoint, &FLASH_Power, &TSEN_Power);
                    pucomm.TX_Ack(PU_GO_WARMUP,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.println("[nominal] Received PU_GO_WARMUP");
                        Mode = WARMUP; 
                        return true;
                        }
                case PU_GO_PREPROFILE:
                    tmp1 = pucomm.RX_PreProfile(&PreProfilePeriod, &PreProfileTMPeriod, &PreProfileDataRate, &TSEN_Power, &ROPC_Power, &FLASH_Power);
                    pucomm.TX_Ack(PU_GO_PREPROFILE,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.println("[nominal] Received PU_GO_PREPROFILE");
                        Mode = PREPROFILE;
                        return true;
                    }
                case PU_GO_PROFILE:
                    if(pucomm.ascii_rx.checksum_valid)
                      DEBUG_SERIAL.println("[nominal] Recevied Valid Profile Checksum");

                    tmp1 = pucomm.RX_Profile(&ProfileDownTime, &ProfileDwellTime, &ProfileUpTime, &ProfileMovingDataRate, &ProfileDwellDataRate, &TSEN_Power, &ROPC_Power, &FLASH_Power, &LoRaTxTM);
                    pucomm.TX_Ack(PU_GO_PROFILE,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.print("[nominal] Received PU_GO_PROFILE, with parameters: ");
                        DEBUG_SERIAL.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",ProfileDownTime, ProfileDwellTime, ProfileUpTime, ProfileMovingDataRate, ProfileDwellDataRate, TSEN_Power, ROPC_Power, FLASH_Power, LoRaTxTM);
                        Mode = PROFILE;
                        return true;
                    }
                case PU_UPDATE_GPS:
                    tmp1 = pucomm.RX_UpdateGPS(&ZephyrGPSTime, &ZephyrLat, &ZephyrLon, &ZephyrAlt);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.println("[nominal] Received PU_UPDATE_GPS");
                        return false;
                    }
                case PU_LORA_STATUS:
                    tmp1 = pucomm.RX_PULoRaStatus(&LoRaStatusRate);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.print("[nominal] Received PU_LORA_STATUS, update rate to: ");
                        DEBUG_SERIAL.println(LoRaStatusRate);
                        LoRaStatTime = millis() + LoRaStatusRate*1000;
                        return false;
                    }
                default:
                    pucomm.TX_Ack(pucomm.ascii_rx.msg_id,false);
                    return false;   
            }
            case ACK_MESSAGE:
                DEBUG_SERIAL.print("[nominal] ACK/NAK for msg: "); DEBUG_SERIAL.println(pucomm.ack_id);
                Serial.print("[nominal] Value: ");
                pucomm.ack_value ? DEBUG_SERIAL.println("ACK") : DEBUG_SERIAL.println("NAK");
                return false;
            case NO_MESSAGE:
            default:
                return false;                    

        }
 }


bool parseFLASH()
{
  String header = "xdata=3D010";
  
  int temp_int;
  String sub;
  char subChar[5];
  

  if ( FLASH_Buff.startsWith(header))
  {
    sub = FLASH_Buff.substring(11,15);
    sub.toCharArray(subChar,5); 
    FLASH_time = (uint16_t)strtol(subChar,NULL,16); //Frames from power on
    DEBUG_SERIAL.print("Flash Time: ");
    DEBUG_SERIAL.print(FLASH_time);
    
    sub = FLASH_Buff.substring(15,19);
    sub.toCharArray(subChar,5); 
    FLASH_fluor = (uint16_t)strtol(subChar,NULL,16); //fluorescence signal count rate (S) in counts
    DEBUG_SERIAL.print(" Fluor: ");
    DEBUG_SERIAL.print( FLASH_fluor);

    sub = FLASH_Buff.substring(19,23);
    sub.toCharArray(subChar,5); 
    FLASH_bkg = (uint16_t)strtol(subChar,NULL,16); //background signal count rate in counts
    DEBUG_SERIAL.print(" bkg: ");
    DEBUG_SERIAL.print(FLASH_bkg);

    sub = FLASH_Buff.substring(23,27);
    sub.toCharArray(subChar,5); 
    temp_int = strtol(subChar,NULL,16);
    FLASH_intT = (uint16_t)temp_int;
    //‘Internal temperature’: Temperature of PMT unit (tpmt), degrees Celsius Tpmt =-21.103*LN(tpmt *0.00061*30/(4096*0.00061-(tpmt *0.00061))) + 97.106
    FLASH_float_T = -21.103*log(double(temp_int) *0.00061*30/(4096*0.00061-(double(temp_int)*0.00061))) + 97.106;
    DEBUG_SERIAL.print(" PMT T: ");
    DEBUG_SERIAL.print(FLASH_float_T);

    sub = FLASH_Buff.substring(27,31);
    sub.toCharArray(subChar,5); 
    temp_int = strtol(subChar,NULL,16);
    FLASH_pmtV = float(temp_int)*0.305; //‘PMT voltage’: photomultiplier voltage (Upmt), Volts Upmt= Upmt *0.305
    DEBUG_SERIAL.print(" PMT V: ");
    DEBUG_SERIAL.print(FLASH_pmtV);

    sub = FLASH_Buff.substring(31,35);
    sub.toCharArray(subChar,5); 
    temp_int = strtol(subChar,NULL,16);
    FLASH_lampI = float(temp_int)*0.0061; //‘Lamp current’: current of VUV lamp (Ilamp), mA. Ilamp= Ilamp *0.0061
    DEBUG_SERIAL.print(" Lamp I: ");
    DEBUG_SERIAL.print(FLASH_lampI);
    
    sub = FLASH_Buff.substring(35,39);
    sub.toCharArray(subChar,5); 
    temp_int = strtol(subChar,NULL,16);
    FLASH_lampV = float(temp_int)*0.123;
    DEBUG_SERIAL.print(" Lamp V: "); //‘Lamp current’: current of VUV lamp (Ilamp), mA. Ilamp= Ilamp *0.0061
    DEBUG_SERIAL.print(FLASH_lampV);

    sub = FLASH_Buff.substring(39,43);
    sub.toCharArray(subChar,5); 
    temp_int = strtol(subChar,NULL,16);
    FLASH_lampT = (uint16_t) temp_int; //‘Lamp temperature’: Temperature of VUV lamp (Tlamp), degrees Celsius Tlamp =-21.103*LN(Tlamp *0.00061*30/(4096*0.00061-( Tlamp *0.00061))) + 97.106
    FLASH_float_lampT = -21.103*log(double(temp_int) *0.00061*30/(4096*0.00061-(double(temp_int)*0.00061))) + 97.106;
    DEBUG_SERIAL.print(" Lamp T: ");
    DEBUG_SERIAL.print(FLASH_float_lampT);

    sub = FLASH_Buff.substring(43,47);
    sub.toCharArray(subChar,5); 
    temp_int = strtol(subChar,NULL,16);
    FLASH_V = float(temp_int)*0.003477;  //Supply voltage’: Supply voltage (Ubat), volts, Ubat= Ubat *0.003477
    DEBUG_SERIAL.print(" In V: ");
    DEBUG_SERIAL.print(FLASH_V);

    sub = FLASH_Buff.substring(47,51);
    sub.toCharArray(subChar,5); 
    temp_int = strtol(subChar,NULL,16);  //'Controller temperature' Temperature of microcontroller (Tmc), degrees Celsius Tmc =(Tmc *0.00061-0.78)/-0.0013+25
    FLASH_T = (float(temp_int) *0.00061-0.78)/-0.0013+25.0;
    DEBUG_SERIAL.print(" uC T: ");
    DEBUG_SERIAL.println(FLASH_T);

    return true;
  }

  return false;
}

void parseOPC() {  
  char * strtokIndx; // this is used by strtok() as an index
  char OPCArray[64];
  //DEBUG_SERIAL.println(OPC_Buff);
  
  OPC_Buff.toCharArray(OPCArray, 64);
  strtokIndx = strtok(OPCArray,",");      // get the first part - the timestamp
  OPC_Time = atoi(strtokIndx); 
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_300 += atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_500 += atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_700 += atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_1000 += atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_2000 += atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_3000 += atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_5000 += atoi(strtokIndx);     // convert this part to an integer
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  OPC_10000 += atoi(strtokIndx);     // convert this part to an integer
}

void parseCommand(String commandToParse)
{
  /* This is where all the commands are interpreted and is the meat of the controll system
   * so far
   * #stop  - switches to idle mode
   * #pump - toggles pump on/off
   * #hk - print house keeping data to port
   * #reset  - force a reset of the T3.6
   * #release,int - energizes the release magnet for int seconds
   * #dl - dumps all the profile data from the buffer
   * #TSENdl - dumps the TSEN docked data
   * #clearProfileBuffer - clears the profile data buffer and resets the index
   * #warmUp,int - switches to warm up mode for int seconds
   * #setHeaters,float1,float2 - changes the setpoint of htr1 and htr2
   * #charge,int - 0 charge off, 1 charge on
   * #profile,int1,int2,int3 - starts a profile for int1 seconds followed by warmUp
   * for int2 seconds and an up profile for int3 seconds
   */
  
  char * strtokIndx; // this is used by strtok() as an index
  char CommandArray[64];
  int int1 = 0;
  int int2 = 0;
  int int3 = 0;
  float flt1 = 0;
  float flt2 = 0;
  float flt3 = 0;
  
  if(commandToParse.startsWith("#stop"))
  {
    DEBUG_SERIAL.println("Switching to idle Mode");
    GONDOLA_SERIAL.print(GONDOLA_Buff);
    DEBUG_Buff = "";
    GONDOLA_Buff = "";
    
    idleMode(); //go to idle mode for almost ever
  }
  else if(commandToParse.startsWith("#hk"))
  {
     DEBUG_SERIAL.println("House Keeping Data: ");
     GONDOLA_SERIAL.print(GONDOLA_Buff);
     commandToParse = "";
     DEBUG_Buff = "";
     GONDOLA_Buff = "";
     readTemps();
     scanAnalog();
     printAnalog();
     printTemps();
  }
  else if(commandToParse.startsWith("#pump"))
  {
     DEBUG_SERIAL.println("Toggling Pump");
     GONDOLA_SERIAL.print(GONDOLA_Buff);
     commandToParse = "";
     DEBUG_Buff = "";
     GONDOLA_Buff = "";
    digitalWrite(PUMP_PWR,!digitalRead(PUMP_PWR));
  }
  else if(commandToParse.startsWith("#reset"))
  {
     DEBUG_SERIAL.println("Rebooting in 3 seconds");
     GONDOLA_SERIAL.print(GONDOLA_Buff);
     delay(3000);
     WRITE_RESTART(0x5FA0004);
     commandToParse = "";
     DEBUG_Buff = "";
    GONDOLA_Buff = "";
    digitalWrite(PUMP_PWR,!digitalRead(PUMP_PWR));
  }
  else if(commandToParse.startsWith("#charge"))
  {
     DEBUG_SERIAL.println("Toggling Charge Enable");
     commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
     GONDOLA_SERIAL.print(GONDOLA_Buff);
     strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
     strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
     int1 = atoi(strtokIndx);     // convert this part to an integer, for the time
     commandToParse = "";
     DEBUG_Buff = "";
     GONDOLA_Buff = "";
     digitalWrite(CHARGER_SHUTDOWN,int1);
  }
  
  
  else if (commandToParse.startsWith("#warmUp"))
    {
      GONDOLA_SERIAL.print(GONDOLA_Buff);
      commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
      strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
  
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      flt1 = atoi(strtokIndx);     // convert this part to an integer, for the time

      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      flt2 = atoi(strtokIndx);     // convert this part to an integer, for the time

      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      flt3 = atoi(strtokIndx);     // convert this part to an integer, for the time

      DEBUG_SERIAL.print("Entering warm up for: ");
      DEBUG_SERIAL.print(int1); DEBUG_SERIAL.println(" Seconds");
 
      commandToParse = "";
      DEBUG_Buff = "";
      GONDOLA_Buff = "";
      //bool warmUpMode(float FLASH_T, float Heater_1_T, float Heater_2_T, bool FLASH_power, bool TSEN_power)
      warmUpMode();
    }
     else if (commandToParse.startsWith("#setHeater"))
    {
      GONDOLA_SERIAL.print(GONDOLA_Buff);
      commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
      strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
  
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      flt1 = atoi(strtokIndx);     // convert this part to an integer, for the time
      
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      flt2 = atoi(strtokIndx);     // convert this part to an integer, for the time

      DEBUG_SERIAL.print("Updating Heater setpoints to HTR1: ");
      DEBUG_SERIAL.print(flt1); DEBUG_SERIAL.print(" C, HTR2: ");
      DEBUG_SERIAL.print(flt2); DEBUG_SERIAL.println(" C");
 
      commandToParse = "";
      DEBUG_Buff = "";
      GONDOLA_Buff = "";

      Heater1Setpoint = flt1;
      Heater2Setpoint = flt2;
    }
    
    else if (commandToParse.startsWith("#profile"))
    {
       GONDOLA_SERIAL.print(GONDOLA_Buff);
      commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
      strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
  
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      int1 = atoi(strtokIndx);     // convert this part to an integer, for the down profile time

      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      int2 = atoi(strtokIndx);     // convert this part to an int for the bottom dwell time

      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      int3 = atoi(strtokIndx);     // convert this part to an int for the up profile time

  
      if (int1 > 2700 || int1 < 0)
      {
        DEBUG_SERIAL.println("Invalid Parameter");
        return;
      }

       if (int2 > 3600 || int2 < 0)
      {
        DEBUG_SERIAL.println("Invalid Parameter");
        return;
      }

      if (int3 > 2700 || int3 < 0)
      {
        DEBUG_SERIAL.println("Invalid Parameter");
        return;
      }

      DEBUG_SERIAL.print("Starting to profile: ");
      DEBUG_SERIAL.print(int1); DEBUG_SERIAL.print(" Seconds down, ");
      DEBUG_SERIAL.print(int2); DEBUG_SERIAL.print(" dwell at bottom ");
      DEBUG_SERIAL.print(int3); DEBUG_SERIAL.println(" Seconds up");
     
      commandToParse = "";
      DEBUG_Buff = "";
      GONDOLA_Buff = "";
      Profile();
      
    }
    else
    {
      GONDOLA_SERIAL.print("Invalid: ");
      DEBUG_SERIAL.print("Invalid: ");
      DEBUG_SERIAL.println(DEBUG_Buff);
      GONDOLA_SERIAL.println(GONDOLA_Buff);
      
      DEBUG_Buff = "";
      GONDOLA_Buff = "";
    }
   
}

bool parseLoRa()
{
  char LoRa_RX_buffer[256] = {0};
  int int1 = 0;

  int BytesToRead = LoRa.available();
  
  for (int i = 0; i <  BytesToRead; i++)
      LoRa_RX_buffer[i] = LoRa.read();
  
  if (strncmp(LoRa_RX_buffer,"TC:",3) == 0)//it is a TC from Zephyr
  { 
      LoRa_RX_buffer[BytesToRead] = '\0'; //null terminate buffer to make a string
      
      if(strncmp(LoRa_RX_buffer,"TC:LoRaStatusTx",16))
      {
        int strtokIndx = strtok(LoRa_RX_buffer,",");      // get the first part - the string we don't care about this
        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        int1 = atoi(strtokIndx);     // convert this part to an integer, for the time
        DEBUG_SERIAL.printf("[nominal] LoRa TC Update LoRa Status TX Rate to %d\n",int1);
        LoRaStatusRate = int1;
        if(LoRaStatusRate != 0)
          LoRaStatTime= millis() + LoRaStatusRate*1000l;
        return true;
      }
      else
        DEBUG_SERIAL.print("[warning] Unrecognised LoRa TC received: ");
        DEBUG_SERIAL.println(LoRa_RX_buffer);
      return false;
  }

  DEBUG_SERIAL.println("[nominal] Unreconized LoRa Packet received");
  return false;

}

void printTemps()
{
  Serial.print(" Pump T: ");
  Serial.print(PUMP_Therm);
  Serial.print(" HTR1 T: ");
  Serial.print(HTR1_Therm);
  Serial.print(" ");
  Serial.print(Heater1Status);
  Serial.print(" HTR2 T: ");
  Serial.print(HTR2_Therm);
  Serial.print(" ");
  Serial.print(Heater2Status);
  //Serial.print(" Spare T: ");
  //Serial.print(Spare_Therm);
}

void printAnalog()
{
  Serial.print("VBatt: ");
  Serial.print(V_Battery);
  Serial.print(" 3v3: ");
  Serial.print(V_3v3);
  Serial.print(" Vin: ");
  Serial.print(V_Input);
  Serial.print(" VPump: ");
  Serial.print(V_Pump);

  Serial.print(" ICharge: ");
  Serial.print(I_Charge);
  Serial.print(" ITSEN: ");
  Serial.print(I_Tsen);
  Serial.print(" IFLASH ");
  Serial.print(I_Flash);
  Serial.print(" IOPC: ");
  Serial.print(I_Opc);

}


bool writeTSENFile(int RecordsToWrite)
{
 TSENFileName = "TSEN" + String(year()) + String(month()) + String(day()) + String(hour()) + String(second()) + ".bin";
 File TSENFile;

 if (RecordsToWrite > TSEN_BUFFER_LENGTH) return false;

  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return false;
  }

  // open a new file, write to file and immediately close it:
  //Serial.println("Writing to File: " + FileName);
  char filename[100];
     TSENFileName.toCharArray(filename, 100);
     
  TSENFile = SD.open(filename, FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (TSENFile) {
    TSENFile.write(TSENData,TSEN_RECORD_LENGTH*RecordsToWrite*2);
    TSENFile.close();
    Serial.print("TSEN File Writen to: "); Serial.println(TSENFileName);
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
    return false;
  }
  return true;
}

bool writeProfileFile(int RecordsToWrite)
{
 String PUFileName = "PU" + String(year()) + String(month()) + String(day()) + String(hour()) + String(second()) + ".bin";
 File PUFile;

 if (RecordsToWrite > TM_BUFFER_LENGTH) return false;

  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return false;
  }

  // open a new file, write to file and immediately close it:
  //Serial.println("Writing to File: " + FileName);
  char filename[100];
     PUFileName.toCharArray(filename, 100);
     
  PUFile = SD.open(filename, FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (PUFile) {
    PUFile.write(ProfileData,TM_RECORD_LENGTH*RecordsToWrite*2);
    PUFile.close();
    Serial.printf("%d Profile Records Writen to: ", RecordsToWrite); Serial.println(PUFileName);
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
    return false;
  }
  return true;
}


time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */

unsigned long processSyncMessage() 
{
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find("T")) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

bool updateRTCfromGPS()
{
    if ((gps.time.age() < 1500) && (gps.satellites.value() > 4))  //we have a GPS time fix within the last 1.5s
    {
        if(timeStatus() != timeSet) //if the time is not set, set it
        {
            setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
            DEBUG_SERIAL.println("Setting RTC to GPS time "); 
            return true;
        }

        if (abs(minute() * 60 + second() - (gps.time.minute()*60 +gps.time.second())) > 2) //if the clock is more than 1 second off
        {
            DEBUG_SERIAL.printf("Updating RTC to GPS time from %d:%d:%d to %u:%u:%u\n", hour(), minute(), second(), gps.time.hour(), gps.time.minute(), gps.time.second());
            DEBUG_SERIAL.println((gps.time.hour()*3600 + gps.time.minute()*60 + gps.time.second()) - (hour()*3600 + minute()*60 + second()));
            setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
            return true;
        }

        return false;

    }
    return false;

}
