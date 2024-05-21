#pragma once

#ifndef FPT_ENGINE_DATA_H
#define FPT_ENGINE_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

struct FPTEngine_Data{
  float mOil_press;
  float mCool_temp;
  float mLoad;
  float mFuel_rate;

  float fsCoolPS;
  float fsCoolSB;
  float fsOil_temp;
  float fsOil_press;
  float fsOil_piston_jet; //No Data
  float fsFuel_press; //No Data
  float fsFuel_temp;
  float fsRaillPress_SB; //No Data
  float fsRaillPress_PS; //No Data
  float fsLoad; 
  float fsEngine_RPM;

  float IES_ITP_PS;
  float IES_ITP_SB; 
  float IES_ITT_PS;
  float IES_ITT_SB; 
  float IES_Load;
  float IES_ET_PS;
  float IES_ET_SB;

  float SPD_SPD_TO_MCU; 
  float SPD_SPD_REQ;
  float SPD_LOAD;
  float SPD_BATVOL;

  float STT_RH;
  float STT_TFU;
};

struct FPTEngine_Alarm{
  bool DiagnosticLight;
  bool Overspeed;
  bool HighTemperature;
  bool LowOilPressure;
  bool BlowByClogging;
  bool AirFilterClogging;
  bool FuelFilterClogging;
  bool OilFilterClogging;
  bool LowWaterLevel;
  bool BatteryWarning;
  bool EngineOilTempHigh;
  bool WaterinFuel;
};

#ifdef __cplusplus
} /*extern "C"*/
#endif


#endif