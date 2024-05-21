/*
	Copyright (C) 2022 Victor Chavez
    This file is part of SimpleJ1939.
    SimpleJ1939 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    SimpleJ1939 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with SimpleJ1939.  If not, see <https://www.gnu.org/licenses/>.
*/
#include "../mcp_can/mcp_can.h"
#include "fpt_engine.h"

bool FPTEngine::Init(INT8U CSPin)
{
  if(pCAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ, CSPin) == CAN_OK){
    pCAN.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.
    CANInitialized = true;
    return 1;
  }else{
    CANInitialized = false;
    return 0;
  }
}
// ------------------------------------------------------------------------
// Transmit CAN message
// ------------------------------------------------------------------------
bool FPTEngine::canTransmit(long lID, unsigned char* pData, int nDataLen)
{
  if (pCAN.sendMsgBuf(lID, CAN_EXTID, nDataLen, pData) == 0)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

// ------------------------------------------------------------------------
// Receive CAN message
// ------------------------------------------------------------------------
bool FPTEngine::canReceive(long* lID, unsigned char* pData, int* nDataLen)
{
  // In case there is a message, put it into the buffer
  while (pCAN.checkReceive() == CAN_MSGAVAIL)
  {
    // Read the message buffer

    pCAN.readMsgBuf(&CANMsgBuffer[nWritePointer].lID,
                     &CANMsgBuffer[nWritePointer].nDataLen,
                     &CANMsgBuffer[nWritePointer].pData[0]);
    if (++nWritePointer == CANMSGBUFFERSIZE)
    {
      nWritePointer = 0;
    }
  }// end while

  // Check ring buffer for a message
  if (nReadPointer != nWritePointer)
  {
    // Read the next message buffer entry
    *nDataLen = CANMsgBuffer[nReadPointer].nDataLen;
    *lID = CANMsgBuffer[nReadPointer].lID;

    for (int nIdx = 0; nIdx < *nDataLen; nIdx++)
    {
      pData[nIdx] = CANMsgBuffer[nReadPointer].pData[nIdx];
    }

    if (++nReadPointer == CANMSGBUFFERSIZE)
    {
      nReadPointer = 0;
    }

    return 1;

  }// end if
  else 
  {
	  return 0;
  }
}// end canReceive

// ------------------------------------------------------------------------
// Refresh Engine Data from J1939 stream
// ------------------------------------------------------------------------
bool FPTEngine::Refresh(FPTEngine_Data* data)
{
  J1939Msg RecvMsg;
  byte nRetCode = 0;
  uint64_t currentMillis = millis();
  if(currentMillis - ECUMillis >= ECU_TIMEOUT)
  {
    Connected = false;
  }
  if(CANInitialized){
    if (canReceive(&RecvMsg.lID, RecvMsg.nData, &RecvMsg.nDataLen))
    {
      long lPriority = RecvMsg.lID & 0x1C000000;
      RecvMsg.nPriority = static_cast<int>(lPriority >> 26);

      RecvMsg.lPGN = RecvMsg.lID & 0x00FFFF00;
      RecvMsg.lPGN = RecvMsg.lPGN >> 8;

      RecvMsg.lID = RecvMsg.lID & 0x000000FF;
      RecvMsg.nSrcAddr = static_cast<int>(RecvMsg.lID);
      if((RecvMsg.nSrcAddr==Address_ECU)||(RecvMsg.nSrcAddr==Address_Converter)){
        Connected = true;
        ECUMillis = currentMillis;
      }
      decode(RecvMsg, data);
      nRetCode = 1;
    }// end if
  }
  return nRetCode;
}// end j1939Receive

// ------------------------------------------------------------------------
// Decode J1939 Message
// ------------------------------------------------------------------------
void FPTEngine::  decode(J1939Msg Msg, FPTEngine_Data* data)
{
    uint16_t temp;
    uint32_t templ;
    if (Msg.nSrcAddr == Address_ECU){
      switch (Msg.lPGN){
        case 0xF003: //Electronic Engine Controller 2
            //data->Accelerator = Msg.nData[1]*0.4;
            data->mLoad = Msg.nData[2];
            data->fsLoad = Msg.nData[2];
            data->IES_Load = Msg.nData[2];
            break;
        case 0xF004: //Electronic Engine Controller 1
            //data->eTorque = Msg.nData[2]-125;
            temp = Msg.nData[3];
            temp += (Msg.nData[4] << 8); 
            data->fsEngine_RPM = temp*0.125;
            //data->SPD_SPD_TO_MCU = temp*0.125;
            break;
        /*case 0xFEE5: //Engine hours, revolutions
            templ = Msg.nData[0];
            templ += (Msg.nData[1] << 8);
            templ += ((uint32_t)(Msg.nData[2]) << 16);
            templ += ((uint32_t)(Msg.nData[3]) << 24);
            data->total_run_hours = templ*0.05;
            break;
        case 0xFEE9: 
            templ = Msg.nData[4];
            templ += (Msg.nData[5] << 8);
            templ += ((uint32_t)(Msg.nData[6]) << 16);
            templ += ((uint32_t)(Msg.nData[7]) << 24);
            data->total_fuel_burned = templ*0.5;
            break;*/
        case 0xFEEE: 
            data->mCool_temp = Msg.nData[0]-40;
            data->fsCoolPS = Msg.nData[0]-40;
            data->fsCoolSB = Msg.nData[0]-40;
            data->fsFuel_temp  = Msg.nData[1]-40;
            temp = Msg.nData[2];
            temp += (Msg.nData[3] << 8); 
            data->fsOil_temp = (temp*0.03125)-273;
            break;
        case 0xFEEF: 
            data->fsOil_press = (Msg.nData[3]*4)*0.01;
            break;
        case 0xFEF2: 
            temp = Msg.nData[0];
            temp += (Msg.nData[1] << 8); 
            data->mFuel_rate = temp *0.05;  
            break;
        /*case 0xFEF5: 
            data->barometric_pressure = (Msg.nData[0] * 0.5)*0.01;
            break;*/
        case 0xFEF6:
            temp = Msg.nData[5];
            temp += (Msg.nData[6] << 8);
            data-> IES_ET_PS = (temp*0.03125)-273;
            data-> IES_ET_SB  = (temp*0.03125)-273;
            //data->boost_pressure = (Msg.nData[1]*2)*0.01;
            data-> IES_ITP_PS = Msg.nData[0]*0.05;
            data-> IES_ITP_SB = Msg.nData[0]*0.05;
            data-> IES_ITT_PS = Msg.nData[0]-40;
            data-> IES_ITT_SB = Msg.nData[0]-40;
            break;
        /*case 0xFEF7: //Vehicle Electrical Power
            temp = Msg.nData[6];
            temp += (Msg.nData[7] << 8);
            data->batt_volt = temp*0.05;
            break;
        case 0xFEF8: 
            data->gear_oil_pressure = (Msg.nData[3]*4) *0.01;
            temp = Msg.nData[4];
            temp += (Msg.nData[5] << 8);
            data->gear_oil_temp = (temp*0.03125)-273;
            break;
        case 0XFF21: 
            temp = (Msg.nData[0] & 0b11000000);
            if(temp>0)
              data_alrm->DiagnosticLight = true;
            else
              data_alrm->DiagnosticLight = false;
            temp = (Msg.nData[1] & 0b11100000);
            if(temp>0)
              data_alrm->HighTemperature = true;
            else
              data_alrm->HighTemperature = false;
            temp = (Msg.nData[1] & 0b00001100);
            if(temp>0)
              data_alrm->Overspeed = true;
            else
              data_alrm->Overspeed = false;
            temp = (Msg.nData[5] & 0b11000000);
            if(temp>0)
              data_alrm->EngineOilTempHigh = true;
            else
              data_alrm->EngineOilTempHigh = false;
            temp = (Msg.nData[5] & 0b00001100);
            if(temp>0)
              data_alrm->WaterinFuel = true;
            else
              data_alrm->WaterinFuel = false;
            temp = (Msg.nData[5] & 0b00000011);
            if(temp>0)
              data_alrm->LowOilPressure = true;
            else
              data_alrm->LowOilPressure = false;
            break;*/
        default:
            break;
        }
    }
    if (Msg.nSrcAddr == Address_Converter){
      switch (Msg.lPGN){
        case 0xFEA3: 
          break;
        /*case 0xFEF8: 
          data->gear_oil_pressure = (Msg.nData[3]*4) *0.01;
          temp = Msg.nData[4];
          temp += (Msg.nData[5] << 8);
          data->gear_oil_temp = (temp*0.03125)-273;
          break;
        case 0xFF13: 
          temp = (Msg.nData[0] & 0b11000000);
          if(temp>0)
            data_alrm->BlowByClogging = true;
          else
            data_alrm->BlowByClogging = false;
          temp = (Msg.nData[0] & 0b00110000);
          if(temp>0)
            data_alrm->AirFilterClogging = true;
          else
            data_alrm->AirFilterClogging = false;
          temp = (Msg.nData[0] & 0b00000011);
          if(temp>0)
            data_alrm->FuelFilterClogging = true;
          else
            data_alrm->FuelFilterClogging = false;
          temp = (Msg.nData[1] & 0b11000000);
          if(temp>0)
            data_alrm->OilFilterClogging = true;
          else
            data_alrm->OilFilterClogging = false;
          temp = (Msg.nData[1] & 0b00001100);
          if(temp>0)
            data_alrm->LowWaterLevel = true;
          else
            data_alrm->LowWaterLevel = false;
          temp = (Msg.nData[2] & 0b11000000);
          if(temp>0)
            data_alrm->BatteryWarning = true;
          else
            data_alrm->BatteryWarning = false;
          break;*/
        default:
          break;
      }
    }
}

// ------------------------------------------------------------------------
// Request specific PGN 
// ------------------------------------------------------------------------
bool FPTEngine::Request(uint8_t msg_priority,uint8_t source_node, uint8_t requester_node, uint16_t pgn)
{
  bool retCode = false;
  if(CANInitialized){
    long lID;
    unsigned char nData[3];
    lID = (uint32_t)msg_priority<<26;
    lID += 0xEA0000;
    lID += source_node<<8;
    lID += requester_node;
    nData[0] = pgn & 0x00FF;
    nData[1] = pgn >> 8;
    nData[2] = 0;
    retCode = canTransmit(lID, nData, 3);
  }
  return retCode;
}