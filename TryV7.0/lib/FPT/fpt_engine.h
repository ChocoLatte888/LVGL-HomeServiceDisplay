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
#ifndef FPTENGINE_HPP
#define FPTENGINE_HPP
#include <Arduino.h>
#include "fpt_engine_data.h"
#include "../mcp_can/mcp_can.h"

#define Address_ECU 0x00
#define Address_Converter 0x03
#define ECU_TIMEOUT  2000
#define CANMSGBUFFERSIZE 350

class FPTEngine
{
  public:
    bool Connected;
    bool Init(INT8U CSPin);
    bool Refresh(FPTEngine_Data* data);
    bool Request(uint8_t msg_priority,uint8_t source_node, uint8_t requester_node, uint16_t pgn);
    //FPTEngine_Data Decoded;
    
  private:
    INT8U   MCPCS;                                                      // Chip Select pin number
    MCP_CAN pCAN;
    int nWritePointer = 0;
    int nReadPointer = 0;
    uint64_t ECUMillis = 0;
    //static constexpr uint16_t CANMSGBUFFERSIZE = 1024;
    bool CANInitialized = false;
    struct CANMsg
    {
      unsigned long lID;
      unsigned char pData[8];
      uint8_t nDataLen;
    };
    struct J1939Msg
    {
        long lPGN,lID;
        byte nPriority, nSrcAddr, nDestAddr, nData[8];
        int nDataLen;    
    };
    CANMsg CANMsgBuffer[CANMSGBUFFERSIZE];
    bool canTransmit(long lID, unsigned char* pData, int nDataLen);
    bool canReceive(long* lID, unsigned char* pData, int* nDataLen);
    void decode(J1939Msg Msg, FPTEngine_Data* data);
};
#endif
