//******************************** UART_PACKET_BUILDER ****************************************
// Copyright (c) 2025
// All Rights Reserved
//*********************************************************************************
//
// File		: UART_Packet_Builder.c
// Summary	: UART packet builder functions
// Note		: None
// Author	: Abhishek
// Date		: 31-12-2025
//
//*********************************************************************************

//******************************* Include Files ***********************************
#include "UART_Packet_Builder.h"
#include "main.h"

//******************************* Local Types *************************************

//***************************** Local Constants ***********************************

//***************************** Extern Variables **********************************

//**************************** Global Variables ***********************************

//****************************** Local Functions **********************************

//***************************** Function Definitions ******************************
//******************************.FUNCTION_HEADER.******************************
//Purpose : To create get response packet
//Inputs  : psResponseValue - structure holding packet data
//Inputs  : ucNumberOfItems - Number of TLVs
//Inputs  : ucLengthOfResponseValue - Length of response value buffer
//Outputs : pucResponsePacket - Response packet buffer
//Return  : none
//**********************************************************************************
void UARTPacketBuilderCreateGetResponsePacket(_sResponseValue *psResponseValue, uint8_t ucNumberOfItems, uint8_t ucLengthOfResponseValue, uint8_t pucResponsePacket[])
 {
     uint8_t ucIndex = 2;            // Start after total length byte of the packet
     uint8_t ucCounter = 0;
     uint8_t ucTotalLength = 0;

     while(ucCounter < ucNumberOfItems)
     {
         _sTLVParams sTLVParam;

         sTLVParam.ucType = 3;       // parameter
         sTLVParam.ucLength = 1;     // since parameter, length = 1
         sTLVParam.ucValueBuffer[0] = psResponseValue->ucParamId;
         uint8_t ucLength = 0;
         ucLength = UARTPacketBuilderCreateTLV(&sTLVParam, pucResponsePacket+ucIndex);

         ucTotalLength += ucLength;

         // TLV holds response data
         _sTLVParams sResponseTLV;
         sResponseTLV.ucType = psResponseValue->ucResponseType;
         sResponseTLV.ucLength = ucLengthOfResponseValue;

         memcpy(sResponseTLV.ucValueBuffer, psResponseValue->ucValueBuffer, sResponseTLV.ucLength);

         ucIndex += ucLength;
         ucLength = 0;
         ucLength = UARTPacketBuilderCreateTLV(&sResponseTLV, pucResponsePacket+ucIndex);
         ucIndex += ucLength;
         ucTotalLength += ucLength;
         ucCounter++;
     }
     pucResponsePacket[0] = 1;
     pucResponsePacket[1] = ucTotalLength;
}

//******************************.FUNCTION_HEADER.******************************
//Purpose : To create set response  packet
//Inputs  : ucParameterID - Parameter ID
//Outputs : ucParameterID - Set response buffer
//Return  : None
//**********************************************************************************
void UARTPacketBuilderCreateSetResponsePacket(uint8_t ucParameterID, uint8_t pucResponsePacket[])
 {
     _sTLVParams sTLVParam;

     sTLVParam.ucType = 3;   //parameter
     sTLVParam.ucLength = 1;
     sTLVParam.ucValueBuffer[0] = ucParameterID;

     uint8_t ucLength = 0;
     ucLength = UARTPacketBuilderCreateTLV(&sTLVParam, pucResponsePacket+2);

     pucResponsePacket[0] = 2;   //set
     pucResponsePacket[1] = ucLength;
 }

//******************************.FUNCTION_HEADER.******************************
//Purpose : To create TLV buffer
//Inputs  : psTLVParam - Structure includes Type, length and value
//Outputs : ucTLVBuffer - TLV buffer
//Return  : true if successful, false otherwise
//**********************************************************************************
uint8_t UARTPacketBuilderCreateTLV(_sTLVParams *psTLVParam, uint8_t ucTLVBuffer[])
{
      uint8_t ucIndex = 0;
      ucTLVBuffer[0] = psTLVParam->ucType;
  	ucIndex++;
      ucTLVBuffer[1] = psTLVParam->ucLength;
  	ucIndex++;
  	memcpy(ucTLVBuffer+ucIndex, psTLVParam->ucValueBuffer, psTLVParam->ucLength);
  	ucIndex += psTLVParam->ucLength;
      return ucIndex;
}
