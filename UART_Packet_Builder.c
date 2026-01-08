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
//Purpose : To create get command packet
//Inputs  : ucParamId - Parameter ID
//Outputs : ucTLVGetPacketBuffer - Get packet buffer
//Return  : none
//**********************************************************************************
void CreateGetPacket(uint8_t ucParamId, uint8_t ucTLVGetPacketBuffer[])
{
	_sTLVParams psTLVParam;
	psTLVParam.ucType = TYPE_PARAMETER;
	psTLVParam.ucLength = VALUE_LENGTH;
	psTLVParam.ucValueBuffer[0] = ucParamId;
	uint8_t ucLengthOfPacket = 0;
	CreateTLVBuffer(&psTLVParam, ucTLVGetPacketBuffer+2, &ucLengthOfPacket);
	ucTLVGetPacketBuffer[0] = GET_COMMAND;
	ucTLVGetPacketBuffer[1] = ucLengthOfPacket;
}

//******************************.FUNCTION_HEADER.******************************
//Purpose : To create set command packet
//Inputs  : ucParamId - Parameter ID
//Inputs  : ucParameterBuffer - Parameter value to set
//Inputs  : ucParamLength - Length of parameter
//Outputs : ucTLVSetPacketBuffer - Set packet buffer
//Return  : None
//**********************************************************************************
void CreateSetPacket(uint8_t ucParamId, uint8_t ucParameterBuffer[], uint8_t ucParamLength, uint8_t ucTLVSetPacketBuffer[])
{
	_sTLVParams psTLVParam;
	psTLVParam.ucType = TYPE_PARAMETER;
	psTLVParam.ucLength = VALUE_LENGTH;
	psTLVParam.ucValueBuffer[0]=ucParamId;
	uint8_t ucLengthOfPacket = 0;
	uint8_t ucTLVLength = 0;

	CreateTLVBuffer(&psTLVParam, ucTLVSetPacketBuffer+2, &ucTLVLength);

	ucLengthOfPacket += ucTLVLength;

	_sTLVParams psSetParamTLV;
	psSetParamTLV.ucType = TYPE_CHAR;
	psSetParamTLV.ucLength = ucParamLength;
	memcpy(psSetParamTLV.ucValueBuffer, ucParameterBuffer, psSetParamTLV.ucLength);
	ucTLVLength = 0;
	CreateTLVBuffer(&psSetParamTLV, ucTLVSetPacketBuffer+(ucLengthOfPacket+2), &ucTLVLength);
	ucLengthOfPacket += ucTLVLength;

	ucTLVSetPacketBuffer[0] = SET_COMMAND;
	ucTLVSetPacketBuffer[1] = ucLengthOfPacket;
}

//******************************.FUNCTION_HEADER.******************************
//Purpose : To create TLV buffer
//Inputs  : psTLVParam - Structure includes Type, length and value
//Outputs  : ucTLVBuffer - TLV buffer
//Outputs : ucLength - Length of TLV buffer
//Return  : true if successful, false otherwise
//**********************************************************************************
void CreateTLVBuffer(_sTLVParams *psTLVParam, uint8_t ucTLVBuffer[], uint8_t *ucLength)
{
	uint8_t ucIndex = 0;
	ucTLVBuffer[0] = psTLVParam->ucType;
	ucIndex ++;
	ucTLVBuffer[1] = psTLVParam->ucLength;
	ucIndex ++;
	if(psTLVParam->ucLength != 1){
	memcpy(ucTLVBuffer+ucIndex, psTLVParam->ucValueBuffer, psTLVParam->ucLength);
	}
	else{
		ucTLVBuffer[ucIndex] = psTLVParam->ucValueBuffer[0]; // since the length is 1 , it  assign to the buffer.
	}
	ucIndex += psTLVParam->ucLength;
	*ucLength += ucIndex;
}
