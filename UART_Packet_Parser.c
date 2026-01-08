//******************************** UART_PACKET_BUILDER ****************************************
// Copyright (c) 2025
// All Rights Reserved
//*********************************************************************************
//
// File		: UART_Packet_Parser.c
// Summary	: UART packet parser functions
// Note		: None
// Author	: Abhishek
// Date		: 31-12-2025
//
//*********************************************************************************

//******************************* Include Files ***********************************
#include "UART_Packet_Parser.h"

//******************************* Local Types *************************************

//***************************** Local Constants ***********************************

//***************************** Extern Variables **********************************

//**************************** Global Variables ***********************************

//****************************** Local Functions **********************************

//***************************** Function Definitions ******************************
//******************************.FUNCTION_HEADER.******************************
//Purpose : To parse packet
//Inputs  : ucUARTPacket - UART packet buffer
//Outputs : psData - structure holding parsed packet data
//Return  : true if successful, false otherwise
//**********************************************************************************
bool ParsePacket(uint8_t ucUARTPacket[], _sPacketData *psData)
{
	psData->ucRequestType = ucUARTPacket[0];
	psData->ucLength = ucUARTPacket[1];

printf("Type Parsed = %d\n", ucUARTPacket[0]);
fflush(stdout);

	psData->ucNumberOfTLVs = GetTLVCount(ucUARTPacket);
	uint8_t ucIndex = 2;
	uint8_t ucCount = 0;

	while(ucIndex < psData->ucLength && ucCount < psData->ucNumberOfTLVs)
	{
		psData->psTlv[ucCount].ucTLVNumber = ucCount;
		psData->psTlv[ucCount].psTlvParam.ucType = ucUARTPacket[ucIndex];
		ucIndex++;
		psData->psTlv[ucCount].psTlvParam.ucLength = ucUARTPacket[ucIndex];
		ucIndex++;

		if(psData->psTlv[ucCount].psTlvParam.ucLength == 0)
		{
			return false;
		}
		else
		{
			uint8_t ucItration;
			for(ucItration = 0; ucItration < psData->psTlv[ucCount].psTlvParam.ucLength; ucItration++ )
			{
				psData->psTlv[ucCount].psTlvParam.ucValueBuffer[ucItration] = ucUARTPacket[ucIndex+ucItration];
			}
			ucIndex += psData->psTlv[ucCount].psTlvParam.ucLength;
		}

		if(ucIndex > psData->ucLength +2)
		{
			return false;
		}

		ucCount++;
	}
	return true;
}

//******************************.FUNCTION_HEADER.******************************
//Purpose : To get TLV count in the packet
//Inputs  : ucUARTPacket - UART packet data buffer
//Outputs : None
//Return  : Number of TLVs
//**********************************************************************************
uint8_t GetTLVCount(uint8_t ucUARTPacket[])
{
	uint8_t ucCount = 0;
	uint8_t ucIndex = 2;

	uint8_t ucTotalLength = ucUARTPacket[1];

	if( ucTotalLength !=0)
	{
		while(ucIndex < ucTotalLength + 2)
		{
			ucIndex++;		// Since first element is tag
			uint8_t ucLength = ucUARTPacket[ucIndex];
			if(ucLength == 0)
			{
				return 0;		// Invalid TLV
			}
			else
			{
				ucIndex += ucLength+1;

			}
			ucCount ++;
		}
	}
	else
	{
		return 0;	// Invalid TLV
	}

	return ucCount;
}


//******************************.FUNCTION_HEADER.******************************
//Purpose : To parse single TLV
//Inputs  : psTLVBuffer - TLV buffer
//Outputs : psTLVData - Structure holding parsed TLV data
//Return  : None
//**********************************************************************************
void ParseTLV(uint8_t *psTLVBuffer, _sTLVParams *psTLVData)
{
	uint8_t ucIndex = 0;
	psTLVData->ucType = psTLVBuffer[ucIndex];
	ucIndex++;

	psTLVData->ucLength = psTLVBuffer[ucIndex];
	ucIndex++;

	uint8_t ucItrator;
	uint8_t ucValueBuffer[psTLVData->ucLength];
	for(ucItrator = 0; ucItrator < psTLVData->ucLength; ucItrator++)
	{
		ucValueBuffer[ucItrator] = psTLVBuffer[ucIndex + ucItrator];
	}

	memcpy(psTLVData->ucValueBuffer, ucValueBuffer, psTLVData->ucLength);
	ucIndex += psTLVData->ucLength;
}
