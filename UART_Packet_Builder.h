/**************************** UART_PACKET_BUILDER.H *****************************/
//  Copyright (c) 2025
//  All Rights Reserved
//*****************************************************************************
//
// File		: UART_Packet_Builder.h
// Summary	: This configurations header file contains structure and function declarations
//		       for build/create UART packet
// Note		: None
// Author	: Abhishek
// Date		: 31-12-2025
//
//*****************************************************************************

#ifndef INC_UART_PACKET_BUILDER_H_
#define INC_UART_PACKET_BUILDER_H_

//******************************* Include Files *******************************
#include <stdio.h>
#include <string.h>
#include <stdint.h>

//******************************* Global Types ********************************

//***************************** Global Constants ******************************
#define UART_GET_COMMAND 1
#define UART_SET_COMMAND 2
#define UART_TYPE_INT 1
#define UART_TYPE_CHAR 2
#define UART_TYPE_PARAMETER 3
#define UART_VALUE_LENGTH 1
//***************************** Global Structure ******************************
typedef struct __sResponseValue{
	uint8_t ucResponseType;
	uint8_t ucParamId;
	uint8_t ucValueBuffer[];
	} _sResponseValue;

typedef struct __sTLVParams
{
	uint8_t ucType;				// Tag/Type of value
	uint8_t ucLength;			// Length of value
	uint8_t ucValueBuffer[10];	// Value
} _sTLVParams;

//**************************** Forward Declarations ***************************

//***************************** Function Declaration **************************
//void CreateGetPacket(uint8_t ucParamId, uint8_t ucTLVGetPacketBuffer[]);
//void CreateSetPacket(uint8_t ucParamId, uint8_t ucParameterBuffer[], uint8_t ucParamLength, uint8_t ucTLVSetPacketBuffer[]);
//void CreateTLVBuffer(_sTLVParams *psTLVParam, uint8_t ucTLVBuffer[], uint8_t *ucLength);

 void UARTPacketBuilderCreateGetResponsePacket(_sResponseValue *psResponseValue, uint8_t ucNumberOfItems, uint8_t ucLengthOfResponseValue, uint8_t pucResponsePacket[]);
 void UARTPacketBuilderCreateSetResponsePacket(uint8_t ucParameterID, uint8_t pucResponsePacket[]);
 uint8_t UARTPacketBuilderCreateTLV(_sTLVParams *psTLVParam, uint8_t ucTLVBuffer[]);

//*********************** Inline Method Implementations ***********************


#endif /* INC_UART_PACKET_BUILDER_H_ */
// EOF
