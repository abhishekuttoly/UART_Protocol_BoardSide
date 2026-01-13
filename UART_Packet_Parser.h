//**************************** UART_PACKET_PARSER.H *****************************
//  Copyright (c) 2025
//  All Rights Reserved
//*****************************************************************************
//
// File		: UART_Packet_Parser.h
// Summary	: This configurations header file contains structure and function declarations
//		       for parse UART packet
// Note		: None
// Author	: Abhishek
// Date		: 31-12-2025
//
//*****************************************************************************

#ifndef INC_UART_PACKET_PARSER_H_
#define INC_UART_PACKET_PARSER_H_

//******************************* Include Files *******************************
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "UART_Packet_Builder.h"

//******************************* Global Types ********************************

//***************************** Global Constants ******************************

//***************************** Global Structure ******************************
typedef struct __sTLV{
	uint8_t	ucTLVNumber;		// count of value
	_sTLVParams psTlvParam;		// TLV data holding structure
	} _sTLV;

typedef struct __sPacketData{
	uint8_t ucRequestType;			// TLV header
	uint8_t ucLength;				// Length of TLV buffer
	uint8_t ucNumberOfTLVs;			// Number of TLVs
	_sTLV psTlv[];		// structure holding multiple TLV data
	} _sPacketData;

//**************************** Forward Declarations ***************************

//***************************** Function Declaration **************************
bool UARTPacketParserParsePacket(uint8_t ucUARTPacket[], _sPacketData *psData);
uint8_t UARTPacketParserGetTLVCount(uint8_t ucUARTPacket[]);
void UARTPacketParserParseTLV(uint8_t *psTLVBuffer, _sTLVParams *psTLVData);
//*********************** Inline Method Implementations ***********************

#endif /* INC_UART_PACKET_PARSER_H_ */
