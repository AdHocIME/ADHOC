/**
 ******************************************************************************
 * @file           : usbd_rndis_if.c
 * @brief          :
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_rndis_if.h"
#include "FreeRTOS.h"
#include "list.h"
#include "task.h"
#include "FreeRTOS_IP.h"
#include "NetworkBufferManagement.h"
#include "FreeRTOS_IP_Private.h"
//#include "hr_gettime.h"

/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_RNDIS
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_RNDIS_Private_TypesDefinitions
 * @{
 */
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
 * @}
 */

/** @defgroup USBD_RNDIS_Private_Defines
 * @{
 */
/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over RNDIS */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
#define DeviceID_8 ((uint8_t*)0x1FFF7A10)

/* USER CODE END PRIVATE_DEFINES */
/**
 * @}
 */

/** @defgroup USBD_RNDIS_Private_Macros
 * @{
 */
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_RNDIS_Private_Variables
 * @{
 */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserRxBufferFS_Temp[64];
static uint64_t rndis_oid_gen_xmit_ok=0;
static uint64_t rndis_oid_gen_rcv_ok=0;

uint16_t UserRxSize=0;
static enum{
	RNDIS_STATE_NORMAL,
	RNDIS_STATE_HALTED
} rndis_state=RNDIS_STATE_HALTED;


/* Send Data over USB RNDIS are stored in this buffer       */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE+44];

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_RNDIS_IF_Exported_Variables
 * @{
 */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_RNDIS_Private_FunctionPrototypes
 * @{
 */
static int8_t RNDIS_Init_FS     (void);
static int8_t RNDIS_DeInit_FS   (void);
static int8_t RNDIS_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t RNDIS_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

static void prvEMACHandlerTask( void *pvParameters );

/* Default the size of the stack used by the EMAC deferred handler task to twice
the size of the stack used by the idle task - but allow this to be overridden in
FreeRTOSConfig.h as configMINIMAL_STACK_SIZE is a user definable constant. */
#ifndef configEMAC_TASK_STACK_SIZE
#define configEMAC_TASK_STACK_SIZE ( 2 * configMINIMAL_STACK_SIZE )
#endif

/* Holds the handle of the task used as a deferred interrupt processor.  The
handle is used so direct notifications can be sent to the task for all EMAC/DMA
related interrupts. */
static TaskHandle_t xEMACTaskHandle = NULL;


USBD_RNDIS_ItfTypeDef USBD_RNDIS_Interface_fops_FS =
{
		RNDIS_Init_FS,
		RNDIS_DeInit_FS,
		RNDIS_Control_FS,
		RNDIS_Receive_FS
};

const uint32_t OID_GEN_SUPPORTED[]={
		RNDIS_OID_GEN_SUPPORTED_LIST,
		RNDIS_OID_GEN_HARDWARE_STATUS,
		RNDIS_OID_GEN_MEDIA_SUPPORTED,
		RNDIS_OID_GEN_MEDIA_IN_USE,
		RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE,
		RNDIS_OID_GEN_LINK_SPEED,
		RNDIS_OID_GEN_TRANSMIT_BLOCK_SIZE,
		RNDIS_OID_GEN_RECEIVE_BLOCK_SIZE,
		RNDIS_OID_GEN_VENDOR_ID,
		RNDIS_OID_GEN_VENDOR_DESCRIPTION,
		RNDIS_OID_GEN_VENDOR_DRIVER_VERSION,
		RNDIS_OID_GEN_CURRENT_PACKET_FILTER,
		RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE,
		RNDIS_OID_GEN_MEDIA_CONNECT_STATUS,
		RNDIS_OID_GEN_PHYSICAL_MEDIUM,
		RNDIS_OID_GEN_XMIT_OK,
		RNDIS_OID_GEN_RCV_OK,
		//			RNDIS_OID_GEN_XMIT_ERROR,
		//			RNDIS_OID_GEN_RCV_ERROR,
		//			RNDIS_OID_GEN_RCV_NO_BUFFER,
		RNDIS_OID_802_3_PERMANENT_ADDRESS,
		RNDIS_OID_802_3_CURRENT_ADDRESS,
		RNDIS_OID_802_3_MULTICAST_LIST,
		RNDIS_OID_802_3_MAC_OPTIONS,
		RNDIS_OID_802_3_MAXIMUM_LIST_SIZE,
		RNDIS_OID_802_3_RCV_ERROR_ALIGNMENT,
		RNDIS_OID_802_3_XMIT_ONE_COLLISION,
		RNDIS_OID_802_3_XMIT_MORE_COLLISIONS,
};

const uint32_t response[]={
		RNDIS_RESPONSE_AVAILABLE,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
};


/* Private functions ---------------------------------------------------------*/

void RNDIS_Disconnect(){
	rndis_oid_gen_xmit_ok=0;
	rndis_oid_gen_rcv_ok=0;
	rndis_state=RNDIS_STATE_HALTED;
	FreeRTOS_NetworkDownFromISR();
}

/**
 * @brief  RNDIS_Init_FS
 *         Initializes the RNDIS media low layer over the FS USB IP
 * @param  None
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t RNDIS_Init_FS(void)
{ 
	/* USER CODE BEGIN 3 */
	/* Set Application Buffers */
	USBD_RNDIS_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
	USBD_RNDIS_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS_Temp);
	RNDIS_Disconnect();
	return (USBD_OK);
	/* USER CODE END 3 */
}

/**
 * @brief  RNDIS_DeInit_FS
 *         DeInitializes the RNDIS media low layer
 * @param  None
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t RNDIS_DeInit_FS(void)
{
	/* USER CODE BEGIN 4 */
	RNDIS_Disconnect();
	return (USBD_OK);
	/* USER CODE END 4 */
}

/**
 * @brief  RNDIS_Control_FS
 *         Manage the RNDIS class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t RNDIS_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
	static const char nome[]="IMBEL TPP-1400";
	static RNDIS_DATA rndis_data;
	uint32_t *buf32=(uint32_t *)pbuf;
	uint8_t len;
	int pos=0;
	USBD_RNDIS_HandleTypeDef *hrndis = (USBD_RNDIS_HandleTypeDef*)hUsbDeviceFS.pClassData;

	switch (cmd)
	{
	case SEND_ENCAPSULATED_COMMAND:
		rndis_data.MessageType=buf32[0];
		rndis_data.RequestId=buf32[2];
		if(buf32[0]==RNDIS_MSG_INIT){
			//SEC RNDIS_MSG_INIT
			RNDIS_Disconnect();
			rndis_data.MajorVersion=buf32[3];
			rndis_data.MinorVersion=buf32[4];
			rndis_data.MaxTransferSize=buf32[5];
			rndis_state=RNDIS_STATE_NORMAL;
			hrndis->TxState=0;
			USBD_RNDIS_TransmitControl(&hUsbDeviceFS, (uint8_t*)response, 8);
		} else if(buf32[0]==RNDIS_MSG_HALT){
			//SEC RNDIS_MSG_HALT
			hrndis->TxState=1;
			RNDIS_Disconnect();
			rndis_state=RNDIS_STATE_HALTED;
		} else if(buf32[0]==RNDIS_MSG_QUERY){
			//SEC RNDIS_MSG_QUERY
			rndis_data.Oid=buf32[3];
			rndis_data.InformationBufferLength=buf32[4];
			rndis_data.InformationBufferOffset=buf32[5];
			rndis_data.DeviceVcHandle=buf32[6];
			USBD_RNDIS_TransmitControl(&hUsbDeviceFS, (uint8_t*)response, 8);
		} else if(buf32[0]==RNDIS_MSG_SET){
			//SEC RNDIS_MSG_SET
			USBD_RNDIS_TransmitControl(&hUsbDeviceFS, (uint8_t*)response, 8);
		} else if(buf32[0]==RNDIS_MSG_RESET){
			//SEC RNDIS_MSG_RESET
			USBD_RNDIS_TransmitControl(&hUsbDeviceFS, (uint8_t*)response, 8);
		} else if(buf32[0]==RNDIS_MSG_KEEPALIVE){
			//SEC RNDIS_MSG_KEEPALIVE
			USBD_RNDIS_TransmitControl(&hUsbDeviceFS, (uint8_t*)response, 8);
		} else {
			//SEC OTHER
		}
		break;
	case GET_ENCAPSULATED_RESPONSE:
		if(rndis_data.MessageType==RNDIS_MSG_INIT){
			//GER RNDIS_MSG_INIT
			buf32[pos++]=RNDIS_MSG_INIT_C;							//MessageType			Specifies the type of message being sent. Set to 0x80000002.
			pos++;
			buf32[pos++]=rndis_data.RequestId;						//RequestId				Specifies the Remote NDIS message ID value. This value is used to match messages sent by the host with device responses.
			buf32[pos++]=RNDIS_STATUS_SUCCESS;						//Status				Specifies RNDIS_STATUS_SUCCESS if the device initialized successfully; otherwise, it specifies an error code that indicates the failure.
			buf32[pos++]=1;											//MajorVersion			Specifies the highest Remote NDIS major protocol version supported by the device.
			buf32[pos++]=0;											//MinorVersion			Specifies the highest Remote NDIS minor protocol version supported by the device.
			buf32[pos++]=RNDIS_DF_CONNECTIONLESS;					//DeviceFlags			Specifies the miniport driver type as either connectionless or connection-oriented. This value can be one of the following:
																	//						RNDIS_DF_CONNECTIONLESS 0x00000001
																	//						RNDIS_DF_CONNECTION_ORIENTED 0x00000002
			buf32[pos++]=RNDIS_MEDIUM_802_3;						//Medium				Specifies the medium supported by the device. Set to RNDIS_MEDIUM_802_3 (0x00000000)
			buf32[pos++]=1;											//MaxPacketsPerMessage	Specifies the maximum number of Remote NDIS data messages that the device can handle in a single transfer to it. This value should be at least one.
			buf32[pos++]=1580;										//MaxTransferSize		Specifies the maximum size in bytes of any single bus data transfer that the device expects to receive from the host.
			buf32[pos++]=0;											//PacketAlignmentFactor	Specifies the byte alignment that the device expects for each Remote NDIS message that is part of a multimessage transfer to it. This value is specified in powers of 2. For example, this value is set to three to indicate 8-byte alignment. This value has a maximum setting of seven, which specifies 128-byte alignment.
			buf32[pos++]=0;											//AFListOffset			Reserved for connection-oriented devices. Set value to zero.
			buf32[pos++]=0;											//AFListSize			Reserved for connection-oriented devices. Set value to zero.
		} else if(rndis_data.MessageType==RNDIS_MSG_QUERY){
			//GER RNDIS_MSG_QUERY OID
			uint32_t temp=0;

			buf32[pos++]=RNDIS_MSG_QUERY_C;
			pos++;
			buf32[pos++]=rndis_data.RequestId;
			buf32[pos++]=RNDIS_STATUS_SUCCESS;

			switch(rndis_data.Oid){
			case RNDIS_OID_GEN_SUPPORTED_LIST:
				temp=sizeof(OID_GEN_SUPPORTED);
				buf32[pos++]=temp;
				buf32[pos++]=16;
				USBD_memcpy(buf32+pos, OID_GEN_SUPPORTED, temp);
				pos+=temp;
				break;
			case RNDIS_OID_GEN_PHYSICAL_MEDIUM:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=0;
				break;
			case RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=1500;
				break;
			case RNDIS_OID_GEN_LINK_SPEED:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=100000/100;
				break;
			case RNDIS_OID_GEN_MEDIA_CONNECT_STATUS:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=NdisMediaStateConnected;
				break;
			case RNDIS_OID_802_3_MAXIMUM_LIST_SIZE:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=1;
				break;
			case RNDIS_OID_802_3_CURRENT_ADDRESS:
				buf32[pos++]=6;
				buf32[pos++]=16;
				buf32[pos++]=0x00757840 | (DeviceID_8[0]<<24);
				buf32[pos++]=DeviceID_8[2]<<8 | DeviceID_8[1];
				len=buf32[1]=pos*4-2;
				break;
			case RNDIS_OID_802_3_PERMANENT_ADDRESS:
				buf32[pos++]=6;
				buf32[pos++]=16;
				buf32[pos++]=0xDD757840;
				buf32[pos++]=0xFFEE;
				len=buf32[1]=pos*4-2;
				break;
			case RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=1558;
				break;
			case RNDIS_OID_GEN_XMIT_OK:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=rndis_oid_gen_xmit_ok & 0xffffffff;
				break;
			case RNDIS_OID_GEN_RCV_OK:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=rndis_oid_gen_rcv_ok & 0xffffffff;
				break;
			case RNDIS_OID_GEN_RCV_ERROR:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=0;
				break;
			case RNDIS_OID_GEN_RCV_NO_BUFFER:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=0;
				break;
			case RNDIS_OID_GEN_XMIT_ERROR:
				buf32[pos++]=4;
				buf32[pos++]=16;
				buf32[pos++]=0;
				break;
			case RNDIS_OID_GEN_VENDOR_ID:
				buf32[pos++]=3;
				buf32[pos++]=20;
				buf32[pos++]=0x00757840;
				len=buf32[1]=pos*4-1;
				break;
			case RNDIS_OID_GEN_VENDOR_DESCRIPTION:
				buf32[pos++]=sizeof(nome);
				buf32[pos++]=16;
				USBD_memcpy((char*)&buf32[pos++], nome, sizeof(nome));
				len=buf32[1]=pos*4+sizeof(nome);
				break;
			default:
				buf32[pos++]=0;
				buf32[pos++]=20;
				break;
			}
		} else if(rndis_data.MessageType==RNDIS_MSG_SET){
			//GER RNDIS_MSG_SET OID
			buf32[pos++]=RNDIS_MSG_SET_C;
			pos++;
			buf32[pos++]=rndis_data.RequestId;
			buf32[pos++]=RNDIS_STATUS_SUCCESS;
			switch(rndis_data.Oid){
			case RNDIS_OID_802_3_PERMANENT_ADDRESS:
				break;
			case RNDIS_OID_GEN_MEDIA_CONNECT_STATUS:
				break;
			case RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE:
				break;
			default:
				break;
			}
		} else if(rndis_data.MessageType==RNDIS_MSG_RESET){
			//GER RNDIS_MSG_RESET
			buf32[pos++]=RNDIS_MSG_RESET_C;
			pos++;
			buf32[pos++]=RNDIS_STATUS_SUCCESS;
			buf32[pos++]=0;
		} else if(rndis_data.MessageType==RNDIS_MSG_KEEPALIVE){
			//GER RNDIS_MSG_KEEPALIVE
			buf32[pos++]=RNDIS_MSG_KEEPALIVE_C;
			pos++;
			buf32[pos++]=rndis_data.RequestId;
			buf32[pos++]=RNDIS_STATUS_SUCCESS;
			//Can send packages

		} else {
			//GER OTHER
		}
		if(!len) len=buf32[1]=pos*4;
		USBD_CtlSendData(&hUsbDeviceFS, pbuf, len);
		break;
	default:
		break;
	}
	return (USBD_OK);
	/* USER CODE END 5 */
}

/**
 * @brief  RNDIS_Receive_FS
 *         Data received over USB OUT endpoint are sent over RNDIS interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         untill exiting this function. If you exit this function before transfer
 *         is complete on RNDIS interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
uint64_t timestamp;
static int8_t RNDIS_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
	BaseType_t xHigherPriorityTaskWoken;
	static uint16_t len=0;

	if(*Len>64){
		*Len=64;
	}
	memcpy(UserRxBufferFS+len, UserRxBufferFS_Temp, *Len);
	len+=(*Len);

	if(*Len!=64 && xEMACTaskHandle!=0){
		UserRxSize=len;
		//timestamp=ullGetHighResolutionTime();
		len=0;
		vTaskNotifyGiveFromISR(xEMACTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		rndis_oid_gen_rcv_ok++;
	} else {
		USBD_RNDIS_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS_Temp);
		USBD_RNDIS_ReceivePacket(&hUsbDeviceFS);
	}
	return (USBD_OK);
	/* USER CODE END 6 */
}

/**
 * @brief  RNDIS_Transmit_FS
 *         Data send over USB IN endpoint are sent over RNDIS interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be send
 * @param  Len: Number of data to be send (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t RNDIS_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
	uint32_t *buffer=(uint32_t*)UserTxBufferFS;
	uint8_t result = USBD_OK;
	/* USER CODE BEGIN 7 */
	USBD_RNDIS_HandleTypeDef *hrndis = (USBD_RNDIS_HandleTypeDef*)hUsbDeviceFS.pClassData;
	if (hrndis->TxState != 0 || rndis_state!=RNDIS_STATE_NORMAL){
		return USBD_BUSY;
	}

	if(Len>APP_TX_DATA_SIZE){
		Len=APP_TX_DATA_SIZE;
	}

	buffer[0]=0x00000001;	//MessageType
	buffer[1]=Len+44;		//MessageLength
	buffer[2]=36;			//DataOffset
	buffer[3]=Len;			//DataLength
	buffer[4]=0;			//OOBDataOffset
	buffer[5]=0;			//OOBDataLength
	buffer[6]=0;			//NumOOBDataElements
	buffer[7]=0;			//PerPacketInfoOffset
	buffer[8]=0;			//PerPacketInfoLength
	buffer[9]=0;			//VcHandle
	buffer[10]=0;			//Reserved

	memcpy(UserTxBufferFS+44, Buf, Len);

	USBD_RNDIS_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len+44);
	result = USBD_RNDIS_TransmitPacket(&hUsbDeviceFS);
	rndis_oid_gen_xmit_ok++;
	/* USER CODE END 7 */
	return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

BaseType_t xNetworkInterfaceInitialise( void ){
	/* When returning non-zero, the stack will become active and
    start DHCP (if configured) */
	BaseType_t ret=0;

	/* The deferred interrupt handler task is created at the highest
	possible priority to ensure the interrupt handler can return directly
	to it.  The task's handle is stored in xEMACTaskHandle so interrupts can
	notify the task when there is something to process. */
	if(rndis_state==RNDIS_STATE_NORMAL){
		ret=1;
		if(xEMACTaskHandle==0){
			xTaskCreate( prvEMACHandlerTask, "EMAC", configEMAC_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, &xEMACTaskHandle );
		}
	}

	return ret;
}


BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxDescriptor, BaseType_t xReleaseAfterSend  ){
	/* Simple network interfaces (as opposed to more efficient zero copy network
	    interfaces) just use Ethernet peripheral driver library functions to copy
	    data from the FreeRTOS+TCP buffer into the peripheral driver's own buffer.
	    This example assumes SendData() is a peripheral driver library function that
	    takes a pointer to the start of the data to be sent and the length of the
	    data to be sent as two separate parameters.  The start of the data is located
	    by pxDescriptor->pucEthernetBuffer.  The length of the data is located
	    by pxDescriptor->xDataLength. */

	uint8_t retries=0;
	while(RNDIS_Transmit_FS( pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength) ){
		vTaskDelay(5);
		retries++;
		if(retries>=5){
			break;
		}
	}

	iptraceNETWORK_INTERFACE_TRANSMIT();

	/* Call the standard trace macro to log the send event. */

	if( xReleaseAfterSend != pdFALSE )
	{
		/* It is assumed SendData() copies the data out of the FreeRTOS+TCP Ethernet
	        buffer.  The Ethernet buffer is therefore no longer needed, and must be
	        freed for re-use. */
		vReleaseNetworkBufferAndDescriptor( pxDescriptor );
	}

	return pdTRUE;

}

//void vNetworkInterfaceAllocateRAMToBuffers( NetworkBufferDescriptor_t pxNetworkBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ] ){
//}

BaseType_t xGetPhyLinkStatus( void ){
		BaseType_t xReturn;

		if( rndis_state == RNDIS_STATE_NORMAL )
		{
			xReturn = pdPASS;
		}
		else
		{
			xReturn = pdFAIL;
		}

		return xReturn;
}

static void prvEMACHandlerTask( void *pvParameters ){
	NetworkBufferDescriptor_t *pxBufferDescriptor;
	size_t xBytesReceived;
	/* Used to indicate that xSendEventStructToIPTask() is being called because
	of an Ethernet receive event. */
	IPStackEvent_t xRxEvent;

	for( ;; )
	{
		/* Wait for the Ethernet MAC interrupt to indicate that another packet
	        has been received.  The task notification is used in a similar way to a
	        counting semaphore to count Rx events, but is a lot more efficient than
	        a semaphore. */
		ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

		/* See how much data was received.  Here it is assumed ReceiveSize() is
	        a peripheral driver function that returns the number of bytes in the
	        received Ethernet frame. */

		xBytesReceived = UserRxSize;
		//timestamp=ullGetHighResolutionTime()-timestamp;

		if( xBytesReceived > 44 )
		{
			xBytesReceived-=44;
			/* Allocate a network buffer descriptor that points to a buffer
	            large enough to hold the received frame.  As this is the simple
	            rather than efficient example the received data will just be copied
	            into this buffer. */
			pxBufferDescriptor = pxGetNetworkBufferWithDescriptor( xBytesReceived, 0 );

			if( pxBufferDescriptor != NULL )
			{
				/* pxBufferDescriptor->pucEthernetBuffer now points to an Ethernet
	                buffer large enough to hold the received data.  Copy the
	                received data into pcNetworkBuffer->pucEthernetBuffer.  Here it
	                is assumed ReceiveData() is a peripheral driver function that
	                copies the received data into a buffer passed in as the function's
	                parameter.  Remember! While is is a simple robust technique -
	                it is not efficient.  An example that uses a zero copy technique
	                is provided further down this page. */
				memcpy(pxBufferDescriptor->pucEthernetBuffer, UserRxBufferFS+44, xBytesReceived);
				UserRxSize=0;
				pxBufferDescriptor->xDataLength = xBytesReceived;

				/* See if the data contained in the received Ethernet frame needs
	                to be processed.  NOTE! It is preferable to do this in
	                the interrupt service routine itself, which would remove the need
	                to unblock this task for packets that don't need processing. */
				if( eConsiderFrameForProcessing( pxBufferDescriptor->pucEthernetBuffer )
						== eProcessBuffer )
				{
					/* The event about to be sent to the TCP/IP is an Rx event. */
					xRxEvent.eEventType = eNetworkRxEvent;

					/* pvData is used to point to the network buffer descriptor that
	                    now references the received data. */
					xRxEvent.pvData = ( void * ) pxBufferDescriptor;

					/* Send the data to the TCP/IP stack. */
					if( xSendEventStructToIPTask( &xRxEvent, 0 ) == pdFALSE )
					{
						/* The buffer could not be sent to the IP task so the buffer
	                        must be released. */
						vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );

						/* Make a call to the standard trace macro to log the
	                        occurrence. */
						iptraceETHERNET_RX_EVENT_LOST();
					}
					else
					{
						/* The message was successfully sent to the TCP/IP stack.
	                        Call the standard trace macro to log the occurrence. */
						iptraceNETWORK_INTERFACE_RECEIVE();
					}
				}
				else
				{
					/* The Ethernet frame can be dropped, but the Ethernet buffer
	                    must be released. */
					vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );
				}
			}
			else
			{
				/* The event was lost because a network buffer was not available.
	                Call the standard trace macro to log the occurrence. */
				iptraceETHERNET_RX_EVENT_LOST();
			}
		}
		USBD_RNDIS_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS_Temp);
		USBD_RNDIS_ReceivePacket(&hUsbDeviceFS);

	}
}



/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

