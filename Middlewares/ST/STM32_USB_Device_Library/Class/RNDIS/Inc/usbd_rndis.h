#pragma once
#include <rndis.h>

typedef struct {
	uint32_t MessageType;						//Specifies the type of message being sent.
	uint32_t RequestId;							//Specifies the Remote NDIS message ID value. This value is used to match messages sent by the host with device responses.
	uint32_t MajorVersion;						//Specifies the Remote NDIS protocol version implemented by the host. The current specification uses RNDIS_MAJOR_VERSION = 1.
	uint32_t MinorVersion;						//Specifies the Remote NDIS protocol version implemented by the host. The current specification uses RNDIS_MINOR_VERSION = 0.
	uint32_t MaxTransferSize;					//Specifies the maximum size in bytes of any single bus data transfer that the host expects to receive from the device. Typically, each bus data transfer accommodates a single Remote NDIS message. However, the device may bundle several Remote NDIS messages that contain data packets into a single transfer (see REMOTE_NDIS_PACKET_MSG).
	uint32_t Oid;								//Specifies the NDIS OID that identifies the parameter being queried.
	uint32_t InformationBufferLength;			//Specifies in bytes the length of the input data for the query. Set to zero when there is no OID input buffer.
	uint32_t InformationBufferOffset;			//Specifies the byte offset, from the beginning of the RequestId field, at which input data for the query is located. Set to zero if there is no OID input buffer.
	uint32_t DeviceVcHandle;					//Reserved for connection-oriented devices. Set to zero.
} RNDIS_DATA;
enum{
	SEND_ENCAPSULATED_COMMAND               =0x00,
	GET_ENCAPSULATED_RESPONSE               =0x01,
};

/**
  ******************************************************************************
  * @file    usbd_rndis.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   header file for the usbd_rbdus.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_RNDIS_H
#define __USB_RNDIS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup usbd_rndis
  * @brief This file is the Header file for usbd_rndis.c
  * @{
  */


/** @defgroup usbd_rndis_Exported_Defines
  * @{
  */
#define RNDIS_IN_EP                                   0x81  /* EP1 for data IN */
#define RNDIS_OUT_EP                                  0x01  /* EP1 for data OUT */
#define RNDIS_CMD_EP                                  0x82  /* EP2 for RNDIS commands */

/* RNDIS Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define RNDIS_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define RNDIS_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define RNDIS_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */

#define USB_RNDIS_CONFIG_DESC_SIZ                     62
#define RNDIS_DATA_HS_IN_PACKET_SIZE                  RNDIS_DATA_HS_MAX_PACKET_SIZE
#define RNDIS_DATA_HS_OUT_PACKET_SIZE                 RNDIS_DATA_HS_MAX_PACKET_SIZE

#define RNDIS_DATA_FS_IN_PACKET_SIZE                  RNDIS_DATA_FS_MAX_PACKET_SIZE
#define RNDIS_DATA_FS_OUT_PACKET_SIZE                 RNDIS_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  RNDIS definitions                                                    */
/*---------------------------------------------------------------------*/
#define RNDIS_SEND_ENCAPSULATED_COMMAND               0x00
#define RNDIS_GET_ENCAPSULATED_RESPONSE               0x01
#define RNDIS_SET_COMM_FEATURE                        0x02
#define RNDIS_GET_COMM_FEATURE                        0x03
#define RNDIS_CLEAR_COMM_FEATURE                      0x04
#define RNDIS_SET_LINE_CODING                         0x20
#define RNDIS_GET_LINE_CODING                         0x21
#define RNDIS_SET_CONTROL_LINE_STATE                  0x22
#define RNDIS_SEND_BREAK                              0x23

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}USBD_RNDIS_LineCodingTypeDef;

typedef struct _USBD_RNDIS_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);
  int8_t (* Receive)       (uint8_t *, uint32_t *);

}USBD_RNDIS_ItfTypeDef;


typedef struct
{
  uint32_t data[RNDIS_DATA_HS_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
}
USBD_RNDIS_HandleTypeDef;



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_RNDIS;
#define USBD_RNDIS_CLASS    &USBD_RNDIS
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_RNDIS_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                      USBD_RNDIS_ItfTypeDef *fops);

uint8_t  USBD_RNDIS_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_RNDIS_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);

uint8_t  USBD_RNDIS_ReceivePacket      (USBD_HandleTypeDef *pdev);

uint8_t  USBD_RNDIS_TransmitPacket     (USBD_HandleTypeDef *pdev);

uint8_t  USBD_RNDIS_TransmitControl(USBD_HandleTypeDef *pdev, uint8_t *buff, uint16_t length);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_RNDIS_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
