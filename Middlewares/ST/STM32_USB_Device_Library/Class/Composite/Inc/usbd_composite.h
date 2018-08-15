/**
  ******************************************************************************
  * @file    usbd_composite.h
  * @author  Luiz Renault
  * @version V2.4.2
  * @date    29-September-2017
  * @brief   header file for the usbd_composite.c file.
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
#ifndef __USB_COMPOSITE_H
#define __USB_COMPOSITE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup usbd_composite
  * @brief This file is the Header file for usbd_composite.c
  * @{
  */ 


/** @defgroup usbd_composite_Exported_Defines
  * @{
  */ 
#define COMPOSITE_IN_EP                                   0x81  /* EP1 for data IN */
#define COMPOSITE_OUT_EP                                  0x01  /* EP1 for data OUT */
#define COMPOSITE_CMD_EP                                  0x82  /* EP2 for COMPOSITE commands */

/* COMPOSITE Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define COMPOSITE_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define COMPOSITE_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define COMPOSITE_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */


#define USB_COMPOSITE_MAX_CLASSES				  		  5
#define USB_COMPOSITE_IFC_ASSOC_DESC_SIZ				  8
#define USB_COMPOSITE_CONFIG_DESC_SIZ                     9
#define COMPOSITE_DATA_HS_IN_PACKET_SIZE                  COMPOSITE_DATA_HS_MAX_PACKET_SIZE
#define COMPOSITE_DATA_HS_OUT_PACKET_SIZE                 COMPOSITE_DATA_HS_MAX_PACKET_SIZE

#define COMPOSITE_DATA_FS_IN_PACKET_SIZE                  COMPOSITE_DATA_FS_MAX_PACKET_SIZE
#define COMPOSITE_DATA_FS_OUT_PACKET_SIZE                 COMPOSITE_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  COMPOSITE definitions                                                    */
/*---------------------------------------------------------------------*/
#define COMPOSITE_SEND_ENCAPSULATED_COMMAND               0x00
#define COMPOSITE_GET_ENCAPSULATED_RESPONSE               0x01
#define COMPOSITE_SET_COMM_FEATURE                        0x02
#define COMPOSITE_GET_COMM_FEATURE                        0x03
#define COMPOSITE_CLEAR_COMM_FEATURE                      0x04
#define COMPOSITE_SET_LINE_CODING                         0x20
#define COMPOSITE_GET_LINE_CODING                         0x21
#define COMPOSITE_SET_CONTROL_LINE_STATE                  0x22
#define COMPOSITE_SEND_BREAK                              0x23

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
}USBD_COMPOSITE_LineCodingTypeDef;

typedef struct _USBD_COMPOSITE_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);   
  int8_t (* Receive)       (uint8_t *, uint32_t *);  

}USBD_COMPOSITE_ItfTypeDef;


typedef struct
{
  uint32_t data[COMPOSITE_DATA_HS_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;    
  uint8_t  *RxBuffer;  
  uint8_t  *TxBuffer;   
  uint32_t RxLength;
  uint32_t TxLength;    
  
  __IO uint32_t TxState;     
  __IO uint32_t RxState;    
}
USBD_COMPOSITE_HandleTypeDef;

typedef struct
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bFirstInterface;
	uint8_t bInterfaceCount;
	uint8_t bFunctionClass;
	uint8_t bFunctionSubClass;
	uint8_t bFunctionProtocol;
	uint8_t iFunction;
} USBD_COMPOSITE_ItfAssocDescriptor;

typedef struct
{
	uint8_t bFunctionClass;
	uint8_t bFunctionSubClass;
	uint8_t bFunctionProtocol;
	uint8_t bInterfaces;
	USBD_ClassTypeDef *pClass;
	void *pClassData;
	void *pUserData;
	uint8_t inEP;
	uint8_t outEP;
	uint8_t inEPn[15];
	uint8_t outEPn[15];
	uint8_t inEPa[15];
	uint8_t outEPa[15];
} USBD_COMPOSITE_ClassData;

/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 
  
/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_COMPOSITE;
#define USBD_COMPOSITE_CLASS    &USBD_COMPOSITE
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */

USBD_StatusTypeDef  USBD_COMPOSITE_RegisterClass(USBD_HandleTypeDef *pdev, uint8_t bFunctionClass, uint8_t bFunctionSubClass, uint8_t bFunctionProtocol);

uint8_t  USBD_COMPOSITE_LL_EP_Conversion  (USBD_HandleTypeDef *pdev, uint8_t  ep_addr);

uint8_t  USBD_COMPOSITE_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                      USBD_COMPOSITE_ItfTypeDef *fops);

uint8_t  USBD_COMPOSITE_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_COMPOSITE_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);
  
uint8_t  USBD_COMPOSITE_ReceivePacket      (USBD_HandleTypeDef *pdev);

uint8_t  USBD_COMPOSITE_TransmitPacket     (USBD_HandleTypeDef *pdev);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_COMPOSITE_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
