/**
  ******************************************************************************
  * @file    usbd_cdc.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   header file for the usbd_cdc.c file.
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
#ifndef __USB_CDC_H
#define __USB_CDC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include "usbd_ioreq.h"
#include "rndis_protocol.h"

#define RNDIS_NOTIFICATION_IN_EP 0x81
#define RNDIS_DATA_IN_EP         0x82
#define RNDIS_DATA_OUT_EP        0x01

#define RNDIS_NOTIFICATION_IN_SZ 0x08
#define RNDIS_DATA_IN_SZ         0x40
#define RNDIS_DATA_OUT_SZ        0x40

#define RNDIS_MTU        1500                           /* MTU value */
#define RNDIS_LINK_SPEED 12000000                       /* Link baudrate (12Mbit/s for USB-FS) */
#define RNDIS_VENDOR     "fetisov"                      /* NIC vendor name */
#define RNDIS_HWADDR     0x20,0x89,0x84,0x6A,0x96,0xAB  /* MAC-address to set to host interface */

//typedef void (*rndis_rxproc_t)(const char *data, int size);
extern USBD_ClassTypeDef  USBD_CDC;
#define USBD_CDC_CLASS    &USBD_CDC

extern usb_eth_stat_t usb_eth_stat;
extern rndis_state_t rndis_state;

//extern rndis_rxproc_t rndis_rxproc;

bool   rndis_can_send(void);
bool   rndis_send(const void *data, int size);


#ifdef __cplusplus
}
#endif

#endif  /* __USB_CDC_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
