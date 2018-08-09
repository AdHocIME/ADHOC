/**
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                CDC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *      
  *  @endverbatim
  *                                  
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

#include <stdio.h>
#include <stdlib.h>

const uint32_t OIDSupportedList[] =
{
    OID_GEN_SUPPORTED_LIST,
    OID_GEN_HARDWARE_STATUS,
    OID_GEN_MEDIA_SUPPORTED,
    OID_GEN_MEDIA_IN_USE,
    OID_GEN_MAXIMUM_FRAME_SIZE,
    OID_GEN_LINK_SPEED,
    OID_GEN_TRANSMIT_BLOCK_SIZE,
    OID_GEN_RECEIVE_BLOCK_SIZE,
    OID_GEN_VENDOR_ID,
    OID_GEN_VENDOR_DESCRIPTION,
    OID_GEN_VENDOR_DRIVER_VERSION,
    OID_GEN_CURRENT_PACKET_FILTER,
    OID_GEN_MAXIMUM_TOTAL_SIZE,
    OID_GEN_PROTOCOL_OPTIONS,
    OID_GEN_MAC_OPTIONS,
    OID_GEN_MEDIA_CONNECT_STATUS,
    OID_GEN_MAXIMUM_SEND_PACKETS,
    OID_802_3_PERMANENT_ADDRESS,
    OID_802_3_CURRENT_ADDRESS,
    OID_802_3_MULTICAST_LIST,
    OID_802_3_MAXIMUM_LIST_SIZE,
    OID_802_3_MAC_OPTIONS
};

#define OID_LIST_LENGTH (sizeof(OIDSupportedList) / sizeof(*OIDSupportedList))
#define ENC_BUF_SIZE    (OID_LIST_LENGTH * 4 + 32)

#define ETH_HEADER_SIZE             14
#define ETH_MAX_PACKET_SIZE         ETH_HEADER_SIZE + RNDIS_MTU
#define ETH_MIN_PACKET_SIZE         60
#define RNDIS_RX_BUFFER_SIZE        (ETH_MAX_PACKET_SIZE + sizeof(rndis_data_packet_t))

/* Default the size of the stack used by the EMAC deferred handler task to twice
the size of the stack used by the idle task - but allow this to be overridden in
FreeRTOSConfig.h as configMINIMAL_STACK_SIZE is a user definable constant. */
#ifndef configEMAC_TASK_STACK_SIZE
	#define configEMAC_TASK_STACK_SIZE ( 2 * configMINIMAL_STACK_SIZE )
#endif
#define EMAC_IF_RX_EVENT        1UL


static const uint8_t station_hwaddr[6] = { RNDIS_HWADDR };
static const uint8_t permanent_hwaddr[6] = { RNDIS_HWADDR };


USBD_HandleTypeDef *pDev;

char rndis_rx_buffer[RNDIS_RX_BUFFER_SIZE];
uint8_t encapsulated_buffer[ENC_BUF_SIZE];
usb_eth_stat_t usb_eth_stat = { 0, 0, 0, 0 };
uint32_t oid_packet_filter = 0x0000000;
__ALIGN_BEGIN char rndis_rx_buffer[RNDIS_RX_BUFFER_SIZE] __ALIGN_END;
__ALIGN_BEGIN uint8_t usb_rx_buffer[RNDIS_DATA_OUT_SZ] __ALIGN_END ;
//rndis_rxproc_t rndis_rxproc = NULL;
uint8_t *rndis_tx_ptr = NULL;
int rndis_first_tx = 1;
int rndis_tx_size = 0;
int rndis_sended = 0;
char data_to_send[300 + 14 + 4];
rndis_state_t rndis_state;
int sended = 0;
static TaskHandle_t xEMACTaskHandle = NULL;
static volatile uint32_t ulISREvents;

char rndis_rx_tcp_buffer[RNDIS_RX_BUFFER_SIZE];
int rndis_tx_tcp_size = 0;


static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t usbd_rndis_sof(USBD_HandleTypeDef *pdev);

static uint8_t rndis_iso_in_incomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t rndis_iso_out_incomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_CDC_GetFSCfgDesc (uint16_t *length);

static uint8_t  *usbd_rndis_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t usbd_cdc_transfer(void *pdev);

static void handle_packet(const char *data, int size);

void rndis_query_cmplt(void *pdev, int status, const void *data, int size);

void rndis_query_cmplt32(void *pdev, int status, uint32_t data);

void rndis_query(void  *pdev);

void rndis_handle_set_msg(void  *pdev);

static void handle_packet(const char *data, int size);

static void rndis_rxproc(const char *data, int size);

static void prvEMACHandlerTask( void *pvParameters );

/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Variables
  * @{
  */ 


/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_CDC = 
{
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_CDC_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_CDC_EP0_RxReady,
  USBD_CDC_DataIn,
  USBD_CDC_DataOut,
  usbd_rndis_sof,
  rndis_iso_in_incomplete,
  rndis_iso_out_incomplete,
  USBD_CDC_GetFSCfgDesc,
  USBD_CDC_GetFSCfgDesc,    
  USBD_CDC_GetFSCfgDesc,
  usbd_rndis_GetDeviceQualifierDesc,
};

#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t usbd_cdc_CfgDesc[] __ALIGN_END =
{
    /* Configuration descriptor */

    9,                                 /* bLength         = 9 bytes. */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType = CONFIGURATION */
    0xDE, 0xAD,                        /* wTotalLength    = sizeof(usbd_cdc_CfgDesc) */
    0x02,                              /* bNumInterfaces  = 2 (RNDIS spec). */
    0x01,                              /* bConfValue      = 1 */
    0x00,                              /* iConfiguration  = unused. */
    0x40,                              /* bmAttributes    = Self-Powered. */
    0x01,                              /* MaxPower        = x2mA */

    /* IAD descriptor */

    0x08, /* bLength */
    0x0B, /* bDescriptorType */
    0x00, /* bFirstInterface */
    0x02, /* bInterfaceCount */
    0xE0, /* bFunctionClass (Wireless Controller) */
    0x01, /* bFunctionSubClass */
    0x03, /* bFunctionProtocol */
    0x00, /* iFunction */

    /* Interface 0 descriptor */

    9,                             /* bLength */
    USB_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType = INTERFACE */
    0x00,                          /* bInterfaceNumber */
    0x00,                          /* bAlternateSetting */
    1,                             /* bNumEndpoints */
    0xE0,                          /* bInterfaceClass: Wireless Controller */
    0x01,                          /* bInterfaceSubClass */
    0x03,                          /* bInterfaceProtocol */
    0,                             /* iInterface */

    /* Interface 0 functional descriptor */

    /* Header Functional Descriptor */
    0x05, /* bFunctionLength */
    0x24, /* bDescriptorType = CS Interface */
    0x00, /* bDescriptorSubtype */
    0x10, /* bcdCDC = 1.10 */
    0x01, /* bcdCDC = 1.10 */

    /* Call Management Functional Descriptor */
    0x05, /* bFunctionLength */
    0x24, /* bDescriptorType = CS Interface */
    0x01, /* bDescriptorSubtype = Call Management */
    0x00, /* bmCapabilities */
    0x01, /* bDataInterface */

    /* Abstract Control Management Functional Descriptor */
    0x04, /* bFunctionLength */
    0x24, /* bDescriptorType = CS Interface */
    0x02, /* bDescriptorSubtype = Abstract Control Management */
    0x00, /* bmCapabilities = Device supports the notification Network_Connection */

    /* Union Functional Descriptor */
    0x05, /* bFunctionLength */
    0x24, /* bDescriptorType = CS Interface */
    0x06, /* bDescriptorSubtype = Union */
    0x00, /* bControlInterface = "RNDIS Communications Control" */
    0x01, /* bSubordinateInterface0 = "RNDIS Ethernet Data" */

    /* Endpoint descriptors for Communication Class Interface */

    7,                            /* bLength         = 7 bytes */
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType = ENDPOINT */
    RNDIS_NOTIFICATION_IN_EP,     /* bEndpointAddr   = IN - EP3 */
    0x03,                         /* bmAttributes    = Interrupt endpoint */
    8, 0,                         /* wMaxPacketSize */
    0x01,                         /* bInterval       = 1 ms polling from host */

    /* Interface 1 descriptor */

    9,                             /* bLength */
    USB_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    0x01,                          /* bInterfaceNumber */
    0x00,                          /* bAlternateSetting */
    2,                             /* bNumEndpoints */
    0x0A,                          /* bInterfaceClass: CDC */
    0x00,                          /* bInterfaceSubClass */
    0x00,                          /* bInterfaceProtocol */
    0x00,                          /* uint8  iInterface */

    /* Endpoint descriptors for Data Class Interface */

    7,                            /* bLength         = 7 bytes */
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType = ENDPOINT [IN] */
    RNDIS_DATA_IN_EP,             /* bEndpointAddr   = IN EP */
    0x02,                         /* bmAttributes    = BULK */
    RNDIS_DATA_IN_SZ, 0,          /* wMaxPacketSize */
    0,                            /* bInterval       = ignored for BULK */

    7,                            /* bLength         = 7 bytes */
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType = ENDPOINT [OUT] */
    RNDIS_DATA_OUT_EP,            /* bEndpointAddr   = OUT EP */
    0x02,                         /* bmAttributes    = BULK */
    RNDIS_DATA_OUT_SZ, 0,         /* wMaxPacketSize */
    0                             /* bInterval       = ignored for BULK */
};

__ALIGN_BEGIN uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_CDC_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
    /* Open EP IN */
	USBD_LL_OpenEP(pdev, RNDIS_NOTIFICATION_IN_EP, USBD_EP_TYPE_INTR, RNDIS_NOTIFICATION_IN_SZ);
    
    /* Open EP OUT */
	USBD_LL_OpenEP(pdev, RNDIS_DATA_IN_EP, USBD_EP_TYPE_BULK, RNDIS_DATA_IN_SZ);

    /* Open EP IN */
	USBD_LL_OpenEP(pdev, RNDIS_DATA_OUT_EP, USBD_EP_TYPE_BULK, RNDIS_DATA_OUT_SZ);
    
    /* Init Xfer states */
    //txState =0;
    //rxState =0;
	pDev = pdev;
    USBD_LL_PrepareReceive(pdev, RNDIS_DATA_OUT_EP, (uint8_t*)usb_rx_buffer, RNDIS_DATA_OUT_SZ);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if( xipInitTaskHandle != NULL )
	{
		vTaskNotifyGiveFromISR( xipInitTaskHandle, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
    return 0;
}

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  USBD_LL_CloseEP(pdev, RNDIS_NOTIFICATION_IN_EP);
  
  USBD_LL_CloseEP(pdev, RNDIS_DATA_IN_EP);
  
  USBD_LL_CloseEP(pdev, RNDIS_DATA_OUT_EP);
  
  return 0;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
	switch (req->bmRequest & USB_REQ_TYPE_MASK){
		case USB_REQ_TYPE_CLASS :
			if (req->wLength){
				if (req->bmRequest & 0x80)
				{
					//USBD_CtlSendData (pdev, encapsulated_buffer, req->wLength);
					USBD_CtlSendData(pdev, encapsulated_buffer, ((rndis_generic_msg_t *)encapsulated_buffer)->MessageLength);
				}
				else
				{
					USBD_CtlPrepareRx (pdev, encapsulated_buffer, req->wLength);
				}
			}
		default:
			break;
	}
	return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	epnum &= 0x0F;
	if (epnum == (RNDIS_DATA_IN_EP & 0x0F)){
		rndis_first_tx = 0;
		rndis_sended += sended;
		rndis_tx_size -= sended;
		rndis_tx_ptr += sended;
		sended = 0;
		usbd_cdc_transfer(pdev);
		if(rndis_tx_size<=0){
			usb_eth_stat.txok++;
		}
	}
	return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uint32_t xfer_count = USBD_LL_GetRxDataSize (pdev, epnum);
	static int rndis_received = 0;
	if (epnum == RNDIS_DATA_OUT_EP){
		if (rndis_received + xfer_count > RNDIS_RX_BUFFER_SIZE){
			usb_eth_stat.rxbad++;
			rndis_received = 0;
		}
		else{
			if (rndis_received + xfer_count <= RNDIS_RX_BUFFER_SIZE){
				memcpy(&rndis_rx_buffer[rndis_received], usb_rx_buffer, xfer_count);
				rndis_received += xfer_count;
				if (xfer_count != RNDIS_DATA_OUT_SZ){
					handle_packet(rndis_rx_buffer, rndis_received);
					rndis_received = 0;
				}
			}
			else{
				rndis_received = 0;
				usb_eth_stat.rxbad++;
			}
		}
	    USBD_LL_PrepareReceive(pdev, RNDIS_DATA_OUT_EP, (uint8_t*)usb_rx_buffer, RNDIS_DATA_OUT_SZ);
	}
    return USBD_OK;
}



/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
	switch (((rndis_generic_msg_t *)encapsulated_buffer)->MessageType){
		case REMOTE_NDIS_INITIALIZE_MSG:
			{
				rndis_initialize_cmplt_t *m;
				m = ((rndis_initialize_cmplt_t *)encapsulated_buffer);
				/* m->MessageID is same as before */
				m->MessageType = REMOTE_NDIS_INITIALIZE_CMPLT;
				m->MessageLength = sizeof(rndis_initialize_cmplt_t);
				m->MajorVersion = RNDIS_MAJOR_VERSION;
				m->MinorVersion = RNDIS_MINOR_VERSION;
				m->Status = RNDIS_STATUS_SUCCESS;
				m->DeviceFlags = RNDIS_DF_CONNECTIONLESS;
				m->Medium = RNDIS_MEDIUM_802_3;
				m->MaxPacketsPerTransfer = 1;
				m->MaxTransferSize = RNDIS_RX_BUFFER_SIZE;
				m->PacketAlignmentFactor = 0;
				m->AfListOffset = 0;
				m->AfListSize = 0;
				rndis_state = rndis_initialized;
				USBD_LL_Transmit(pdev, RNDIS_NOTIFICATION_IN_EP, (uint8_t *)"\x01\x00\x00\x00\x00\x00\x00\x00", 8);
			}
			break;

		case REMOTE_NDIS_QUERY_MSG:
			rndis_query(pdev);
			break;

		case REMOTE_NDIS_SET_MSG:
			rndis_handle_set_msg(pdev);
			break;

		case REMOTE_NDIS_RESET_MSG:
			{
				rndis_reset_cmplt_t * m;
				m = ((rndis_reset_cmplt_t *)encapsulated_buffer);
				rndis_state = rndis_uninitialized;
				m->MessageType = REMOTE_NDIS_RESET_CMPLT;
				m->MessageLength = sizeof(rndis_reset_cmplt_t);
				m->Status = RNDIS_STATUS_SUCCESS;
				m->AddressingReset = 1; /* Make it look like we did something */
				/* m->AddressingReset = 0; - Windows halts if set to 1 for some reason */
				USBD_LL_Transmit(pdev,RNDIS_NOTIFICATION_IN_EP,(uint8_t *)"\x01\x00\x00\x00\x00\x00\x00\x00",8);
			}
			break;

		case REMOTE_NDIS_KEEPALIVE_MSG:
			{
				rndis_keepalive_cmplt_t * m;
				m = (rndis_keepalive_cmplt_t *)encapsulated_buffer;
				m->MessageType = REMOTE_NDIS_KEEPALIVE_CMPLT;
				m->MessageLength = sizeof(rndis_keepalive_cmplt_t);
				m->Status = RNDIS_STATUS_SUCCESS;
			}
			/* We have data to send back */
			USBD_LL_Transmit(pdev,RNDIS_NOTIFICATION_IN_EP,(uint8_t *)"\x01\x00\x00\x00\x00\x00\x00\x00",8);
			break;

		default:
			break;
	}
	return USBD_OK;
}

/**
  * @brief  USBD_CDC_GetFSCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_CDC_GetFSCfgDesc (uint16_t *length){
    *length = sizeof(usbd_cdc_CfgDesc);
    usbd_cdc_CfgDesc[2] = sizeof(usbd_cdc_CfgDesc) & 0xFF;
    usbd_cdc_CfgDesc[3] = (sizeof(usbd_cdc_CfgDesc) >> 8) & 0xFF;
    return usbd_cdc_CfgDesc;
}

// Start Of Frame event management
static uint8_t usbd_rndis_sof(USBD_HandleTypeDef *pdev){
  //rndis_send();
  return usbd_cdc_transfer(pdev);
}

static uint8_t rndis_iso_in_incomplete(USBD_HandleTypeDef *pdev, uint8_t epnum){
  return usbd_cdc_transfer(pdev);
}

static uint8_t rndis_iso_out_incomplete(USBD_HandleTypeDef *pdev, uint8_t epnum){
  return usbd_cdc_transfer(pdev);
}

static uint8_t  *usbd_rndis_GetDeviceQualifierDesc (uint16_t *length){
  *length = sizeof (USBD_DeviceQualifierDesc);
  return USBD_DeviceQualifierDesc;
}

static uint8_t usbd_cdc_transfer(void *pdev){
	if (sended != 0 || rndis_tx_ptr == NULL || rndis_tx_size <= 0 || rndis_state!=rndis_data_initialized) return USBD_OK;
	if (rndis_first_tx)
	{
		static uint8_t first[RNDIS_DATA_IN_SZ];
		rndis_data_packet_t *hdr;

		hdr = (rndis_data_packet_t *)first;
		memset(hdr, 0, sizeof(rndis_data_packet_t));
		hdr->MessageType = REMOTE_NDIS_PACKET_MSG;
		hdr->MessageLength = sizeof(rndis_data_packet_t) + rndis_tx_size;
		hdr->DataOffset = sizeof(rndis_data_packet_t) - offsetof(rndis_data_packet_t, DataOffset);
		hdr->DataLength = rndis_tx_size;

		sended = RNDIS_DATA_IN_SZ - sizeof(rndis_data_packet_t);
		if (sended > rndis_tx_size) sended = rndis_tx_size;
		memcpy(first + sizeof(rndis_data_packet_t), rndis_tx_ptr, sended);

		USBD_LL_Transmit(pdev, RNDIS_DATA_IN_EP, (uint8_t *)first, sizeof(rndis_data_packet_t) + sended);
	}
	else
	{
		int n = rndis_tx_size;
		if (n > RNDIS_DATA_IN_SZ) n = RNDIS_DATA_IN_SZ;
		USBD_LL_Transmit(pdev, RNDIS_DATA_IN_EP, rndis_tx_ptr, n);
		sended = n;
	}
	return USBD_OK;
}

static void handle_packet(const char *data, int size){
	rndis_data_packet_t *p;
	p = (rndis_data_packet_t *)data;
	if (size < sizeof(rndis_data_packet_t)) return;
	if (p->MessageType != REMOTE_NDIS_PACKET_MSG || p->MessageLength != size) return;
	if (p->DataOffset + offsetof(rndis_data_packet_t, DataOffset) + p->DataLength != size)
	{
		usb_eth_stat.rxbad++;
		return;
	}
	usb_eth_stat.rxok++;
	rndis_rxproc(&rndis_rx_buffer[p->DataOffset + offsetof(rndis_data_packet_t, DataOffset)], p->DataLength);
}

static const char *rndis_vendor = RNDIS_VENDOR;

void rndis_query(void  *pdev){
	switch (((rndis_query_msg_t *)encapsulated_buffer)->Oid){
		case OID_GEN_SUPPORTED_LIST:         rndis_query_cmplt(pdev, RNDIS_STATUS_SUCCESS, OIDSupportedList, 4 * OID_LIST_LENGTH); return;
		case OID_GEN_VENDOR_DRIVER_VERSION:  rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, 0x00001000);  return;
		case OID_802_3_CURRENT_ADDRESS:      rndis_query_cmplt(pdev, RNDIS_STATUS_SUCCESS, &station_hwaddr, 6); return;
		case OID_802_3_PERMANENT_ADDRESS:    rndis_query_cmplt(pdev, RNDIS_STATUS_SUCCESS, &permanent_hwaddr, 6); return;
		case OID_GEN_MEDIA_SUPPORTED:        rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, NDIS_MEDIUM_802_3); return;
		case OID_GEN_MEDIA_IN_USE:           rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, NDIS_MEDIUM_802_3); return;
		case OID_GEN_PHYSICAL_MEDIUM:        rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, NDIS_MEDIUM_802_3); return;
		case OID_GEN_HARDWARE_STATUS:        rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, 0); return;
		case OID_GEN_LINK_SPEED:             rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, RNDIS_LINK_SPEED / 100); return;
		case OID_GEN_VENDOR_ID:              rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, 0x00FFFFFF); return;
		case OID_GEN_VENDOR_DESCRIPTION:     rndis_query_cmplt(pdev, RNDIS_STATUS_SUCCESS, rndis_vendor, strlen(rndis_vendor) + 1); return;
		case OID_GEN_CURRENT_PACKET_FILTER:  rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, oid_packet_filter); return;
		case OID_GEN_MAXIMUM_FRAME_SIZE:     rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, ETH_MAX_PACKET_SIZE - ETH_HEADER_SIZE); return;
		case OID_GEN_MAXIMUM_TOTAL_SIZE:     rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, ETH_MAX_PACKET_SIZE); return;
		case OID_GEN_TRANSMIT_BLOCK_SIZE:    rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, ETH_MAX_PACKET_SIZE); return;
		case OID_GEN_RECEIVE_BLOCK_SIZE:     rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, ETH_MAX_PACKET_SIZE); return;
		case OID_GEN_MEDIA_CONNECT_STATUS:   rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, NDIS_MEDIA_STATE_CONNECTED); return;
		case OID_GEN_RNDIS_CONFIG_PARAMETER: rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, 0); return;
		case OID_802_3_MAXIMUM_LIST_SIZE:    rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, 1); return;
		case OID_802_3_MULTICAST_LIST:       rndis_query_cmplt32(pdev, RNDIS_STATUS_NOT_SUPPORTED, 0); return;
		case OID_802_3_MAC_OPTIONS:          rndis_query_cmplt32(pdev, RNDIS_STATUS_NOT_SUPPORTED, 0); return;
		case OID_GEN_MAC_OPTIONS:            rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, /*MAC_OPT*/ 0); return;
		case OID_802_3_RCV_ERROR_ALIGNMENT:  rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, 0); return;
		case OID_802_3_XMIT_ONE_COLLISION:   rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, 0); return;
		case OID_802_3_XMIT_MORE_COLLISIONS: rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, 0); return;
		case OID_GEN_XMIT_OK:                rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, usb_eth_stat.txok); return;
		case OID_GEN_RCV_OK:                 rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, usb_eth_stat.rxok); return;
		case OID_GEN_RCV_ERROR:              rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, usb_eth_stat.rxbad); return;
		case OID_GEN_XMIT_ERROR:             rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, usb_eth_stat.txbad); return;
		case OID_GEN_RCV_NO_BUFFER:          rndis_query_cmplt32(pdev, RNDIS_STATUS_SUCCESS, 0); return;
		default:                             rndis_query_cmplt(pdev, RNDIS_STATUS_FAILURE, NULL, 0); return;
	}
}

void rndis_query_cmplt(void *pdev, int status, const void *data, int size){
	rndis_query_cmplt_t *c;
	c = (rndis_query_cmplt_t *)encapsulated_buffer;
	c->MessageType = REMOTE_NDIS_QUERY_CMPLT;
	c->MessageLength = sizeof(rndis_query_cmplt_t) + size;
	c->InformationBufferLength = size;
	c->InformationBufferOffset = 16;
	c->Status = status;
	memcpy(c + 1, data, size);
	USBD_LL_Transmit(pdev, RNDIS_NOTIFICATION_IN_EP, (uint8_t *)"\x01\x00\x00\x00\x00\x00\x00\x00", 8);
}

void rndis_query_cmplt32(void *pdev, int status, uint32_t data){
	rndis_query_cmplt_t *c;
	c = (rndis_query_cmplt_t *)encapsulated_buffer;
	c->MessageType = REMOTE_NDIS_QUERY_CMPLT;
	c->MessageLength = sizeof(rndis_query_cmplt_t) + 4;
	c->InformationBufferLength = 4;
	c->InformationBufferOffset = 16;
	c->Status = status;
	*(uint32_t *)(c + 1) = data;
	USBD_LL_Transmit(pdev, RNDIS_NOTIFICATION_IN_EP, (uint8_t *)"\x01\x00\x00\x00\x00\x00\x00\x00", 8);
}

#define INFBUF ((uint32_t *)((uint8_t *)&(m->RequestId) + m->InformationBufferOffset))
#define CFGBUF ((rndis_config_parameter_t *) INFBUF)
#define PARMNAME  ((uint8_t *)CFGBUF + CFGBUF->ParameterNameOffset)
#define PARMVALUE ((uint8_t *)CFGBUF + CFGBUF->ParameterValueOffset)
#define PARMVALUELENGTH	CFGBUF->ParameterValueLength
#define PARM_NAME_LENGTH 25 /* Maximum parameter name length */

void rndis_handle_set_msg(void  *pdev){
	rndis_set_cmplt_t *c;
	rndis_set_msg_t *m;
	rndis_Oid_t oid;

	c = (rndis_set_cmplt_t *)encapsulated_buffer;
	m = (rndis_set_msg_t *)encapsulated_buffer;

	/* Never have longer parameter names than PARM_NAME_LENGTH */
	/*
	char parmname[PARM_NAME_LENGTH+1];
	uint8_t i;
	int8_t parmlength;
	*/

	/* The parameter name seems to be transmitted in uint16_t, but */
	/* we want this in uint8_t. Hence have to throw out some info... */

	/*
	if (CFGBUF->ParameterNameLength > (PARM_NAME_LENGTH*2))
	{
		parmlength = PARM_NAME_LENGTH * 2;
	}
	else
	{
		parmlength = CFGBUF->ParameterNameLength;
	}
	i = 0;
	while (parmlength > 0)
	{
		// Convert from uint16_t to char array.
		parmname[i] = (char)*(PARMNAME + 2*i); // FSE! FIX IT!
		parmlength -= 2;
		i++;
	}
	*/

	oid = m->Oid;
	c->MessageType = REMOTE_NDIS_SET_CMPLT;
	c->MessageLength = sizeof(rndis_set_cmplt_t);
	c->Status = RNDIS_STATUS_SUCCESS;

	switch (oid)
	{
		/* Parameters set up in 'Advanced' tab */
		case OID_GEN_RNDIS_CONFIG_PARAMETER:
			{
                rndis_config_parameter_t *p;
				char *ptr = (char *)m;
				ptr += sizeof(rndis_generic_msg_t);
				ptr += m->InformationBufferOffset;
				p = (rndis_config_parameter_t *)ptr;
				//rndis_handle_config_parm(ptr, p->ParameterNameOffset, p->ParameterValueOffset, p->ParameterNameLength, p->ParameterValueLength);
			}
			break;

		/* Mandatory general OIDs */
		case OID_GEN_CURRENT_PACKET_FILTER:
			oid_packet_filter = *INFBUF;
			if (oid_packet_filter)
			{
				//rndis_packetFilter(oid_packet_filter);
				rndis_state = rndis_data_initialized;
			}
			else
			{
				rndis_state = rndis_initialized;
			}
			break;

		case OID_GEN_CURRENT_LOOKAHEAD:
			break;

		case OID_GEN_PROTOCOL_OPTIONS:
			break;

		/* Mandatory 802_3 OIDs */
		case OID_802_3_MULTICAST_LIST:
			break;

		/* Power Managment: fails for now */
		case OID_PNP_ADD_WAKE_UP_PATTERN:
		case OID_PNP_REMOVE_WAKE_UP_PATTERN:
		case OID_PNP_ENABLE_WAKE_UP:
		default:
			c->Status = RNDIS_STATUS_FAILURE;
			break;
	}

	/* c->MessageID is same as before */
	USBD_LL_Transmit(pdev, RNDIS_NOTIFICATION_IN_EP, (uint8_t *)"\x01\x00\x00\x00\x00\x00\x00\x00", 8);
	return;
}

BaseType_t xNetworkInterfaceInitialise( void )
{
	xTaskCreate( prvEMACHandlerTask, "EMAC", configEMAC_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, &xEMACTaskHandle );

    return pdTRUE;
}

BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxDescriptor, BaseType_t xReleaseAfterSend ){
    /* Simple network interfaces (as opposed to more efficient zero copy network
    interfaces) just use Ethernet peripheral driver library functions to copy
    data from the FreeRTOS+TCP buffer into the peripheral driver's own buffer.
    This example assumes SendData() is a peripheral driver library function that
    takes a pointer to the start of the data to be sent and the length of the
    data to be sent as two separate parameters.  The start of the data is located
    by pxDescriptor->pucEthernetBuffer.  The length of the data is located
    by pxDescriptor->xDataLength. */
	if (pxDescriptor->xDataLength <= 0 ||
			pxDescriptor->xDataLength > ETH_MAX_PACKET_SIZE ||
		rndis_tx_size > 0) return pdFALSE;

	//__disable_irq();
	//rndis_first_tx = true;
	//memcpy((void *) data_to_send, (void *) pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength);
	//rndis_sended = 0;
	//rndis_tx_ptr = (uint8_t *)data_to_send;
	//rndis_tx_size = pxDescriptor->xDataLength;
	//usbd_cdc_transfer(pDev);
	__enable_irq();
    /* Call the standard trace macro to log the send event. */
    iptraceNETWORK_INTERFACE_TRANSMIT();
	if( xReleaseAfterSend != pdFALSE ){
        /* It is assumed SendData() copies the data out of the FreeRTOS+TCP Ethernet
        buffer.  The Ethernet buffer is therefore no longer needed, and must be
        freed for re-use. */
        vReleaseNetworkBufferAndDescriptor( pxDescriptor );
    }
    return pdTRUE;
}

static void rndis_rxproc(const char *data, int size){
	memcpy((void *) rndis_rx_tcp_buffer, (void *) data, size);
	rndis_tx_tcp_size = size;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Ethernet RX-Complete callback function, elsewhere declared as weak. */
	ulISREvents |= EMAC_IF_RX_EVENT;
	if( xEMACTaskHandle != NULL )
	{
		vTaskNotifyGiveFromISR( xEMACTaskHandle, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
static void prvEMACHandlerTask( void *pvParameters )
{

	NetworkBufferDescriptor_t *pxBufferDescriptor;
	size_t xBytesReceived;
	/* Used to indicate that xSendEventStructToIPTask() is being called because
	of an Ethernet receive event. */
	IPStackEvent_t xRxEvent;
    for( ;; )
    {
        ulTaskNotifyTake( pdFALSE, portMAX_DELAY );
		xBytesReceived = rndis_tx_tcp_size;
		if( xBytesReceived > 0 ){
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
				memcpy((void *) pxBufferDescriptor->pucEthernetBuffer, (void *) rndis_rx_tcp_buffer, xBytesReceived);
				pxBufferDescriptor->xDataLength = xBytesReceived;

				/* See if the data contained in the received Ethernet frame needs
				to be processed.  NOTE! It is preferable to do this in
				the interrupt service routine itself, which would remove the need
				to unblock this task for packets that don't need processing. */
				if( eConsiderFrameForProcessing( pxBufferDescriptor->pucEthernetBuffer )
																	  == eProcessBuffer ){
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
    }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
