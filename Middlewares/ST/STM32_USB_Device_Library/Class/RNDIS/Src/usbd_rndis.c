/**
 ******************************************************************************
 * @file    usbd_rndis.c
 * @author  MCD Application Team
 * @version V2.4.2
 * @date    11-December-2015
 * @brief   This file provides the high layer firmware functions to manage the
 *          following functionalities of the USB RNDIS Class:
 *           - Initialization and Configuration of high and low layer
 *           - Enumeration as RNDIS Device (and enumeration for each implemented memory interface)
 *           - OUT/IN data transfer
 *           - Command IN transfer (class requests management)
 *           - Error management
 *
 *  @verbatim
 *
 *          ===================================================================
 *                                RNDIS Class Driver Description
 *          ===================================================================
 *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
 *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
 *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
 *           This driver implements the following aspects of the specification:
 *             - Device descriptor management
 *             - Configuration descriptor management
 *             - Enumeration as RNDIS device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
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
#include "usbd_rndis.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */


/** @defgroup USBD_RNDIS
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_RNDIS_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */


/** @defgroup USBD_RNDIS_Private_Defines
 * @{
 */
/**
 * @}
 */


/** @defgroup USBD_RNDIS_Private_Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup USBD_RNDIS_Private_FunctionPrototypes
 * @{
 */


static uint8_t  USBD_RNDIS_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t  USBD_RNDIS_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t  USBD_RNDIS_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t  USBD_RNDIS_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_RNDIS_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_RNDIS_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_RNDIS_GetFSCfgDesc (uint16_t *length);

static uint8_t  *USBD_RNDIS_GetHSCfgDesc (uint16_t *length);

static uint8_t  *USBD_RNDIS_GetOtherSpeedCfgDesc (uint16_t *length);

static uint8_t  *USBD_RNDIS_GetOtherSpeedCfgDesc (uint16_t *length);

uint8_t  *USBD_RNDIS_GetDeviceQualifierDescriptor (uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_RNDIS_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

/** @defgroup USBD_RNDIS_Private_Variables
 * @{
 */


/* RNDIS interface class callbacks structure */
USBD_ClassTypeDef  USBD_RNDIS =
{
		USBD_RNDIS_Init,
		USBD_RNDIS_DeInit,
		USBD_RNDIS_Setup,
		NULL,                 /* EP0_TxSent, */
		USBD_RNDIS_EP0_RxReady,
		USBD_RNDIS_DataIn,
		USBD_RNDIS_DataOut,
		NULL,
		NULL,
		NULL,
		USBD_RNDIS_GetHSCfgDesc,
		USBD_RNDIS_GetFSCfgDesc,
		USBD_RNDIS_GetOtherSpeedCfgDesc,
		USBD_RNDIS_GetDeviceQualifierDescriptor,
};

/* USB RNDIS device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_RNDIS_CfgHSDesc[USB_RNDIS_CONFIG_DESC_SIZ] __ALIGN_END =
{
		//INTERFACE ASSOCIATION DESCRIPTOR
		0x08,																							//array index 0
		0x0B,	///bDescriptorType
		0x00,	///bFirstInterface																		//array index 2
		0x02,	///bInterfaceCount
		0xE0,    ///iInterfaceClass: Class Code: Wireless
		0x01,    ///iInterfaceSubClass: RF Controller
		0x03,    ///bInterfaceProtocol: Remote NDIS
		0x00,	///iFunction																			//array index 7

		//SIZE: 9 + 7 + 9 + 7 + 7 = 39
		///INTERFACE DESCRIPTOR(0)
		0x09,    ///bLength: Length of this descriptor													//array index 8
		0x04,    ///bDescriptorType: Interface Descriptor Type
		0x00,    ///bInterfaceNumber: Interface Number													//array index 10
		0x00,    ///bAlternateSetting: Alternate setting for this interface
		0x01,    ///bNumEndpoints: Number of endpoints in this interface excluding endpoint 0
		0xE0,    ///iInterfaceClass: Class Code: Wireless
		0x01,    ///iInterfaceSubClass: RF Controller
		0x03,    ///bInterfaceProtocol: Remote NDIS
		0x00,    ///iInterface: String index

		0x05,
		0x24,
		0x00,
		0x10,
		0x01,

		0x05,
		0x24,
		0x01,
		0x00,
		0x01,

		0x04,
		0x24,
		0x02,
		0x00,

		///ENDPOINT DESCRIPTOR(0)
		0x07,    ///bLength: Length of this descriptor													//array index 17
		0x05,    ///bDescriptorType: Endpoint Descriptor Type
		0x81,    ///bEndpointAddress: Endpoint address (IN,EP3)											//array index 19
		0x03,    ///bmAttributes: Transfer Type: INTERRUPT_TRANSFER
		0x08,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///wMaxPacketSize: Endpoint Size
		0x01,    ///bIntervall: Polling Intervall

		///INTERFACE DESCRIPTOR(5)
		0x09,    ///bLength: Length of this descriptor													//array index 24
		0x04,    ///bDescriptorType: Interface Descriptor Type
		0x01,    ///bInterfaceNumber: Interface Number													//array index 26
		0x00,    ///bAlternateSetting: Alternate setting for this interface
		0x02,    ///bNumEndpoints: Number of endpoints in this interface excluding endpoint 0
		0x0A,    ///iInterfaceClass: Class Code: CDC_DATA
		0x00,    ///iInterfaceSubClass: SubClass Code
		0x00,    ///bInterfaceProtocol: Protocol Code
		0x00,    ///iInterface: String index

		///ENDPOINT DESCRIPTOR(1)
		0x07,    ///bLength: Length of this descriptor													//array index 33
		0x05,    ///bDescriptorType: Endpoint Descriptor Type
		0x82,    ///bEndpointAddress: Endpoint address (IN,EP1)											//array index 35
		0x02,    ///bmAttributes: Transfer Type: BULK_TRANSFER
		0x40,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///bIntervall: Polling Intervall

		///ENDPOINT DESCRIPTOR(2)
		0x07,    ///bLength: Length of this descriptor													//array index 40
		0x05,    ///bDescriptorType: Endpoint Descriptor Type
		0x02,    ///bEndpointAddress: Endpoint address (OUT,EP1)										//array index 42
		0x02,    ///bmAttributes: Transfer Type: BULK_TRANSFER
		0x40,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///bIntervall: Polling Intervall														//array index 46
		/*---------------------------------------------------------------------------*/
} ;

/* USB RNDIS device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_RNDIS_CfgFSDesc[USB_RNDIS_CONFIG_DESC_SIZ] __ALIGN_END =
{
		//SIZE: 9+9+5+5+4+7+9+7+7=
		/*Configuration Descriptor*/
		0x09,   /* bLength: Configuration Descriptor size */
		USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
		USB_RNDIS_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
		0x00,
		0x02,   /* bNumInterfaces: 2 interface */
		0x01,   /* bConfigurationValue: Configuration value */
		0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
		0xC0,   /* bmAttributes: self powered */
		0xFA,   /* MaxPower 0 mA */

		///INTERFACE DESCRIPTOR(0)
		0x09,    ///bLength: Length of this descriptor													//array index 8
		0x04,    ///bDescriptorType: Interface Descriptor Type
		0x00,    ///bInterfaceNumber: Interface Number													//array index 10
		0x00,    ///bAlternateSetting: Alternate setting for this interface
		0x01,    ///bNumEndpoints: Number of endpoints in this interface excluding endpoint 0
		0xE0,    ///iInterfaceClass: Class Code: Wireless
		0x01,    ///iInterfaceSubClass: RF Controller
		0x03,    ///bInterfaceProtocol: Remote NDIS
		0x00,    ///iInterface: String index

		0x05,
		0x24,
		0x00,
		0x10,
		0x01,

		0x05,
		0x24,
		0x01,
		0x00,
		0x01,

		0x04,
		0x24,
		0x02,
		0x00,

		///ENDPOINT DESCRIPTOR(0)
		0x07,    					//bLength: Length of this descriptor													//array index 17
		0x05,    					//bDescriptorType: Endpoint Descriptor Type
		RNDIS_CMD_EP,    			//bEndpointAddress: Endpoint address (IN,EP3)											//array index 19
		0x03,    					//bmAttributes: Transfer Type: INTERRUPT_TRANSFER
		RNDIS_CMD_PACKET_SIZE,    	//wMaxPacketSize: Endpoint Size
		0x00,    					//wMaxPacketSize: Endpoint Size
		0x01,    					//bIntervall: Polling Intervall

		///INTERFACE DESCRIPTOR(5)
		0x09,    ///bLength: Length of this descriptor													//array index 24
		0x04,    ///bDescriptorType: Interface Descriptor Type
		0x01,    ///bInterfaceNumber: Interface Number													//array index 26
		0x00,    ///bAlternateSetting: Alternate setting for this interface
		0x02,    ///bNumEndpoints: Number of endpoints in this interface excluding endpoint 0
		0x0A,    ///iInterfaceClass: Class Code: CDC_DATA
		0x00,    ///iInterfaceSubClass: SubClass Code
		0x00,    ///bInterfaceProtocol: Protocol Code
		0x00,    ///iInterface: String index

		///ENDPOINT DESCRIPTOR(1)
		0x07,    ///bLength: Length of this descriptor													//array index 33
		0x05,    ///bDescriptorType: Endpoint Descriptor Type
		RNDIS_IN_EP,    ///bEndpointAddress: Endpoint address (IN,EP1)											//array index 35
		0x02,    ///bmAttributes: Transfer Type: BULK_TRANSFER
		0x40,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///bIntervall: Polling Intervall

		///ENDPOINT DESCRIPTOR(2)
		0x07,    ///bLength: Length of this descriptor													//array index 40
		0x05,    ///bDescriptorType: Endpoint Descriptor Type
		RNDIS_OUT_EP,    ///bEndpointAddress: Endpoint address (OUT,EP1)										//array index 42
		0x02,    ///bmAttributes: Transfer Type: BULK_TRANSFER
		RNDIS_DATA_FS_MAX_PACKET_SIZE,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///bIntervall: Polling Intervall														//array index 46
		/*---------------------------------------------------------------------------*/
} ;

__ALIGN_BEGIN uint8_t USBD_RNDIS_OtherSpeedCfgDesc[USB_RNDIS_CONFIG_DESC_SIZ] __ALIGN_END =
{
		//INTERFACE ASSOCIATION DESCRIPTOR
		0x08,																							//array index 0
		0x0B,	///bDescriptorType
		0x00,	///bFirstInterface																		//array index 2
		0x02,	///bInterfaceCount
		0xE0,    ///iInterfaceClass: Class Code: Wireless
		0x01,    ///iInterfaceSubClass: RF Controller
		0x03,    ///bInterfaceProtocol: Remote NDIS
		0x00,	///iFunction																			//array index 7

		//SIZE: 9 + 7 + 9 + 7 + 7 = 39
		///INTERFACE DESCRIPTOR(0)
		0x09,    ///bLength: Length of this descriptor													//array index 8
		0x04,    ///bDescriptorType: Interface Descriptor Type
		0x00,    ///bInterfaceNumber: Interface Number													//array index 10
		0x00,    ///bAlternateSetting: Alternate setting for this interface
		0x01,    ///bNumEndpoints: Number of endpoints in this interface excluding endpoint 0
		0xE0,    ///iInterfaceClass: Class Code: Wireless
		0x01,    ///iInterfaceSubClass: RF Controller
		0x03,    ///bInterfaceProtocol: Remote NDIS
		0x00,    ///iInterface: String index

		0x05,
		0x24,
		0x00,
		0x10,
		0x01,

		0x05,
		0x24,
		0x01,
		0x00,
		0x01,

		0x04,
		0x24,
		0x02,
		0x00,

		///ENDPOINT DESCRIPTOR(0)
		0x07,    ///bLength: Length of this descriptor													//array index 17
		0x05,    ///bDescriptorType: Endpoint Descriptor Type
		0x81,    ///bEndpointAddress: Endpoint address (IN,EP3)											//array index 19
		0x03,    ///bmAttributes: Transfer Type: INTERRUPT_TRANSFER
		0x08,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///wMaxPacketSize: Endpoint Size
		0x01,    ///bIntervall: Polling Intervall

		///INTERFACE DESCRIPTOR(5)
		0x09,    ///bLength: Length of this descriptor													//array index 24
		0x04,    ///bDescriptorType: Interface Descriptor Type
		0x01,    ///bInterfaceNumber: Interface Number													//array index 26
		0x00,    ///bAlternateSetting: Alternate setting for this interface
		0x02,    ///bNumEndpoints: Number of endpoints in this interface excluding endpoint 0
		0x0A,    ///iInterfaceClass: Class Code: CDC_DATA
		0x00,    ///iInterfaceSubClass: SubClass Code
		0x00,    ///bInterfaceProtocol: Protocol Code
		0x00,    ///iInterface: String index

		///ENDPOINT DESCRIPTOR(1)
		0x07,    ///bLength: Length of this descriptor													//array index 33
		0x05,    ///bDescriptorType: Endpoint Descriptor Type
		0x82,    ///bEndpointAddress: Endpoint address (IN,EP1)											//array index 35
		0x02,    ///bmAttributes: Transfer Type: BULK_TRANSFER
		0x40,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///bIntervall: Polling Intervall

		///ENDPOINT DESCRIPTOR(2)
		0x07,    ///bLength: Length of this descriptor													//array index 40
		0x05,    ///bDescriptorType: Endpoint Descriptor Type
		0x02,    ///bEndpointAddress: Endpoint address (OUT,EP1)										//array index 42
		0x02,    ///bmAttributes: Transfer Type: BULK_TRANSFER
		0x40,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///wMaxPacketSize: Endpoint Size
		0x00,    ///bIntervall: Polling Intervall														//array index 46
		/*---------------------------------------------------------------------------*/
};


/**
 * @}
 */

/** @defgroup USBD_RNDIS_Private_Functions
 * @{
 */

/**
 * @brief  USBD_RNDIS_Init
 *         Initialize the RNDIS interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t  USBD_RNDIS_Init (USBD_HandleTypeDef *pdev,
		uint8_t cfgidx)
{
	uint8_t ret = 0;
	USBD_RNDIS_HandleTypeDef   *hrndis;

	if(pdev->dev_speed == USBD_SPEED_HIGH  )
	{
		/* Open EP IN */
		USBD_LL_OpenEP(pdev,
				RNDIS_IN_EP,
				USBD_EP_TYPE_BULK,
				RNDIS_DATA_HS_IN_PACKET_SIZE);

		/* Open EP OUT */
		USBD_LL_OpenEP(pdev,
				RNDIS_OUT_EP,
				USBD_EP_TYPE_BULK,
				RNDIS_DATA_HS_OUT_PACKET_SIZE);

	}
	else
	{
		/* Open EP IN */
		USBD_LL_OpenEP(pdev,
				RNDIS_IN_EP,
				USBD_EP_TYPE_BULK,
				RNDIS_DATA_FS_IN_PACKET_SIZE);

		/* Open EP OUT */
		USBD_LL_OpenEP(pdev,
				RNDIS_OUT_EP,
				USBD_EP_TYPE_BULK,
				RNDIS_DATA_FS_OUT_PACKET_SIZE);
	}
	/* Open Command IN EP */
	USBD_LL_OpenEP(pdev,
			RNDIS_CMD_EP,
			USBD_EP_TYPE_INTR,
			RNDIS_CMD_PACKET_SIZE);


	pdev->pClassData = USBD_malloc(sizeof (USBD_RNDIS_HandleTypeDef));

	if(pdev->pClassData == NULL)
	{
		ret = 1;
	}
	else
	{
		hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;

		/* Init  physical Interface components */
		((USBD_RNDIS_ItfTypeDef *)pdev->pUserData)->Init();

		/* Init Xfer states */
		hrndis->TxState =0;
		hrndis->RxState =0;

		if(pdev->dev_speed == USBD_SPEED_HIGH  )
		{
			/* Prepare Out endpoint to receive next packet */
			USBD_LL_PrepareReceive(pdev,
					RNDIS_OUT_EP,
					hrndis->RxBuffer,
					RNDIS_DATA_HS_OUT_PACKET_SIZE);
		}
		else
		{
			/* Prepare Out endpoint to receive next packet */
			USBD_LL_PrepareReceive(pdev,
					RNDIS_OUT_EP,
					hrndis->RxBuffer,
					RNDIS_DATA_FS_OUT_PACKET_SIZE);
		}


	}
	return ret;
}

/**
 * @brief  USBD_RNDIS_Init
 *         DeInitialize the RNDIS layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t  USBD_RNDIS_DeInit (USBD_HandleTypeDef *pdev,
		uint8_t cfgidx)
{
	uint8_t ret = 0;

	/* Open EP IN */
	USBD_LL_CloseEP(pdev,
			RNDIS_IN_EP);

	/* Open EP OUT */
	USBD_LL_CloseEP(pdev,
			RNDIS_OUT_EP);

	/* Open Command IN EP */
	USBD_LL_CloseEP(pdev,
			RNDIS_CMD_EP);


	/* DeInit  physical Interface components */
	if(pdev->pClassData != NULL)
	{
		((USBD_RNDIS_ItfTypeDef *)pdev->pUserData)->DeInit();
		USBD_free(pdev->pClassData);
		pdev->pClassData = NULL;
	}

	return ret;
}

/**
 * @brief  USBD_RNDIS_Setup
 *         Handle the RNDIS specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t  USBD_RNDIS_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	USBD_RNDIS_HandleTypeDef   *hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;
	static uint8_t ifalt = 0;

	switch (req->bmRequest & USB_REQ_TYPE_MASK)
	{
	case USB_REQ_TYPE_CLASS :
		if (req->wLength)
		{
			if (req->bmRequest & 0x80)
			{
				((USBD_RNDIS_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest, (uint8_t *)hrndis->data, req->wLength);
				USBD_CtlSendData (pdev, (uint8_t *)hrndis->data, req->wLength);
			}
			else
			{
				hrndis->CmdOpCode = req->bRequest;
				hrndis->CmdLength = req->wLength;

				USBD_CtlPrepareRx (pdev, (uint8_t *)hrndis->data, req->wLength);
			}

		}
		else
		{
			((USBD_RNDIS_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest, (uint8_t*)req, 0);
		}
		break;

	case USB_REQ_TYPE_STANDARD:
		switch (req->bRequest)
		{
		case USB_REQ_GET_INTERFACE :
			USBD_CtlSendData (pdev, &ifalt, 1);
			break;

		case USB_REQ_SET_INTERFACE :
			break;
		}

		default:
			break;
	}
	return USBD_OK;
}

/**
 * @brief  USBD_RNDIS_DataIn
 *         Data sent on non-control IN endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t  USBD_RNDIS_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	USBD_RNDIS_HandleTypeDef   *hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;

	if(pdev->pClassData != NULL)
	{

		hrndis->TxState = 0;

		return USBD_OK;
	}
	else
	{
		return USBD_FAIL;
	}
}

/**
 * @brief  USBD_RNDIS_DataOut
 *         Data received on non-control Out endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t  USBD_RNDIS_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	USBD_RNDIS_HandleTypeDef   *hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;

	/* Get the received data length */
	hrndis->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);

	/* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */
	if(pdev->pClassData != NULL)
	{
		((USBD_RNDIS_ItfTypeDef *)pdev->pUserData)->Receive(hrndis->RxBuffer, &hrndis->RxLength);

		return USBD_OK;
	}
	else
	{
		return USBD_FAIL;
	}
}



/**
 * @brief  USBD_RNDIS_DataOut
 *         Data received on non-control Out endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t  USBD_RNDIS_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
	USBD_RNDIS_HandleTypeDef   *hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;

	if((pdev->pUserData != NULL) && (hrndis->CmdOpCode != 0xFF))
	{
		((USBD_RNDIS_ItfTypeDef *)pdev->pUserData)->Control(hrndis->CmdOpCode,
				(uint8_t *)hrndis->data,
				hrndis->CmdLength);
		hrndis->CmdOpCode = 0xFF;

	}
	return USBD_OK;
}

/**
 * @brief  USBD_RNDIS_GetFSCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_RNDIS_GetFSCfgDesc (uint16_t *length)
{
	*length = sizeof (USBD_RNDIS_CfgFSDesc);
	return USBD_RNDIS_CfgFSDesc;
}

/**
 * @brief  USBD_RNDIS_GetHSCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_RNDIS_GetHSCfgDesc (uint16_t *length)
{
	*length = sizeof (USBD_RNDIS_CfgHSDesc);
	return USBD_RNDIS_CfgHSDesc;
}

/**
 * @brief  USBD_RNDIS_GetCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_RNDIS_GetOtherSpeedCfgDesc (uint16_t *length)
{
	*length = sizeof (USBD_RNDIS_OtherSpeedCfgDesc);
	return USBD_RNDIS_OtherSpeedCfgDesc;
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t  *USBD_RNDIS_GetDeviceQualifierDescriptor (uint16_t *length)
{
	*length = sizeof (USBD_RNDIS_DeviceQualifierDesc);
	return USBD_RNDIS_DeviceQualifierDesc;
}

/**
 * @brief  USBD_RNDIS_RegisterInterface
 * @param  pdev: device instance
 * @param  fops: CD  Interface callback
 * @retval status
 */
uint8_t  USBD_RNDIS_RegisterInterface  (USBD_HandleTypeDef   *pdev,
		USBD_RNDIS_ItfTypeDef *fops)
{
	uint8_t  ret = USBD_FAIL;

	if(fops != NULL)
	{
		pdev->pUserData= fops;
		ret = USBD_OK;
	}

	return ret;
}

/**
 * @brief  USBD_RNDIS_SetTxBuffer
 * @param  pdev: device instance
 * @param  pbuff: Tx Buffer
 * @retval status
 */
uint8_t  USBD_RNDIS_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
		uint8_t  *pbuff,
		uint16_t length)
{
	USBD_RNDIS_HandleTypeDef   *hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;

	hrndis->TxBuffer = pbuff;
	hrndis->TxLength = length;

	return USBD_OK;
}


/**
 * @brief  USBD_RNDIS_SetRxBuffer
 * @param  pdev: device instance
 * @param  pbuff: Rx Buffer
 * @retval status
 */
uint8_t  USBD_RNDIS_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
		uint8_t  *pbuff)
{
	USBD_RNDIS_HandleTypeDef   *hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;

	hrndis->RxBuffer = pbuff;

	return USBD_OK;
}

/**
 * @brief  USBD_RNDIS_DataOut
 *         Data received on non-control Out endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
uint8_t  USBD_RNDIS_TransmitPacket(USBD_HandleTypeDef *pdev)
{
	USBD_RNDIS_HandleTypeDef   *hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;

	if(pdev->pClassData != NULL)
	{
		if(hrndis->TxState == 0)
		{
			/* Tx Transfer in progress */
			hrndis->TxState = 1;

			/* Transmit next packet */
			USBD_LL_Transmit(pdev,
					RNDIS_IN_EP,
					hrndis->TxBuffer,
					hrndis->TxLength);

			return USBD_OK;
		}
		else
		{
			return USBD_BUSY;
		}
	}
	else
	{
		return USBD_FAIL;
	}
}


/**
 * @brief  USBD_RNDIS_ReceivePacket
 *         prepare OUT Endpoint for reception
 * @param  pdev: device instance
 * @retval status
 */
uint8_t  USBD_RNDIS_ReceivePacket(USBD_HandleTypeDef *pdev)
{
	USBD_RNDIS_HandleTypeDef   *hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;

	/* Suspend or Resume USB Out process */
	if(pdev->pClassData != NULL)
	{
		if(pdev->dev_speed == USBD_SPEED_HIGH  )
		{
			/* Prepare Out endpoint to receive next packet */
			USBD_LL_PrepareReceive(pdev,
					RNDIS_OUT_EP,
					hrndis->RxBuffer,
					RNDIS_DATA_HS_OUT_PACKET_SIZE);
		}
		else
		{
			/* Prepare Out endpoint to receive next packet */
			USBD_LL_PrepareReceive(pdev,
					RNDIS_OUT_EP,
					hrndis->RxBuffer,
					RNDIS_DATA_FS_OUT_PACKET_SIZE);
		}
		return USBD_OK;
	}
	else
	{
		return USBD_FAIL;
	}
}






/**
 * @brief  USBD_RNDIS_DataOut
 *         Data received on non-control Out endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
uint8_t  USBD_RNDIS_TransmitControl(USBD_HandleTypeDef *pdev, uint8_t *buff, uint16_t length)
{
//	USBD_RNDIS_HandleTypeDef   *hrndis = (USBD_RNDIS_HandleTypeDef*) pdev->pClassData;

//	if(pdev->pClassData != NULL)
//	{
//		if(hrndis->TxState == 0)
//		{
//			/* Tx Transfer in progress */
//			hrndis->TxState = 1;
//
//			/* Transmit next packet */
			USBD_LL_Transmit(pdev, RNDIS_CMD_EP, buff, length);

			return USBD_OK;
//		}
//		else
//		{
//			return USBD_BUSY;
//		}
//	}
//	else
//	{
//		return USBD_FAIL;
//	}
}



/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
