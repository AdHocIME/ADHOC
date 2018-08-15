/**
 ******************************************************************************
 * @file    usbd_composite.c
 * @author  MCD Application Team
 * @version V2.4.2
 * @date    11-December-2015
 * @brief   This file provides the high layer firmware functions to manage the
 *          following functionalities of the USB COMPOSITE Class:
 *           - Initialization and Configuration of high and low layer
 *           - Enumeration as COMPOSITE Device (and enumeration for each implemented memory interface)
 *           - OUT/IN data transfer
 *           - Command IN transfer (class requests management)
 *           - Error management
 *
 *  @verbatim
 *
 *          ===================================================================
 *                                COMPOSITE Class Driver Description
 *          ===================================================================
 *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
 *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
 *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
 *           This driver implements the following aspects of the specification:
 *             - Device descriptor management
 *             - Configuration descriptor management
 *             - Enumeration as COMPOSITE device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
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
#include <usbd_composite.h>
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */


/** @defgroup USBD_COMPOSITE
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_COMPOSITE_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */


/** @defgroup USBD_COMPOSITE_Private_Defines
 * @{
 */
/**
 * @}
 */


/** @defgroup USBD_COMPOSITE_Private_Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup USBD_COMPOSITE_Private_FunctionPrototypes
 * @{
 */


static uint8_t  USBD_COMPOSITE_Init (USBD_HandleTypeDef *pdev,
		uint8_t cfgidx);

static uint8_t  USBD_COMPOSITE_DeInit (USBD_HandleTypeDef *pdev,
		uint8_t cfgidx);

static uint8_t  USBD_COMPOSITE_Setup (USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req);

static uint8_t  USBD_COMPOSITE_DataIn (USBD_HandleTypeDef *pdev,
		uint8_t epnum);

static uint8_t  USBD_COMPOSITE_DataOut (USBD_HandleTypeDef *pdev,
		uint8_t epnum);

static uint8_t  USBD_COMPOSITE_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_COMPOSITE_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_COMPOSITE_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_COMPOSITE_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_COMPOSITE_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  *USBD_COMPOSITE_GetFSCfgDesc (uint16_t *length);

static uint8_t  *USBD_COMPOSITE_GetHSCfgDesc (uint16_t *length);

static uint8_t  *USBD_COMPOSITE_GetOtherSpeedCfgDesc (uint16_t *length);

static uint8_t  *USBD_COMPOSITE_GetOtherSpeedCfgDesc (uint16_t *length);

uint8_t  *USBD_COMPOSITE_GetDeviceQualifierDescriptor (uint16_t *length);


USBD_COMPOSITE_ClassData usbd_composite_class_data[USB_COMPOSITE_MAX_CLASSES];
uint8_t usbd_composite_pClass_count=0;
static uint8_t descriptor[1024];
static uint16_t descriptor_size=0;
static uint8_t itf_num=0;
static uint8_t inEP=1;
static uint8_t outEP=1;



/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

/** @defgroup USBD_COMPOSITE_Private_Variables
 * @{
 */


/* COMPOSITE interface class callbacks structure */
USBD_ClassTypeDef  USBD_COMPOSITE =
{
		USBD_COMPOSITE_Init,
		USBD_COMPOSITE_DeInit,
		USBD_COMPOSITE_Setup,
		USBD_COMPOSITE_EP0_TxReady,
		USBD_COMPOSITE_EP0_RxReady,
		USBD_COMPOSITE_DataIn,
		USBD_COMPOSITE_DataOut,
		USBD_COMPOSITE_SOF,
		USBD_COMPOSITE_IsoINIncomplete,
		USBD_COMPOSITE_IsoOutIncomplete,
		USBD_COMPOSITE_GetHSCfgDesc,
		USBD_COMPOSITE_GetFSCfgDesc,
		USBD_COMPOSITE_GetOtherSpeedCfgDesc,
		USBD_COMPOSITE_GetDeviceQualifierDescriptor,
};

/* USB COMPOSITE device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_COMPOSITE_CfgHSDesc[USB_COMPOSITE_CONFIG_DESC_SIZ] __ALIGN_END =
{
		/*Configuration Descriptor*/
		0x09,   /* bLength: Configuration Descriptor size */
		USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
		USB_COMPOSITE_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
		0x00,
		0x02,   /* bNumInterfaces: 2 interface */
		0x01,   /* bConfigurationValue: Configuration value */
		0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
		0x80,   /* bmAttributes: bus powered */
		0xFA,   /* MaxPower 500 mA */
} ;

/* USB COMPOSITE device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_COMPOSITE_CfgFSDesc[USB_COMPOSITE_CONFIG_DESC_SIZ] __ALIGN_END =
{
		/*Configuration Descriptor*/
		0x09,   /* bLength: Configuration Descriptor size */
		USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
		USB_COMPOSITE_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
		0x00,
		0x02,   /* bNumInterfaces: 2 interface */
		0x01,   /* bConfigurationValue: Configuration value */
		0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
		0x80,   /* bmAttributes: bus powered */
		0xFA,   /* MaxPower 500 mA */
} ;

__ALIGN_BEGIN uint8_t USBD_COMPOSITE_OtherSpeedCfgDesc[USB_COMPOSITE_CONFIG_DESC_SIZ] __ALIGN_END =
{ 
		0x09,   /* bLength: Configuation Descriptor size */
		USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,
		USB_COMPOSITE_CONFIG_DESC_SIZ,
		0x00,
		0x02,   /* bNumInterfaces: 2 interfaces */
		0x01,   /* bConfigurationValue: */
		0x04,   /* iConfiguration: */
		0x80,   /* bmAttributes: bus powered */
		0xFA,   /* MaxPower 500 mA */
};

__ALIGN_BEGIN uint8_t USBD_COMPOSITE_IfcAssocDesc[USB_COMPOSITE_IFC_ASSOC_DESC_SIZ] __ALIGN_END =
{
		0x08,		//  0	bLength				1	08h
		0x0B,		//	1	bDescriptorType		1	0Bh	Interface Association
		0x00,		//  2	bFirstInterface		1	02h
		0x02,		//  3	bInterfaceCount		1	02h
		0x02,		//  4	bFunctionClass		1	EFh
		0x02,		//  5	bFunctionSubClass	1	02h
		0x00,		//  6	bFunctionProtocol	1	01h
		0x00		//  7	iFunction			1	00h	""
};


/**
 * @}
 */

/** @defgroup USBD_COMPOSITE_Private_Functions
 * @{
 */

/**
 * @brief  USBD_COMPOSITE_Init
 *         Initialize the COMPOSITE interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	uint8_t ret = 0;
	uint8_t index=0;

	for(index=0 ; index<usbd_composite_pClass_count;index++){
		pdev->pClassData=usbd_composite_class_data[index].pClassData;
		pdev->pUserData=usbd_composite_class_data[index].pUserData;

		ret|=usbd_composite_class_data[index].pClass->Init(pdev, cfgidx);

		usbd_composite_class_data[index].pClassData=pdev->pClassData;
		usbd_composite_class_data[index].pUserData=pdev->pUserData;
	}
	return ret;
}

/**
 * @brief  USBD_COMPOSITE_Init
 *         DeInitialize the COMPOSITE layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_DeInit (USBD_HandleTypeDef *pdev,
		uint8_t cfgidx)
{
	uint8_t ret = 0;

	uint8_t index=0;

	for(index=0 ; index<usbd_composite_pClass_count;index++){
		pdev->pClassData=usbd_composite_class_data[index].pClassData;
		pdev->pUserData=usbd_composite_class_data[index].pUserData;

		ret|=usbd_composite_class_data[index].pClass->DeInit(pdev, cfgidx);

		usbd_composite_class_data[index].pClassData=pdev->pClassData;
		usbd_composite_class_data[index].pUserData=pdev->pUserData;
	}

	return ret;
}

/**
 * @brief  USBD_COMPOSITE_Setup
 *         Handle the COMPOSITE specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_Setup (USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req)
{
	uint8_t status=USBD_OK;
	uint8_t itf=0;
	uint8_t index=-1;
	uint8_t i=0;

	switch(req->bmRequest & 0x1F) {
	case USB_REQ_RECIPIENT_INTERFACE:
		for(index=0;index<usbd_composite_pClass_count;index++){
			itf+=usbd_composite_class_data[index].bInterfaces;
			if(LOBYTE(req->wIndex)<itf)
				break;
		}
		break;
	case USB_REQ_RECIPIENT_ENDPOINT:
		for(index=0;index<usbd_composite_pClass_count;index++){
			if(req->wIndex & 0x80){
				for(i=0;i<usbd_composite_class_data[index].inEP;i++){
					if(usbd_composite_class_data[index].inEPa[i]==(req->wIndex & 0x7F)){
						break;
					}
				}
			} else {
				for(i=0;i<usbd_composite_class_data[index].outEP;i++){
					if(usbd_composite_class_data[index].outEPa[i]==LOBYTE(req->wIndex)){
						break;
					}
				}
			}
			if(i!=usbd_composite_class_data[index].inEP){
				break;
			}
		}
		break;
	}
	if(index!=-1 && index!=usbd_composite_pClass_count){
		pdev->pClassData=usbd_composite_class_data[index].pClassData;
		pdev->pUserData=usbd_composite_class_data[index].pUserData;

		if(usbd_composite_class_data[index].pClass->Setup){
			status=usbd_composite_class_data[index].pClass->Setup(pdev, req);
		}

		usbd_composite_class_data[index].pClassData=pdev->pClassData;
		usbd_composite_class_data[index].pUserData=pdev->pUserData;
	}

	return status;
}

uint8_t USBD_COMPOSITE_GetClassIndexFromEP(uint8_t epnum){
	uint8_t index=0;
	uint8_t i=0;
	for(index=0;index<usbd_composite_pClass_count;index++){
		if(epnum & 0x80){
			for(i=0;i<usbd_composite_class_data[index].inEP;i++){
				if(usbd_composite_class_data[index].inEPa[i]==(epnum & 0x7F)){
					return index;
				}
			}
		} else {
			for(i=0;i<usbd_composite_class_data[index].outEP;i++){
				if(usbd_composite_class_data[index].outEPa[i]==epnum){
					return index;
				}
			}
		}
	}
	return -1;
}

/**
 * @brief  USBD_COMPOSITE_DataIn
 *         Data sent on non-control IN endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uint8_t status=USBD_OK;
	uint8_t index=0;
	uint8_t i=0;

	for(index=0;index<usbd_composite_pClass_count;index++){
		for(i=0;i<usbd_composite_class_data[index].inEP;i++){
			if(usbd_composite_class_data[index].inEPa[i]==epnum){
				pdev->pClassData=usbd_composite_class_data[index].pClassData;
				pdev->pUserData=usbd_composite_class_data[index].pUserData;

				if(usbd_composite_class_data[index].pClass->DataOut){
					status|=usbd_composite_class_data[index].pClass->DataIn(pdev, usbd_composite_class_data[index].inEPn[i]);
				}

				usbd_composite_class_data[index].pClassData=pdev->pClassData;
				usbd_composite_class_data[index].pUserData=pdev->pUserData;
				return status;
			}
		}
	}
	return status;
}

/**
 * @brief  USBD_COMPOSITE_DataOut
 *         Data received on non-control Out endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
	uint8_t status=USBD_OK;
	uint8_t index=0;
	for(index=0;index<usbd_composite_pClass_count;index++){
		uint8_t i=0;
		for(i=0;i<usbd_composite_class_data[index].outEP;i++){
			if(usbd_composite_class_data[index].outEPa[i]==epnum){
				pdev->pClassData=usbd_composite_class_data[index].pClassData;
				pdev->pUserData=usbd_composite_class_data[index].pUserData;

				if(usbd_composite_class_data[index].pClass->DataOut){
					status|=usbd_composite_class_data[index].pClass->DataOut(pdev, usbd_composite_class_data[index].outEPn[i]);
				}

				usbd_composite_class_data[index].pClassData=pdev->pClassData;
				usbd_composite_class_data[index].pUserData=pdev->pUserData;
				return status;
			}
		}
	}
	return status;
}



/**
 * @brief  USBD_COMPOSITE_DataOut
 *         Data received on non-control Out endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t  USBD_COMPOSITE_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
	uint8_t status=USBD_OK;
	uint8_t index;
	for(index=0;index<usbd_composite_pClass_count;index++){
		pdev->pClassData=usbd_composite_class_data[index].pClassData;
		pdev->pUserData=usbd_composite_class_data[index].pUserData;

		if(usbd_composite_class_data[index].pClass->EP0_RxReady){
			status|=usbd_composite_class_data[index].pClass->EP0_RxReady(pdev);
		}

		usbd_composite_class_data[index].pClassData=pdev->pClassData;
		usbd_composite_class_data[index].pUserData=pdev->pUserData;
	}
	return status;
}

/**
  * @brief  USBD_COMPOSITE_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_COMPOSITE_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
	uint8_t status=USBD_OK;
	uint8_t index;
	for(index=0;index<usbd_composite_pClass_count;index++){
		pdev->pClassData=usbd_composite_class_data[index].pClassData;
		pdev->pUserData=usbd_composite_class_data[index].pUserData;

		if(usbd_composite_class_data[index].pClass->EP0_TxSent){
			status|=usbd_composite_class_data[index].pClass->EP0_TxSent(pdev);
		}

		usbd_composite_class_data[index].pClassData=pdev->pClassData;
		usbd_composite_class_data[index].pUserData=pdev->pUserData;
	}
	return status;
}
/**
  * @brief  USBD_COMPOSITE_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_COMPOSITE_SOF (USBD_HandleTypeDef *pdev)
{
	uint8_t status=USBD_OK;
	uint8_t index;
	for(index=0;index<usbd_composite_pClass_count;index++){
		pdev->pClassData=usbd_composite_class_data[index].pClassData;
		pdev->pUserData=usbd_composite_class_data[index].pUserData;

		if(usbd_composite_class_data[index].pClass->SOF){
			status|=usbd_composite_class_data[index].pClass->SOF(pdev);
		}

		usbd_composite_class_data[index].pClassData=pdev->pClassData;
		usbd_composite_class_data[index].pUserData=pdev->pUserData;
	}
	return status;
}


/**
  * @brief  USBD_COMPOSITE_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_COMPOSITE_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uint8_t status=USBD_OK;
	uint8_t index=0;
	for(index=0;index<usbd_composite_pClass_count;index++){
		uint8_t i=0;
		for(i=0;i<usbd_composite_class_data[index].inEP;i++){
			if(usbd_composite_class_data[index].inEPa[i]==epnum){
				pdev->pClassData=usbd_composite_class_data[index].pClassData;
				pdev->pUserData=usbd_composite_class_data[index].pUserData;

				if(usbd_composite_class_data[index].pClass->IsoINIncomplete){
					status|=usbd_composite_class_data[index].pClass->IsoINIncomplete(pdev, usbd_composite_class_data[index].inEPn[i]);
				}

				usbd_composite_class_data[index].pClassData=pdev->pClassData;
				usbd_composite_class_data[index].pUserData=pdev->pUserData;
				return status;
			}
		}
	}
	return status;
}
/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_COMPOSITE_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

	uint8_t status=USBD_OK;
	uint8_t index=0;
	for(index=0;index<usbd_composite_pClass_count;index++){
		uint8_t i=0;
		for(i=0;i<usbd_composite_class_data[index].outEP;i++){
			if(usbd_composite_class_data[index].outEPa[i]==epnum){
				pdev->pClassData=usbd_composite_class_data[index].pClassData;
				pdev->pUserData=usbd_composite_class_data[index].pUserData;

				if(usbd_composite_class_data[index].pClass->IsoOUTIncomplete){
					status|=usbd_composite_class_data[index].pClass->IsoOUTIncomplete(pdev, usbd_composite_class_data[index].outEPn[i]);
				}

				usbd_composite_class_data[index].pClassData=pdev->pClassData;
				usbd_composite_class_data[index].pUserData=pdev->pUserData;
				return status;
			}
		}
	}
	return status;
}


/**
 * @brief  USBD_COMPOSITE_GetFSCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_COMPOSITE_GetFSCfgDesc (uint16_t *length)
{
	*length=descriptor_size;
	return descriptor;
}

/**
 * @brief  USBD_COMPOSITE_GetHSCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_COMPOSITE_GetHSCfgDesc (uint16_t *length)
{
	return USBD_COMPOSITE_GetFSCfgDesc(length);
}

/**
 * @brief  USBD_COMPOSITE_GetCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_COMPOSITE_GetOtherSpeedCfgDesc (uint16_t *length)
{
	return USBD_COMPOSITE_GetFSCfgDesc(length);
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t  *USBD_COMPOSITE_GetDeviceQualifierDescriptor (uint16_t *length)
{
	*length = sizeof (USBD_COMPOSITE_DeviceQualifierDesc);
	return USBD_COMPOSITE_DeviceQualifierDesc;
}

USBD_StatusTypeDef  USBD_COMPOSITE_RegisterClass(USBD_HandleTypeDef *pdev, uint8_t bFunctionClass, uint8_t bFunctionSubClass, uint8_t bFunctionProtocol){
	USBD_StatusTypeDef   status = USBD_OK;
	uint8_t lastIfc=-1;
	if(descriptor_size==0){
		USBD_memcpy(descriptor, USBD_COMPOSITE_CfgFSDesc, USB_COMPOSITE_CONFIG_DESC_SIZ);
		descriptor_size+=USB_COMPOSITE_CONFIG_DESC_SIZ;
	}

	if(pdev->pClass != 0 && pdev->pClass != &USBD_COMPOSITE && usbd_composite_pClass_count<USB_COMPOSITE_MAX_CLASSES)
	{
		/* link the class to the USB Device handle */
		usbd_composite_class_data[usbd_composite_pClass_count].bFunctionClass=bFunctionClass;
		usbd_composite_class_data[usbd_composite_pClass_count].bFunctionSubClass=bFunctionSubClass;
		usbd_composite_class_data[usbd_composite_pClass_count].bFunctionProtocol=bFunctionProtocol;
		usbd_composite_class_data[usbd_composite_pClass_count].pClass=pdev->pClass;
		usbd_composite_class_data[usbd_composite_pClass_count].pClassData=pdev->pClassData;
		usbd_composite_class_data[usbd_composite_pClass_count].pUserData=pdev->pUserData;

		uint16_t length_temp;
		uint8_t *descriptor_temp=usbd_composite_class_data[usbd_composite_pClass_count].pClass->GetFSConfigDescriptor(&length_temp);
		uint8_t *descriptor_end=descriptor_temp+length_temp;
		USBD_COMPOSITE_ItfAssocDescriptor *itfAssocDescriptor;

		while(descriptor_temp<descriptor_end){
			uint8_t *descriptor_current=descriptor+descriptor_size;
			USBD_memcpy(descriptor_current, descriptor_temp, descriptor_temp[0]);
			descriptor_size+=descriptor_temp[0];
			descriptor_temp+=descriptor_temp[0];
			uint8_t itf_num_temp=0;

			switch(descriptor_current[1]){
			case 0x02: // Configuration descriptor
				itf_num_temp=descriptor_current[4];
				descriptor_size-=descriptor_current[0];
				USBD_memcpy(descriptor_current, USBD_COMPOSITE_IfcAssocDesc, USB_COMPOSITE_IFC_ASSOC_DESC_SIZ);
				descriptor_size+=USB_COMPOSITE_IFC_ASSOC_DESC_SIZ;

				itfAssocDescriptor=(USBD_COMPOSITE_ItfAssocDescriptor*)(descriptor_current);
				itfAssocDescriptor->bFirstInterface=itf_num;													//Use next available interface
				itfAssocDescriptor->bInterfaceCount=itf_num_temp;
				itfAssocDescriptor->bFunctionClass=bFunctionClass;
				itfAssocDescriptor->bFunctionSubClass=bFunctionSubClass;
				itfAssocDescriptor->bFunctionProtocol=bFunctionProtocol;
				break;
			case 0x04: // Interface descriptor
				if(descriptor_current[2]!=lastIfc){ // Check if same interface different configuration.
					lastIfc=descriptor_current[2];
					descriptor_current[2]=itf_num++;
					usbd_composite_class_data[usbd_composite_pClass_count].bInterfaces++;
				} else {
					descriptor_current[2]=itf_num-1;
				}
				break;
			case 0x05: // Endpoint descriptor
				if(descriptor_current[2] & 0x80) // Check if IN EP
				{
					usbd_composite_class_data[usbd_composite_pClass_count].inEPn[usbd_composite_class_data[usbd_composite_pClass_count].inEP]=descriptor_current[2] & 0x7F;
//					usbd_composite_class_data[usbd_composite_pClass_count].inEPa[usbd_composite_class_data[usbd_composite_pClass_count].inEP++]=descriptor_current[2] & 0x7F;
//					inEP++;
					usbd_composite_class_data[usbd_composite_pClass_count].inEPa[usbd_composite_class_data[usbd_composite_pClass_count].inEP++]=inEP;
					descriptor_current[2]=inEP++ | 0x80;
				} else {
					usbd_composite_class_data[usbd_composite_pClass_count].outEPn[usbd_composite_class_data[usbd_composite_pClass_count].outEP]=descriptor_current[2] & 0x7F;
//					usbd_composite_class_data[usbd_composite_pClass_count].outEPa[usbd_composite_class_data[usbd_composite_pClass_count].outEP++]=descriptor_current[2] & 0x7F;
//					outEP++;
					usbd_composite_class_data[usbd_composite_pClass_count].outEPa[usbd_composite_class_data[usbd_composite_pClass_count].outEP++]=outEP;
					descriptor_current[2]=outEP++;
				}
				break;
			case 0x24: // CS Interface
				switch(descriptor_current[2]){
				case 0x01: //Check if Union Functional Descriptor
					descriptor_current[4]=itf_num-1+descriptor_current[4];
					break;
				case 0x06: //Check if Union Functional Descriptor
					descriptor_current[3]=itf_num-1+descriptor_current[3];
					descriptor_current[4]=itf_num-1+descriptor_current[4];
					break;
				}
				break;
			default:
				break;
			}
		}

		descriptor[2]=LOBYTE(descriptor_size);		//Update Config Descritor Total Size
		descriptor[3]=HIBYTE(descriptor_size);	//Update Config Descritor Total Size
		descriptor[4]=itf_num;			//Update the total interface count

		usbd_composite_pClass_count++;
		pdev->pClass = &USBD_COMPOSITE;

		status = USBD_OK;
	}
	else
	{
		USBD_ErrLog("Invalid Class handle");
		status = USBD_FAIL;
	}

	return status;
}

uint8_t  USBD_COMPOSITE_LL_EP_Conversion  (USBD_HandleTypeDef *pdev, uint8_t  ep_addr){
	uint8_t index=0;
	uint8_t i=0;
	if((ep_addr & 0x7f)==0){
		return ep_addr;
	}
	for(index=0;index<usbd_composite_pClass_count;index++){
		if(pdev->pClassData==usbd_composite_class_data[index].pClassData && pdev->pUserData==usbd_composite_class_data[index].pUserData){
			if(ep_addr & 0x80){
				for(i=0;i<usbd_composite_class_data[index].inEP;i++){
					if(usbd_composite_class_data[index].inEPn[i]==(ep_addr & 0x7f)){
						return usbd_composite_class_data[index].inEPa[i] | 0x80;
					}
				}
			} else {
				for(i=0;i<usbd_composite_class_data[index].outEP;i++){
					if(usbd_composite_class_data[index].outEPn[i]==ep_addr){
						return usbd_composite_class_data[index].outEPa[i];
					}
				}
			}
		}
	}
	return ep_addr;
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
