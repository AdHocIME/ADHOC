#include <usbd_composite.h>
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_rndis.h"
#include "usbd_rndis_if.h"

USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_DEVICE_Init(void)
{
	USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);


//	USBD_RegisterClass(&hUsbDeviceFS, &USBD_AUDIO);
//	USBD_AUDIO_RegisterInterface(&hUsbDeviceFS, &USBD_AUDIO_fops_FS);
//	USBD_COMPOSITE_RegisterClass(&hUsbDeviceFS, 0x01, 0x00, 0x00);

//	USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
//	USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
//	USBD_COMPOSITE_RegisterClass(&hUsbDeviceFS, 0x02, 0x02, 0x00);

//	USBD_RegisterClass(&hUsbDeviceFS, &USBD_MSC);
//	USBD_MSC_RegisterStorage(&hUsbDeviceFS, &USBD_Storage_Interface_fops_FS);
//	USBD_COMPOSITE_RegisterClass(&hUsbDeviceFS, 0x08, 0x06, 0x50);

	USBD_RegisterClass(&hUsbDeviceFS, &USBD_RNDIS);
	USBD_RNDIS_RegisterInterface(&hUsbDeviceFS, &USBD_RNDIS_Interface_fops_FS);
	USBD_COMPOSITE_RegisterClass(&hUsbDeviceFS, 0xE0, 0x01, 0x03);

	USBD_Start(&hUsbDeviceFS);

}
