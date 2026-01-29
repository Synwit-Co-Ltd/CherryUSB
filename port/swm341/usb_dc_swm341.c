#include "usbd_core.h"

#undef USB_FEATURE_REMOTE_WAKEUP
#undef USB_FEATURE_ENDPOINT_HALT
#include "SWM341.h"

#define CONFIG_USBDEV_EP_NUM	8

/* Endpoint state */
struct usb_dc_ep_state {
	uint8_t ep_reg;		// peripheral endpoint register for this Endpoint
    uint16_t ep_mps;    /* Endpoint max packet size */
    uint8_t ep_type;    /* Endpoint type */
    uint8_t ep_stalled; /* Endpoint stall flag */
    uint8_t *xfer_buf;
    uint32_t xfer_len;
    uint32_t xfered_len;
};

/* Driver state */
struct swm341_udc {
	uint8_t ep_reg;		// free peripheral endpoint register
    volatile uint8_t dev_addr;
    struct usb_dc_ep_state in_ep[CONFIG_USBDEV_EP_NUM];  /*!< IN endpoint parameters*/
    struct usb_dc_ep_state out_ep[CONFIG_USBDEV_EP_NUM]; /*!< OUT endpoint parameters */
} g_swm341_udc;


__WEAK void usb_dc_low_level_init(void)
{
	SYS->USBCR |= (1 << SYS_USBCR_RST48M_Pos); __DSB();
	SYS->USBCR |= (1 << SYS_USBCR_RST12M_Pos); __DSB();
	SYS->USBCR |= (1 << SYS_USBCR_RSTPLL_Pos); __DSB();
	
	SYS->USBCR &= ~SYS_USBCR_ROLE_Msk;
	SYS->USBCR |= (3 << SYS_USBCR_ROLE_Pos);
	
	SYS->USBCR |= (1 << SYS_USBCR_VBUS_Pos);
	
	SYS->CLKEN0 |= (0x01 << SYS_CLKEN0_USB_Pos);
	
	USBD->DEVCR = (3 << USBD_DEVCR_SPEED_Pos)  |
				  (1 << USBD_DEVCR_DEVICE_Pos) |
				  (1 << USBD_DEVCR_CSRDONE_Pos);
	
	USBD->DEVIE = (1 << USBD_DEVIE_RST_Pos)   |
				  (1 << USBD_DEVIE_SETUP_Pos) |
				  (1 << USBD_DEVIE_SETCFG_Pos);
	
	USBD->EPIE = 0;
	
	NVIC_EnableIRQ(USB_IRQn);
	
	USBD_PullUp_Enable();
}

__WEAK void usb_dc_low_level_deinit(void)
{
	NVIC_DisableIRQ(USB_IRQn);
	
	USBD_PullUp_Disable();
	
	USBD->DEVIE = 0;
	USBD->EPIE = 0;
}


int usb_dc_init(uint8_t busid)
{
    usb_dc_low_level_init();
	
    return 0;
}

int usb_dc_deinit(uint8_t busid)
{
	usb_dc_low_level_deinit();
	
    return 0;
}

int usbd_set_address(uint8_t busid, const uint8_t addr)
{
	g_swm341_udc.dev_addr = addr;
	
    return 0;
}

int usbd_set_remote_wakeup(uint8_t busid)
{
    return -1;
}

uint8_t usbd_get_port_speed(uint8_t busid)
{
    return USB_SPEED_FULL;
}

int usbd_ep_open(uint8_t busid, const struct usb_endpoint_descriptor *ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep->bEndpointAddress);
	uint8_t ep_dir = USB_EP_GET_DIR(ep->bEndpointAddress);
	
	struct usb_dc_ep_state * hEP = ep_dir ? &g_swm341_udc.in_ep[ep_idx] : &g_swm341_udc.out_ep[ep_idx];
	
	hEP->ep_reg = g_swm341_udc.ep_reg++;
	hEP->ep_mps = USB_GET_MAXPACKETSIZE(ep->wMaxPacketSize);
	hEP->ep_type = USB_GET_ENDPOINT_TYPE(ep->bmAttributes);
	
	USBD_EPConfig(hEP->ep_reg, ep_idx, ep_dir, hEP->ep_type, hEP->ep_mps, ep_idx ? 1 : 0, 0, 0);
	
	USBD->EPIE |= (1 << ep_idx) << (ep_dir ? 0 : 16);
	
    return 0;
}

int usbd_ep_close(uint8_t busid, const uint8_t ep)
{
    return 0;
}

int usbd_ep_set_stall(uint8_t busid, const uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	
	if(USB_EP_DIR_IS_IN(ep))
        USBD_TxStall(ep_idx);
    else
        USBD_RxStall(ep_idx);
	
    return 0;
}

int usbd_ep_clear_stall(uint8_t busid, const uint8_t ep)
{
    return 0;
}

int usbd_ep_is_stalled(uint8_t busid, const uint8_t ep, uint8_t *stalled)
{
    return 0;
}

int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
	
    if (!data && data_len) {
        return -1;
    }
	
	struct usb_dc_ep_state * hEP = &g_swm341_udc.in_ep[ep_idx];
	
    hEP->xfer_buf = (uint8_t *)data;
    hEP->xfer_len = data_len;
    hEP->xfered_len = 0;
	
	USBD_TxWrite(ep_idx, (uint8_t *)data, MIN(data_len, hEP->ep_mps));
	
    return 0;
}

int usbd_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
	
    if (!data && data_len) {
        return -1;
    }
	
	struct usb_dc_ep_state * hEP = &g_swm341_udc.out_ep[ep_idx];
	
    hEP->xfer_buf = (uint8_t *)data;
    hEP->xfer_len = data_len;
    hEP->xfered_len = 0;
	
	USBD_RxReady(ep_idx);
	
    return 0;
}

void USBD_IRQHandler(uint8_t busid)
{
	struct usb_dc_ep_state * hEP;
	
	uint32_t devif = USBD->DEVIF;
    uint32_t epif  = USBD->EPIF;
	
    if(devif & USBD_DEVIF_RST_Msk)
    {
        USBD->DEVIF = USBD_DEVIF_RST_Msk;
		
		memset(&g_swm341_udc, 0, sizeof(struct swm341_udc));
		
        usbd_event_reset_handler(busid);
    }
	else if(devif & USBD_DEVIF_SETCFG_Msk)
	{
		USBD->DEVIF = USBD_DEVIF_SETCFG_Msk;
		
		uint32_t len;
		struct usb_setup_packet setup;
		setup.bRequest = USB_REQUEST_SET_CONFIGURATION;
		setup.wValue = (USBD->DEVSR & USBD_DEVSR_CFG_Msk) >> USBD_DEVSR_CFG_Pos;
		
		bool usbd_std_device_req_handler(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
        usbd_std_device_req_handler(busid, &setup, NULL, &len);
	}
    else if(devif & USBD_DEVIF_SETUP_Msk)
    {
		USBD->SETUPSR = USBD_SETUPSR_DONE_Msk;
		
        uint32_t SetupBuff[2];
		SetupBuff[0] = USBD->SETUPD1;
        SetupBuff[1] = USBD->SETUPD2;
		
        usbd_event_ep0_setup_complete_handler(busid, (uint8_t *)SetupBuff);
    }
	else
    {
        for(uint8_t ep_reg = 0; ep_reg < 8; ep_reg++)
        {
            uint8_t ep_dir = USBD->EPCFG[ep_reg] & USBD_EPCFG_DIR_Msk;
            uint8_t ep_nbr = USBD->EPCFG[ep_reg] & USBD_EPCFG_EPNR_Msk;
			
            uint8_t ep_addr = ep_nbr | (ep_dir ? 0x80 : 0x00);
			
            if(ep_dir)
            {
				hEP = &g_swm341_udc.in_ep[ep_nbr];
				
                if(epif & (1 << ep_nbr))
                {
                    if(USBD_TxSuccess(ep_nbr))
                    {
                        uint16_t size = MIN(hEP->xfer_len - hEP->xfered_len, hEP->ep_mps);
						
						hEP->xfer_buf += size;
                        hEP->xfered_len += size;
						
                        /* if more data to send, send it; otherwise, alert CherryUSB that we've finished */
                        if(hEP->xfered_len == hEP->xfer_len)
							usbd_event_ep_in_complete_handler(busid, ep_addr, hEP->xfered_len);
                        else
							USBD_TxWrite(ep_nbr, hEP->xfer_buf, MIN(hEP->xfer_len - hEP->xfered_len, hEP->ep_mps));
                    }
                    USBD_TxIntClr(ep_nbr);
                }
            }
            else
            {
				hEP = &g_swm341_udc.out_ep[ep_nbr];
				
                if(epif & (1 << (16 + ep_nbr)))
                {
                    USBD_RxIntClr();
                    if(USBD_RxSuccess())
                    {
                        uint16_t size = USBD_RxRead(hEP->xfer_buf, hEP->ep_mps);
						
                        hEP->xfer_buf += size;
                        hEP->xfered_len += size;
						
                        /* when the transfer is finished, alert CherryUSB; otherwise, accept more data */
                        if((hEP->xfered_len == hEP->xfer_len) || (size < hEP->ep_mps))
							usbd_event_ep_out_complete_handler(busid, ep_addr, hEP->xfered_len);
                        else
                            USBD_RxReady(ep_nbr);
                    }
                }
            }
        }
    }
}