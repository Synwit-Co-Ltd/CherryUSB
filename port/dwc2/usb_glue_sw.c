/*
 * Copyright (c) 2026, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 * CherryUSB glue layer for Synwit SWM350 (dwc2 ip)
 */
#include "usbd_core.h"
#include "usbh_core.h"
#include "usb_dwc2_param.h"
#include "usb_dwc2_reg.h"

/* SWM350.h redefines these with different spelling but the same values */
#undef USB_FEATURE_REMOTE_WAKEUP
#undef USB_FEATURE_ENDPOINT_HALT
#include "SWM350.h"


__WEAK void USBD_IRQHandler(uint8_t busid)
{
    (void)busid;
}

__WEAK void USBH_IRQHandler(uint8_t busid)
{
    (void)busid;
}

typedef void (*usb_dwc2_irq)(uint8_t busid);

static usb_dwc2_irq g_usb_dwc2_irq;
static volatile uint8_t g_usb_dwc2_busid = 0;


const struct dwc2_user_params param_swm350 = {
#ifdef CONFIG_USB_HS
    .phy_type = DWC2_PHY_TYPE_PARAM_UTMI,
#else
    .phy_type = DWC2_PHY_TYPE_PARAM_FS,
#endif
    .phy_utmi_width = 16,

    .device_dma_enable = false,         // slave-only mode
    .device_dma_desc_enable = false,
    .device_rx_fifo_size = 284,         // (5 + 8) + 1024 / 4 + 2 * 8 + 2
    .device_tx_fifo_size = {
        [0] = 16,   // 64 byte, control
        [1] = 256,  // 1024 byte
        [2] = 128,  // 512 byte
        [3] = 128,  // 512 byte
        [4] = 128,  // 512 byte
        [5] = 128,  // 512 byte
        [6] = 0,
        [7] = 0,
        [8] = 0,
        [9] = 0,
        [10] = 0,
        [11] = 0,
        [12] = 0,
        [13] = 0,
        [14] = 0,
        [15] = 0 },

    .host_dma_desc_enable = false,
    .host_rx_fifo_size = 284,        /* (5 + 8) + 1024 / 4 + 2 * 8 + 2 */
    .host_nperio_tx_fifo_size = 128, /* 512 byte, max packet size for non-periodic transfer */
    .host_perio_tx_fifo_size = 256,  /* 1024 byte, largest of all periodic transfers */

    .device_gccfg = 0,
    .host_gccfg = 0,

    .b_session_valid_override = false,
};

#ifndef CONFIG_USB_DWC2_CUSTOM_PARAM
void dwc2_get_user_params(uint32_t reg_base, struct dwc2_user_params *params)
{
    (void)reg_base;

    memcpy(params, &param_swm350, sizeof(struct dwc2_user_params));
#ifdef CONFIG_USB_DWC2_CUSTOM_FIFO
    struct usb_dwc2_user_fifo_config s_dwc2_fifo_config;

    dwc2_get_user_fifo_config(reg_base, &s_dwc2_fifo_config);

    params->device_rx_fifo_size = s_dwc2_fifo_config.device_rx_fifo_size;
    for (uint8_t i = 0; i < MAX_EPS_CHANNELS; i++) {
        params->device_tx_fifo_size[i] = s_dwc2_fifo_config.device_tx_fifo_size[i];
    }
#endif
}
#endif


/* Reset sequence from the vendor USBD_Init()/USBH_Init(): the RSTAPP/RSTMAC/RSTPHY
 * bits are active-low, hold all of them in reset first, then release the phy, and
 * finally release the mac and the app layer.
 */
static void usb_swm350_hw_reset(void)
{
    SYS->CLKEN0 |= SYS_CLKEN0_USB_Msk;
    SW_DelayMS(1);

    SYS->USBCFG &= ~(SYS_USBCFG_RSTAPP_Msk | SYS_USBCFG_RSTMAC_Msk | SYS_USBCFG_RSTPHY_Msk);
    SW_DelayMS(1);
    SYS->USBCFG |= SYS_USBCFG_RSTPHY_Msk;
    SW_DelayMS(1);
    SYS->USBCFG |= SYS_USBCFG_RSTMAC_Msk | SYS_USBCFG_RSTAPP_Msk;
    SW_DelayMS(3);
}

void usb_dc_low_level_init(uint8_t busid)
{
    g_usb_dwc2_irq = USBD_IRQHandler;
    g_usb_dwc2_busid = busid;

    usb_swm350_hw_reset();
    SYS->USBCFG = (SYS->USBCFG & ~SYS_USBCFG_ROLE_Msk) | (3 << SYS_USBCFG_ROLE_Pos);    // device

    /* GINTMSK and GAHBCFG.GINT are 0 right after the reset above, so enabling the
     * vector here cannot fire before usb_dc_init() sets the core up.
     */
    CLIC_EnableIRQ(USB_IRQn);
}

void usb_dc_low_level_deinit(uint8_t busid)
{
    (void)busid;

    CLIC_DisableIRQ(USB_IRQn);

    SYS->USBCFG &= ~(SYS_USBCFG_RSTAPP_Msk | SYS_USBCFG_RSTMAC_Msk | SYS_USBCFG_RSTPHY_Msk);
    SYS->CLKEN0 &= ~SYS_CLKEN0_USB_Msk;
}

void usb_hc_low_level_init(struct usbh_bus *bus)
{
    g_usb_dwc2_irq = USBH_IRQHandler;
    g_usb_dwc2_busid = bus->busid;

    usb_swm350_hw_reset();
    SYS->USBCFG = (SYS->USBCFG & ~SYS_USBCFG_ROLE_Msk) | (2 << SYS_USBCFG_ROLE_Pos);    // host

    CLIC_EnableIRQ(USB_IRQn);
}

void usb_hc_low_level_deinit(struct usbh_bus *bus)
{
    (void)bus;

    CLIC_DisableIRQ(USB_IRQn);

    SYS->USBCFG &= ~(SYS_USBCFG_RSTAPP_Msk | SYS_USBCFG_RSTMAC_Msk | SYS_USBCFG_RSTPHY_Msk);
    SYS->CLKEN0 &= ~SYS_CLKEN0_USB_Msk;
}

/* usb interrupt vector name defined in startup_SWM350.S */
__attribute__((interrupt))
void USB_Handler(void)
{
    if (g_usb_dwc2_irq) {
        g_usb_dwc2_irq(g_usb_dwc2_busid);
    }
}

void usbd_dwc2_delay_ms(uint8_t ms)
{
    SW_DelayMS(ms);
}

uint32_t usbd_dwc2_get_system_clock(void)
{
    return SystemCoreClock;
}
