/*
 * Copyright (c) 2026, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 * CherryUSB glue layer for Synwit SWM350 (dwc2 ip)
 */
#include "usbd_core.h"
#include "usbh_core.h"
#include "usb_dwc2_param.h"
#include "usb_dwc2_reg.h" /* for the usb_dc_low_level_xxx / usbd_dwc2_xxx prototypes */

/* SWM350.h redefines these with different spelling but the same values */
#undef USB_FEATURE_REMOTE_WAKEUP
#undef USB_FEATURE_ENDPOINT_HALT
#include "SWM350.h"

/* SWM350 integrates one dwc2 otg core with an internal UTMI+ (16-bit) HS phy:
 * global regs @ USBG_BASE, device regs @ USBG_BASE + 0x800, host regs @ USBG_BASE + 0x400,
 * data fifo @ USBG_BASE + 0x1000 + 0x1000 * n. All of them match the standard dwc2 layout,
 * so usb_dc_dwc2.c / usb_hc_dwc2.c can drive the core directly.
 *
 * Pass USBG_BASE as the reg_base argument:
 *     usbd_initialize(0, USBG_BASE, NULL);
 *     usbh_initialize(0, USBG_BASE, hub_event_handler);
 *
 * PHY mapping: phy_type = UTMI (default here) clears GUSBCFG.SERIAL and sets GUSBCFG.PHY16b,
 * which is the vendor's HS configuration. phy_type = FS sets GUSBCFG bit6, the same bit as
 * SWM350's USBG_USBCFG_SERIAL, so it selects the USB 1.1 serial transceiver instead.
 *
 * Host mode note: usb_hc_dwc2.c requires the core to have the internal dma architecture
 * (GHWCFG2.ARCH == INT_DMA) and asserts on it, while the Synwit host stack is slave-only.
 * Device mode is not affected (it runs in slave mode here).
 */

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

/* Total dfifo usage is 668 words, exactly the budget the vendor stack partitions
 * (RX 284 + non-periodic TX 128 + periodic TX 256), so it fits the hw dfifo.
 *
 * 128 words is 512 byte, which is what the cherryusb hs demos use for bulk
 * (`#ifdef CONFIG_USB_HS #define CDC_MAX_MPS 512`). 16 words is the minimum the core
 * accepts and the smallest fifo the vendor driver ever assigns (64 byte packets).
 *
 * The per endpoint depths are also limited by the core's power_on values, which
 * usb_dc_dwc2.c validates at init with USB_ASSERT_MSG. Should that assert fire, it
 * prints the depth the core actually allows, for example:
 *   "device_tx_fifo_size[1] cannot be larger than power_on_value 64"
 * Lower the matching entry below to that value and keep the total within 668 words.
 */
const struct dwc2_user_params param_swm350 = {
    .phy_type = DWC2_PHY_TYPE_PARAM_UTMI, /* internal UTMI+ phy, HS capable */
    .phy_utmi_width = 16,

    .device_dma_enable = false, /* slave mode, same as the vendor stack */
    .device_dma_desc_enable = false,
    .device_rx_fifo_size = 284, /* (5 + 8) + 1024 / 4 + 2 * 8 + 2, same as the vendor stack */
    .device_tx_fifo_size = {
        [0] = 16,  /* 64 byte, control */
        [1] = 128, /* 512 byte, bulk in */
        [2] = 128, /* 512 byte, bulk in */
        [3] = 64,  /* 256 byte */
        [4] = 32,  /* 128 byte */
        [5] = 16,  /* 64 byte, interrupt in */
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

    /* The vendor stack leaves the role to the ID pin. Set this to true
     * (GOTGCTL BVALOEN | BVALOVAL) only if the board does not wire the VBUS pin.
     */
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

/* The role field (SYS->USBCFG.ROLE) is left to the ID pin by default, exactly like the
 * vendor USBD_Init()/USBH_Init() do; the device/host mode is forced by the core level
 * GUSBCFG.FDMOD/FHMOD bits which dwc2_set_mode() programs. Define
 * CONFIG_SWM350_FORCE_ROLE to 1 in usb_config.h if the board has no usable ID pin and
 * the OTG wrapper keeps selecting the wrong role. Values are from the csl header:
 * 2 host, 3 device.
 */
#ifndef CONFIG_SWM350_FORCE_ROLE
#define CONFIG_SWM350_FORCE_ROLE 0
#endif

static void usb_swm350_set_role(uint32_t role)
{
#if CONFIG_SWM350_FORCE_ROLE
    SYS->USBCFG = (SYS->USBCFG & ~SYS_USBCFG_ROLE_Msk) | (role << SYS_USBCFG_ROLE_Pos);
#else
    (void)role;
#endif
}

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
    usb_swm350_set_role(3); /* device */

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
    usb_swm350_set_role(2); /* host */

    /* VBUS 5V supply (if the board has a switch) must be driven by the application */
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
