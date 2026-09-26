/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbh_core.h"
#include "usbh_hub.h"

#undef USB_FEATURE_REMOTE_WAKEUP
#undef USB_FEATURE_ENDPOINT_HALT
#include "SWM341.h"

/*
 * The SWM341 USB host controller executes one transaction at a time: a token
 * written to USBH.TOKEN starts a SETUP/IN/OUT transaction, the answer (ACK,
 * NAK, STALL or error) is reported through USBH.IF.RXSTAT and USBH.SR.RESP.
 * Data of at most one packet (up to 1023 bytes, TXBUF/RXBUF are 1KB) can be
 * carried by each transaction.
 *
 * This driver therefore serializes all URBs into one queue. The URB at the
 * head of the queue owns the hardware and walks through its SETUP/DATA/STATUS
 * (control) or packet-by-packet (bulk/interrupt) sequence, one token per
 * completion interrupt. On NAK the URB is rotated to the tail of the queue so
 * that a NAKing HID pipe cannot starve bulk transfers.
 */

#ifndef CONFIG_USB_SWM341_URB_NUM
#define CONFIG_USB_SWM341_URB_NUM 8
#endif

/* USBH.TOKEN.TYPE field values */
#define SWM341_TOKEN_TYPE_OUT      1
#define SWM341_TOKEN_TYPE_IN       9
#define SWM341_TOKEN_TYPE_SETUP    13

/* USBH.TOKEN.SPEED field values */
#define SWM341_TOKEN_SPEED_LOW     2
#define SWM341_TOKEN_SPEED_FULL    3

/*
 * The Synwit library only issues a token when the frame remaining counter is
 * in this window, keeping transactions away from the SOF boundary. When the
 * current position is outside the window, wait (bounded) until it re-enters.
 */
#define SWM341_FRAME_REMAIN_MIN    2000
#define SWM341_FRAME_REMAIN_MAX    10000

#define SWM341_PORT_CHANGE_MSK     (USBH_PORTSR_CONNCHG_Msk | USBH_PORTSR_ENACHG_Msk | \
                                    USBH_PORTSR_SUSPCHG_Msk | USBH_PORTSR_RSTCHG_Msk)

/* consecutive bus errors before an URB is failed */
#define SWM341_ERROR_RETRIES       3

enum swm341_ctrl_stage {
    SWM341_CTRL_STAGE_SETUP = 0,
    SWM341_CTRL_STAGE_DATA_IN,
    SWM341_CTRL_STAGE_DATA_OUT,
    SWM341_CTRL_STAGE_STATUS_IN,
    SWM341_CTRL_STAGE_STATUS_OUT,
};

struct swm341_urb_priv {
    bool inuse;
    usb_slist_t list;
    struct usbh_urb *urb;
    uint8_t stage;         /* control transfer stage, enum swm341_ctrl_stage */
    uint8_t data_toggle;  /* DATA0/DATA1 of the running non-control transfer */
    uint8_t toggle_in;    /* control transfer IN toggle */
    uint8_t toggle_out;   /* control transfer OUT toggle */
    uint8_t error_retries;
    uint16_t xfer_req;    /* bytes carried by the token in flight */
    uint32_t xfer_len;    /* bytes remaining in the current transfer/stage */
    uint8_t *xfer_buf;
    usb_osal_sem_t waitsem; /* valid while urb->timeout > 0 */
};

struct swm341_hcd {
    usb_slist_t urb_list;       /* queued URBs, the head one owns the hardware */
    volatile bool xfer_busy;    /* a token has been issued and is not answered yet */
    volatile bool xfer_abandon; /* the answered token belongs to a killed URB */
    volatile bool port_csc;     /* connect status change latch */
    volatile bool port_pec;     /* port enable change latch */
    volatile bool port_pe;      /* port enabled */
    uint8_t ep_toggle_addr[16]; /* data toggle map of non-control endpoints */
    uint8_t ep_toggle[16];
    struct swm341_urb_priv urb_pool[CONFIG_USB_SWM341_URB_NUM];
} g_swm341_hcd[CONFIG_USBHOST_MAX_BUS];

/* data toggle of non-control endpoints must survive URB resubmission */
static uint8_t swm341_toggle_get(struct swm341_hcd *hcd, uint8_t ep_addr)
{
    for (uint8_t i = 0; i < 16; i++) {
        if (hcd->ep_toggle_addr[i] == ep_addr) {
            return hcd->ep_toggle[i];
        }
    }
    return 0;
}

static void swm341_toggle_set(struct swm341_hcd *hcd, uint8_t ep_addr, uint8_t toggle)
{
    uint8_t i;

    for (i = 0; i < 16; i++) {
        if (hcd->ep_toggle_addr[i] == ep_addr) {
            break;
        }
        if (hcd->ep_toggle_addr[i] == 0xFF) {
            hcd->ep_toggle_addr[i] = ep_addr;
            break;
        }
    }
    if (i < 16) {
        hcd->ep_toggle[i] = toggle;
    }
}

static void swm341_toggle_clear(struct swm341_hcd *hcd, uint8_t ep_addr)
{
    for (uint8_t i = 0; i < 16; i++) {
        if (hcd->ep_toggle_addr[i] == ep_addr) {
            hcd->ep_toggle_addr[i] = 0xFF;
            hcd->ep_toggle[i] = 0;
        }
    }
}

static void swm341_toggle_clear_all(struct swm341_hcd *hcd)
{
    memset(hcd->ep_toggle_addr, 0xFF, sizeof(hcd->ep_toggle_addr));
    memset(hcd->ep_toggle, 0, sizeof(hcd->ep_toggle));
}

static struct swm341_urb_priv *swm341_urb_cur(struct swm341_hcd *hcd)
{
    return usb_slist_first_entry_or_null(&hcd->urb_list, struct swm341_urb_priv, list);
}

static void swm341_wait_frame_window(void)
{
    uint32_t guard = 0;
    uint32_t remain = USBH->FRAMERM;

    while ((remain >= SWM341_FRAME_REMAIN_MAX) || (remain <= SWM341_FRAME_REMAIN_MIN)) {
        /* bounded wait, window opens at latest after the next SOF */
        if (++guard > 60000u) {
            break;
        }
        remain = USBH->FRAMERM;
    }
}

static void swm341_token_send(uint8_t type, uint8_t addr, uint8_t ep,
                              uint8_t datax, uint8_t speed, uint16_t size)
{
    swm341_wait_frame_window();

    USBH->TOKEN = ((uint32_t)addr << USBH_TOKEN_ADDR_Pos) |
                  ((uint32_t)ep << USBH_TOKEN_EPNR_Pos) |
                  ((uint32_t)datax << USBH_TOKEN_DATAX_Pos) |
                  ((uint32_t)type << USBH_TOKEN_TYPE_Pos) |
                  ((uint32_t)speed << USBH_TOKEN_SPEED_Pos) |
                  ((uint32_t)size << USBH_TOKEN_TRSZ_Pos);
}

/*
 * Write one packet into the host TX FIFO.
 *
 * Two details are taken from the vendor driver (SWM341_usbh.c), which the
 * TinyUSB SWM341 hcd is also built on:
 *  - USBH->TXTRSZ is programmed before the FIFO is touched. Writing the FIFO
 *    first leaves its data port unarmed and the store raises a precise bus
 *    error, observed as BFAR = 0x40005000 = USBH->TXBUF, PC inside usb_memcpy.
 *  - the packet is written byte by byte, as every Synwit host implementation
 *    does. The FIFO accepted word stores in device mode (USBD_TxWrite uses
 *    USBD_memcpy), but keep the byte loop here since that is the only form
 *    the host FIFO is documented and exercised with.
 *
 * memcpy() is deliberately not used: usb_config.h leaves
 * CONFIG_USB_MEMCPY_DISABLE undefined, so usb_memcpy() replaces it and takes
 * its 32-bit word path whenever source and destination are both dword aligned,
 * which g_setup_buffer and ep0_request_buffer always are.
 */
static void swm341_txbuf_write(const uint8_t *src, uint16_t size)
{
    volatile uint8_t *fifo = (volatile uint8_t *)USBH->TXBUF;

    /* volatile keeps the compiler from widening this into word stores */
    for (uint16_t i = 0; i < size; i++) {
        fifo[i] = src[i];
    }
}

static void swm341_urb_next_token(struct swm341_urb_priv *priv)
{
    struct usbh_urb *urb = priv->urb;
    uint16_t mps = USB_GET_MAXPACKETSIZE(urb->ep->wMaxPacketSize);
    uint8_t speed = (urb->hport->speed == USB_SPEED_LOW) ? SWM341_TOKEN_SPEED_LOW : SWM341_TOKEN_SPEED_FULL;
    uint8_t addr = urb->hport->dev_addr;
    uint16_t size;

    if (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes) == USB_ENDPOINT_TYPE_CONTROL) {
        switch (priv->stage) {
            case SWM341_CTRL_STAGE_SETUP:
                USBH->TXTRSZ = 8;
                swm341_txbuf_write((const uint8_t *)urb->setup, 8);
                priv->xfer_req = 8;
                swm341_token_send(SWM341_TOKEN_TYPE_SETUP, addr, 0, 0, speed, 8);
                break;
            case SWM341_CTRL_STAGE_DATA_IN:
                size = MIN(priv->xfer_len, mps);
                priv->xfer_req = size;
                swm341_token_send(SWM341_TOKEN_TYPE_IN, addr, 0, priv->toggle_in, speed, size);
                break;
            case SWM341_CTRL_STAGE_DATA_OUT:
                size = MIN(priv->xfer_len, mps);
                USBH->TXTRSZ = size;
                swm341_txbuf_write(priv->xfer_buf, size);
                priv->xfer_req = size;
                swm341_token_send(SWM341_TOKEN_TYPE_OUT, addr, 0, priv->toggle_out, speed, size);
                break;
            case SWM341_CTRL_STAGE_STATUS_IN:
                priv->xfer_req = 0;
                swm341_token_send(SWM341_TOKEN_TYPE_IN, addr, 0, priv->toggle_in, speed, 0);
                break;
            case SWM341_CTRL_STAGE_STATUS_OUT:
            default:
                USBH->TXTRSZ = 0;
                priv->xfer_req = 0;
                swm341_token_send(SWM341_TOKEN_TYPE_OUT, addr, 0, priv->toggle_out, speed, 0);
                break;
        }
    } else {
        uint8_t ep = urb->ep->bEndpointAddress & 0x0f;

        if (urb->ep->bEndpointAddress & 0x80) {
            size = MIN(priv->xfer_len, mps);
            priv->xfer_req = size;
            swm341_token_send(SWM341_TOKEN_TYPE_IN, addr, ep, priv->data_toggle, speed, size);
        } else {
            size = MIN(priv->xfer_len, mps);
            USBH->TXTRSZ = size;
            swm341_txbuf_write(priv->xfer_buf, size);
            priv->xfer_req = size;
            swm341_token_send(SWM341_TOKEN_TYPE_OUT, addr, ep, priv->data_toggle, speed, size);
        }
    }
}

/* start the next token of the queue head, must run in a critical section */
static void swm341_engine_kick(struct swm341_hcd *hcd)
{
    struct swm341_urb_priv *priv = swm341_urb_cur(hcd);

    if (priv && !hcd->xfer_busy && !hcd->xfer_abandon) {
        hcd->xfer_busy = true;
        swm341_urb_next_token(priv);
    }
}

static void swm341_urb_complete(struct swm341_hcd *hcd, struct swm341_urb_priv *priv, int errorcode)
{
    struct usbh_urb *urb = priv->urb;

    usb_slist_remove(&hcd->urb_list, &priv->list);

    if (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes) == USB_ENDPOINT_TYPE_CONTROL) {
        /*
         * USB spec: CLEAR_FEATURE(ENDPOINT_HALT), SET_CONFIGURATION and
         * SET_INTERFACE reset the data toggle of the affected endpoints.
         */
        if ((errorcode == 0) && urb->setup) {
            struct usb_setup_packet *setup = urb->setup;

            if (((setup->bmRequestType & USB_REQUEST_TYPE_MASK) == USB_REQUEST_STANDARD) &&
                (setup->bRequest == USB_REQUEST_CLEAR_FEATURE) &&
                ((setup->bmRequestType & USB_REQUEST_RECIPIENT_MASK) == USB_REQUEST_RECIPIENT_ENDPOINT) &&
                (setup->wValue == USB_FEATURE_ENDPOINT_HALT)) {
                swm341_toggle_clear(hcd, setup->wIndex & 0x8f);
            } else if (((setup->bmRequestType & USB_REQUEST_TYPE_MASK) == USB_REQUEST_STANDARD) &&
                       ((setup->bRequest == USB_REQUEST_SET_CONFIGURATION) ||
                        (setup->bRequest == USB_REQUEST_SET_INTERFACE))) {
                swm341_toggle_clear_all(hcd);
            }
        }
    } else {
        /* remember the next expected toggle for the next submission */
        swm341_toggle_set(hcd, urb->ep->bEndpointAddress & 0x8f, priv->data_toggle);
    }

    urb->errorcode = errorcode;

    if (urb->timeout > 0) {
        /* the submitting thread owns the pool slot and frees it */
        usb_osal_sem_give(priv->waitsem);
    } else {
        urb->hcpriv = NULL;
        priv->urb = NULL;
        priv->inuse = false;
    }

    if (urb->complete) {
        if (urb->errorcode < 0) {
            urb->complete(urb->arg, urb->errorcode);
        } else {
            urb->complete(urb->arg, urb->actual_length);
        }
    }
}

static void swm341_kill_all_urbs(struct swm341_hcd *hcd, int errorcode)
{
    struct swm341_urb_priv *priv;

    while ((priv = swm341_urb_cur(hcd)) != NULL) {
        if ((hcd->urb_list.next == &priv->list) && hcd->xfer_busy) {
            hcd->xfer_abandon = true;
        }
        swm341_urb_complete(hcd, priv, errorcode);
    }
}

/* process the response of the token in flight, called from the IRQ */
static void swm341_handle_response(struct swm341_hcd *hcd)
{
    struct swm341_urb_priv *priv;
    struct usbh_urb *urb;
    uint8_t resp = (USBH->SR & USBH_SR_RESP_Msk) >> USBH_SR_RESP_Pos;
    uint16_t rx_size;

    /* the RXSTAT interrupt means the token in flight has been answered */
    hcd->xfer_busy = false;

    if (hcd->xfer_abandon) {
        /* belongs to an URB killed while the token was in flight, drop it */
        hcd->xfer_abandon = false;
        swm341_engine_kick(hcd);
        return;
    }

    priv = swm341_urb_cur(hcd);
    if (priv == NULL) {
        return;
    }
    urb = priv->urb;

    switch (resp) {
        case USBR_ACK:
            priv->error_retries = 0;

            if (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes) == USB_ENDPOINT_TYPE_CONTROL) {
                switch (priv->stage) {
                    case SWM341_CTRL_STAGE_SETUP:
                        /* data and status stages of a control transfer start from DATA1 */
                        priv->toggle_in = 1;
                        priv->toggle_out = 1;
                        if (priv->xfer_len != 0) {
                            priv->stage = (urb->setup->bmRequestType & 0x80) ? SWM341_CTRL_STAGE_DATA_IN : SWM341_CTRL_STAGE_DATA_OUT;
                        } else {
                            priv->stage = (urb->setup->bmRequestType & 0x80) ? SWM341_CTRL_STAGE_STATUS_OUT : SWM341_CTRL_STAGE_STATUS_IN;
                        }
                        break;
                    case SWM341_CTRL_STAGE_DATA_IN:
                        rx_size = (USBH->SR & USBH_SR_TRSZ_Msk) >> USBH_SR_TRSZ_Pos;
                        if (rx_size > priv->xfer_req) {
                            rx_size = priv->xfer_req;
                        }
                        memcpy(priv->xfer_buf, (uint8_t *)USBH->RXBUF, rx_size);
                        priv->xfer_buf += rx_size;
                        priv->xfer_len -= rx_size;
                        urb->actual_length += rx_size;
                        priv->toggle_in ^= 1;
                        /* short packet terminates the data stage */
                        if ((rx_size < priv->xfer_req) || (priv->xfer_len == 0)) {
                            priv->stage = SWM341_CTRL_STAGE_STATUS_OUT;
                        }
                        break;
                    case SWM341_CTRL_STAGE_DATA_OUT:
                        priv->xfer_buf += priv->xfer_req;
                        priv->xfer_len -= priv->xfer_req;
                        urb->actual_length += priv->xfer_req;
                        priv->toggle_out ^= 1;
                        if (priv->xfer_len == 0) {
                            priv->stage = SWM341_CTRL_STAGE_STATUS_IN;
                        }
                        break;
                    case SWM341_CTRL_STAGE_STATUS_IN:
                    case SWM341_CTRL_STAGE_STATUS_OUT:
                    default:
                        swm341_urb_complete(hcd, priv, 0);
                        break;
                    }
            } else {
                if (urb->ep->bEndpointAddress & 0x80) {
                    rx_size = (USBH->SR & USBH_SR_TRSZ_Msk) >> USBH_SR_TRSZ_Pos;
                    if (rx_size > priv->xfer_req) {
                        rx_size = priv->xfer_req;
                    }
                    memcpy(priv->xfer_buf, (uint8_t *)USBH->RXBUF, rx_size);
                    priv->xfer_buf += rx_size;
                    priv->xfer_len -= rx_size;
                    urb->actual_length += rx_size;
                    priv->data_toggle ^= 1;
                    if ((rx_size < priv->xfer_req) || (priv->xfer_len == 0)) {
                        swm341_urb_complete(hcd, priv, 0);
                        break;
                    }
                } else {
                    priv->xfer_buf += priv->xfer_req;
                    priv->xfer_len -= priv->xfer_req;
                    urb->actual_length += priv->xfer_req;
                    priv->data_toggle ^= 1;
                    if (priv->xfer_len == 0) {
                        swm341_urb_complete(hcd, priv, 0);
                        break;
                    }
                }
            }
            break;

        case USBR_NAK:
            priv->error_retries = 0;
            if (hcd->urb_list.next->next != NULL) {
                /* more URBs are waiting: yield the bus to them */
                usb_slist_t *node = hcd->urb_list.next;
                usb_slist_remove(&hcd->urb_list, node);
                usb_slist_add_tail(&hcd->urb_list, node);
            }
            break;

        case USBR_STALL:
            swm341_urb_complete(hcd, priv, -USB_ERR_STALL);
            break;

        default: /* CRC, stuff, toggle, timeout, ... bus errors */
            if (++priv->error_retries < SWM341_ERROR_RETRIES) {
                break; /* retry the same token */
            }
            swm341_urb_complete(hcd, priv, -USB_ERR_IO);
            break;
    }

    swm341_engine_kick(hcd);
}

static int usbh_reset_port(struct usbh_bus *bus, const uint8_t port)
{
    struct swm341_hcd *hcd = &g_swm341_hcd[bus->hcd.hcd_id];

    (void)port;

    hcd->port_pe = 0;
    hcd->port_pec = 0;
    swm341_toggle_clear_all(hcd);

    /* writing RESET issues a port reset pulse, hardware re-enables the port
     * and raises RSTCHG when the reset sequence is finished */
    USBH->PORTSR = USBH_PORTSR_RESET_Msk;

    for (int i = 0; i < 200; i++) {
        if (USBH->PORTSR & USBH_PORTSR_ENA_Msk) {
            break;
        }
        usb_osal_msleep(1);
    }

    usb_osal_msleep(10);

    /* clear the change flags raised by the reset */
    USBH->PORTSR = SWM341_PORT_CHANGE_MSK;

    hcd->port_pe = (USBH->PORTSR & USBH_PORTSR_ENA_Msk) ? 1 : 0;

    return 0;
}

__WEAK void usb_hc_low_level_init(struct usbh_bus *bus)
{
    (void)bus;

    SYS->USBCR = 0;
    for (uint32_t i = 0; i < CyclesPerUs; i++) {
        __NOP();
    }
    SYS->USBCR |= (1 << SYS_USBCR_RST48M_Pos);
    __DSB();
    SYS->USBCR |= (1 << SYS_USBCR_RST12M_Pos);
    __DSB();
    SYS->USBCR |= (1 << SYS_USBCR_RSTPLL_Pos);
    __DSB();
    for (uint32_t i = 0; i < CyclesPerUs; i++) {
        __NOP();
    }

    SYS->USBCR &= ~SYS_USBCR_ROLE_Msk;
    SYS->USBCR |= (2 << SYS_USBCR_ROLE_Pos); /* host role */

    SYS->USBCR |= (1 << SYS_USBCR_VBUS_Pos);

    SYS->CLKEN0 |= (0x01 << SYS_CLKEN0_USB_Pos);
}

__WEAK void usb_hc_low_level_deinit(struct usbh_bus *bus)
{
    (void)bus;
}

int usb_hc_init(struct usbh_bus *bus)
{
    struct swm341_hcd *hcd = &g_swm341_hcd[bus->hcd.hcd_id];

    usb_hc_low_level_init(bus);

    memset(hcd, 0, sizeof(struct swm341_hcd));
    usb_slist_init(&hcd->urb_list);
    swm341_toggle_clear_all(hcd);

    for (uint8_t i = 0; i < CONFIG_USB_SWM341_URB_NUM; i++) {
        hcd->urb_pool[i].waitsem = usb_osal_sem_create(0);
    }

    USBH->CR = USBH_CR_FLUSHFF_Msk;

    USBD->DEVCR = (0 << USBD_DEVCR_DEVICE_Pos) | /* host mode */
                  (3 << USBD_DEVCR_SPEED_Pos) |
                  (1 << USBD_DEVCR_CSRDONE_Pos);

    /* clear pending flags before unmasking */
    USBH->IF = USBH_IF_RXSTAT_Msk | USBH_IF_ABVTHR_Msk | USBH_IF_BLWTHR_Msk |
               USBH_IF_SOF_Msk | USBH_IF_PORT_Msk | USBH_IF_OTG_Msk;

	NVIC_SetPriority(USB_IRQn, 5);   /* ±ØÐë²»µÍÓÚ configMAX_SYSCALL_INTERRUPT_PRIORITY */
    NVIC_EnableIRQ(USB_IRQn);

    USBH->IE = USBH_IF_RXSTAT_Msk | USBH_IF_PORT_Msk;

    /* power the port, a device attach then raises the PORT interrupt */
    USBH->PORTSR = USBH_PORTSR_POWER_Msk;

    return 0;
}

int usb_hc_deinit(struct usbh_bus *bus)
{
    struct swm341_hcd *hcd = &g_swm341_hcd[bus->hcd.hcd_id];

    USBH->IE = 0;
    NVIC_DisableIRQ(USB_IRQn);

    swm341_kill_all_urbs(hcd, -USB_ERR_SHUTDOWN);

    for (uint8_t i = 0; i < CONFIG_USB_SWM341_URB_NUM; i++) {
        usb_osal_sem_delete(hcd->urb_pool[i].waitsem);
    }

    USBH->PORTSR = USBH_PORTSR_CLRPOWER_Msk;

    usb_hc_low_level_deinit(bus);
    return 0;
}

uint16_t usbh_get_frame_number(struct usbh_bus *bus)
{
    (void)bus;
    return (uint16_t)USBH->FRAMENR;
}

int usbh_roothub_control(struct usbh_bus *bus, struct usb_setup_packet *setup, uint8_t *buf)
{
    struct swm341_hcd *hcd = &g_swm341_hcd[bus->hcd.hcd_id];
    uint8_t nports = 1; /* the SWM341 host has a single port */
    uint8_t port = setup->wIndex;
    uint32_t status;

    if (setup->bmRequestType & USB_REQUEST_RECIPIENT_DEVICE) {
        switch (setup->bRequest) {
            case HUB_REQUEST_CLEAR_FEATURE:
                switch (setup->wValue) {
                    case HUB_FEATURE_HUB_C_LOCALPOWER:
                        break;
                    case HUB_FEATURE_HUB_C_OVERCURRENT:
                        break;
                    default:
                        return -USB_ERR_INVAL;
                }
                break;
            case HUB_REQUEST_SET_FEATURE:
                switch (setup->wValue) {
                    case HUB_FEATURE_HUB_C_LOCALPOWER:
                        break;
                    case HUB_FEATURE_HUB_C_OVERCURRENT:
                        break;
                    default:
                        return -USB_ERR_INVAL;
                }
                break;
            case HUB_REQUEST_GET_STATUS:
                memset(buf, 0, 4);
                break;
            default:
                break;
        }
    } else if (setup->bmRequestType & USB_REQUEST_RECIPIENT_OTHER) {
        switch (setup->bRequest) {
            case HUB_REQUEST_CLEAR_FEATURE:
                if (!port || port > nports) {
                    return -USB_ERR_INVAL;
                }

                switch (setup->wValue) {
                    case HUB_PORT_FEATURE_ENABLE:
                        USBH->PORTSR = USBH_PORTSR_CLRENA_Msk;
                        hcd->port_pe = 0;
                        break;
                    case HUB_PORT_FEATURE_SUSPEND:
                    case HUB_PORT_FEATURE_C_SUSPEND:
                    case HUB_PORT_FEATURE_POWER:
                        break;
                    case HUB_PORT_FEATURE_C_CONNECTION:
                        USBH->PORTSR = USBH_PORTSR_CONNCHG_Msk;
                        hcd->port_csc = 0;
                        break;
                    case HUB_PORT_FEATURE_C_ENABLE:
                        USBH->PORTSR = USBH_PORTSR_ENACHG_Msk;
                        hcd->port_pec = 0;
                        break;
                    case HUB_PORT_FEATURE_C_OVER_CURREN:
                        break;
                    case HUB_PORT_FEATURE_C_RESET:
                        USBH->PORTSR = USBH_PORTSR_RSTCHG_Msk;
                        break;
                    default:
                        return -USB_ERR_INVAL;
                }
                break;
            case HUB_REQUEST_SET_FEATURE:
                if (!port || port > nports) {
                    return -USB_ERR_INVAL;
                }

                switch (setup->wValue) {
                    case HUB_PORT_FEATURE_RESET:
                        usbh_reset_port(bus, port);
                        break;
                    case HUB_PORT_FEATURE_SUSPEND:
                        /* suspend/resume not supported by this driver */
                        break;
                    case HUB_PORT_FEATURE_POWER:
                        USBH->PORTSR = USBH_PORTSR_POWER_Msk;
                        break;
                    default:
                        return -USB_ERR_INVAL;
                }
                break;
            case HUB_REQUEST_GET_STATUS:
                if (!port || port > nports) {
                    return -USB_ERR_INVAL;
                }

                status = 0;
                if (hcd->port_csc) {
                    status |= (1 << HUB_PORT_FEATURE_C_CONNECTION);
                }
                if (hcd->port_pec) {
                    status |= (1 << HUB_PORT_FEATURE_C_ENABLE);
                }

                /* the hub thread checks CONNECTION before resetting the port,
                 * so report it independently of the port enable state */
                if (USBH->PORTSR & USBH_PORTSR_CONN_Msk) {
                    status |= (1 << HUB_PORT_FEATURE_CONNECTION);
                }
                if (USBH->PORTSR & USBH_PORTSR_ENA_Msk) {
                    status |= (1 << HUB_PORT_FEATURE_ENABLE);
                    if (USBH->PORTSR & USBH_PORTSR_SPEED_Msk) {
                        status |= (1 << HUB_PORT_FEATURE_LOWSPEED);
                    }
                }

                status |= (1 << HUB_PORT_FEATURE_POWER);
                memcpy(buf, &status, 4);
                break;
            default:
                break;
        }
    }
    return 0;
}

int usbh_submit_urb(struct usbh_urb *urb)
{
    struct usbh_bus *bus;
    struct swm341_hcd *hcd;
    struct swm341_urb_priv *priv = NULL;
    size_t flags;
    int ret = 0;

    if (!urb || !urb->hport || !urb->ep || !urb->hport->bus) {
        return -USB_ERR_INVAL;
    }

    if (!urb->hport->connected) {
        return -USB_ERR_NOTCONN;
    }

    if (urb->errorcode == -USB_ERR_BUSY) {
        return -USB_ERR_BUSY;
    }

    switch (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes)) {
        case USB_ENDPOINT_TYPE_CONTROL:
        case USB_ENDPOINT_TYPE_BULK:
        case USB_ENDPOINT_TYPE_INTERRUPT:
            break;
        case USB_ENDPOINT_TYPE_ISOCHRONOUS:
        /* fall through */
        default:
            return -USB_ERR_NOTSUPP;
    }

    bus = urb->hport->bus;
    hcd = &g_swm341_hcd[bus->hcd.hcd_id];

    flags = usb_osal_enter_critical_section();
    for (uint8_t i = 0; i < CONFIG_USB_SWM341_URB_NUM; i++) {
        if (!hcd->urb_pool[i].inuse) {
            priv = &hcd->urb_pool[i];
            priv->inuse = true;
            break;
        }
    }
    usb_osal_leave_critical_section(flags);

    if (priv == NULL) {
        return -USB_ERR_NOMEM;
    }

    priv->urb = urb;
    priv->error_retries = 0;
    priv->xfer_req = 0;
    priv->xfer_buf = urb->transfer_buffer;
    priv->xfer_len = urb->transfer_buffer_length;

    if (USB_GET_ENDPOINT_TYPE(urb->ep->bmAttributes) == USB_ENDPOINT_TYPE_CONTROL) {
        priv->stage = SWM341_CTRL_STAGE_SETUP;
        priv->toggle_in = 0;
        priv->toggle_out = 0;
    } else {
        priv->data_toggle = swm341_toggle_get(hcd, urb->ep->bEndpointAddress & 0x8f);
    }

    urb->hcpriv = priv;
    urb->errorcode = -USB_ERR_BUSY;
    urb->actual_length = 0;

    flags = usb_osal_enter_critical_section();
    usb_slist_add_tail(&hcd->urb_list, &priv->list);
    swm341_engine_kick(hcd);
    usb_osal_leave_critical_section(flags);

    if (urb->timeout > 0) {
        ret = usb_osal_sem_take(priv->waitsem, urb->timeout);
        if (ret < 0) {
            urb->timeout = 0;
            usbh_kill_urb(urb);
            return -USB_ERR_TIMEOUT;
        }
        urb->timeout = 0;
        ret = urb->errorcode;

        flags = usb_osal_enter_critical_section();
        urb->hcpriv = NULL;
        priv->urb = NULL;
        priv->inuse = false;
        usb_osal_leave_critical_section(flags);
    }

    return ret;
}

int usbh_kill_urb(struct usbh_urb *urb)
{
    struct swm341_hcd *hcd;
    struct swm341_urb_priv *priv;
    size_t flags;

    if (!urb || !urb->hcpriv || !urb->hport->bus) {
        return -USB_ERR_INVAL;
    }

    hcd = &g_swm341_hcd[urb->hport->bus->hcd.hcd_id];

    flags = usb_osal_enter_critical_section();

    priv = (struct swm341_urb_priv *)urb->hcpriv;

    if ((hcd->urb_list.next == &priv->list) && hcd->xfer_busy) {
        /* the token in flight will be answered later, discard the response */
        hcd->xfer_abandon = true;
    }

    swm341_urb_complete(hcd, priv, -USB_ERR_SHUTDOWN);

    usb_osal_leave_critical_section(flags);
    return 0;
}

void USBH_IRQHandler(uint8_t busid)
{
    struct usbh_bus *bus = &g_usbhost_bus[busid];
    struct swm341_hcd *hcd = &g_swm341_hcd[bus->hcd.hcd_id];
    uint32_t intr = USBH->IF;
    uint32_t portsr;

    if (intr & USBH_IF_PORT_Msk) {
        portsr = USBH->PORTSR;

        /* write 1 to the change bits to clear them */
        USBH->PORTSR = portsr & SWM341_PORT_CHANGE_MSK;
        USBH->IF = USBH_IF_PORT_Msk;

        if (portsr & USBH_PORTSR_CONNCHG_Msk) {
            hcd->port_csc = 1;

            if (portsr & USBH_PORTSR_CONN_Msk) {
                USB_LOG_INFO("Device attached\r\n");
            } else {
                USB_LOG_INFO("Device detached\r\n");
                hcd->port_pe = 0;
                swm341_toggle_clear_all(hcd);
                swm341_kill_all_urbs(hcd, -USB_ERR_SHUTDOWN);
            }
            bus->hcd.roothub.int_buffer[0] = (1 << 1);
            usbh_hub_thread_wakeup(&bus->hcd.roothub);
        }

        if (portsr & USBH_PORTSR_ENACHG_Msk) {
            hcd->port_pec = 1;

            if (portsr & USBH_PORTSR_ENA_Msk) {
                hcd->port_pe = 1;
            } else {
                /* babble or severe bus error: transfers cannot continue */
                hcd->port_pe = 0;
                swm341_kill_all_urbs(hcd, -USB_ERR_IO);
            }
            bus->hcd.roothub.int_buffer[0] = (1 << 1);
            usbh_hub_thread_wakeup(&bus->hcd.roothub);
        }
    }

    if (intr & USBH_IF_RXSTAT_Msk) {
        USBH->IF = USBH_IF_RXSTAT_Msk;
        swm341_handle_response(hcd);
    }
}
