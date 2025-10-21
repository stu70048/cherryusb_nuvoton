/*
 * Copyright (c) 2025, Kenny Tseng
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include "NuMicro.h"
#include "usbd_core.h"

// Compile-time configuration (that can be externally overridden if necessary)
// Maximum number of endpoints
#ifndef USBD_MAX_ENDPOINT_NUM
    #define USBD_MAX_ENDPOINT_NUM           (USBD_MAX_EP)
#endif

// Maximum packet size for Endpoint 0
#ifndef USBD_EP0_MAX_PACKET_SIZE
    #define USBD_EP0_MAX_PACKET_SIZE        (64)
#endif

// Maximum RAM for endpoint buffers
#ifndef USBD_BUF_SIZE
    #define USBD_BUF_SIZE                   (1536)
#endif

// Macros
// Macro for porting compatibility
#define USBD_t          USBD_T
#define USBD_EP_t       USBD_EP_T

#define PERIPH_SETUP_BUF_BASE  0
#define PERIPH_SETUP_BUF_LEN   8
#define PERIPH_EP0_BUF_BASE    (PERIPH_SETUP_BUF_BASE + PERIPH_SETUP_BUF_LEN)
#define PERIPH_EP0_BUF_LEN     USBD_EP0_MAX_PACKET_SIZE
#define PERIPH_EP1_BUF_BASE    (PERIPH_EP0_BUF_BASE + PERIPH_EP0_BUF_LEN)
#define PERIPH_EP1_BUF_LEN     USBD_EP0_MAX_PACKET_SIZE
#define PERIPH_EP2_BUF_BASE    (PERIPH_EP1_BUF_BASE + PERIPH_EP1_BUF_LEN)

#define USBD_HW(port) ((USBD_t *)g_usbdev_bus[port].reg_base)

typedef enum
{
    PERIPH_EP0 = 0,
    PERIPH_EP1 = 1,
    PERIPH_EP2 = 2,
    PERIPH_EP3 = 3,
    PERIPH_EP4 = 4,
    PERIPH_EP5 = 5,
    PERIPH_EP6 = 6,
    PERIPH_EP7 = 7,
    PERIPH_EP8 = 8,
    PERIPH_EP9 = 9,
    PERIPH_EP10 = 10,
    PERIPH_EP11 = 11,
    PERIPH_EP12 = 12,
    PERIPH_EP13 = 13,
    PERIPH_EP14 = 14,
    PERIPH_EP15 = 15,
    PERIPH_EP16 = 16,
    PERIPH_EP17 = 17,
    PERIPH_EP18 = 18,
    PERIPH_EP19 = 19,
    PERIPH_EP20 = 20,
    PERIPH_EP21 = 21,
    PERIPH_EP22 = 22,
    PERIPH_EP23 = 23,
    PERIPH_EP24 = 24,
    PERIPH_EP25 = 25,
    PERIPH_EP26 = 26,
    PERIPH_EP27 = 27,
    PERIPH_EP28 = 28,
    PERIPH_EP29 = 29,
    PERIPH_EP30 = 30,
    PERIPH_EP31 = 31,
    PERIPH_MAX_EP = USBD_MAX_ENDPOINT_NUM,
} EP_Num_t;

// Endpoint information
typedef struct
{
    uint16_t ep_mps;                        // Endpoint max packet size
    uint8_t ep_type;                        // Endpoint type 
    uint8_t ep_stalled;                     // Endpoint stall flag 
    uint8_t *volatile xfer_buf;             // Pointer to buf
    volatile uint32_t xfer_len;             // Number of bytes to transfer
    volatile uint32_t actual_xfer_len;      // Number of totally transferred bytes
    volatile uint32_t xfer_transferring;    // Number of transferred bytes in last transfer
    uint8_t ep_addr;                        // Endpoint address
} usb_dc_ep_state;

/* Driver state */
typedef struct
{
    uint8_t dev_addr;
    volatile uint8_t usbd_set_address_flag;
    uint32_t bufseg_addr;                     // Allocated USB RAM
    volatile uint8_t setup_packet[8];         // Setup Packet data
    usb_dc_ep_state ep_state[PERIPH_MAX_EP];  // Endpoint parameters
} usb_dc_config_priv;

static usb_dc_config_priv g_nuvoton_udc[1];
// Auxiliary functions
static USBD_EP_t *USBD_EndpointEntry(uint8_t busid, uint8_t ep_addr, bool add)
{
    EP_Num_t ep_index;
    usb_dc_ep_state *ep_state;
    USBD_t *husbd = USBD_HW(busid);
    USBD_EP_t *periph_ep;

    for (ep_index = PERIPH_EP0, ep_state = &g_nuvoton_udc->ep_state[PERIPH_EP0], periph_ep = husbd->EP;
         ep_index < PERIPH_MAX_EP;
         ep_index++, ep_state++, periph_ep++)
    {
        if (add)
        {
            if (0 == (periph_ep->CFG & USBD_CFG_STATE_Msk)) return periph_ep;
        }
        else
        {
            if (ep_state->ep_addr == ep_addr) return periph_ep;
        }
    }

    return NULL;
}

static void USBD_EndpointConfigureBuffer(uint8_t busid)
{
    // USB RAM beyond what we've allocated above is available to the user(Control endpoint and setup package used.)
    g_nuvoton_udc->bufseg_addr = PERIPH_EP2_BUF_BASE;

    // Reconfigures the buffer segmentation for all enabled endpoints.
    EP_Num_t ep_index;
    USBD_EP_t *periph_ep;

    for (ep_index = PERIPH_EP2, periph_ep = &USBD_HW(busid)->EP[PERIPH_EP2]; ep_index < PERIPH_MAX_EP; ep_index++, periph_ep++)
    {
        if (0 == (periph_ep->CFG & USBD_CFG_STATE_Msk)) continue;

        // Update the Endpoint Buffer Segmentation
        periph_ep->BUFSEG = g_nuvoton_udc->bufseg_addr;
        g_nuvoton_udc->bufseg_addr += g_nuvoton_udc->ep_state[ep_index].ep_mps;
    }
}

//__WEAK void usb_dc_low_level_init(void)
//{
//}

//__WEAK void usb_dc_low_level_deinit(void)
//{
//}

int usb_fs_dc_init(uint8_t busid)
{
    USBD_t *husbd = USBD_HW(busid);
    memset(&g_nuvoton_udc, 0, sizeof(g_nuvoton_udc));

//    usb_dc_low_level_init();

    /*****************************************************/
    // Initial USB engine
    USBD->ATTR = 0x7D0ul;

    /*****************************************************/

    // Clear USB-related interrupts before enable interrupt
    husbd->INTSTS = (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);
    // Enable USB-related interrupts.
    husbd->INTEN |= (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);
    // Disable software-disconnect function
    husbd->SE0 &= ~USBD_DRVSE0;
    /*****************************************************/
    NVIC_EnableIRQ(USBD_IRQn);
    return 0;
}

int usb_fs_dc_deinit(uint8_t busid)
{
    USBD_t *husbd = USBD_HW(busid);
    NVIC_DisableIRQ(USBD_IRQn);
    USBD->ATTR = 0x00000040;
    // Enable software-disconnect function.
    husbd->SE0 |= USBD_DRVSE0;
    return 0;
}

int usbd_fs_set_address(uint8_t busid, const uint8_t addr)
{
    USBD_t *husbd = USBD_HW(busid);
    uint8_t usbd_addr = husbd->FADDR;

    if ((usbd_addr == 0) && (usbd_addr != addr))
    {
        g_nuvoton_udc->dev_addr = addr;
        g_nuvoton_udc->usbd_set_address_flag = 1;
    }

    return 0;
}

int usbd_fs_set_remote_wakeup(uint8_t busid)
{
    (void) busid;
    return -1;
}

uint8_t usbd_fs_get_port_speed(uint8_t busid)
{
    (void) busid;
    return USB_SPEED_FULL;
}

int usbd_fs_ep_close(uint8_t busid, const uint8_t ep)
{
    USBD_t *husbd = USBD_HW(busid);
    USBD_EP_t *periph_ep = USBD_EndpointEntry(busid, ep, false);

    if (periph_ep != NULL)
    {
        usb_dc_ep_state *ep_state = &g_nuvoton_udc->ep_state[periph_ep - husbd->EP];
        // Clear Endpoint information
        memset((void *)ep_state, 0, sizeof(usb_dc_ep_state));
        // Clear Endpoint configure
        periph_ep->CFG = 0U;
    }

    return 0;
}

int usbd_fs_ep_open(uint8_t busid, const struct usb_endpoint_descriptor *ep)
{
    uint32_t ep_dir_mask, ep_type_mask;
    uint8_t ep_num = USB_EP_GET_IDX(ep->bEndpointAddress);
    uint8_t ep_dir = USB_EP_DIR_IS_IN(ep->bEndpointAddress);
    USBD_t *husbd = USBD_HW(busid);

    // Unconfigure Endpoint the endpoint if it has been used
    usbd_fs_ep_close(busid, ep->bEndpointAddress);

    // Open periph endpoint
    USBD_EP_t *periph_ep = USBD_EndpointEntry(busid, ep->bEndpointAddress, true);

    // Error if all periph endpoints used
    if (periph_ep == NULL)
    {
        return -1;
    }

    usb_dc_ep_state *ep_state = &g_nuvoton_udc->ep_state[periph_ep - husbd->EP];

    // Error if USB buffer is insufficient
    if (g_nuvoton_udc->bufseg_addr + USB_GET_MAXPACKETSIZE(ep->wMaxPacketSize) > USBD_BUF_SIZE)
    {
        return -2;
    }

    // Store max packet size information
    ep_state->ep_mps = USB_GET_MAXPACKETSIZE(ep->wMaxPacketSize);

    // configured endpoint
    if (ep_num != 0U)
    {
        ep_dir_mask = ep_dir ? USBD_CFG_EPMODE_IN : USBD_CFG_EPMODE_OUT;

        if (USB_GET_ENDPOINT_TYPE(ep->bmAttributes) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
            ep_type_mask = USBD_CFG_TYPE_ISO;
        else
            ep_type_mask = 0;

        periph_ep->CFG = (ep_dir_mask | ep_type_mask | ep_num);
        ep_state->ep_addr = ep->bEndpointAddress;
    }
    else
    {
        // Control endpoint
        if (ep_dir)
        {
            husbd->EP[PERIPH_EP0].CFG = USBD_CFG_CSTALL_Msk | USBD_CFG_EPMODE_IN;
            g_nuvoton_udc->ep_state[PERIPH_EP0].ep_addr = 0x80;
        }
        else
        {
            husbd->EP[PERIPH_EP1].CFG = USBD_CFG_CSTALL_Msk | USBD_CFG_EPMODE_OUT;
            g_nuvoton_udc->ep_state[PERIPH_EP1].ep_addr = 0x00;
        }
    }

    // Reconfigures the buffer segmentation for all enabled endpoints(must configure EP before configure buffer segmentation)
    USBD_EndpointConfigureBuffer(busid);
    return 0;
}

int usbd_fs_ep_set_stall(uint8_t busid, const uint8_t ep)
{
    USBD_EP_t *periph_ep = USBD_EndpointEntry(busid, ep, false);
    periph_ep->CFGP |= USBD_CFGP_SSTALL_Msk;
    return 0;
}

int usbd_fs_ep_clear_stall(uint8_t busid, const uint8_t ep)
{
    USBD_EP_t *periph_ep = USBD_EndpointEntry(busid, ep, false);
    periph_ep->CFGP &= ~USBD_CFGP_SSTALL_Msk;
    periph_ep->CFG &= ~USBD_CFG_DSQSYNC_Msk;
    return 0;
}

int usbd_fs_ep_is_stalled(uint8_t busid, const uint8_t ep, uint8_t *stalled)
{
    USBD_EP_t *periph_ep = USBD_EndpointEntry(busid, ep, false);
    *stalled = (periph_ep->CFGP & USBD_CFGP_SSTALL_Msk) > 0 ? 1 : 0;
    return 0;
}

int usbd_fs_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len)
{
    if (!data && data_len)
    {
        return -1;
    }

    USBD_EP_t *periph_ep = USBD_EndpointEntry(busid, ep, false);

    if (periph_ep == NULL)
    {
        return -2;
    }

    USBD_t *husbd = USBD_HW(busid);
    usb_dc_ep_state *ep_state = &g_nuvoton_udc->ep_state[periph_ep - husbd->EP];

    ep_state->xfer_buf = (uint8_t *)data;
    ep_state->xfer_len = data_len;
    ep_state->actual_xfer_len = 0;

    ep_state->xfer_transferring = MIN(ep_state->ep_mps, data_len);

    if (ep == 0x80)
    {
        // Set EP0 IN token PID to DATA1
        periph_ep->CFG |= USBD_CFG_DSQSYNC_Msk;
    }

    // Prepare the data for next IN transfer
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + periph_ep->BUFSEG), (uint8_t *)ep_state->xfer_buf, ep_state->xfer_transferring);
    // Trigger
    periph_ep->MXPLD = ep_state->xfer_transferring;

    return 0;
}

int usbd_fs_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t data_len)
{
    if (!data && data_len)
    {
        return -1;
    }

    USBD_EP_t *periph_ep = USBD_EndpointEntry(busid, ep, false);

    if (periph_ep == NULL)
    {
        return -2;
    }

    USBD_t *husbd = USBD_HW(busid);
    usb_dc_ep_state *ep_state = &g_nuvoton_udc->ep_state[periph_ep - husbd->EP];

    ep_state->xfer_buf = (uint8_t *)data;
    ep_state->xfer_len = data_len;
    ep_state->actual_xfer_len = 0;

    // Trigger to get next OUT transfer
    periph_ep->MXPLD = ep_state->ep_mps;

    return 0;
}

// Event functions *****************************************************************
static void USBD_BusReset(uint8_t busid)
{
    USBD_t *husbd = USBD_HW(busid);
    // Clear Endpoints information
    memset((void *)g_nuvoton_udc->ep_state, 0U, PERIPH_MAX_EP * sizeof(usb_dc_ep_state));

    for (EP_Num_t ep_index = PERIPH_EP0; ep_index < PERIPH_MAX_EP; ep_index++)
    {
        husbd->EP[ep_index].CFG = 0U;
    }

    husbd->FADDR = 0U;
    g_nuvoton_udc->dev_addr = 0;

    // Buffer for setup packet
    husbd->STBUFSEG = PERIPH_SETUP_BUF_BASE;
    // Buffer for control endpoint, share same buffer.
    husbd->EP[PERIPH_EP0].BUFSEG = PERIPH_EP0_BUF_BASE;
    husbd->EP[PERIPH_EP1].BUFSEG = PERIPH_EP1_BUF_BASE;

    // USB RAM beyond what we've allocated above is available to the user(Control endpoint and setup package used.)
    g_nuvoton_udc->bufseg_addr = PERIPH_EP2_BUF_BASE;
}

NVT_ITCM void USBDn_IRQHandler(uint8_t busid)
{
    USBD_t *husbd = USBD_HW(busid);
    volatile uint32_t u32IntSts = husbd->INTSTS;
    volatile uint32_t u32EpIntSts = husbd->EPINTSTS;
    volatile uint32_t u32BusSts = husbd->ATTR & 0x300f;

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        husbd->INTSTS = USBD_INTSTS_FLDET;

        if (husbd->VBUSDET & USBD_VBUSDET_VBUSDET_Msk)
        {
            // USB Plug In
            husbd->ATTR |= 0x7D0U;
            usbd_event_connect_handler(busid);
        }
        else
        {
            // USB Un-plug
            husbd->ATTR &= ~USBD_USB_EN;
            usbd_event_disconnect_handler(busid);
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_WAKEUP)
    {
        // Clear event flag
        husbd->INTSTS = USBD_INTSTS_WAKEUP;
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        // Clear event flag
        husbd->INTSTS = USBD_INTSTS_BUS;

        if (u32BusSts & USBD_STATE_USBRST)
        {
            // Bus reset
            USBD_BusReset(busid);
            // Enable USB and enable PHY
            husbd->ATTR |= 0x7D0U;

            usbd_event_reset_handler(busid);
        }

        if (u32BusSts & USBD_STATE_SUSPEND)
        {
            /* Enable USB but disable PHY */
            husbd->ATTR &= ~USBD_PHY_EN;
            usbd_event_suspend_handler(busid);
        }

        if (u32BusSts & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            husbd->ATTR |= 0x7D0U;
            usbd_event_resume_handler(busid);
        }
    }

    //------------------------------------------------------------------

    if (u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            /* Clear event flag */
            husbd->INTSTS = USBD_INTSTS_SETUP;
            /* Clear the data IN/OUT ready flag of control end-points */
            husbd->EP[PERIPH_EP0].CFGP |= USBD_CFGP_CLRRDY_Msk;
            husbd->EP[PERIPH_EP1].CFGP |= USBD_CFGP_CLRRDY_Msk;

            USBD_MemCopy((uint8_t *)g_nuvoton_udc->setup_packet, (uint8_t *)USBD_BUF_BASE, 8U);
            usbd_event_ep0_setup_complete_handler(busid, (uint8_t *)g_nuvoton_udc->setup_packet);
        }

        // EP event
        if (u32EpIntSts)
        {
            EP_Num_t ep_index;
            uint32_t mask;
            USBD_EP_t *periph_ep;
            usb_dc_ep_state *ep_state;

            for (ep_index = PERIPH_EP0, mask = USBD_EPINTSTS_EPEVT0_Msk, periph_ep = &husbd->EP[PERIPH_EP0], ep_state = &g_nuvoton_udc->ep_state[PERIPH_EP0]; 
                 ep_index < PERIPH_MAX_EP; 
                 ep_index++, mask <<= 1U, periph_ep++, ep_state++)
            {
                if (u32EpIntSts & mask)
                {
                    // Clear event flag
                    husbd->EPINTSTS = mask;
                    uint8_t const ep_addr = g_nuvoton_udc->ep_state[ep_index].ep_addr;

                    if (USB_EP_DIR_IS_IN(ep_addr))
                    {
                        // Update transferred number
                        ep_state->actual_xfer_len += ep_state->xfer_transferring;

                        if (ep_state->actual_xfer_len == ep_state->xfer_len)
                        {

                            if (g_nuvoton_udc->usbd_set_address_flag == 1)
                            {
                                husbd->FADDR = g_nuvoton_udc->dev_addr;
                                g_nuvoton_udc->usbd_set_address_flag = 0;
                            }

                            // If all data was transferred
                            usbd_event_ep_in_complete_handler(busid, ep_addr, ep_state->actual_xfer_len);
                        }
                        else
                        {
                            // If there is more data to transfer
                            uint8_t *data_to_transfer = ep_state->xfer_buf + ep_state->actual_xfer_len;
                            uint32_t num_to_transfer  = MIN(ep_state->xfer_len - ep_state->actual_xfer_len, ep_state->ep_mps);

                            ep_state->xfer_transferring = num_to_transfer;

                            // Prepare the data for next IN transfer
                            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + periph_ep->BUFSEG), (uint8_t *)data_to_transfer, num_to_transfer);
                            // Trigger
                            periph_ep->MXPLD = num_to_transfer;
                        }
                    }
                    else
                    {
                        uint32_t num_transferred = periph_ep->MXPLD;

                        USBD_MemCopy((uint8_t *)ep_state->xfer_buf + ep_state->actual_xfer_len, (uint8_t *)(USBD_BUF_BASE + periph_ep->BUFSEG), num_transferred);
                        ep_state->actual_xfer_len += num_transferred;

                        if ((num_transferred < ep_state->ep_mps) || (ep_state->actual_xfer_len == ep_state->xfer_len))
                        {
                            // If all data was transferred
                            usbd_event_ep_out_complete_handler(busid, ep_addr, ep_state->actual_xfer_len);
                        }
                        else
                        {
                            // Trigger next OUT transfer if data remains
                            periph_ep->MXPLD = ep_state->ep_mps;
                        }
                    }
                }
            }
        }
    }
}

NVT_ITCM void USBD_IRQHandler(uint8_t busid)
{
    if (USBD_HW(busid) == USBD)
    {
        USBDn_IRQHandler(0);
    }
}


