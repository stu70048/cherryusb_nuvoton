#include "usbd_core.h"

extern int usb_fs_dc_init(uint8_t busid);
extern int usb_fs_dc_deinit(uint8_t busid);
extern int usbd_fs_set_address(uint8_t busid, const uint8_t addr);
extern int usbd_fs_set_remote_wakeup(uint8_t busid);
extern uint8_t usbd_fs_get_port_speed(uint8_t busid);
extern int usbd_fs_ep_open(uint8_t busid, const struct usb_endpoint_descriptor *ep);
extern int usbd_fs_ep_close(uint8_t busid, const uint8_t ep);
extern int usbd_fs_ep_set_stall(uint8_t busid, const uint8_t ep);
extern int usbd_fs_ep_clear_stall(uint8_t busid, const uint8_t ep);
extern int usbd_fs_ep_is_stalled(uint8_t busid, const uint8_t ep, uint8_t *stalled);
extern int usbd_fs_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len);
extern int usbd_fs_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t data_len);


extern int usb_hs_dc_init(uint8_t busid);
extern int usb_hs_dc_deinit(uint8_t busid);
extern int usbd_hs_set_address(uint8_t busid, const uint8_t addr);
extern int usbd_hs_set_remote_wakeup(uint8_t busid);
extern uint8_t usbd_hs_get_port_speed(uint8_t busid);
extern int usbd_hs_ep_open(uint8_t busid, const struct usb_endpoint_descriptor *ep);
extern int usbd_hs_ep_close(uint8_t busid, const uint8_t ep);
extern int usbd_hs_ep_set_stall(uint8_t busid, const uint8_t ep);
extern int usbd_hs_ep_clear_stall(uint8_t busid, const uint8_t ep);
extern int usbd_hs_ep_is_stalled(uint8_t busid, const uint8_t ep, uint8_t *stalled);
extern int usbd_hs_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len);
extern int usbd_hs_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t data_len);


__WEAK void usb_dc_low_level_init(void)
{
}

__WEAK void usb_dc_low_level_deinit(void)
{
}

int usb_dc_init(uint8_t busid)
{
    printf("usb_dc_init:%d\n", busid);

    if (busid == 0)
        return usb_fs_dc_init(busid);
    else if (busid == 1)
        return usb_hs_dc_init(busid);
    else
        return -1;
}

int usb_dc_deinit(uint8_t busid)
{
    if (busid == 0)
        return usb_fs_dc_deinit(busid);
    else if (busid == 1)
        return usb_hs_dc_deinit(busid);
    else
        return -1;
}

int usbd_set_address(uint8_t busid, const uint8_t addr)
{
    if (busid == 0)
        return usbd_fs_set_address(busid, addr);
    else if (busid == 1)
        return usbd_hs_set_address(busid, addr);
    else
        return 0;
}

int usbd_set_remote_wakeup(uint8_t busid)
{
    if (busid == 0)
        return usbd_fs_set_remote_wakeup(busid);
    else if (busid == 1)
        return usbd_hs_set_remote_wakeup(busid);
    else
        return -1;
}

uint8_t usbd_get_port_speed(uint8_t busid)
{
    if (busid == 0)
        return usbd_fs_get_port_speed(busid);
    else if (busid == 1)
        return usbd_hs_get_port_speed(busid);
    else
        return USB_SPEED_FULL;
}

int usbd_ep_open(uint8_t busid, const struct usb_endpoint_descriptor *ep)
{
    if (busid == 0)
        return usbd_fs_ep_open(busid, ep);
    else if (busid == 1)
        return usbd_hs_ep_open(busid, ep);
    else
        return 0;
}

int usbd_ep_close(uint8_t busid, const uint8_t ep)
{
    if (busid == 0)
        return usbd_fs_ep_close(busid, ep);
    else if (busid == 1)
        return usbd_hs_ep_close(busid, ep);
    else
        return 0;
}

int usbd_ep_set_stall(uint8_t busid, const uint8_t ep)
{
    if (busid == 0)
        return usbd_fs_ep_set_stall(busid, ep);
    else if (busid == 1)
        return usbd_hs_ep_set_stall(busid, ep);
    else
        return 0;
}

int usbd_ep_clear_stall(uint8_t busid, const uint8_t ep)
{
    if (busid == 0)
        return usbd_fs_ep_clear_stall(busid, ep);
    else if (busid == 1)
        return usbd_hs_ep_clear_stall(busid, ep);
    else
        return 0;
}

int usbd_ep_is_stalled(uint8_t busid, const uint8_t ep, uint8_t *stalled)
{
    if (busid == 0)
        return usbd_fs_ep_is_stalled(busid, ep, stalled);
    else if (busid == 1)
        return usbd_hs_ep_is_stalled(busid, ep, stalled);
    else
        return 0;
}

int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len)
{
    if (busid == 0)
        return usbd_fs_ep_start_write(busid, ep, data, data_len);
    else if (busid == 1)
        return usbd_hs_ep_start_write(busid, ep, data, data_len);
    else
        return 0;
}

int usbd_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t data_len)
{
    if (busid == 0)
        return usbd_fs_ep_start_read(busid, ep, data, data_len);
    else if (busid == 1)
        return usbd_hs_ep_start_read(busid, ep, data, data_len);
    else
        return 0;
}
