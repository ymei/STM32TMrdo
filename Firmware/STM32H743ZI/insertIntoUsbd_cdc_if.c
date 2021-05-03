/** \file insertIntoUsbd_cdc_if.c
 * Central hub for user-defined USB CDC handling functions.
 * To be #include "" into the STM32CubeMX generated usbd_cdc_if.c file.
 */
/*
 * Copyright (c)
 *
 *     Meson Dynamics
 *
 * All rights reserved.
 */

volatile uint8_t usb_recv_buf[APP_RX_DATA_SIZE+1] = {0};

void usb_recv(uint8_t* buf, uint32_t *len)
{
    uint32_t i;
    if(usb_recv_buf[0]) {
        /* Previous data not fully consumed. */
    }
    /* Reverse the data index order. */
    usb_recv_buf[*len +1] = 0;
    for(i=0; i<*len; i++)
        usb_recv_buf[i] = buf[*len-1 - i];
}
