/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 J Devine, M Lambrichts
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MICROPY_INCLUDED_NRF_COMMON_HAL_BUSIO_JACDAC_H
#define MICROPY_INCLUDED_NRF_COMMON_HAL_BUSIO_JACDAC_H

#include "common-hal/microcontroller/Pin.h"
#include "py/obj.h"

#include "nrfx_uarte.h"
#include "nrf/timers.h"

#define JD_POOL_SIZE 20
#define JD_RX_SIZE 10
#define JD_TX_SIZE 10


// 255 minus size of the serial header, rounded down to 4
#define JD_SERIAL_PAYLOAD_SIZE 236
#define JD_MAX_FRAME_SIZE 252


// structure for a jacdac frame
struct _jd_frame_t {
    uint16_t crc;
    uint8_t size;
    uint8_t flags;

    uint64_t device_identifier;

    uint8_t data[JD_SERIAL_PAYLOAD_SIZE + 4];
} __attribute__((__packed__, aligned(4)));
typedef struct _jd_frame_t jd_frame_t;


typedef struct busio_jacdac_obj {
    mp_obj_base_t base;
    uint8_t pin;
    uint16_t status;

    nrfx_uarte_t* uarte;
    nrfx_timer_t* timer;

    void (*tim_cb)(struct busio_jacdac_obj*);

    jd_frame_t* buffer_pool[JD_POOL_SIZE];
    jd_frame_t* rx_queue[JD_RX_SIZE];
    jd_frame_t* tx_queue[JD_TX_SIZE];

    jd_frame_t* rx_buffer;
    jd_frame_t* tx_buffer;
} busio_jacdac_obj_t;

typedef void (*cb_t)(busio_jacdac_obj_t*);

/*
typedef struct {
    uint32_t bus_state;
    uint32_t bus_lo_error;
    uint32_t bus_uart_error;
    uint32_t bus_timeout_error;
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_dropped;
} jd_diagnostics_t;
jd_diagnostics_t *jd_get_diagnostics(void);
*/

void jacdac_reset(void);


#endif // MICROPY_INCLUDED_NRF_COMMON_HAL_BUSIO_JACDAC_H
