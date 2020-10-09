/*
 * This file is part of the Micro Python project, http://micropython.org/
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

#ifndef MICROPY_INCLUDED_SHARED_BINDINGS_BUSIO_JACDAC_H
#define MICROPY_INCLUDED_SHARED_BINDINGS_BUSIO_JACDAC_H

#include "common-hal/microcontroller/Pin.h"
#include "common-hal/busio/JACDAC.h"
#include "py/ringbuf.h"

extern const mp_obj_type_t busio_jacdac_type;


// Construct an underlying UART object.
extern void common_hal_busio_jacdac_construct(busio_jacdac_obj_t *context, const mcu_pin_obj_t * pin);

extern void common_hal_busio_jacdac_deinit(busio_jacdac_obj_t *context);
extern bool common_hal_busio_jacdac_deinited(busio_jacdac_obj_t *context);

extern int common_hal_busio_jacdac_send(busio_jacdac_obj_t *context, const uint8_t *data, size_t len);
extern int common_hal_busio_jacdac_receive(busio_jacdac_obj_t *context, uint8_t *data, size_t len);

#endif  // MICROPY_INCLUDED_SHARED_BINDINGS_BUSIO_JACDAC_H
