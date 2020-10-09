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

#include <stdint.h>

#include "shared-bindings/busio/JACDAC.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/util.h"

#include "lib/utils/buffer_helper.h"
#include "lib/utils/context_manager_helpers.h"
#include "lib/utils/interrupt_char.h"

#include "py/ioctl.h"
#include "py/objproperty.h"
#include "py/runtime.h"
#include "supervisor/shared/translate.h"


//| class JACDAC:
//|     """A bidirectional single wire serial protocol"""
//|     def __init__(self, pin: microcontroller.Pin) -> None:
//|         """A common bidirectional serial protocol that uses an an agreed upon speed
//|         rather than a shared clock line.
//|
//|         :param ~microcontroller.Pin pin: the pin to transmit with.
//|         ...
//|
STATIC mp_obj_t busio_jacdac_make_new(const mp_obj_type_t *type, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // Always initially allocate the UART (so also JACDAC) object within the long-lived heap.
    // This is needed to avoid crashes with certain UART implementations which
    // cannot accomodate being moved after creation. (See
    // https://github.com/adafruit/circuitpython/issues/1056)
    busio_jacdac_obj_t *self = m_new_ll_obj(busio_jacdac_obj_t);
    self->base.type = &busio_jacdac_type;

    enum { ARG_pin };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_pin, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const mcu_pin_obj_t* pin = validate_obj_is_free_pin_or_none(args[ARG_pin].u_obj);

    if ( (pin == NULL)) {
        mp_raise_ValueError(translate("pin can not be None"));
    }

    common_hal_busio_jacdac_construct(self, pin);
    return (mp_obj_t)self;
}

//|     def deinit(self) -> None:
//|         """Deinitialises JACDAC and releases any hardware resources for reuse."""
//|         ...
//|
STATIC mp_obj_t busio_jacdac_obj_deinit(mp_obj_t self_in) {
    busio_jacdac_obj_t *self = MP_OBJ_TO_PTR(self_in);
    common_hal_busio_jacdac_deinit(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(busio_jacdac_deinit_obj, busio_jacdac_obj_deinit);

STATIC void check_for_deinit(busio_jacdac_obj_t *self) {
    if (common_hal_busio_jacdac_deinited(self)) {
        raise_deinited_error();
    }
}

//|     def __enter__(self) -> JACDAC:
//|         """No-op used by Context Managers."""
//|         ...
//|
//  Provided by context manager helper.

//|     def __exit__(self) -> None:
//|         """Automatically deinitializes the hardware when exiting a context. See
//|         :ref:`lifetime-and-contextmanagers` for more info."""
//|         ...
//|
STATIC mp_obj_t busio_jacdac_obj___exit__(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    common_hal_busio_jacdac_deinit(args[0]);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(busio_jacdac___exit___obj, 4, 4, busio_jacdac_obj___exit__);





STATIC mp_obj_t busio_jacdac_send(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buffer, ARG_start, ARG_end };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_buffer,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_start,      MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_end,        MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = INT_MAX} },
    };
    busio_jacdac_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    check_for_deinit(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // setup buffer
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_READ);
    int32_t start = args[ARG_start].u_int;
    size_t length = bufinfo.len;
    normalize_buffer_bounds(&start, args[ARG_end].u_int, &length);

    // empty buffer
    if (length == 0) {
        return mp_obj_new_bool(0);
    }

    return mp_obj_new_bool(common_hal_busio_jacdac_send(self, ((uint8_t*)bufinfo.buf) + start, length));
}
MP_DEFINE_CONST_FUN_OBJ_KW(busio_jacdac_send_obj, 2, busio_jacdac_send);




STATIC mp_obj_t busio_jacdac_receive(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buffer, ARG_start, ARG_end };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_buffer,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_start,      MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_end,        MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = INT_MAX} },
    };
    busio_jacdac_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    check_for_deinit(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // setup buffer
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_WRITE);
    int32_t start = args[ARG_start].u_int;
    size_t length = bufinfo.len;
    normalize_buffer_bounds(&start, args[ARG_end].u_int, &length);

    // empty buffer
    if (length == 0) {
        return mp_obj_new_bool(0);
    }

    return mp_obj_new_bool(common_hal_busio_jacdac_receive(self, ((uint8_t*)bufinfo.buf) + start, length));
}
MP_DEFINE_CONST_FUN_OBJ_KW(busio_jacdac_receive_obj, 2, busio_jacdac_receive);

// https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
static uint32_t hash_fnv1(const void *data, unsigned len) {
    const uint8_t *d = (const uint8_t *)data;
    uint32_t h = 0x811c9dc5;
    while (len--)
        h = (h * 0x1000193) ^ *d++;
    return h;
}

static uint32_t jd_hash(uint8_t* buf, size_t length, int bits) {
    if (bits < 1)
        return 0;

    uint32_t h = hash_fnv1(buf, length);

    if (bits >= 32)
        return h;
    else
        return ((h ^ (h >> bits)) & ((1 << bits) - 1));
}


STATIC mp_obj_t busio_jacdac_hash(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buffer };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_buffer,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    busio_jacdac_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    check_for_deinit(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // setup buffer
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_READ);
    int32_t start = 0;
    size_t length = bufinfo.len;
    normalize_buffer_bounds(&start, INT_MAX, &length);

    // empty buffer
    if (length == 0) {
        return 0;
    }

    uint32_t h = jd_hash(((uint8_t*)bufinfo.buf), length, 30);

    vstr_t vstr;
    vstr_init_len(&vstr, 4);

    vstr.buf[0] = 0x41 + h % 26;
    vstr.buf[1] = 0x41 + (h / 26) % 26;
    vstr.buf[2] = 0x30 + (h / (26 * 26)) % 10;
    vstr.buf[3] = 0x30 + (h / (26 * 26 * 10)) % 10;

    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}
MP_DEFINE_CONST_FUN_OBJ_KW(busio_jacdac_hash_obj, 2, busio_jacdac_hash);


STATIC const mp_rom_map_elem_t busio_jacdac_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit),       MP_ROM_PTR(&busio_jacdac_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__),    MP_ROM_PTR(&default___enter___obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__),     MP_ROM_PTR(&busio_jacdac___exit___obj) },

    { MP_ROM_QSTR(MP_QSTR_send),     MP_ROM_PTR(&busio_jacdac_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_receive),  MP_ROM_PTR(&busio_jacdac_receive_obj) },
    { MP_ROM_QSTR(MP_QSTR_hash),  MP_ROM_PTR(&busio_jacdac_hash_obj) },
};
STATIC MP_DEFINE_CONST_DICT(busio_jacdac_locals_dict, busio_jacdac_locals_dict_table);

const mp_obj_type_t busio_jacdac_type = {
    { &mp_type_type },
    .name = MP_QSTR_JACDAC,
    .make_new = busio_jacdac_make_new,
    .locals_dict = (mp_obj_dict_t*)&busio_jacdac_locals_dict,
};
