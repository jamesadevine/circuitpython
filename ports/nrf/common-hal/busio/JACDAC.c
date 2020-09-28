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

#include "shared-bindings/microcontroller/__init__.h"
#include "shared-bindings/busio/JACDAC.h"

#include "lib/utils/interrupt_char.h"
#include "py/mpconfig.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "py/runtime.h"
#include "supervisor/shared/translate.h"

#include "nrfx_config.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"


#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define JD_FRAME_SIZE(pkt) ((pkt)->size + 12)
#define JD_PERIPHERALS 2

#define JD_LOG_SIZE     512
volatile uint32_t jd_log[JD_LOG_SIZE] = {0};
static uint32_t logidx = 0;

void log_char(char c) {
    jd_log[logidx] = c;
    logidx = (logidx + 1) % JD_LOG_SIZE;
}


// states
#define ACTIVE 0x03
#define RX_ACTIVE 0x01
#define TX_ACTIVE 0x02

#define TX_CONFIGURED 0x04
#define RX_CONFIGURED 0x08

#define TX_PENDING 0x10


static void set_timer_for_ticks(busio_jacdac_obj_t* self);
static void set_timer(busio_jacdac_obj_t* self, int delta, cb_t cb);


/*************************************************************************************
*   Configuration
*/


busio_jacdac_obj_t* jd_instance = NULL;

static nrfx_uarte_t nrfx_uartes[] = {
#if NRFX_CHECK(NRFX_UARTE0_ENABLED)
    NRFX_UARTE_INSTANCE(0),
#endif
#if NRFX_CHECK(NRFX_UARTE1_ENABLED)
    NRFX_UARTE_INSTANCE(1),
#endif
};


/*************************************************************************************
*   Helper methods
*/
static int8_t irq_disabled;
static uint32_t seed;

static void target_enable_irq(void) {
    irq_disabled--;
    if (irq_disabled <= 0) {
        irq_disabled = 0;
        __enable_irq();
    }
}

static void target_disable_irq(void) {
    __disable_irq();
    irq_disabled++;
}

static void target_panic(void) {
    mp_raise_RuntimeError(translate("JACDAC error"));
}

static void seed_random(uint32_t s) {
    seed = (seed * 0x1000193) ^ s;
}

static uint32_t get_random(void) {
    if (seed == 0)
        seed_random(13);

    // xorshift algorithm
    uint32_t x = seed;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    seed = x;
    return x;
}

// return v +/- 25% or so
static uint32_t random_around(uint32_t v) {
    uint32_t mask = 0xfffffff;
    while (mask > v)
        mask >>= 1;
    return (v - (mask >> 1)) + (get_random() & mask);
}

static uint16_t jd_crc16(const void *data, uint32_t size) {
    const uint8_t *ptr = (const uint8_t *)data;
    uint16_t crc = 0xffff;
    while (size--) {
        uint8_t d = *ptr++;
        uint8_t x = (crc >> 8) ^ d;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }
    return crc;
}



/*************************************************************************************
*   GPIO helper
*/


static void gpio_set(busio_jacdac_obj_t* self, uint32_t value) {
    if (nrf_gpio_pin_dir_get(self->pin) != NRF_GPIO_PIN_DIR_OUTPUT)
        nrf_gpio_cfg_output(self->pin);
    nrf_gpio_pin_write(self->pin, value);
}

static uint32_t gpio_get(busio_jacdac_obj_t* self) {
    if (nrf_gpio_pin_dir_get(self->pin) != NRF_GPIO_PIN_DIR_INPUT)
        nrf_gpio_cfg_input(self->pin, NRF_GPIO_PIN_PULLUP);
    return nrf_gpio_pin_read(self->pin);
}

static inline bool gpio_is_output(busio_jacdac_obj_t* self) {
    return nrf_gpio_pin_dir_get(self->pin) == NRF_GPIO_PIN_DIR_OUTPUT;
}

static inline void enable_gpio_interrupts(busio_jacdac_obj_t* self) {
    nrf_gpio_cfg_input(self->pin, NRF_GPIO_PIN_PULLUP);
    nrfx_gpiote_in_event_enable(self->pin, true);
}

static inline void disable_gpio_interrupts(busio_jacdac_obj_t* self) {
    nrfx_gpiote_in_event_disable(self->pin);
}


/*************************************************************************************
*   Status helper
*/


// set the given status
static inline void set_status(busio_jacdac_obj_t* self, uint16_t status) {
    self->status |= status;
}

// clear the given status
static inline void clr_status(busio_jacdac_obj_t* self, uint16_t status) {
    self->status &= ~(status);
}

// check if the given status is set or not
static inline bool is_status(busio_jacdac_obj_t* self, uint16_t status) {
    return self->status & status;
}


/*************************************************************************************
*   UART configuration
*/


static void uart_configure_tx(busio_jacdac_obj_t* self, int enable) {
    if (enable && !is_status(self, TX_CONFIGURED)) {
        NRF_P0->DIR |= (1 << self->pin);
        NRF_P0->PIN_CNF[self->pin] =  3 << 2; // this overrides DIR setting above
        self->uarte->p_reg->PSEL.TXD = self->pin;
        self->uarte->p_reg->EVENTS_ENDTX = 0;
        self->uarte->p_reg->ENABLE = 8;
        while(!(self->uarte->p_reg->ENABLE));
        set_status(self, TX_CONFIGURED);
    } else if (!enable && is_status(self, TX_CONFIGURED)) {
        self->uarte->p_reg->TASKS_STOPTX = 1;
        while(self->uarte->p_reg->TASKS_STOPTX);
        self->uarte->p_reg->ENABLE = 0;
        while((self->uarte->p_reg->ENABLE));
        self->uarte->p_reg->PSEL.TXD = 0xFFFFFFFF;
        clr_status(self, TX_CONFIGURED);
    }
}

static void uart_configure_rx(busio_jacdac_obj_t* self, int enable) {
    if (enable && !is_status(self, RX_CONFIGURED)) {
        NRF_P0->DIR &= ~(1 << self->pin);
        NRF_P0->PIN_CNF[self->pin] =  3 << 2; // this overrides DIR setting above
        self->uarte->p_reg->PSEL.RXD = self->pin;
        self->uarte->p_reg->EVENTS_ENDRX = 0;
        self->uarte->p_reg->EVENTS_ERROR = 0;
        self->uarte->p_reg->ERRORSRC = self->uarte->p_reg->ERRORSRC;
        self->uarte->p_reg->ENABLE = 8;
        while(!(self->uarte->p_reg->ENABLE));
        set_status(self, RX_CONFIGURED);
    } else if (!enable && is_status(self, RX_CONFIGURED)) {
        self->uarte->p_reg->TASKS_STOPRX = 1;
        while(self->uarte->p_reg->TASKS_STOPRX);
        self->uarte->p_reg->ENABLE = 0;
        while((self->uarte->p_reg->ENABLE));
        self->uarte->p_reg->PSEL.RXD = 0xFFFFFFFF;
        clr_status(self, RX_CONFIGURED);
    }
}

static inline void enable_rx_interrupts(busio_jacdac_obj_t* self) {
    self->uarte->p_reg->INTENSET = (UARTE_INTENSET_ENDRX_Msk | UARTE_INTENSET_ERROR_Msk);
}

static inline void disable_rx_interrupts(busio_jacdac_obj_t* self) {
    self->uarte->p_reg->INTENCLR = (UARTE_INTENCLR_ENDRX_Msk | UARTE_INTENSET_ERROR_Msk);
}

static inline void enable_tx_interrupts(busio_jacdac_obj_t* self) {
    self->uarte->p_reg->INTENSET = (UARTE_INTENSET_ENDTX_Msk);
}

static inline void disable_tx_interrupts(busio_jacdac_obj_t* self) {
    self->uarte->p_reg->INTENCLR = (UARTE_INTENCLR_ENDTX_Msk);
}


/*************************************************************************************
*   Pin configuration
*/


// set the JACDAC pin to act as the UART tx pin
static inline void set_pin_tx(busio_jacdac_obj_t* self) {
    if (is_status(self, TX_CONFIGURED))
        return;

    uart_configure_rx(self, 0);
    uart_configure_tx(self, 1);
    enable_tx_interrupts(self);
}

// set the JACDAC pin to act as the UART rx pin
static inline void set_pin_rx(busio_jacdac_obj_t* self) {
    if (is_status(self, RX_CONFIGURED))
        return;

    uart_configure_tx(self, 0);
    uart_configure_rx(self, 1);
    enable_rx_interrupts(self);
}

// set the JACDAC pin to act as a gpio pin
static inline void set_pin_gpio(busio_jacdac_obj_t* self) {
    uart_configure_tx(self, 0);
    uart_configure_rx(self, 0);
}

static void stop_rx(busio_jacdac_obj_t* self) {
    disable_tx_interrupts(self);
    disable_rx_interrupts(self);

    nrfx_uarte_tx_abort(self->uarte);
    nrfx_uarte_rx_abort(self->uarte);
}

static void rx_timeout(busio_jacdac_obj_t* self) {
    target_disable_irq();
    //jd_diagnostics.bus_timeout_error++;

    // disable uart
    stop_rx(self);

    set_pin_gpio(self);
    clr_status(self, RX_ACTIVE);

    target_enable_irq();
}

static void setup_rx_timeout(busio_jacdac_obj_t* self) {
    uint32_t* data = (uint32_t*) self->rx_buffer;
    if (data[0] == 0)
        rx_timeout(self); // didn't get any data after lo-pulse

    // got the size - set timeout for whole packet
    set_timer(self, JD_FRAME_SIZE(self->rx_buffer) * 12 + 60, rx_timeout);
}

static void start_rx(busio_jacdac_obj_t* self) {
    if (is_status(self, RX_ACTIVE))
        target_panic();
    set_status(self, RX_ACTIVE);

    // reset buffer
    self->rx_buffer->crc = 0;
    self->rx_buffer->size = 0;
    self->rx_buffer->flags = 0;

    while(nrf_gpio_pin_read(self->pin) == 0);

    set_pin_rx(self);

    nrfx_uarte_rx(self->uarte, (uint8_t*) self->rx_buffer, sizeof(jd_frame_t));

    set_timer(self, 100, setup_rx_timeout);

    target_enable_irq();
}

static void rx_done(busio_jacdac_obj_t *self) {
    disable_rx_interrupts(self);

    if (!is_status(self, RX_ACTIVE))
        target_panic();

    set_pin_gpio(self);
    clr_status(self, RX_ACTIVE);
    set_timer_for_ticks(self);

    // check crc
    uint16_t crc = jd_crc16((uint8_t *)self->rx_buffer + 2, self->rx_buffer->size - 2);
    if (crc != self->rx_buffer->crc) {
        //.bus_uart_error++;
        return;
    }

    // save_and_create_new_frame(self);
    //jd_diagnostics.packets_received++;
    //if (!save_and_create_new_frame(self))
    //    jd_diagnostics.packets_dropped++;

    stop_rx(self);

    // restart normal operation
    enable_gpio_interrupts(self);
}


/*************************************************************************************
*   JACDAC - transmitting
*/

static void start_tx(busio_jacdac_obj_t* self) {
    log_char('S');
    clr_status(self, TX_PENDING);
    set_status(self, TX_ACTIVE);

    uint32_t val = gpio_get(self);
    // try to pull the line low, provided it currently reads as high
    if (val == 0) {
        clr_status(self, TX_ACTIVE);
        set_timer_for_ticks(self);
        set_status(self, TX_PENDING);
        return;
    }

    // start pulse (11-15µs)
    gpio_set(self, 0);
    common_hal_mcu_delay_us(10);

    // start-data gap (40-89µs)
    gpio_set(self, 1);
    common_hal_mcu_delay_us(37);

    // setup UART tx
    set_pin_tx(self);
    enable_tx_interrupts(self);
    nrfx_uarte_tx(self->uarte, (uint8_t*)self->tx_buffer, JD_FRAME_SIZE(self->tx_buffer));
}

static void tx_done(busio_jacdac_obj_t *self) {
    log_char('D');
    disable_tx_interrupts(self);

     if (!is_status(self, TX_ACTIVE))
        target_panic();

    set_pin_gpio(self);

    // end pulse (11-15µs)
    gpio_set(self, 0);
    common_hal_mcu_delay_us(10);
    gpio_set(self, 1);

    // restart idle operation
    clr_status(self, TX_ACTIVE);
    set_timer_for_ticks(self);

    enable_gpio_interrupts(self);
}


/*************************************************************************************
*   Timers
*/
static void set_timer(busio_jacdac_obj_t* self, int delta, cb_t cb) {
    self->tim_cb = cb;
    nrfx_timer_compare(self->timer, NRF_TIMER_CC_CHANNEL0, nrfx_timer_capture(self->timer, NRF_TIMER_CC_CHANNEL1) + delta, true);
}

static void set_timer_for_ticks(busio_jacdac_obj_t* self) {
    log_char('s');
    if (!is_status(self, RX_ACTIVE)) {

        int delta = 10000;
        log_char('R');
        cb_t cb = set_timer_for_ticks;

        if (is_status(self, TX_PENDING) && !is_status(self, TX_ACTIVE)){
            log_char('q');
            delta = random_around(150);
            cb = start_tx;
        }

        set_timer(self, delta, cb);
    }
}



/*************************************************************************************
*   Interrupt handlers
*/

// interrupt handler for timers
static void timer_irq(nrf_timer_event_t event_type, void* context) {
    busio_jacdac_obj_t* self = (busio_jacdac_obj_t*) context;
    log_char('i');
    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        cb_t f = self->tim_cb;
        if (f) {
            log_char('O');
            self->tim_cb = NULL;
            f(self);
        }
    }
}


// interrupt handler for UART
static void uart_irq(const nrfx_uarte_event_t* event, void* context) {
    busio_jacdac_obj_t* self = (busio_jacdac_obj_t*) context;

    switch ( event->type ) {
        case NRFX_UARTE_EVT_RX_DONE:
            rx_done(self);
            break;

        case NRFX_UARTE_EVT_TX_DONE:
            tx_done(self);
            break;

        case NRFX_UARTE_EVT_ERROR:
            // Possible Error source is Overrun, Parity, Framing, Break
            if ((event->data.error.error_mask & NRF_UARTE_ERROR_BREAK_MASK) && is_status(self, RX_ACTIVE))
                rx_done(self);
            break;
     }
}

// interrupt handler for GPIO
static void gpiote_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    disable_gpio_interrupts(jd_instance);
    start_rx(jd_instance);
}

static void initialize_gpio(busio_jacdac_obj_t *self) {
    nrfx_gpiote_in_config_t cfg = {
        .sense = NRF_GPIOTE_POLARITY_HITOLO,
        .pull = NRF_GPIO_PIN_PULLUP, // idle_state ? NRF_GPIO_PIN_PULLDOWN : NRF_GPIO_PIN_PULLUP,
        .is_watcher = false, // nrf_gpio_cfg_watcher vs nrf_gpio_cfg_input
        .hi_accuracy = true,
        .skip_gpio_setup = false
    };

    nrfx_gpiote_in_init(self->pin, &cfg, gpiote_callback);
    nrfx_gpiote_in_event_enable(self->pin, true);

    enable_gpio_interrupts(self);
}

static void initialize_timer(busio_jacdac_obj_t *self) {
    self->timer = nrf_peripherals_allocate_timer();
    if (self->timer == NULL) {
        target_panic();
        mp_raise_RuntimeError(translate("All timers in use"));
    }

    nrfx_timer_config_t timer_config = {
        .frequency = NRF_TIMER_FREQ_1MHz,
        .mode = NRF_TIMER_MODE_TIMER,
        .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = 1,
        .p_context = self
    };

    nrfx_timer_init(self->timer, &timer_config, &timer_irq);
    nrfx_timer_enable(self->timer);
}

static void initialize_uart(busio_jacdac_obj_t *self) {
    self->uarte = NULL;
    for (size_t i = 0; i < MP_ARRAY_SIZE(nrfx_uartes); i++) {
        if ((nrfx_uartes[i].p_reg->ENABLE & UARTE_ENABLE_ENABLE_Msk) == 0) {
            self->uarte = &nrfx_uartes[i];
            break;
        }
    }

    if (self->uarte == NULL)
        mp_raise_ValueError(translate("All UART peripherals are in use"));

    nrfx_uarte_config_t uart_config = {
        .pseltxd = NRF_UARTE_PSEL_DISCONNECTED,
        .pselrxd = NRF_UARTE_PSEL_DISCONNECTED,
        .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
        .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
        .p_context = self,
        .baudrate = NRF_UARTE_BAUDRATE_1000000,
        .interrupt_priority = 1,
        .hal_cfg = {
            .hwfc = NRF_UARTE_HWFC_DISABLED,
            .parity = NRF_UARTE_PARITY_EXCLUDED,
        }
    };

    nrfx_uarte_init(self->uarte, &uart_config, uart_irq);
}

/*************************************************************************************
*   Public JACDAC methods
*/
void common_hal_busio_jacdac_construct(busio_jacdac_obj_t* self, const mcu_pin_obj_t* pin) {

    if (jd_instance != NULL)
    {
        self = jd_instance;
        self->refcount++;
        return;
    }

    jd_instance = self;

    self->refcount = 1;
    self->pin = pin->number;
    self->status = 0;
    self->rx_buffer = gc_alloc(sizeof(jd_frame_t), false, true);
    self->tx_buffer = gc_alloc(sizeof(jd_frame_t), false, true);

    claim_pin(pin);
    // memset(self->rxArray, 0, sizeof(jd_frame_t*) * JD_RX_ARRAY_SIZE);
    // memset(self->txArray, 0, sizeof(jd_frame_t*) * JD_TX_ARRAY_SIZE);
    self->txTail = 0;
    self->txHead = 0;
    self->rxTail = 0;
    self->rxHead = 0;

    initialize_gpio(self);
    initialize_uart(self);
    initialize_timer(self);

    log_char('I');
}


void common_hal_busio_jacdac_deinit(busio_jacdac_obj_t* self) {
    if (common_hal_busio_jacdac_deinited(self))
        return;

    nrfx_gpiote_in_event_disable(self->pin);
    nrfx_gpiote_in_uninit(self->pin);

    self->refcount--;

    if (self->refcount == 0) {
        nrf_peripherals_free_timer(self->timer);
        nrfx_uarte_uninit(self->uarte);
        reset_pin_number(self->pin);
        self->pin = NO_PIN;
    }
}

bool common_hal_busio_jacdac_deinited(busio_jacdac_obj_t *self) {
    return self->pin == NO_PIN || jd_instance == NULL;
}

void common_hal_busio_jacdac_send(busio_jacdac_obj_t *self, const uint8_t *data, size_t len) {

    memcpy(self->tx_buffer, data, MIN(len, 254));

    // notify that a new frame is present
    set_status(self, TX_PENDING);
    if (!is_status(self, ACTIVE))
    {
        set_timer_for_ticks(self);
        log_char('p');
    }

    log_char('T');
}

void common_hal_busio_jacdac_receive(busio_jacdac_obj_t *self, uint8_t *data, size_t len) {
    // jd_frame_t* t = popRxArray(self);

    // if (t != NULL)
    //     memcpy(data, t, sizeof(*t));

    // gc_free(t);
}

void jacdac_reset(void) {
    if (!jd_instance)
        return;

    // reset gpiote
    if ( nrfx_gpiote_is_init() ) {
        nrfx_gpiote_uninit();
    }

    nrfx_gpiote_init(NRFX_GPIOTE_CONFIG_IRQ_PRIORITY);

    nrf_peripherals_free_timer(jd_instance->timer);

    // reset uart
    for (size_t i = 0 ; i < MP_ARRAY_SIZE(nrfx_uartes); i++)
        nrfx_uarte_uninit(jd_instance->uarte);

    jd_instance = NULL;
}