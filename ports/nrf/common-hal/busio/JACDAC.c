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

#define JD_LOG_SIZE     512
volatile uint32_t jd_log[JD_LOG_SIZE] = {0};
uint32_t logidx = 0;

static inline void log_char(char c) {
    jd_log[logidx] = c;
    logidx = (logidx + 1) % JD_LOG_SIZE;
}


#define JD_FRAME_SIZE(pkt) ((pkt)->size + 12)
#define JD_PERIPHERALS 2


// states
#define ACTIVE 0x03
#define RX_ACTIVE 0x01
#define TX_ACTIVE 0x02

#define TX_CONFIGURED 0x04
#define RX_CONFIGURED 0x08

#define TX_PENDING 0x10

#define JD_INST_ARRAY_SIZE  4
busio_jacdac_obj_t* jd_instances[JD_INST_ARRAY_SIZE] = { NULL };

static void tim_set_timer(busio_jacdac_obj_t* self, int delta, cb_t cb);
static void initial_rx_timeout(busio_jacdac_obj_t* self);
void tx_start(busio_jacdac_obj_t *self);

static inline void cfg_dbg_pins(void){
    nrf_gpio_cfg_output(4); // P0
    nrf_gpio_cfg_output(5); // P1
    nrf_gpio_cfg_output(3); // P2
}

static inline void set_P0(int val) {
    nrf_gpio_pin_write(4, val);
}

static inline void set_P1(int val) {
    nrf_gpio_pin_write(5, val);
}

static inline void set_P2(int val) {
    nrf_gpio_pin_write(3, val);
}

static uint32_t seed;

static void seed_random(uint32_t s) {
    seed = (seed * 0x1000193) ^ s;
}

static uint32_t get_random(void) {
    if (seed == 0)
        seed_random(13 + *((uint32_t*)0x20032e20));

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

/*************************************************************************************
*   Configuration
*/


// static busio_jacdac_obj_t* peripherals[JD_PERIPHERALS];
// static void allocate_peripheral(busio_jacdac_obj_t* peripheral) {
//     for (uint8_t i = 0; i < JD_PERIPHERALS; i++) {
//         if (peripherals[i] == NULL) {
//             peripherals[i] = peripheral;
//             return;
//         }
//     }

//     mp_raise_RuntimeError(translate("All JACDAC peripherals in use"));
// }

// static void free_peripheral(busio_jacdac_obj_t* peripheral) {
//     for (uint8_t i = 0; i < JD_PERIPHERALS; i++) {
//         if (peripherals[i] != NULL && peripherals[i]->pin == peripheral->pin) {
//             peripherals[i] = NULL;
//             return;
//         }
//     }
// }

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


// static int8_t irq_disabled;

// static void target_enable_irq(void) {
//     irq_disabled--;
//     if (irq_disabled <= 0) {
//         irq_disabled = 0;
//         __enable_irq();
//     }
// // }

// static void target_disable_irq(void) {
//     __disable_irq();
//     irq_disabled++;
// }

static void target_panic(void) {
    log_char('P');
    while (1) {}
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

//static jd_diagnostics_t jd_diagnostics;
//jd_diagnostics_t *jd_get_diagnostics(void) {
//    jd_diagnostics.bus_state = 0;
//    return &jd_diagnostics;
//}


/*************************************************************************************
*   GPIO helper
*/


static void gpio_set(busio_jacdac_obj_t* self, uint32_t value) {
    nrf_gpio_cfg_output(self->pin);
    nrf_gpio_pin_write(self->pin, value);
}

static uint32_t gpio_get(busio_jacdac_obj_t* self) {
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


jd_frame_t* buffer_from_pool(busio_jacdac_obj_t* self) {
    jd_frame_t* ret = NULL;

    __disable_irq();
    for (int i = 0; i < JD_POOL_SIZE; i++)
        if (self->buffer_pool[i])
        {
            ret = self->buffer_pool[i];
            self->buffer_pool[i] = NULL;
            break;
        }
    __enable_irq();

    return ret;
}

int move_to_rx_queue(busio_jacdac_obj_t* self, jd_frame_t* f) {
    int i;

    for (i = 0; i < JD_RX_SIZE; i++)
        if (self->rx_queue[i] == NULL)
        {
            self->rx_queue[i] = f;
            break;
        }

    if (i == JD_RX_SIZE)
        return -1;

    return 0;
}

int move_to_tx_queue(busio_jacdac_obj_t* self, jd_frame_t* f) {
    int i;

    for (i = 0; i < JD_TX_SIZE; i++)
        if (self->tx_queue[i] == NULL)
        {
            self->tx_queue[i] = f;
            break;
        }

    if (i == JD_TX_SIZE)
        return -1;

    return 1;
}

void return_buffer_to_pool(busio_jacdac_obj_t* self, jd_frame_t* buf) {
    __disable_irq();
    for (int i = 0; i < JD_POOL_SIZE; i++)
        if (self->buffer_pool[i] == NULL)
        {
            self->buffer_pool[i] = buf;
            break;
        }
    __enable_irq();
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
    self->status &= ~status;
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
        self->uarte->p_reg->INTENSET = (UARTE_INTENSET_ENDTX_Msk);
        self->uarte->p_reg->ENABLE = 8;
        while(!(self->uarte->p_reg->ENABLE));
        set_status(self, TX_CONFIGURED);
    } else if (!enable && is_status(self, TX_CONFIGURED)) {
        self->uarte->p_reg->TASKS_STOPTX = 1;
        while(self->uarte->p_reg->TASKS_STOPTX);
        self->uarte->p_reg->ENABLE = 0;
        while((self->uarte->p_reg->ENABLE));
        self->uarte->p_reg->INTENCLR = (UARTE_INTENCLR_ENDTX_Msk);
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
        self->uarte->p_reg->INTENSET = (UARTE_INTENSET_ENDRX_Msk | UARTE_INTENSET_ERROR_Msk);
        self->uarte->p_reg->ENABLE = 8;
        while(!(self->uarte->p_reg->ENABLE));
        set_status(self, RX_CONFIGURED);
    } else if (!enable && is_status(self, RX_CONFIGURED)) {
        self->uarte->p_reg->TASKS_STOPRX = 1;
        while(self->uarte->p_reg->TASKS_STOPRX);
        self->uarte->p_reg->ENABLE = 0;
        while((self->uarte->p_reg->ENABLE));
        self->uarte->p_reg->INTENCLR = (UARTE_INTENCLR_ENDRX_Msk | UARTE_INTENCLR_ERROR_Msk);
        self->uarte->p_reg->PSEL.RXD = 0xFFFFFFFF;
        clr_status(self, RX_CONFIGURED);
    }
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
}

// set the JACDAC pin to act as the UART rx pin
static inline void set_pin_rx(busio_jacdac_obj_t* self) {
    if (is_status(self, RX_CONFIGURED))
        return;

    uart_configure_tx(self, 0);
    uart_configure_rx(self, 1);
}

// set the JACDAC pin to act as a gpio pin
static inline void set_pin_gpio(busio_jacdac_obj_t* self) {
    uart_configure_tx(self, 0);
    uart_configure_rx(self, 0);
}


/*************************************************************************************
*   Receiving
*/
static void stop_uart_dma(busio_jacdac_obj_t* self) {
    nrfx_uarte_tx_abort(self->uarte);
    nrfx_uarte_rx_abort(self->uarte);
}

static void rx_timeout(busio_jacdac_obj_t* self) {
    log_char('?');
    set_P1(0);

    // if (!is_status(self, RX_ACTIVE))
    //     target_panic();

    // disable uart
    stop_uart_dma(self);

    set_pin_gpio(self);
    clr_status(self, RX_ACTIVE);
    // restart normal operation
    enable_gpio_interrupts(self);
}

static void rx_start(busio_jacdac_obj_t* self) {
    log_char('r');
    if (is_status(self, RX_ACTIVE))
        target_panic();

    set_P1(1);

    disable_gpio_interrupts(self);
    set_status(self, RX_ACTIVE);

    // TODO set timer for ticks.
    gpio_get(self);
    tim_set_timer(self, 1000, rx_timeout);
    while(nrf_gpio_pin_read(self->pin) == 0);

    set_pin_rx(self);

    // if we don't receive these bytes (CRC) after 200 us assume an error
    uint8_t* b = (uint8_t *)self->rx_buffer;
    b[0] = 0;
    b[1] = 0;
    b[2] = 0;
    b[3] = 0;

    nrfx_uarte_rx(self->uarte, (uint8_t*) self->rx_buffer, sizeof(jd_frame_t));
    tim_set_timer(self, 200, initial_rx_timeout);
}

static void rx_done(busio_jacdac_obj_t *self) {
    set_P1(0);

    log_char('R');
    // clear any upcoming timer interrupts
    nrfx_timer_capture(self->timer, NRF_TIMER_CC_CHANNEL0);

    if (!is_status(self, RX_ACTIVE))
        target_panic();

    set_pin_gpio(self);
    clr_status(self, RX_ACTIVE);

    gpio_get(self);
    while(nrf_gpio_pin_read(self->pin) == 0);


    // check size
    uint32_t txSize = sizeof(*self->rx_buffer);
    uint32_t declaredSize = JD_FRAME_SIZE(self->rx_buffer);
    if (txSize < declaredSize || declaredSize == 0) {
        //jd_diagnostics.bus_uart_error++;
        log_char('V');
        enable_gpio_interrupts(self);
        return;
    }

    // check crc
    uint16_t crc = jd_crc16((uint8_t *)self->rx_buffer + 2, declaredSize - 2);
    if (crc != self->rx_buffer->crc) {
        //.bus_uart_error++;
        log_char('M');
        // target_panic();
        enable_gpio_interrupts(self);
        return;
    }

    jd_frame_t* rx = self->rx_buffer;
    self->rx_buffer = buffer_from_pool(self);

    int ret = move_to_rx_queue(self, rx);

    // drop but ensure memory is not left floating around...
    if (ret == -1)
        return_buffer_to_pool(self, rx);

    stop_uart_dma(self);
    // restart normal operation
    enable_gpio_interrupts(self);

    if (is_status(self, TX_PENDING))
        tim_set_timer(self, random_around(150), tx_start);
}


/*************************************************************************************
*   JACDAC - transmitting
*/
void tx_start(busio_jacdac_obj_t *self) {
    if ((self->status & TX_ACTIVE) || (self->status & RX_ACTIVE))
        target_panic();

    log_char('T');

    if (self->tx_buffer == NULL)
    {
        __disable_irq();
        for (int i = 0; i < JD_RX_SIZE; i++)
            if (self->tx_queue[i])
            {
                self->tx_buffer = self->tx_queue[i];
                self->tx_queue[i] = NULL;
                break;
            }
        __enable_irq();

        if (self->tx_buffer == NULL)
        {
            clr_status(self, TX_PENDING);
            return;
        }
    }

    // try to pull the line low, provided it currently reads as high
    if (gpio_get(self) == 0) {
        set_P0(0);
        set_status(self, TX_PENDING);
        return;
    }

    disable_gpio_interrupts(self);

    set_P0(1);
    log_char(';');
    gpio_set(self, 0);
    // start pulse (11-15µs)
    common_hal_mcu_delay_us(10);
    clr_status(self, TX_PENDING);
    set_status(self, TX_ACTIVE);

    // start-data gap (40-89µs)
    gpio_set(self, 1);
    uint16_t* data = (uint16_t*)self->tx_buffer;
    *data = jd_crc16((uint8_t *)self->tx_buffer + 2, JD_FRAME_SIZE(self->tx_buffer) - 2);
    common_hal_mcu_delay_us(19);

    // setup UART tx
    set_pin_tx(self);
    nrfx_uarte_tx(self->uarte, (uint8_t*) self->tx_buffer, JD_FRAME_SIZE(self->tx_buffer));
}

static void tx_done(busio_jacdac_obj_t *self) {
    set_P0(0);
    log_char('t');

     if (!is_status(self, TX_ACTIVE))
        target_panic();

    set_pin_gpio(self);

    // end pulse (11-15µs)
    gpio_set(self, 0);
    common_hal_mcu_delay_us(10);
    return_buffer_to_pool(self, self->tx_buffer);
    self->tx_buffer = NULL;
    gpio_set(self, 1);

    uart_configure_tx(self, 0);

    // restart idle operation
    clr_status(self, TX_ACTIVE);

    __disable_irq();
    bool more = false;
    for (int i = 0; i < JD_RX_SIZE; i++)
        if (self->tx_queue[i])
        {
            more = true;
            break;
        }
    __enable_irq();

    if (more)
    {
        set_status(self, TX_PENDING);
        tim_set_timer(self, random_around(150), tx_start);
    }

    enable_gpio_interrupts(self);
}

static void tim_set_timer(busio_jacdac_obj_t* self, int delta, cb_t cb) {
    self->tim_cb = cb;
    uint32_t now = nrfx_timer_capture(self->timer, NRF_TIMER_CC_CHANNEL1);
    nrfx_timer_compare(self->timer, NRF_TIMER_CC_CHANNEL0, now + delta, true);
}

/*************************************************************************************
*   Interrupt handlers
*/


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
            log_char('E');
            log_char((char)event->data.error.error_mask);
            // Possible Error source is Overrun, Parity, Framing, Break
            if (is_status(self, RX_ACTIVE))
            {
                log_char('&');
                if ((event->data.error.error_mask & NRF_UARTE_ERROR_BREAK_MASK))
                    rx_done(self);
            }
            else if (is_status(self, TX_ACTIVE))
            {
                log_char('*');
            }

            break;
     }
}

static void initial_rx_timeout(busio_jacdac_obj_t* self)
{
    log_char('N');
    if (is_status(self, RX_ACTIVE)) {
        uint8_t* b = (uint8_t *)self->rx_buffer;

        if (b[0] == 0 && b[1] == 0)
        {
            log_char('Y');
            rx_timeout(self);
            return;
        }

        log_char('I');
        tim_set_timer(self, JD_FRAME_SIZE(self->rx_buffer) * 12 + 60, rx_timeout);
    }
}

// interrupt handler for timers
static void timer_irq(nrf_timer_event_t event_type, void* context) {
    log_char('i');
    set_P2(1);
    busio_jacdac_obj_t* self = (busio_jacdac_obj_t*) context;

    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        if (self->tim_cb)
            self->tim_cb(self);
    }
    set_P2(0);
}

// interrupt handler for GPIO
static void gpiote_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {

    busio_jacdac_obj_t* self = NULL;

    for (int i = 0; i < JD_INST_ARRAY_SIZE; i++)
        if (jd_instances[i]->pin == pin)
        {
            self = jd_instances[i];
            break;
        }

    if (self && !is_status(self, TX_ACTIVE))
        rx_start(self);
}


/*************************************************************************************
*   Initialization
*/

static void initialize_gpio(busio_jacdac_obj_t *self) {

    nrfx_gpiote_in_config_t cfg = {
        .sense = NRF_GPIOTE_POLARITY_HITOLO,
        .pull = NRF_GPIO_PIN_PULLUP, // idle_state ? NRF_GPIO_PIN_PULLDOWN : NRF_GPIO_PIN_PULLUP,
        .is_watcher = false, // nrf_gpio_cfg_watcher vs nrf_gpio_cfg_input
        .hi_accuracy = true,
        .skip_gpio_setup = false
    };

    nrfx_gpiote_init(0);
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
        .interrupt_priority = 2,
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
    log_char('A');

    bool found = false;
    for (int i = 0; i < JD_INST_ARRAY_SIZE; i++)
        if (jd_instances[i] == self)
        {
            found = true;
            break;
        }

    if (!found)
    {
        int i;
        for (i = 0; i < JD_INST_ARRAY_SIZE; i++)
            if (jd_instances[i] == NULL)
            {
                jd_instances[i] = self;
                break;
            }

        if (i == JD_INST_ARRAY_SIZE)
            target_panic();
    }

    self->pin = pin->number;
    self->status = 0;
    self->tim_cb = NULL;

    for (int i = 0; i < JD_POOL_SIZE; i++)
        self->buffer_pool[i] = m_malloc(sizeof(jd_frame_t), true);

    for (int i = 0; i < JD_RX_SIZE; i++)
        self->rx_queue[i] = NULL;

    for (int i = 0; i < JD_TX_SIZE; i++)
        self->tx_queue[i] = NULL;

    self->rx_buffer = buffer_from_pool(self);
    self->tx_buffer = NULL;

    cfg_dbg_pins();

    claim_pin(pin);
    initialize_timer(self);
    initialize_gpio(self);
    initialize_uart(self);

    log_char('B');
}


void common_hal_busio_jacdac_deinit(busio_jacdac_obj_t* self) {
    if (common_hal_busio_jacdac_deinited(self))
        return;

    nrfx_gpiote_in_event_disable(self->pin);
    nrfx_gpiote_in_uninit(self->pin);

    nrf_peripherals_free_timer(self->timer);

    // uart
    if (self->uarte)
    {
        nrfx_uarte_tx_abort(self->uarte);
        nrfx_uarte_rx_abort(self->uarte);
        nrfx_uarte_uninit(self->uarte);
    }

    // pin
    reset_pin_number(self->pin);

    for (int i = 0; i < JD_POOL_SIZE; i++)
    {
        if (self->buffer_pool[i])
        {
            m_free(self->buffer_pool[i]);
            self->buffer_pool[i] = NULL;
        }
    }

    for (int i = 0; i < JD_RX_SIZE; i++)
    {
        if (self->rx_queue[i])
        {
            m_free(self->rx_queue[i]);
            self->rx_queue[i] = NULL;
        }
    }

    for (int i = 0; i < JD_TX_SIZE; i++)
    {
        if (self->tx_queue[i])
        {
            m_free(self->tx_queue[i]);
            self->tx_queue[i] = NULL;
        }
    }

    if (self->rx_buffer)
    {
        m_free(self->rx_buffer);
        self->rx_buffer = NULL;
    }

    if (self->tx_buffer)
    {
        m_free(self->tx_buffer);
        self->tx_buffer = NULL;
    }

    self->pin = NO_PIN;
}

bool common_hal_busio_jacdac_deinited(busio_jacdac_obj_t *self) {
    return self->pin == NO_PIN;
}

int common_hal_busio_jacdac_send(busio_jacdac_obj_t *self, const uint8_t *data, size_t len) {
    jd_frame_t* f = buffer_from_pool(self);

    if (f == NULL)
        return -1;

    memcpy(f, data, MIN(len, JD_MAX_FRAME_SIZE));

    int ret = move_to_tx_queue(self, f);

    if (ret == -1)
    {
        return_buffer_to_pool(self, f);
        return -1;
    }

    if (!is_status(self, TX_PENDING | TX_ACTIVE | RX_ACTIVE))
    {
        set_status(self, TX_PENDING);
        tim_set_timer(self, 100, tx_start);
    }

    return 1;
}

int common_hal_busio_jacdac_receive(busio_jacdac_obj_t *self, uint8_t *data, size_t len) {

    jd_frame_t *f = NULL;

    __disable_irq();
    for (int i = 0; i < JD_RX_SIZE; i++)
        if (self->rx_queue[i])
        {
            f = self->rx_queue[i];
            self->rx_queue[i] = NULL;
            break;
        }
    __enable_irq();

    if (f)
    {
        memcpy(data, f, sizeof(jd_frame_t));
        return_buffer_to_pool(self, f);
        return 1;
    }

    return 0;
}

void jacdac_reset(void) {
    log_char('}');

    for (int i = 0; i < JD_INST_ARRAY_SIZE; i++)
    {
        if (jd_instances[i])
        {
            common_hal_busio_jacdac_deinit(jd_instances[i]);
            jd_instances[i] = NULL;
        }
    }
}