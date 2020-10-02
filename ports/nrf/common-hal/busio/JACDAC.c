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
static uint32_t logidx = 0;

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

busio_jacdac_obj_t* jd_instance = NULL;


static void set_timer_for_ticks(busio_jacdac_obj_t* self);
static void set_timer(busio_jacdac_obj_t* self, int delta, cb_t cb);


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
// TODO remove debug

static void led1On(void) {
    nrf_gpio_cfg_output(pin_P1_02.number);
    nrf_gpio_pin_write(pin_P1_02.number, 1);
}

static void led1Off(void) {
    nrf_gpio_cfg_output(pin_P1_02.number);
    nrf_gpio_pin_write(pin_P1_02.number, 0);
}

static void led1Toggle(void) {
    nrf_gpio_cfg_output(pin_P1_02.number);
    nrf_gpio_pin_toggle(pin_P1_02.number);
}

static void led2On(void) {
    nrf_gpio_cfg_output(pin_P1_05.number);
    nrf_gpio_pin_write(pin_P1_05.number, 1);
}

static void led2Off(void) {
    nrf_gpio_cfg_output(pin_P1_05.number);
    nrf_gpio_pin_write(pin_P1_05.number, 0);
}

static void led2Toggle(void) {
    nrf_gpio_cfg_output(pin_P1_05.number);
    nrf_gpio_pin_toggle(pin_P1_05.number);
}

static void led3On(void) {
    nrf_gpio_cfg_output(pin_P1_06.number);
    nrf_gpio_pin_write(pin_P1_06.number, 1);
}

static void led3Off(void) {
    nrf_gpio_cfg_output(pin_P1_06.number);
    nrf_gpio_pin_write(pin_P1_06.number, 0);
}

static void led3Toggle(void) {
    nrf_gpio_cfg_output(pin_P1_06.number);
    nrf_gpio_pin_toggle(pin_P1_06.number);
}

static void led4On(void) {
    nrf_gpio_cfg_output(pin_P1_07.number);
    nrf_gpio_pin_write(pin_P1_07.number, 1);
}

static void led4Off(void) {
    nrf_gpio_cfg_output(pin_P1_07.number);
    nrf_gpio_pin_write(pin_P1_07.number, 0);
}

static void led4Toggle(void) {
    nrf_gpio_cfg_output(pin_P1_07.number);
    nrf_gpio_pin_toggle(pin_P1_07.number);
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


static int8_t irq_disabled;

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
    target_disable_irq();

     // TODO debug remove
    common_hal_mcu_delay_us(10000);
    led1On();
    led2On();
    led3On();
    led4On();
    // TODO debug remove

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

void move_to_rx_queue(busio_jacdac_obj_t* self, jd_frame_t* f) {
    int i;

    for (i = 0; i < JD_RX_SIZE; i++)
        if (self->rx_queue[i] == NULL)
        {
            self->rx_queue[i] = f;
            break;
        }

    if (i == JD_RX_SIZE)
        target_panic();
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


/*************************************************************************************
*   Receiving
*/
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
    log_char('L');
    if (is_status(self, RX_ACTIVE))
        target_panic();

    disable_gpio_interrupts(self);
    set_status(self, RX_ACTIVE);

    // TODO set timer for ticks.
    gpio_get(self);
    while(nrf_gpio_pin_read(self->pin) == 0);

    set_pin_rx(self);

    led4Toggle(); // TODO debug remove
    nrfx_uarte_rx(self->uarte, (uint8_t*) self->rx_buffer, sizeof(jd_frame_t));

    set_timer(self, 200, setup_rx_timeout);

    target_enable_irq();
}

static void rx_done(busio_jacdac_obj_t *self) {
    led4Toggle(); // TODO debug remove

    log_char('P');

    disable_rx_interrupts(self);

    if (!is_status(self, RX_ACTIVE))
        target_panic();

    set_pin_gpio(self);
    clr_status(self, RX_ACTIVE);

    gpio_get(self);
    while(nrf_gpio_pin_read(self->pin) == 0);


    // check size
    uint32_t txSize = sizeof(*self->rx_buffer);
    uint32_t declaredSize = JD_FRAME_SIZE(self->rx_buffer);
    if (txSize < declaredSize) {
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
        target_panic();
        enable_gpio_interrupts(self);
        return;
    }

    jd_frame_t* rx = self->rx_buffer;
    self->rx_buffer = buffer_from_pool(self);

    move_to_rx_queue(self, rx);

    stop_rx(self);
    // restart normal operation
    enable_gpio_interrupts(self);
}


/*************************************************************************************
*   JACDAC - transmitting
*/

static void tx_done(busio_jacdac_obj_t *self) {
    disable_tx_interrupts(self);

     if (!is_status(self, TX_ACTIVE))
        target_panic();

    set_pin_gpio(self);

    // end pulse (11-15µs)
    gpio_set(self, 0);
    common_hal_mcu_delay_us(11);
    gpio_set(self, 1);

    uart_configure_tx(self, 0);

    // restart idle operation
    clr_status(self, TX_ACTIVE);

    enable_gpio_interrupts(self);
}


/*************************************************************************************
*   Timers
*/
static void tick(busio_jacdac_obj_t* self) {
    led1Toggle(); // TODO debug remove
    set_timer_for_ticks(self);
}

static void set_timer(busio_jacdac_obj_t* self, int delta, cb_t cb) {
    target_disable_irq();

    self->tim_cb = cb;

    uint32_t t = nrfx_timer_us_to_ticks(self->timer, delta);

    nrfx_timer_clear(self->timer);
    nrfx_timer_compare(self->timer, NRF_TIMER_CC_CHANNEL0, t, true);

    target_enable_irq();
}

static void set_timer_for_ticks(busio_jacdac_obj_t* self) {
    target_disable_irq();

    if (!is_status(self, RX_ACTIVE)) {
        set_timer(self, 10000, tick);
    }

    target_enable_irq();
}


/*************************************************************************************
*   Interrupt handlers
*/


// interrupt handler for UART
static void uart_irq(const nrfx_uarte_event_t* event, void* context) {
    busio_jacdac_obj_t* self = (busio_jacdac_obj_t*) context;

    switch ( event->type ) {
        case NRFX_UARTE_EVT_RX_DONE:
            led4Toggle(); // TODO debug remove
            // rx_done(self);
            break;

        case NRFX_UARTE_EVT_TX_DONE:
            led3Toggle(); // TODO debug remove
            tx_done(self);
            break;

        case NRFX_UARTE_EVT_ERROR:
            // Possible Error source is Overrun, Parity, Framing, Break
            if ((event->data.error.error_mask & NRF_UARTE_ERROR_BREAK_MASK) && is_status(self, RX_ACTIVE))
                rx_done(self);
            else
                led2Toggle(); // TODO debug remove
            break;
     }
}

// interrupt handler for timers
static void timer_irq(nrf_timer_event_t event_type, void* context) {
    log_char('o');
    busio_jacdac_obj_t* self = (busio_jacdac_obj_t*) context;

    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        cb_t f = self->tim_cb;
        if (f != NULL) {
            self->tim_cb = NULL;
            f(self);
        }
    }
}

// interrupt handler for GPIO
static void gpiote_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    if (jd_instance && pin == jd_instance->pin && !is_status(jd_instance, TX_ACTIVE))
        start_rx(jd_instance);
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
    log_char('T');
    if (self->timer_refcount == 0) {
        self->timer = nrf_peripherals_allocate_timer();
        if (self->timer == NULL) {
            target_panic();
            mp_raise_RuntimeError(translate("All timers in use"));
        }

        nrfx_timer_config_t timer_config = {
            .frequency = NRF_TIMER_FREQ_16MHz,
            .mode = NRF_TIMER_MODE_TIMER,
            .bit_width = NRF_TIMER_BIT_WIDTH_32,
            .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
            .p_context = self
        };

        nrfx_timer_init(self->timer, &timer_config, &timer_irq);
        // nrfx_timer_enable(self->timer);
    }
    self->timer_refcount++;

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

    // if (jd_instance) {
    //     self = jd_instance;
    //     return;
    // }

    log_char('A');

    jd_instance = self;

    self->pin = pin->number;
    self->status = 0;
    self->tim_cb = NULL;

    for (int i = 0; i < JD_POOL_SIZE; i++)
        self->buffer_pool[i] = m_malloc(sizeof(jd_frame_t), true);

    for (int i = 0; i < JD_RX_SIZE; i++)
        self->rx_queue[i] = NULL;

    self->rx_buffer = buffer_from_pool(self);
    self->tx_buffer = m_malloc(sizeof(jd_frame_t), true);

    // allocate_peripheral(self);

    // TODO debug remove
    led1Toggle();
    led2Toggle();
    led3Toggle();
    led4Toggle();
    led1Off();
    led2Off();
    led3Off();
    led4Off();
    // TODO debug remove

    claim_pin(pin);
    initialize_timer(self);
    initialize_gpio(self);
    initialize_uart(self);

    set_timer_for_ticks(self);

    log_char('B');
}


void common_hal_busio_jacdac_deinit(busio_jacdac_obj_t* self) {

    log_char('D');
    // free_peripheral(self);

    if (common_hal_busio_jacdac_deinited(self))
        return;

    nrfx_gpiote_in_event_disable(self->pin);
    nrfx_gpiote_in_uninit(self->pin);

    // timer
    self->timer_refcount--;
    if (self->timer_refcount == 0) {
        nrf_peripherals_free_timer(self->timer);
    }

    // uart
    if (self->uarte != NULL)
        nrfx_uarte_uninit(self->uarte);

    // pin
    reset_pin_number(self->pin);
    self->pin = NO_PIN;
}

bool common_hal_busio_jacdac_deinited(busio_jacdac_obj_t *self) {
    return self->pin == NO_PIN;
}

int common_hal_busio_jacdac_send(busio_jacdac_obj_t *self, const uint8_t *data, size_t len) {
    if ((self->status & TX_ACTIVE) || (self->status & RX_ACTIVE))
        target_panic();

    log_char('C');

    // try to pull the line low, provided it currently reads as high
    if (gpio_get(self) == 0) {
        clr_status(self, TX_ACTIVE);
        return -1;
    }

    disable_gpio_interrupts(self);
    gpio_set(self, 0);
    // start pulse (11-15µs)
    common_hal_mcu_delay_us(9);
    set_status(self, TX_ACTIVE);

    // start-data gap (40-89µs)
    gpio_set(self, 1);
    *(uint16_t*)data = jd_crc16((uint8_t *)data + 2, len - 2);
    common_hal_mcu_delay_us(19);
    log_char((char)len);
    memcpy(self->tx_buffer, data, MIN(len, 254));

    // setup UART tx
    set_pin_tx(self);
    nrfx_uarte_tx(self->uarte, (uint8_t*) self->tx_buffer, MIN(len, 254));
    return 0;
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
    log_char('R');
    // reset gpiote
    // if ( nrfx_gpiote_is_init() ) {
    //     nrfx_gpiote_uninit();
    // }
    // nrfx_gpiote_init(NRFX_GPIOTE_CONFIG_IRQ_PRIORITY);


    // // free all peripherals
    // for (uint8_t i = 0; i < JD_PERIPHERALS; i++) {

    //     // reset timers
    //     if (peripherals[i]->timer != NULL)
    //         nrf_peripherals_free_timer(peripherals[i]->timer);
    //     peripherals[i]->timer_refcount = 0;


    //     peripherals[i] = NULL;
    // }

    // // reset uart
    // for (size_t i = 0 ; i < MP_ARRAY_SIZE(nrfx_uartes); i++)
    //     nrfx_uarte_uninit(&nrfx_uartes[i]);
}