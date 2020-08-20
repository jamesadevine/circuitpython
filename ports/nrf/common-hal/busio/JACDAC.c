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
#include "py/stream.h"
#include "supervisor/shared/translate.h"



#include "nrfx_config.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"



#include <string.h>
#include <stdio.h>
#include <stdlib.h>

busio_jacdac_obj_t* self;

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#define TX_CONFIGURED       ((uint16_t)0x02)
#define RX_CONFIGURED       ((uint16_t)0x04)
//#define FIRST_BREAK         ((uint16_t)0x08)


#define JD_FRAME_SIZE(pkt) ((pkt)->size + 12)


#define DEVICE_NO_RESOURCES -1005
#define DEVICE_COMPONENT_RUNNING 0x1000
#define DEVICE_OK 0


#define SWS_EVT_DATA_RECEIVED       1
#define SWS_EVT_DATA_SENT           2
#define SWS_EVT_ERROR               3
#define SWS_EVT_DATA_DROPPED        4


#define JD_RX_ARRAY_SIZE 10
#define JD_TX_ARRAY_SIZE 10

// buffers
static volatile uint8_t txHead;
static volatile uint8_t txTail;
static volatile uint8_t rxHead;
static volatile uint8_t rxTail;

static jd_frame_t* rxArray[JD_RX_ARRAY_SIZE];
static jd_frame_t* txArray[JD_TX_ARRAY_SIZE];

// timer
static nrfx_uarte_t* uarte;
static nrfx_timer_t* timer;
static volatile uint8_t timer_refcount = 0;


static void set_tick_timer(uint8_t statusClear);
static void tim_set_timer(int delta, cb_t cb);

static void jd_line_falling(void);


/****************************************************************************/

typedef enum {
    SINGLE_WIRE_RX = 0,
    SINGLE_WIRE_TX,
    SINGLE_WIRE_DISCONNECTED
} single_wire_mode;

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

/****************************************************************************/

// expression to examine, and return value in case of failing
#define _VERIFY_ERR(_exp) \
    do {\
      uint32_t _err = (_exp);\
      if (NRFX_SUCCESS != _err ) {\
        mp_raise_msg_varg(&mp_type_RuntimeError, translate("error = 0x%08lX"), _err);\
      }\
    }while(0)

static nrfx_uarte_t nrfx_uartes[] = {
#if NRFX_CHECK(NRFX_UARTE0_ENABLED)
    NRFX_UARTE_INSTANCE(0),
#endif
#if NRFX_CHECK(NRFX_UARTE1_ENABLED)
    NRFX_UARTE_INSTANCE(1),
#endif
};

/****************************************************************************/

static int configureTx(int enable) {
    if (enable && !(self->sws_status & TX_CONFIGURED)) {
        NRF_P0->DIR |= (1 << self->pin);
        NRF_P0->PIN_CNF[self->pin] =  3 << 2; // this overrides DIR setting above
        uarte->p_reg->PSEL.TXD = self->pin;
        uarte->p_reg->EVENTS_ENDTX = 0;
        uarte->p_reg->ENABLE = 8;
        while(!(uarte->p_reg->ENABLE));
        self->sws_status |= TX_CONFIGURED;
    } else if (self->sws_status & TX_CONFIGURED) {
        uarte->p_reg->TASKS_STOPTX = 1;
        while(uarte->p_reg->TASKS_STOPTX);
        uarte->p_reg->ENABLE = 0;
        while((uarte->p_reg->ENABLE));
        uarte->p_reg->PSEL.TXD = 0xFFFFFFFF;
        self->sws_status &= ~TX_CONFIGURED;
    }

    return DEVICE_OK;
}

static int configureRx(int enable) {
    if (enable && !(self->sws_status & RX_CONFIGURED)) {
        NRF_P0->DIR &= ~(1 << self->pin);
        NRF_P0->PIN_CNF[self->pin] =  3 << 2; // this overrides DIR setting above
        uarte->p_reg->PSEL.RXD = self->pin;
        uarte->p_reg->EVENTS_ENDRX = 0;
        uarte->p_reg->EVENTS_ERROR = 0;
        uarte->p_reg->ERRORSRC = uarte->p_reg->ERRORSRC;
        uarte->p_reg->ENABLE = 8;
        while(!(uarte->p_reg->ENABLE));
        self->sws_status |= RX_CONFIGURED;
    } else if (enable == 0 && self->sws_status & RX_CONFIGURED) {
        uarte->p_reg->TASKS_STOPRX = 1;
        while(uarte->p_reg->TASKS_STOPRX);
        uarte->p_reg->ENABLE = 0;
        while((uarte->p_reg->ENABLE));
        uarte->p_reg->PSEL.RXD = 0xFFFFFFFF;
        self->sws_status &= ~RX_CONFIGURED;
    }

    return DEVICE_OK;
}

static int sws_setMode(single_wire_mode mode) {
    if (mode == SINGLE_WIRE_RX) {
        configureTx(0);
        configureRx(1);
    } else if (mode == SINGLE_WIRE_TX) {
        configureRx(0);
        configureTx(1);
    } else {
        configureTx(0);
        configureRx(0);
    }
    return DEVICE_OK;
}

/****************************************************************************/

static void configureRxInterrupt(int enable) {
    if (enable) {
        led2On();
        uarte->p_reg->INTENSET = (UARTE_INTENSET_ENDRX_Msk | UARTE_INTENSET_ERROR_Msk);
    } else {
        led2Off();
        uarte->p_reg->INTENCLR = (UARTE_INTENCLR_ENDRX_Msk | UARTE_INTENSET_ERROR_Msk);
    }
}

static void configureTxInterrupt(int enable) {
    if (enable) {
        led4On();
        uarte->p_reg->INTENSET = (UARTE_INTENSET_ENDTX_Msk);
    } else {
        led4Off();
        uarte->p_reg->INTENCLR = (UARTE_INTENCLR_ENDTX_Msk);
    }
}


/****************************************************************************/

static int sws_sendDMA(const uint8_t* data, int len) {


    if (!(self->sws_status & TX_CONFIGURED))
        sws_setMode(SINGLE_WIRE_TX);

    configureTxInterrupt(1);

    // EasyDMA can only access SRAM
    uint8_t* tx_buf = (uint8_t*) data;
    if ( !nrfx_is_in_ram(data) ) {
        // TODO: If this is not too big, we could allocate it on the stack.
        tx_buf = (uint8_t *) gc_alloc(len, false, false);
        memcpy(tx_buf, data, len);
    }

    nrfx_uarte_tx(uarte, tx_buf, len);



    // Wait for write to complete.
    while ( nrfx_uarte_tx_in_progress(uarte) ) {
        RUN_BACKGROUND_TASKS;
    }


    if ( !nrfx_is_in_ram(data) ) {
        gc_free(tx_buf);
    }

/*
    uarte->p_reg->TXD.PTR = (uint32_t)data;
    uarte->p_reg->TXD.MAXCNT = len;

    configureTxInterrupt(1);

    uarte->p_reg->TASKS_STARTTX = 1;
*/

    return DEVICE_OK;
}

/****************************************************************************/

static int sws_receiveDMA(uint8_t* data, int len) {
    if (!(self->sws_status & RX_CONFIGURED))
        sws_setMode(SINGLE_WIRE_RX);

    //uarte->p_reg->RXD.PTR = (uint32_t)data;
    //uarte->p_reg->RXD.MAXCNT = len;

    configureRxInterrupt(1);

    //uarte->p_reg->TASKS_STARTRX = 1;

    _VERIFY_ERR(nrfx_uarte_rx(uarte, data, len));
    return DEVICE_OK;
}

/****************************************************************************/

static int sws_abortDMA(void) {
    configureTxInterrupt(0);
    configureRxInterrupt(0);

    nrfx_uarte_tx_abort(uarte);
    nrfx_uarte_rx_abort(uarte);

    return DEVICE_OK;
}

/****************************************************************************/




/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void gpio_set(uint32_t value) {
    if (nrf_gpio_pin_dir_get(self->pin) != NRF_GPIO_PIN_DIR_OUTPUT)
        nrf_gpio_cfg_output(self->pin);
    nrf_gpio_pin_write(self->pin, value);
}

static uint32_t gpio_get(void) {
    nrf_gpio_cfg_input(self->pin, NRF_GPIO_PIN_PULLUP);
    return nrf_gpio_pin_read(self->pin);
}

static uint32_t gpio_get_high_impedence(void) {
    nrf_gpio_cfg_input(self->pin, NRF_GPIO_PIN_NOPULL);
    return nrf_gpio_pin_read(self->pin);
}

static void gpio_enable_interrupts(void) {
    nrfx_gpiote_in_event_enable(self->pin, true);
}

static void gpio_disable_interrupts(void) {
    nrfx_gpiote_in_event_disable(self->pin);
}

static uint32_t gpio_get_and_set(uint32_t value) {
    // set output but connect input buffer
    nrf_gpio_cfg(
        self->pin,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_pin_write(self->pin, value);

    return nrf_gpio_pin_read(self->pin);
}

static bool gpio_isOutput(void) {
    return nrf_gpio_pin_dir_get(self->pin) == NRF_GPIO_PIN_DIR_OUTPUT;
}



/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static int8_t irq_disabled;

static void target_enable_irq(void) {
    irq_disabled--;
    if (irq_disabled <= 0) {
        irq_disabled = 0;
        __enable_irq();
    }
}

static void target_disable_irq(void) {
    //common_hal_mcu_disable_interrupts
    // always disable just in case - it's just one instruction
    __disable_irq();
    irq_disabled++;
    // this used to disable here, only if irq_disabled==1 - this was a race
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static uint32_t seed;
static void jd_seed_random(uint32_t s) {
    seed = (seed * 0x1000193) ^ s;
}

static uint32_t jd_random(void) {
    if (seed == 0)
        jd_seed_random(13);

    // xorshift algorithm
    uint32_t x = seed;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    seed = x;
    return x;
}

// return v +/- 25% or so
static uint32_t jd_random_around(uint32_t v) {
    uint32_t mask = 0xfffffff;
    while (mask > v)
        mask >>= 1;
    return (v - (mask >> 1)) + (jd_random() & mask);
}

static void jd_panic(void) {
    target_disable_irq();

    common_hal_mcu_delay_us(10000);

    //DMESG("*** CODAL PANIC : [%d]", statusCode);
    led1On();
    led2On();
    led3On();
    led4On();
    while (1) {}
}

// https://wiki.nicksoft.info/mcu:pic16:crc-16:home
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


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/






/***********************************************************/

// FIFO circular buffers.
static jd_frame_t* popRxArray(void) {
    // nothing to pop
    if (rxTail == rxHead)
        return NULL;

    target_disable_irq();
    uint8_t nextHead = (rxHead + 1) % JD_RX_ARRAY_SIZE;
    jd_frame_t* p = rxArray[rxHead];
    rxArray[rxHead] = NULL;
    rxHead = nextHead;
    target_enable_irq();

    return p;
}

// FIFO circular buffers.
static jd_frame_t* popTxArray(void) {
    // nothing to pop
    if (txTail == txHead)
        return NULL;


    target_disable_irq();
    uint8_t nextHead = (txHead + 1) % JD_TX_ARRAY_SIZE;
    jd_frame_t* p = txArray[txHead];
    txArray[txHead] = NULL;
    txHead = nextHead;
    target_enable_irq();

    return p;
}

// FIFO circular buffers.
static int addToTxArray(jd_frame_t* frame) {
    uint8_t nextTail = (txTail + 1) % JD_TX_ARRAY_SIZE;

    if (nextTail == txHead)
        return DEVICE_NO_RESOURCES;

    // add our buffer to the array before updating the head
    // this ensures atomicity.
    txArray[txTail] = frame;
    target_disable_irq();
    txTail = nextTail;
    target_enable_irq();

    return DEVICE_OK;
}

// FIFO circular buffers.
static int addToRxArray(jd_frame_t* frame) {
     uint8_t nextTail = (rxTail + 1) % JD_RX_ARRAY_SIZE;

    if (nextTail == rxHead)
        return DEVICE_NO_RESOURCES;

    // add our buffer to the array before updating the head
    // this ensures atomicity.
    rxArray[rxTail] = frame;
    target_disable_irq();
    rxTail = nextTail;
    target_enable_irq();

    return DEVICE_OK;
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#define STATUS_IN_RX 0x01
#define STATUS_IN_TX 0x02

static volatile uint8_t hw_status;


static void jd_tx_completed(int errCode);
static void jd_rx_completed(void);
static void setup_exti(void);


static int uart_start_tx(const jd_frame_t* frame) {

    if (hw_status & STATUS_IN_TX) {
        jd_panic();
    }



    target_disable_irq();
    if (hw_status & STATUS_IN_RX) {
        target_enable_irq();
        return -1;
    }



    gpio_disable_interrupts();

    int val = gpio_get_high_impedence(); //sws->p.getDigitalValue();
    target_enable_irq();

    // try to pull the line low, provided it currently reads as high
    if (val == 0 || gpio_get_and_set(0)) {
        // we failed - the line was low - start reception
        // jd_line_falling() would normally execute from EXTI, which has high
        // priority - we simulate this by completely disabling IRQs
        target_disable_irq();
        jd_line_falling();
        target_enable_irq();
        return -1;
    }


    //target_wait_us(9);
    common_hal_mcu_delay_us(11);

    hw_status |= STATUS_IN_TX;

    gpio_set(1);
    //sws->p.setDigitalValue(1);


    // LOG("start tx @%d", (int)tim_get_micros());
    //target_wait_us(40);
    common_hal_mcu_delay_us(40);

    //pin_pulse();

    sws_sendDMA((const uint8_t*) frame, JD_FRAME_SIZE(frame));


    return 0;
}

static void uart_start_rx(jd_frame_t* data, uint32_t maxbytes) {
    // LOG("start rx @%d", (int)tim_get_micros());
    if (hw_status & STATUS_IN_RX)
        jd_panic();
    hw_status |= STATUS_IN_RX;
    sws_receiveDMA((uint8_t *)data, maxbytes);
    //pin_log(0);
}

static void uart_disable(void) {
    //pin_pulse();
    sws_abortDMA();
    hw_status = 0;
    setup_exti();
    //pin_pulse();
}

static int uart_wait_high(void) {
    int timeout = 1000; // should be around 100-1000us
    while (timeout-- > 0 && gpio_get() == 0);
    if (timeout <= 0)
        return -1;
    return 0;
}



static void setup_exti(void) {
    // LOG("setup exti; %d", sws->p.name);
    sws_setMode(SINGLE_WIRE_DISCONNECTED);

    gpio_get();
    gpio_enable_interrupts();


    // force transition to output so that the pin is reconfigured.
    // also drive the bus high for a little bit.
    // TODO this is problematic as it may drive the line high, while another device is transmitting,
    // in case we're called by rx_timeout()
    //sws->p.setDigitalValue(1);
    //sws->p.getDigitalValue(PullMode::Up);
    //sws->p.eventOn(DEVICE_PIN_INTERRUPT_ON_EDGE);

}



/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#define JD_SERIAL_PAYLOAD_SIZE 236

#define JD_WR_OVERHEAD 7
#define JD_TIM_OVERHEAD 20

#define STATUS_IN_RX 0x01
#define STATUS_IN_TX 0x02



#define TIMEOUT_CC                  0
#define TX_CALLBACK_CC              0

#define JD_STATUS_RX_ACTIVE 0x01
#define JD_STATUS_TX_ACTIVE 0x02
#define JD_STATUS_TX_QUEUED 0x04


static cb_t tim_cb;

static volatile uint8_t jd_status;
//static uint8_t currEvent;


static jd_frame_t _rxBuffer[2];
static jd_frame_t* rxFrame = &_rxBuffer[0];
static jd_frame_t* txFrame;
//static uint64_t nextAnnounce;
static uint8_t txPending;
//static uint8_t annCounter;

static jd_diagnostics_t jd_diagnostics;
jd_diagnostics_t *jd_get_diagnostics(void) {
    jd_diagnostics.bus_state = 0;
    return &jd_diagnostics;
}


/***********************************************************/


/***********************************************************/






static void tx_done(void) {
    //signal_write(0);
    set_tick_timer(JD_STATUS_TX_ACTIVE);
}

static void jd_tx_completed(int errCode) {
    //LOG("tx done: %d", errCode);
    //app_frame_sent(txFrame); // TODO
    txFrame = NULL;
    tx_done();
}



/*
static void check_announce(void) {
    if (tim_get_micros() > nextAnnounce) {
        // pulse_log_pin();
        if (nextAnnounce)
            app_queue_annouce();
        nextAnnounce = tim_get_micros() + 499000 + (jd_random() & 0x7ff);
    }
}
*/








static void rx_timeout(void) {

    led2On();
    led3On();
    led4On();

    target_disable_irq();
    jd_diagnostics.bus_timeout_error++;
    //ERROR("RX timeout");
    uart_disable();
    //signal_read(0);
    set_tick_timer(JD_STATUS_RX_ACTIVE);
    target_enable_irq();
    //signal_error();
}

static void setup_rx_timeout(void) {
    uint32_t *p = (uint32_t *)rxFrame;
    if (p[0] == 0 && p[1] == 0)
        rx_timeout(); // didn't get any data after lo-pulse
    // got the size - set timeout for whole packet
    tim_set_timer(JD_FRAME_SIZE(rxFrame) * 12 + 60, rx_timeout);
}

static void jd_line_falling(void) {
    //LOG("line fall");
    //log_pin_set(1, 1);
    //pulse_log_pin();
    //signal_read(1);

    target_disable_irq();
    // no need to disable IRQ - we're at the highest IRQ level
    if (jd_status & JD_STATUS_RX_ACTIVE)
        jd_panic();
    jd_status |= JD_STATUS_RX_ACTIVE;

    // 1us faster than memset() on SAMD21
    uint32_t *p = (uint32_t *)rxFrame;
    p[0] = 0;
    p[1] = 0;
    p[2] = 0;
    p[3] = 0;



    // otherwise we can enable RX in the middle of LO pulse
    if (uart_wait_high() < 0) {
        // line didn't get high in 1ms or so - bail out
        rx_timeout();
        return;
    }
    // pulse1();
    // target_wait_us(2);

    uart_start_rx(rxFrame, sizeof(*rxFrame));
    //log_pin_set(1, 0);

    tim_set_timer(200, setup_rx_timeout);
    target_enable_irq();
}


static void jd_rx_completed(void) {
    //if (annCounter++ == 0)
    //    check_announce();

    //LOG("rx cmpl");
    jd_frame_t* frame = m_new_ll_obj(jd_frame_t);
    memcpy(frame, rxFrame, sizeof(*rxFrame));
    //jd_frame_t *frame = rxFrame;

    if (rxFrame == &_rxBuffer[0])
        rxFrame = &_rxBuffer[1];
    else
        rxFrame = &_rxBuffer[0];

    //signal_read(0);
    set_tick_timer(JD_STATUS_RX_ACTIVE);

    uint32_t txSize = sizeof(*frame);
    uint32_t declaredSize = JD_FRAME_SIZE(frame);
    if (txSize < declaredSize) {
        //ERROR("frame too short");
        jd_diagnostics.bus_uart_error++;
        return;
    }


    uint16_t crc = jd_crc16((uint8_t *)frame + 2, declaredSize - 2);
    if (crc != frame->crc) {
        //ERROR("crc mismatch");
        jd_diagnostics.bus_uart_error++;
        return;
    }

    jd_diagnostics.packets_received++;

    //if (frame->data[0] == 'a')

    // pulse1();
    // int err = app_handle_frame(frame);
    int err = addToRxArray(frame);



    if (err) {
        jd_diagnostics.packets_dropped++;
    }
}


static void jd_packet_ready(void) {
    target_disable_irq();
    txPending = 1;
    if (jd_status == 0) {
        set_tick_timer(0);
    }
    target_enable_irq();
}


/*
TODO
static int jd_is_running(void) {
    return nextAnnounce != 0;
}

static int jd_is_busy(void) {
    return jd_status != 0;
}
*/

static void flush_tx_buffer(void) {

    // pulse1();
    /*
    if (annCounter++ == 0)
        check_announce();
    */



    //LOG("flush %d", jd_status);
    target_disable_irq();
    if (jd_status & (JD_STATUS_RX_ACTIVE | JD_STATUS_TX_ACTIVE)) {
        target_enable_irq();
        return;
    }

    jd_status |= JD_STATUS_TX_ACTIVE;
    target_enable_irq();

    txPending = 0;
    if (!txFrame) {
        //txFrame = app_pull_frame();
        txFrame = popTxArray();
        if (txFrame == NULL) {
            tx_done();
            return;
        }
    }

    //signal_write(1);
    if (uart_start_tx(txFrame) < 0) {
        // ERROR("race on TX");
        jd_diagnostics.bus_lo_error++;
        tx_done();
        txPending = 1;

        return;
    }



    set_tick_timer(0);
}


static void tick(void) {
    led1Toggle();


    //check_announce();
    set_tick_timer(0);
}

static void tim_set_timer(int delta, cb_t cb) {
     // compensate for overheads
    delta -= JD_TIM_OVERHEAD;
    if (delta < 20)
        delta = 20;
/*
    uint64_t ticks = delta * 31250ULL;
    if (ticks > UINT32_MAX) {
        mp_raise_ValueError(translate("timeout duration exceeded the maximum supported value"));
    }
*/
    target_disable_irq();

    tim_cb = cb;

    //uint64_t t = delta * ((SystemCoreClock) / (1000UL * 1000UL));

    uint32_t t = nrfx_timer_us_to_ticks(timer, delta);

    nrfx_timer_clear(timer);
    nrfx_timer_compare(timer, NRF_TIMER_CC_CHANNEL0, t, true);
    //nrfx_timer_resume(timer);

    //system_timer_event_after_us(delta, DEVICE_ID, currEvent);
    //system_timer_cancel_event(DEVICE_ID, prev); // make sure we don't get the same slot

    target_enable_irq();
}


static void set_tick_timer(uint8_t statusClear) {

    target_disable_irq();
    if (statusClear) {
        // LOG("st %d @%d", statusClear, jd_status);
        jd_status &= ~statusClear;
    }
    if ((jd_status & JD_STATUS_RX_ACTIVE) == 0) {
        if (txPending && !(jd_status & JD_STATUS_TX_ACTIVE)) {
            //pulse1();
            // the JD_WR_OVERHEAD value should be such, that the time from pulse1() above
            // to beginning of low-pulse generated by the current device is exactly 150us
            // (when the line below is uncommented)
            // tim_set_timer(150 - JD_WR_OVERHEAD, flush_tx_queue);
            jd_status |= JD_STATUS_TX_QUEUED;
            tim_set_timer(jd_random_around(150) - JD_WR_OVERHEAD, flush_tx_buffer);
        } else {
            jd_status &= ~JD_STATUS_TX_QUEUED;
            tim_set_timer(10000, tick);
        }
    }
    target_enable_irq();

}




static void sws_done(uint8_t errCode) {
    //pin_pulse();
    //pin_pulse();

    // LOG("sws_done %d @%d", errCode, (int)tim_get_micros());


    switch (errCode) {
    case SWS_EVT_DATA_SENT:
        if (hw_status & STATUS_IN_TX) {
            hw_status &= ~STATUS_IN_TX;
            sws_setMode(SINGLE_WIRE_DISCONNECTED);
            // force reconfigure
            //sws->p.getDigitalValue();
            // send break signal
            //sws->p.setDigitalValue(0);
            gpio_set(0);
            //target_wait_us(11);
            common_hal_mcu_delay_us(11);
            //sws->p.setDigitalValue(1);
            gpio_set(1);
            jd_tx_completed(0);
        }
        break;
        /*
    case SWS_EVT_ERROR: // brk condition
        if (!(hw_status & STATUS_IN_RX)) {
            //LOG("SWS error");
            //target_panic(122);
            jd_panic();
        } else {
            return;
        }
        break;
        */
    case SWS_EVT_DATA_RECEIVED:

        // LOG("DMA overrun");
        // sws->getBytesReceived() always returns 1 on NRF
        if (hw_status & STATUS_IN_RX) {
            hw_status &= ~STATUS_IN_RX;
            sws_setMode(SINGLE_WIRE_DISCONNECTED);
            jd_rx_completed();
        } else {
            //LOG("double complete");
            //target_panic(122);
            jd_panic();
        }
        sws_abortDMA();
        break;
    }
    setup_exti();

    //pin_pulse();

}




/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

static void uart_irq(const nrfx_uarte_event_t* event, void* context) {

    uint8_t eventValue = 0;
    switch ( event->type ) {
        case NRFX_UARTE_EVT_RX_DONE:
            configureRxInterrupt(0);
            eventValue = SWS_EVT_DATA_RECEIVED;
            led3Toggle();
            break;

        case NRFX_UARTE_EVT_TX_DONE:
            configureTxInterrupt(0);
            eventValue = SWS_EVT_DATA_SENT;
            led2Toggle();
            break;

        case NRFX_UARTE_EVT_ERROR:
            // Possible Error source is Overrun, Parity, Framing, Break


            //led2Toggle();
            if (event->data.error.error_mask & NRF_UARTE_ERROR_BREAK_MASK && hw_status & STATUS_IN_RX) {
                eventValue = SWS_EVT_DATA_RECEIVED;
                configureRxInterrupt(0);
                led1Toggle();
            }

            if (event->data.error.error_mask & NRF_UARTE_ERROR_FRAMING_MASK)
                led3Toggle();

            break;
     }
     /*
     NRF_UARTE_ERROR_OVERRUN_MASK = UARTE_ERRORSRC_OVERRUN_Msk, ///< Overrun error.
    NRF_UARTE_ERROR_PARITY_MASK  = UARTE_ERRORSRC_PARITY_Msk,  ///< Parity error.
    NRF_UARTE_ERROR_FRAMING_MASK = UARTE_ERRORSRC_FRAMING_Msk, ///< Framing error.
    NRF_UARTE_ERROR_BREAK_MASK   = UARTE_ERRORSRC_BREAK_Msk    ///< Break error.
*/
    if (eventValue > 0) {
        led4Toggle();
        sws_done(eventValue);
    }
}

static void timer_irq(nrf_timer_event_t event_type, void* p_context) {
    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        cb_t f = tim_cb;
        if (f != NULL) {
            tim_cb = NULL;
            f();
        }
    }
}


static void gpiote_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    //led2Toggle();

    if (gpio_isOutput()) {
        // LOG("in send already");
        return;
    }

    //sws->p.eventOn(DEVICE_PIN_EVENT_NONE);
    gpio_disable_interrupts();
    jd_line_falling();
}

/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/


static void initialize_buffers(void) {
    memset(rxArray, 0, sizeof(jd_frame_t*) * JD_RX_ARRAY_SIZE);
    memset(txArray, 0, sizeof(jd_frame_t*) * JD_TX_ARRAY_SIZE);
    txTail = 0;
    txHead = 0;
    rxTail = 0;
    rxHead = 0;
}

static void initialize_gpio(void) {
    //sws_setMode(SINGLE_WIRE_DISCONNECTED);
/*
    nrfx_gpiote_in_config_t gpio_config = {
        .sense = NRF_GPIOTE_POLARITY_TOGGLE,
        .pull = NRF_GPIO_PIN_PULLUP, // idle_state ? NRF_GPIO_PIN_PULLDOWN : NRF_GPIO_PIN_PULLUP,
        .is_watcher = false, // nrf_gpio_cfg_watcher vs nrf_gpio_cfg_input
        .hi_accuracy = true,
        .skip_gpio_setup = true
    };
    /*/
    nrfx_gpiote_in_config_t cfg = {
        .sense = NRF_GPIOTE_POLARITY_HITOLO,
        .pull = NRF_GPIO_PIN_PULLUP, // idle_state ? NRF_GPIO_PIN_PULLDOWN : NRF_GPIO_PIN_PULLUP,
        .is_watcher = false, // nrf_gpio_cfg_watcher vs nrf_gpio_cfg_input
        .hi_accuracy = true,
        .skip_gpio_setup = false
    };

    nrfx_gpiote_in_init(self->pin, &cfg, gpiote_callback);
    nrfx_gpiote_in_event_enable(self->pin, true);


    setup_exti();
}

static void initialize_timer(void) {
    if (timer_refcount == 0) {
        timer = nrf_peripherals_allocate_timer();
        if (timer == NULL) {
            jd_panic();
            mp_raise_RuntimeError(translate("All timers in use"));
        }

        nrfx_timer_config_t timer_config = {
            .frequency = NRF_TIMER_FREQ_16MHz,
            .mode = NRF_TIMER_MODE_TIMER,
            .bit_width = NRF_TIMER_BIT_WIDTH_32,
            .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
        };

        nrfx_timer_init(timer, &timer_config, &timer_irq);
        nrfx_timer_enable(timer);
    }
    timer_refcount++;

}

static void initialize_uart(void) {

    uarte = NULL;

    for (size_t i = 0; i < MP_ARRAY_SIZE(nrfx_uartes); i++) {
        if ((nrfx_uartes[i].p_reg->ENABLE & UARTE_ENABLE_ENABLE_Msk) == 0) {
            uarte = &nrfx_uartes[i];
            break;
        }
    }

    if (uarte == NULL)
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

    _VERIFY_ERR(nrfx_uarte_init(uarte, &uart_config, uart_irq));

    //configureRxInterrupt(0);
    //configureTxInterrupt(0);
    //sws_setMode(SINGLE_WIRE_DISCONNECTED);
}


void common_hal_busio_jacdac_construct(busio_jacdac_obj_t* context, const mcu_pin_obj_t* pin) {
    self = context;
    self->pin = pin->number;
    self->sws_status = 0;


    led1Toggle();
    led2Toggle();
    led3Toggle();
    led4Toggle();
    led1Off();
    led2Off();
    led3Off();
    led4Off();



    initialize_timer();

    //nrf_gpio_cfg_input(self->pin, NRF_GPIO_PIN_PULLUP);
    claim_pin(pin);

    initialize_gpio();

    initialize_buffers();
    initialize_uart();

    self->sws_status |= DEVICE_COMPONENT_RUNNING;

    set_tick_timer(0);


    //check_announce();
}


void common_hal_busio_jacdac_deinit(busio_jacdac_obj_t* context) {
    led2Toggle();

    self = context;

    if (common_hal_busio_jacdac_deinited(self))
        return;

    nrfx_gpiote_in_event_disable(self->pin);
    nrfx_gpiote_in_uninit(self->pin);

    // timer
    timer_refcount--;
    if (timer_refcount == 0) {
        nrf_peripherals_free_timer(timer);
    }


    // uart
    if (uarte != NULL)
        nrfx_uarte_uninit(uarte);

    // pin
    reset_pin_number(self->pin);
    self->pin = NO_PIN;
}

bool common_hal_busio_jacdac_deinited(busio_jacdac_obj_t *context) {
    return self->pin == NO_PIN;
}


uint8_t common_hal_busio_jacdac_send(busio_jacdac_obj_t *context, const uint32_t *data, size_t len) {
    self = context;

    jd_frame_t* t = m_new_ll_obj(jd_frame_t);

    t->size = 16;
    t->flags = 0;
    t->device_identifier = 0x747E48326EB44CF8;
    t->data[0] = 12;

    uint32_t declaredSize = JD_FRAME_SIZE(t);
    t->crc = jd_crc16((uint8_t *)t + 2, declaredSize - 2);

/*
    uint16_t crc;
    uint8_t size;
    uint8_t flags;

    uint64_t device_identifier;

    uint8_t data[JD_SERIAL_PAYLOAD_SIZE + 4];
    */
    addToTxArray(t);
    jd_packet_ready();
    return 0; //
    //return sws_sendDMA(data, len);
}

uint8_t common_hal_busio_jacdac_receive(busio_jacdac_obj_t *context, uint8_t *data, size_t len) {
    jd_frame_t* t = popRxArray();

    if (t != NULL)
        memcpy(data, t, sizeof(*t));

    return 0;
}


/****************************************************************************/

void jacdac_reset(void) {

    if ( nrfx_gpiote_is_init() ) {
        nrfx_gpiote_uninit();
    }
    nrfx_gpiote_init(NRFX_GPIOTE_CONFIG_IRQ_PRIORITY);

    if (timer != NULL) {
        nrf_peripherals_free_timer(timer);
    }
    timer_refcount = 0;

/*
    // gpio
    if ( nrfx_gpiote_is_init() ) {
        nrfx_gpiote_uninit();
    }
    nrfx_gpiote_init(NRFX_GPIOTE_CONFIG_IRQ_PRIORITY);

    // buffers
    initialize_buffers();
    */
/*

    // uart
    for (size_t i = 0 ; i < MP_ARRAY_SIZE(nrfx_uartes); i++) {
        nrfx_uarte_uninit(&nrfx_uartes[i]);
    }
*/
/*
    jd_status = 0;
    hw_status = 0;
    self->sws_status = 0;

*/


    //reset_pin_number(self->pin);
    //self->pin = NO_PIN;
}


