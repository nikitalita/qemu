/*
 * QEMU MOS6522 VIA emulation
 *
 * Copyright (c) 2004-2007 Fabrice Bellard
 * Copyright (c) 2007 Jocelyn Mayer
 * Copyright (c) 2018 Mark Cave-Ayland
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MOS6522_H
#define MOS6522_H

#include "exec/memory.h"
#include "hw/sysbus.h"
#include "hw/ide/internal.h"
#include "hw/input/adb.h"

/**
 * MOS6522Timer:
 * @counter_value: counter value at load time
 */
typedef struct MOS6522Timer {
    int index;
    uint16_t latch;
    uint16_t counter_value;
    int64_t load_time;
    int64_t next_irq_time;
    uint64_t frequency;
    QEMUTimer *timer;
} MOS6522Timer;

/**
 * MOS6522State:
 * @b: B-side data
 * @a: A-side data
 * @dirb: B-side direction (1=output)
 * @dira: A-side direction (1=output)
 * @sr: Shift register
 * @acr: Auxiliary control register
 * @pcr: Peripheral control register
 * @ifr: Interrupt flag register
 * @ier: Interrupt enable register
 * @anh: A-side data, no handshake
 * @last_b: last value of B register
 * @last_acr: last value of ACR register
 */
typedef struct MOS6522State {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion mem;
    /* cuda registers */
    uint8_t b;
    uint8_t a;
    uint8_t dirb;
    uint8_t dira;
    uint8_t sr;
    uint8_t acr;
    uint8_t pcr;
    uint8_t ifr;
    uint8_t ier;
    uint8_t anh;

    ADBBusState adb_bus;
    MOS6522Timer timers[2];

    uint32_t tick_offset;
    uint64_t frequency;

    uint8_t last_b;
    uint8_t last_acr;

    /* MacOS 9 is racy and requires a delay upon setting the SR_INT bit */
    uint64_t sr_delay_ns;
    QEMUTimer *sr_delay_timer;

    int data_in_size;
    int data_in_index;
    int data_out_index;

    qemu_irq irq;
    uint16_t adb_poll_mask;
    uint8_t autopoll_rate_ms;
    uint8_t autopoll;
    uint8_t data_in[128];
    uint8_t data_out[16];
    QEMUTimer *adb_poll_timer;
} MOS6522State;

#define TYPE_MOS6522 "cuda"
#define MOS6522(obj) OBJECT_CHECK(MOS6522State, (obj), TYPE_MOS6522)

typedef struct MOS6522DeviceClass
{
    DeviceClass parent_class;

    void (*portB_write)(MOS6522State *dev);
    void (*portA_write)(MOS6522State *dev);
    uint64_t (*get_timer1_counter_value)(MOS6522State *dev);
    uint64_t (*get_timer2_counter_value)(MOS6522State *dev);
} MOS6522DeviceClass;

#define MOS6522_DEVICE_CLASS(cls) \
    OBJECT_CLASS_CHECK(MOS6522DeviceClass, (cls), TYPE_MOS6522)
#define MOS6522_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(MOS6522DeviceClass, (obj), TYPE_MOS6522)

#endif /* MOS6522_H */
