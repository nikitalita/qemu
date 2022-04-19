/*
 *  Copyright (c) 2012-2018 Laurent Vivier <laurent@vivier.eu>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef HW_AUDIO_ASC_H
#define HW_AUDIO_ASC_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "audio/audio.h"

enum {
    ASC_TYPE_ASC    = 0,  /* original discrete Apple Sound Chip */
    ASC_TYPE_EASC   = 1   /* discrete Enhanced Apple Sound Chip */
};

#define ASC_FIFO_OFFSET    0x0
#define ASC_FIFO_SIZE      0x400

#define ASC_REG_OFFSET     0x800
#define ASC_REG_SIZE       0x60

#define ASC_EXTREG_OFFSET  0xf00
#define ASC_EXTREG_SIZE    0x20

typedef struct ASCFIFOState {
    int index;

    MemoryRegion mem_fifo;
    uint8_t fifo[ASC_FIFO_SIZE];
    uint8_t int_status;

    int cnt;
    int wptr;
    int rptr;

    MemoryRegion mem_extregs;
    uint8_t extregs[ASC_EXTREG_SIZE];

    int xa_cnt;
    uint8_t xa_val;
    uint8_t xa_flags;
    int16_t xa_last[2];
} ASCFIFOState;

struct ASCState {
    SysBusDevice parent_obj;

    uint8_t type;
    MemoryRegion asc;
    MemoryRegion mem_fifo;
    MemoryRegion mem_regs;
    MemoryRegion mem_extregs;

    QEMUSoundCard card;
    SWVoiceOut *voice;
    int8_t *mixbuf;
    int left, pos, samples, shift;

    qemu_irq irq;

    ASCFIFOState fifos[2];

    uint8_t regs[ASC_REG_SIZE];
};

#define TYPE_ASC "apple-sound-chip"
OBJECT_DECLARE_SIMPLE_TYPE(ASCState, ASC)

#endif
