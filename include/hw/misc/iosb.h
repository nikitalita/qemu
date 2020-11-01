/*
 * IOSB
 *
 */

#ifndef HW_MEM_IOSB_H
#define HW_MEM_IOSB_H

typedef struct IOSBState {
    SysBusDevice parent_obj;

    MemoryRegion mem_regs;
    uint32_t regs[6];
} IOSBState;

#define TYPE_IOSB "IOSB"
#define IOSB(obj) OBJECT_CHECK(IOSBState, (obj), TYPE_IOSB)

#endif
