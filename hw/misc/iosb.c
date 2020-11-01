/*
 *
 */

#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/sysbus.h"
#include "hw/misc/iosb.h"
#include "trace.h"

#define IOSB_SIZE       0x2000

#define IOSB_CONFIG     0
#define IOSB_CONFIG2    1
#define IOSB_SONIC_SCSI 2
#define IOSB_REVISION   3
#define IOSB_SCSI_RESID 4
#define IOSB_BRIGHTNESS 5
#define IOSB_TIMEOUT    6

static const VMStateDescription vmstate_IOSB = {
    .name = "IOSB",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t IOSB_read(void *opaque, hwaddr addr,
                          unsigned size)
{
    IOSBState *s = IOSB(opaque);
    uint64_t val = 0;

    addr >>= 8;
    val = s->regs[addr];

    trace_IOSB_read(addr, size, val);
    return val;
}

static void IOSB_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    IOSBState *s = IOSB(opaque);

    addr >>= 8;
    s->regs[addr] = val;

    trace_IOSB_write(addr, size, val);
}

static const MemoryRegionOps IOSB_mmio_ops = {
    .read = IOSB_read,
    .write = IOSB_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .endianness = DEVICE_BIG_ENDIAN,
};

static void IOSB_init(Object *obj)
{
    IOSBState *s = IOSB(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mem_regs, obj, &IOSB_mmio_ops, s, "IOSB",
                          IOSB_SIZE);
    sysbus_init_mmio(sbd, &s->mem_regs);
}

static void IOSB_reset(DeviceState *d)
{
    IOSBState *s = IOSB(d);

    s->regs[IOSB_CONFIG] = 1; /* BCLK 33 MHz */
}

static void IOSB_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->reset = IOSB_reset;
    dc->vmsd = &vmstate_IOSB;
}

static TypeInfo IOSB_info = {
    .name          = TYPE_IOSB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IOSBState),
    .instance_init = IOSB_init,
    .class_init    = IOSB_class_init,
};

static void IOSB_register_types(void)
{
    type_register_static(&IOSB_info);
}

type_init(IOSB_register_types)
