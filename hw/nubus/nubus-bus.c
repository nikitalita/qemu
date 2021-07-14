/*
 * QEMU Macintosh Nubus
 *
 * Copyright (c) 2013-2018 Laurent Vivier <laurent@vivier.eu>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "hw/nubus/nubus.h"
#include "qapi/error.h"
#include "trace.h"


static NubusBus *nubus_find(void)
{
    /* Returns NULL unless there is exactly one nubus device */
    return NUBUS_BUS(object_resolve_path_type("", TYPE_NUBUS_BUS, NULL));
}

static MemTxResult nubus_slot_write(void *opaque, hwaddr addr, uint64_t val,
                                    unsigned size, MemTxAttrs attrs)
{
    trace_nubus_slot_write(addr, val, size);
    return MEMTX_DECODE_ERROR;
}

static MemTxResult nubus_slot_read(void *opaque, hwaddr addr, uint64_t *data,
                                   unsigned size, MemTxAttrs attrs)
{
    trace_nubus_slot_read(addr, size);
    return MEMTX_DECODE_ERROR;
}

static const MemoryRegionOps nubus_slot_ops = {
    .read_with_attrs  = nubus_slot_read,
    .write_with_attrs = nubus_slot_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static MemTxResult nubus_super_slot_write(void *opaque, hwaddr addr,
                                          uint64_t val, unsigned size,
                                          MemTxAttrs attrs)
{
    trace_nubus_super_slot_write(addr, val, size);
    return MEMTX_DECODE_ERROR;
}

static MemTxResult nubus_super_slot_read(void *opaque, hwaddr addr,
                                         uint64_t *data, unsigned size,
                                         MemTxAttrs attrs)
{
    trace_nubus_super_slot_read(addr, size);
    return MEMTX_DECODE_ERROR;
}

static const MemoryRegionOps nubus_super_slot_ops = {
    .read_with_attrs = nubus_super_slot_read,
    .write_with_attrs = nubus_super_slot_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void nubus_realize(BusState *bus, Error **errp)
{
    NubusBus *nubus = NUBUS_BUS(bus);

    if (!nubus_find()) {
        error_setg(errp, "at most one %s device is permitted", TYPE_NUBUS_BUS);
        return;
    }

    address_space_init(&nubus->nubus_as, &nubus->nubus_mr, "nubus");
}

static void nubus_init(Object *obj)
{
    NubusBus *nubus = NUBUS_BUS(obj);

    memory_region_init(&nubus->nubus_mr, obj, "nubus", 0x100000000);

    memory_region_init_io(&nubus->super_slot_io, obj, &nubus_super_slot_ops,
                          nubus, "nubus-super-slots",
                          NUBUS_SUPER_SLOT_NB * NUBUS_SUPER_SLOT_SIZE);
    memory_region_add_subregion(&nubus->nubus_mr, 0x0, &nubus->super_slot_io);

    memory_region_init_io(&nubus->slot_io, obj, &nubus_slot_ops,
                          nubus, "nubus-slots",
                          NUBUS_SLOT_NB * NUBUS_SLOT_SIZE);
    memory_region_add_subregion(&nubus->nubus_mr,
                                NUBUS_SUPER_SLOT_NB * NUBUS_SUPER_SLOT_SIZE,
                                &nubus->slot_io);

    nubus->slot_available_mask = 0xffff;
}

static void nubus_class_init(ObjectClass *oc, void *data)
{
    BusClass *bc = BUS_CLASS(oc);

    bc->realize = nubus_realize;
}

static const TypeInfo nubus_bus_info = {
    .name = TYPE_NUBUS_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(NubusBus),
    .instance_init = nubus_init,
    .class_init = nubus_class_init,
};

static void nubus_register_types(void)
{
    type_register_static(&nubus_bus_info);
}

type_init(nubus_register_types)
