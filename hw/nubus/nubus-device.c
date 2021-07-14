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
#include "qemu/datadir.h"
#include "hw/loader.h"
#include "hw/nubus/nubus.h"
#include "qapi/error.h"
#include "qemu/error-report.h"


static void nubus_device_realize(DeviceState *dev, Error **errp)
{
    NubusBus *nubus = NUBUS_BUS(qdev_get_parent_bus(dev));
    NubusDevice *nd = NUBUS_DEVICE(dev);
    char *name, *path;
    hwaddr slot_offset;
    int64_t size;
    uint16_t s;
    int ret;

    if (nd->slot == -1) {
        /* No slot specified, find first available free slot */
        s = ctz32(nubus->slot_available_mask);
        if (s) {
            nd->slot = s;
        } else {
            error_setg(errp, "Cannot register nubus card, no free slot "
                             "available");
            return;
        }
    } else {
        /* Slot specified, make sure the slot is available */
        if (nd->slot < NUBUS_FIRST_SLOT || nd->slot > NUBUS_LAST_SLOT) {
            error_setg(errp, "Cannot register nubus card, slot must be "
                             "between %d and %d", NUBUS_FIRST_SLOT,
                             NUBUS_LAST_SLOT);
            return;
        }

        if (!(nubus->slot_available_mask & (1UL << nd->slot))) {
            error_setg(errp, "Cannot register nubus card, slot %d is "
                             "unavailable or already occupied", nd->slot);
            return;
        }
    }

    nubus->slot_available_mask &= ~(1UL << nd->slot);

    /* Super */
    slot_offset = nd->slot * NUBUS_SUPER_SLOT_SIZE;

    name = g_strdup_printf("nubus-super-slot-%x", nd->slot);
    memory_region_init(&nd->super_slot_mem, OBJECT(dev), name,
                        NUBUS_SUPER_SLOT_SIZE);
    memory_region_add_subregion(&nubus->super_slot_io, slot_offset,
                                &nd->super_slot_mem);
    g_free(name);

    /* Normal */
    slot_offset = nd->slot * NUBUS_SLOT_SIZE;

    name = g_strdup_printf("nubus-slot-%x", nd->slot);
    memory_region_init(&nd->slot_mem, OBJECT(dev), name, NUBUS_SLOT_SIZE);
    memory_region_add_subregion(&nubus->slot_io, slot_offset,
                                &nd->slot_mem);
    g_free(name);

    /* Declaration ROM */
    if (nd->romfile != NULL) {
        path = qemu_find_file(QEMU_FILE_TYPE_BIOS, nd->romfile);
        if (path == NULL) {
            path = g_strdup(nd->romfile);
        }

        size = get_image_size(path);
        if (size < 0) {
            error_setg(errp, "failed to find romfile \"%s\"", nd->romfile);
            g_free(path);
            return;
        } else if (size == 0) {
            error_setg(errp, "romfile \"%s\" is empty", nd->romfile);
            g_free(path);
            return;
        } else if (size > NUBUS_DECL_ROM_MAX_SIZE) {
            error_setg(errp, "romfile \"%s\" too large (size cannot exceed 64K)",
                       nd->romfile);
            g_free(path);
            return;
        }

        name = g_strdup_printf("nubus-slot-%x-declaration-rom", nd->slot);
        memory_region_init_rom(&nd->decl_rom, OBJECT(dev), name, size,
                               &error_fatal);
        ret = load_image_mr(path, &nd->decl_rom);
        g_free(path);
        if (ret < 0) {
            warn_report("nubus-device: could not load prom '%s'", nd->romfile);
        }
        memory_region_add_subregion(&nd->slot_mem, NUBUS_SLOT_SIZE - size,
                                    &nd->decl_rom);
    }
}

static Property nubus_device_properties[] = {
    DEFINE_PROP_INT32("slot", NubusDevice, slot, -1),
    DEFINE_PROP_STRING("romfile", NubusDevice, romfile),
    DEFINE_PROP_END_OF_LIST()
};

static void nubus_device_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = nubus_device_realize;
    dc->bus_type = TYPE_NUBUS_BUS;
    device_class_set_props(dc, nubus_device_properties);
}

static const TypeInfo nubus_device_type_info = {
    .name = TYPE_NUBUS_DEVICE,
    .parent = TYPE_DEVICE,
    .abstract = true,
    .instance_size = sizeof(NubusDevice),
    .class_init = nubus_device_class_init,
};

static void nubus_register_types(void)
{
    type_register_static(&nubus_device_type_info);
}

type_init(nubus_register_types)
