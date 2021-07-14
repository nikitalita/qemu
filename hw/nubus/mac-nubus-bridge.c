/*
 *  Copyright (c) 2013-2018 Laurent Vivier <laurent@vivier.eu>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/nubus/mac-nubus-bridge.h"


static void mac_nubus_bridge_init(Object *obj)
{
    MacNubusState *s = MAC_NUBUS_BRIDGE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->bus = NUBUS_BUS(qbus_create(TYPE_NUBUS_BUS, DEVICE(s), NULL));

    /* Macintosh only has slots 0x9 to 0xe available */
    s->bus->slot_available_mask = 0x7e00;

    /* Aliases for slots 0x9 to 0xe */
    memory_region_init_alias(&s->super_slot_alias, obj, "super-slot-alias",
                             &s->bus->nubus_mr,
                             9 * NUBUS_SUPER_SLOT_SIZE,
                             6 * NUBUS_SUPER_SLOT_SIZE);

    memory_region_init_alias(&s->slot_alias, obj, "slot-alias",
                             &s->bus->nubus_mr,
                             NUBUS_SLOT_BASE + 9 * NUBUS_SLOT_SIZE,
                             6 * NUBUS_SLOT_SIZE);

    sysbus_init_mmio(sbd, &s->super_slot_alias);
    sysbus_init_mmio(sbd, &s->slot_alias);
}

static void mac_nubus_bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Nubus bridge";
}

static const TypeInfo mac_nubus_bridge_info = {
    .name          = TYPE_MAC_NUBUS_BRIDGE,
    .parent        = TYPE_NUBUS_BRIDGE,
    .instance_init = mac_nubus_bridge_init,
    .instance_size = sizeof(MacNubusState),
    .class_init    = mac_nubus_bridge_class_init,
};

static void mac_nubus_bridge_register_types(void)
{
    type_register_static(&mac_nubus_bridge_info);
}

type_init(mac_nubus_bridge_register_types)
