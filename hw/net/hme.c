/*
 * QEMU Happy Meal Ethernet emulation
 *
 * Copyright (c) 2017 Mark Cave-Ayland
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

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "net/net.h"

#define HME_REG_SIZE       0x8000

/* Change to 1 to enable debugging */
#define DEBUG_HME 1

#define DPRINTF(fmt, ...) do { \
    if (DEBUG_HME) { \
        printf("HME: " fmt , ## __VA_ARGS__); \
    } \
} while (0);

#define TYPE_HME "hme"
#define HME(obj) OBJECT_CHECK(HMEState, (obj), TYPE_HME)

typedef struct HMEState {
    /*< private >*/
    PCIDevice parent_obj;
    
    NICState *nic;
    NICConf conf;

    MemoryRegion hme;
    MemoryRegion sebreg;
    MemoryRegion etxreg;
    MemoryRegion erxreg;
    MemoryRegion macreg;
    MemoryRegion mifreg;
} HMEState;

static Property hme_properties[] = {
    DEFINE_NIC_PROPERTIES(HMEState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void hme_seb_write(void *opaque, hwaddr addr,
                          uint64_t val, unsigned size)
{
    DPRINTF("hme_seb_write %" HWADDR_PRIx " %lx\n", addr, val);
}

static uint64_t hme_seb_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    DPRINTF("hme_seb_read %" HWADDR_PRIx "\n", addr);

    return 0;
}

static const MemoryRegionOps hme_seb_ops = {
    .read = hme_seb_read,
    .write = hme_seb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void hme_etx_write(void *opaque, hwaddr addr,
                          uint64_t val, unsigned size)
{
    DPRINTF("hme_etx_write %" HWADDR_PRIx " %lx\n", addr, val);
}

static uint64_t hme_etx_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    DPRINTF("hme_etx_read %" HWADDR_PRIx "\n", addr);

    return 0;
}

static const MemoryRegionOps hme_etx_ops = {
    .read = hme_etx_read,
    .write = hme_etx_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void hme_erx_write(void *opaque, hwaddr addr,
                          uint64_t val, unsigned size)
{
    DPRINTF("hme_erx_write %" HWADDR_PRIx " %lx\n", addr, val);
}

static uint64_t hme_erx_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    DPRINTF("hme_erx_read %" HWADDR_PRIx "\n", addr);

    return 0;
}

static const MemoryRegionOps hme_erx_ops = {
    .read = hme_erx_read,
    .write = hme_erx_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void hme_mac_write(void *opaque, hwaddr addr,
                          uint64_t val, unsigned size)
{
    DPRINTF("hme_mac_write %" HWADDR_PRIx " %lx\n", addr, val);
}

static uint64_t hme_mac_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    DPRINTF("hme_mac_read %" HWADDR_PRIx "\n", addr);

    return 0;
}

static const MemoryRegionOps hme_mac_ops = {
    .read = hme_mac_read,
    .write = hme_mac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void hme_mif_write(void *opaque, hwaddr addr,
                          uint64_t val, unsigned size)
{
    DPRINTF("hme_mif_write %" HWADDR_PRIx " %lx\n", addr, val);
}

static uint64_t hme_mif_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    DPRINTF("hme_mif_read %" HWADDR_PRIx "\n", addr);

    return 0;
}

static const MemoryRegionOps hme_mif_ops = {
    .read = hme_mif_read,
    .write = hme_mif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void hme_realize(PCIDevice *pci_dev, Error **errp)
{
    HMEState *s = HME(pci_dev);
    uint8_t *pci_conf;

    pci_conf = pci_dev->config;
    pci_conf[PCI_INTERRUPT_PIN] = 1;    /* interrupt pin A */

    memory_region_init(&s->hme, OBJECT(pci_dev), "hme", HME_REG_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->hme);
    
    memory_region_init_io(&s->sebreg, OBJECT(pci_dev), &hme_seb_ops, 0,
                          "hme.seb", 0x2000);
    memory_region_add_subregion(&s->hme, 0, &s->sebreg);

    memory_region_init_io(&s->etxreg, OBJECT(pci_dev), &hme_etx_ops, 0,
                          "hme.etx", 0x2000);
    memory_region_add_subregion(&s->hme, 0x2000, &s->etxreg);

    memory_region_init_io(&s->erxreg, OBJECT(pci_dev), &hme_erx_ops, 0,
                          "hme.erx", 0x2000);
    memory_region_add_subregion(&s->hme, 0x4000, &s->erxreg);

    memory_region_init_io(&s->macreg, OBJECT(pci_dev), &hme_mac_ops, 0,
                          "hme.mac", 0x1000);
    memory_region_add_subregion(&s->hme, 0x6000, &s->macreg);

    memory_region_init_io(&s->mifreg, OBJECT(pci_dev), &hme_mif_ops, 0,
                          "hme.mif", 0x1000);
    memory_region_add_subregion(&s->hme, 0x7000, &s->mifreg);

    return;
}

static void hme_exit(PCIDevice *pci_dev)
{
    printf("hme_exit!\n");

    return;
}

static void hme_instance_init(Object *obj)
{
    printf("hme_instance_init!\n");

    return;
}

static void hme_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = hme_realize;
    k->exit = hme_exit;
    k->vendor_id = PCI_VENDOR_ID_SUN;
    k->device_id = PCI_DEVICE_ID_SUN_HME;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    //dc->vmsd = &vmstate_hme;
    dc->props = hme_properties;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo hme_info = {
    .name          = TYPE_HME,
    .parent        = TYPE_PCI_DEVICE,
    .class_init    = hme_class_init,
    .instance_size = sizeof(HMEState),
    .instance_init = hme_instance_init,
};

static void hme_register_types(void)
{
    type_register_static(&hme_info);
}

type_init(hme_register_types)
