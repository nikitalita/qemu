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
#include "hw/net/mii.h"
#include "net/net.h"

#define HME_REG_SIZE           0x8000

#define HME_SEB_REG_SIZE       0x2000

#define HME_SEBI_RESET         0x0
#define HME_SEB_RESET_ETX      0x1
#define HME_SEB_RESET_ERX      0x2

#define HME_ETX_REG_SIZE       0x2000
#define HME_ERX_REG_SIZE       0x2000
#define HME_MAC_REG_SIZE       0x1000

#define HME_MIF_REG_SIZE       0x20

#define HME_MIFI_FO            0xc
#define HME_MIF_FO_ST          0xc0000000    /* Start of frame */
#define HME_MIF_FO_ST_SHIFT    30
#define HME_MIF_FO_OPC         0x30000000    /* Opcode */
#define HME_MIF_FO_OPC_SHIFT   28
#define HME_MIF_FO_PHYAD       0x0f800000    /* PHY Address */
#define HME_MIF_FO_PHYAD_SHIFT 23
#define HME_MIF_FO_REGAD       0x007c0000    /* Register Address */
#define HME_MIF_FO_REGAD_SHIFT 18
#define HME_MIF_FO_TAMSB       0x20000    /* Turn-around MSB */
#define HME_MIF_FO_TALSB       0x10000    /* Turn-around LSB */
#define HME_MIF_FO_DATA        0xffff     /* data to read or write */

#define HME_MIFI_CFG           0x10
#define HME_MIF_CFG_MDI0       0x100
#define HME_MIF_CFG_MDI1       0x200 

/* Wired HME PHY addresses */
#define	HME_PHYAD_INTERNAL     1
#define	HME_PHYAD_EXTERNAL     0

#define MII_COMMAND_START      0x1
#define MII_COMMAND_READ       0x2
#define MII_COMMAND_WRITE      0x1


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
    
    uint32_t sebregs[HME_SEB_REG_SIZE >> 2];
    uint32_t etxregs[HME_ETX_REG_SIZE >> 2];
    uint32_t erxregs[HME_ERX_REG_SIZE >> 2];
    uint32_t macregs[HME_MAC_REG_SIZE >> 2];
    uint32_t mifregs[HME_MIF_REG_SIZE >> 2];
    
    uint16_t miiregs[0x20];
} HMEState;

static Property hme_properties[] = {
    DEFINE_NIC_PROPERTIES(HMEState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void hme_reset_tx(HMEState *s)
{
    /* Indicate TX reset complete */
    s->sebregs[HME_SEBI_RESET] &= ~HME_SEB_RESET_ETX;
}

static void hme_reset_rx(HMEState *s)
{
    /* Indicate RX reset complete */
    s->sebregs[HME_SEBI_RESET] &= ~HME_SEB_RESET_ERX;
}

static void hme_seb_write(void *opaque, hwaddr addr,
                          uint64_t val, unsigned size)
{
    HMEState *s = HME(opaque);

    DPRINTF("hme_seb_write %" HWADDR_PRIx " %lx\n", addr, val);

    switch (addr) {
    case HME_SEBI_RESET:
        if (val & HME_SEB_RESET_ETX) {
            hme_reset_tx(s);
        }
        if (val & HME_SEB_RESET_ERX) {
            hme_reset_rx(s);
        }
        val = s->sebregs[HME_SEBI_RESET >> 2];
        break;
    }

    s->sebregs[addr >> 2] = val; 
}

static uint64_t hme_seb_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    HMEState *s = HME(opaque);

    DPRINTF("hme_seb_read %" HWADDR_PRIx "\n", addr);

    return s->sebregs[addr >> 2];
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
    HMEState *s = HME(opaque);

    DPRINTF("hme_etx_write %" HWADDR_PRIx " %lx\n", addr, val);
    
    s->etxregs[addr >> 2] = val;
}

static uint64_t hme_etx_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    HMEState *s = HME(opaque);

    DPRINTF("hme_etx_read %" HWADDR_PRIx "\n", addr);

    return s->etxregs[addr >> 2];
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
    HMEState *s = HME(opaque);

    DPRINTF("hme_erx_write %" HWADDR_PRIx " %lx\n", addr, val);
    
    s->erxregs[addr >> 2] = val;
}

static uint64_t hme_erx_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    HMEState *s = HME(opaque);

    DPRINTF("hme_erx_read %" HWADDR_PRIx "\n", addr);

    return s->erxregs[addr >> 2];    return 0;
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
    HMEState *s = HME(opaque);

    DPRINTF("hme_mac_write %" HWADDR_PRIx " %lx\n", addr, val);
    
    s->macregs[addr >> 2] = val;
}

static uint64_t hme_mac_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    HMEState *s = HME(opaque);

    DPRINTF("hme_mac_read %" HWADDR_PRIx "\n", addr);

    return s->macregs[addr >> 2];
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

static void hme_mii_write(HMEState *s, uint8_t reg, uint16_t data)
{
    DPRINTF("hme_mii_write %x %x\n", reg, data);
    
    switch (reg) {
    case MII_BMCR:
        if (data & MII_BMCR_RESET) {
            /* Autoclear reset bit, enable auto negotiation */
            data &= ~MII_BMCR_RESET;
            data |= MII_BMCR_AUTOEN;
        }        
        if (data & MII_BMCR_ANRESTART) {
            /* Autoclear auto negotiation restart */
            data &= ~MII_BMCR_ANRESTART;

            /* Indicate negotiation complete at 100Mbps FD */
            s->miiregs[MII_ANLPAR] |= MII_ANLPAR_TXFD;
            s->miiregs[MII_BMSR] |= (MII_BMSR_AN_COMP | MII_BMSR_LINK_ST);
        }
        break;
    }
    
    s->miiregs[reg] = data;
}

static uint16_t hme_mii_read(HMEState *s, uint8_t reg)
{
    DPRINTF("hme_mii_read %x %x\n", reg, s->miiregs[reg]);

    return s->miiregs[reg];
}

static void hme_mif_write(void *opaque, hwaddr addr,
                          uint64_t val, unsigned size)
{
    HMEState *s = HME(opaque);
    uint8_t cmd, reg;
    uint16_t data;
    
    DPRINTF("hme_mif_write %" HWADDR_PRIx " %lx\n", addr, val);

    switch (addr) {
    case HME_MIFI_CFG:
        /* Mask the read-only bits */
        val &= ~(HME_MIF_CFG_MDI0 | HME_MIF_CFG_MDI1);
        val |= s->mifregs[HME_MIFI_CFG >> 2] &
               (HME_MIF_CFG_MDI0 | HME_MIF_CFG_MDI1);
        break;

    case HME_MIFI_FO:
        /* Detect start of MII command */
        if ((val & HME_MIF_FO_ST) >> HME_MIF_FO_ST_SHIFT
            != MII_COMMAND_START) {
            break;
        }
        
        /* Internal phy only */
        if ((val & HME_MIF_FO_PHYAD) >> HME_MIF_FO_PHYAD_SHIFT
            != HME_PHYAD_INTERNAL) {
            break;
        }
        
        cmd = (val & HME_MIF_FO_OPC) >> HME_MIF_FO_OPC_SHIFT;
        reg = (val & HME_MIF_FO_REGAD) >> HME_MIF_FO_REGAD_SHIFT;
        data = (val & HME_MIF_FO_DATA);

        switch (cmd) {
        case MII_COMMAND_WRITE:
            hme_mii_write(s, reg, data);
            break;

        case MII_COMMAND_READ:
            val &= ~HME_MIF_FO_DATA;
            val |= hme_mii_read(s, reg);
            break;
        }

        val |= HME_MIF_FO_TALSB;        
        break;
    }

    s->mifregs[addr >> 2] = val;
}

static uint64_t hme_mif_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    HMEState *s = HME(opaque);
    uint64_t val;

    val = s->mifregs[addr >> 2];
    
    DPRINTF("hme_mif_read %" HWADDR_PRIx " %lx\n", addr, val);
    return val;
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
    
    memory_region_init_io(&s->sebreg, OBJECT(pci_dev), &hme_seb_ops, s,
                          "hme.seb", HME_SEB_REG_SIZE);
    memory_region_add_subregion(&s->hme, 0, &s->sebreg);

    memory_region_init_io(&s->etxreg, OBJECT(pci_dev), &hme_etx_ops, s,
                          "hme.etx", HME_ETX_REG_SIZE);
    memory_region_add_subregion(&s->hme, 0x2000, &s->etxreg);

    memory_region_init_io(&s->erxreg, OBJECT(pci_dev), &hme_erx_ops, s,
                          "hme.erx", HME_ERX_REG_SIZE);
    memory_region_add_subregion(&s->hme, 0x4000, &s->erxreg);

    memory_region_init_io(&s->macreg, OBJECT(pci_dev), &hme_mac_ops, s,
                          "hme.mac", HME_MAC_REG_SIZE);
    memory_region_add_subregion(&s->hme, 0x6000, &s->macreg);

    memory_region_init_io(&s->mifreg, OBJECT(pci_dev), &hme_mif_ops, s,
                          "hme.mif", HME_MIF_REG_SIZE);
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

static void hme_reset(DeviceState *ds)
{
    HMEState *s = HME(ds);
    
    printf("hme reset!\n");

    /* Configure internal transceiver */
    s->mifregs[HME_MIFI_CFG >> 2] |= HME_MIF_CFG_MDI0;
    
    /* Advetise 100Mbps FD */
    s->miiregs[MII_ANAR] = MII_ANAR_TXFD;
    s->miiregs[MII_BMSR] = MII_BMSR_100TX_FD;
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
    dc->reset = hme_reset;
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
