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
#include "net/eth.h"

#define HME_REG_SIZE                   0x8000

#define HME_SEB_REG_SIZE               0x2000

#define HME_SEBI_RESET                 0x0
#define HME_SEB_RESET_ETX              0x1
#define HME_SEB_RESET_ERX              0x2

//#define HME_SEBI_STAT                  0x100
#define HME_SEBI_STAT                  0x108
#define HME_SEB_STAT_RXTOHOST          0x10000    /* pkt moved from rx fifo->memory */
#define HME_SEB_STAT_MIFIRQ            0x800000   /* mif needs attention */
#define HME_SEB_STAT_HOSTTOTX          0x1000000  /* pkt moved from memory->tx fifo */
#define HME_SEB_STAT_TXALL             0x2000000  /* all pkts in fifo transmitted */

//#define HME_SEBI_IMASK                 0x104
#define HME_SEBI_IMASK                 0x10c

#define HME_ETX_REG_SIZE               0x2000

#define HME_ETXI_PENDING               0x0     /* Pending/wakeup */

#define HME_ETXI_RING                  0x8     /* Descriptor Ring pointer */
#define HME_ETXI_RING_ADDR             0xffffff00
#define HME_ETXI_RING_OFFSET           0xff

#define HME_ETXI_RSIZE                 0x2c    /* Ring size */

#define HME_ERX_REG_SIZE               0x2000

#define HME_ERXI_CFG                   0x0
#define HME_ERX_CFG_RINGSIZE           0x300
#define HME_ERX_CFG_RINGSIZE_SHIFT     9
#define HME_ERX_CFG_BYTEOFFSET         0x38    /* RX first byte offset */
#define HME_ERX_CFG_BYTEOFFSET_SHIFT   3

#define HME_ERXI_RING                  0x4
#define HME_ERXI_RING_ADDR             0xffffff00
#define HME_ERXI_RING_OFFSET           0xff

#define HME_MAC_REG_SIZE               0x1000

#define HME_MIF_REG_SIZE               0x20

#define HME_MIFI_FO                    0xc
#define HME_MIF_FO_ST                  0xc0000000    /* Start of frame */
#define HME_MIF_FO_ST_SHIFT            30
#define HME_MIF_FO_OPC                 0x30000000    /* Opcode */
#define HME_MIF_FO_OPC_SHIFT           28
#define HME_MIF_FO_PHYAD               0x0f800000    /* PHY Address */
#define HME_MIF_FO_PHYAD_SHIFT         23
#define HME_MIF_FO_REGAD               0x007c0000    /* Register Address */
#define HME_MIF_FO_REGAD_SHIFT         18
#define HME_MIF_FO_TAMSB               0x20000    /* Turn-around MSB */
#define HME_MIF_FO_TALSB               0x10000    /* Turn-around LSB */
#define HME_MIF_FO_DATA                0xffff     /* data to read or write */

#define HME_MIFI_CFG                   0x10
#define HME_MIF_CFG_MDI0               0x100
#define HME_MIF_CFG_MDI1               0x200 

#define HME_MIFI_IMASK                 0x14

#define HME_MIFI_STAT                  0x18        /* Status (ro, auto-clear) */


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

/* All descriptor values are stored BE */
typedef struct HMEDesc {
    uint32_t status;
    uint32_t buffer;
} HMEDesc;

/* Maximum size of buffer */
#define HME_FIFO_SIZE          0x800

/* Size of TX/RX descriptor */
#define HME_DESC_SIZE          0x8

#define HME_XD_OWN             0x80000000    /* ownership: 1=hw, 0=sw */
#define HME_XD_OFL             0x40000000    /* buffer overflow (rx) */
#define HME_XD_RXLENMSK        0x3fff0000    /* packet length mask (rx) */
#define HME_XD_RXLENSHIFT      16
#define HME_XD_TXLENMSK        0x00001fff

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

static void hme_update_irq(HMEState *s)
{
    PCIDevice *d = PCI_DEVICE(s);
    int level;
    
    /* MIF interrupt mask (16-bit) */
    uint32_t mifmask = ~(s->mifregs[HME_MIFI_IMASK >> 2]) & 0xffff;
    uint32_t mif = s->mifregs[HME_MIFI_STAT >> 2] & mifmask;

    DPRINTF("irq mif: %x\n", mif);
    
    /* Main SEB interrupt mask (include MIF status from above) */
    uint32_t sebimask = s->sebregs[HME_SEBI_IMASK >> 2];
    DPRINTF("sebimask: %x\n", sebimask);
    uint32_t sebimask2 = ~(s->sebregs[HME_SEBI_IMASK >> 2]);
    DPRINTF("sebimask2: %x\n", sebimask2);
    uint32_t sebstat = s->sebregs[HME_SEBI_STAT >> 2];
    DPRINTF("sebstat: %x\n", sebstat);
    
    uint32_t sebmask = ~(s->sebregs[HME_SEBI_IMASK >> 2]) &
                       ~HME_SEB_STAT_MIFIRQ;
    uint32_t seb = s->sebregs[HME_SEBI_STAT >> 2] & sebmask;
    if (mif) {
        seb |= HME_SEB_STAT_MIFIRQ;
    }
    
    level = (seb ? 1 : 0);
    
    DPRINTF("irq level: %x  seb: %x\n", level, seb);
    pci_set_irq(d, level);
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
    uint64_t val = s->sebregs[addr >> 2];
    
    switch (addr) {
    case HME_SEBI_STAT:
        /* Autoclear status (except MIF) */
        s->sebregs[HME_SEBI_STAT >> 2] &= HME_SEB_STAT_MIFIRQ;
        hme_update_irq(s);
        break;
    }
    
    DPRINTF("hme_seb_read %" HWADDR_PRIx " %lx\n", addr, val);

    return val;
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

static void hme_transmit(HMEState *s);

static void hme_etx_write(void *opaque, hwaddr addr,
                          uint64_t val, unsigned size)
{
    HMEState *s = HME(opaque);

    DPRINTF("hme_etx_write %" HWADDR_PRIx " %lx\n", addr, val);
    
    switch (addr) {
    case HME_ETXI_PENDING:
        if (val) {
            hme_transmit(s);
	}
        break;
    }
    
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
    
    switch (addr) {
    case HME_MIFI_STAT:
        /* Autoclear MIF interrupt status */
        s->mifregs[HME_MIFI_STAT >> 2] = 0;
        hme_update_irq(s);
        break;
    }
    
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

static void hme_transmit_frame(HMEState *s, uint8_t *buf, int size)
{
    qemu_send_packet(qemu_get_queue(s->nic), buf, size);
}

static inline int hme_get_tx_ring_count(HMEState *s)
{
    return s->etxregs[HME_ETXI_RSIZE];
}

static inline int hme_get_tx_ring_nr(HMEState *s)
{
    return s->etxregs[HME_ETXI_RING >> 2] & HME_ETXI_RING_OFFSET;
}

static inline void hme_set_tx_ring_nr(HMEState *s, int i)
{
    uint32_t ring = s->etxregs[HME_ETXI_RING >> 2] & ~HME_ETXI_RING_OFFSET;
    ring |= i & HME_ETXI_RING_OFFSET;

    s->etxregs[HME_ETXI_RING >> 2] = ring;
}

static void hme_transmit(HMEState *s)
{
    PCIDevice *d = PCI_DEVICE(s);
    dma_addr_t tb, addr;
    uint32_t status, buffer;
    int cr, len, count;
    uint8_t xmit_buffer[HME_FIFO_SIZE];

    DPRINTF("hme_transmit!\n");
    
    tb = s->etxregs[HME_ETXI_RING >> 2] & HME_ETXI_RING_ADDR;
    cr = hme_get_tx_ring_nr(s);
    
    DPRINTF("tb " DMA_ADDR_FMT "  %d\n", tb, cr);
    
    pci_dma_read(d, tb + cr * HME_DESC_SIZE, &status, 4);
    pci_dma_read(d, tb + cr * HME_DESC_SIZE + 4, &buffer, 4);

    DPRINTF("-- status: %" PRIx32 "\n", status);
    
    count = 0;
    while ((status & HME_XD_OWN) && count <= hme_get_tx_ring_count(s)) {
        /* Copy data into transmit buffer */
        addr = buffer;
        len = status & HME_XD_TXLENMSK;

        // TODO: Tx overflow? In sebregs
        if (len > HME_FIFO_SIZE) {
            len = HME_FIFO_SIZE;
        }

        DPRINTF("  addr: " DMA_ADDR_FMT " len: %d\n", addr, len);
        pci_dma_read(d, addr, &xmit_buffer, len);

        hme_transmit_frame(s, xmit_buffer, len);

        /* Return descriptor back to OS */
        status &= ~HME_XD_OWN;
        pci_dma_write(d, tb + cr * HME_DESC_SIZE, &status, 4);

        cr++;
        if (cr >= hme_get_tx_ring_count(s)) {
            cr = 0;
        }

        hme_set_tx_ring_nr(s, cr);

        /* Indicate TX complete */
        uint32_t intstatus = s->sebregs[HME_SEBI_STAT >> 2];
        intstatus |= HME_SEB_STAT_HOSTTOTX;
        s->sebregs[HME_SEBI_STAT >> 2] = intstatus;

        /* Autoclear TX pending */
        s->etxregs[HME_ETXI_PENDING >> 2] = 0;

        hme_update_irq(s);
        count++;
    }
    
    /* TX FIFO now clear */
    uint32_t intstatus = s->sebregs[HME_SEBI_STAT >> 2];
    intstatus |= HME_SEB_STAT_TXALL;
    s->sebregs[HME_SEBI_STAT >> 2] = intstatus;
    hme_update_irq(s);
}

static int hme_can_receive(NetClientState *nc)
{
    return 1;
}

static void hme_set_link_status(NetClientState *nc)
{
    return;
}

static inline int hme_get_rx_ring_count(HMEState *s)
{
    uint32_t rings = (s->erxregs[HME_ERXI_CFG >> 2] & HME_ERX_CFG_RINGSIZE)
                      >> HME_ERX_CFG_RINGSIZE_SHIFT;

    switch (rings) {
    case 0:
        return 32;
    case 1:
        return 64;
    case 2:
        return 128;
    case 3:
        return 256;
    }

    return 0;
}

static inline int hme_get_rx_ring_nr(HMEState *s)
{
    return s->erxregs[HME_ERXI_RING >> 2] & HME_ERXI_RING_OFFSET;
}

static inline void hme_set_rx_ring_nr(HMEState *s, int i)
{
    uint32_t ring = s->erxregs[HME_ERXI_RING >> 2] & ~HME_ERXI_RING_OFFSET;
    ring |= i & HME_ERXI_RING_OFFSET;

    s->erxregs[HME_ERXI_RING >> 2] = ring;
}

#define MIN_BUF_SIZE 60

static ssize_t hme_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    DPRINTF("RECEIVED PACKET %zu\n", size);

    HMEState *s = qemu_get_nic_opaque(nc);
    PCIDevice *d = PCI_DEVICE(s);
    dma_addr_t rb, addr, rxoffset;
    uint32_t status, buffer, buffersize;
    uint8_t buf1[60];
    int nr, cr, len;

    /* if too small buffer, then expand it */
    if (size < MIN_BUF_SIZE) {
        memcpy(buf1, buf, size);
        memset(buf1 + size, 0, MIN_BUF_SIZE - size);
        buf = buf1;
        size = MIN_BUF_SIZE;
    }
    
    rb = s->erxregs[HME_ERXI_RING >> 2] & HME_ERXI_RING_ADDR;
    nr = hme_get_rx_ring_count(s);
    cr = hme_get_rx_ring_nr(s);
    
    DPRINTF("rb " DMA_ADDR_FMT "  %d  (nr: %d)\n", rb, cr, nr);

    pci_dma_read(d, rb + cr * HME_DESC_SIZE, &status, 4);
    pci_dma_read(d, rb + cr * HME_DESC_SIZE + 4, &buffer, 4);

    rxoffset = (s->erxregs[HME_ERXI_CFG >> 2] & HME_ERX_CFG_BYTEOFFSET) >>
                HME_ERX_CFG_BYTEOFFSET_SHIFT;

    addr = buffer + rxoffset;
    buffersize = (status & HME_XD_RXLENMSK) >> HME_XD_RXLENSHIFT;
    
    /* Detect receive overflow */
    len = size;
    if (size > buffersize) {
        status |= HME_XD_OFL;
        len = buffersize;
    }

    DPRINTF("desc address " DMA_ADDR_FMT " - status %x with buffersize %d\n", addr, status, buffersize);
    pci_dma_write(d, addr, buf, len);

    struct ip_header *ip = (struct ip_header *)buf;
    uint8_t ip_protocol = ip->ip_p;
    
    DPRINTF("protocol is %x\n", ip_protocol);

    
    /* Update status owner */
    status &= ~HME_XD_OWN;
    status &= ~HME_XD_RXLENMSK;
    status |= len << HME_XD_RXLENSHIFT;

    DPRINTF("status is now %x\n", status);
    
    pci_dma_write(d, rb + cr * HME_DESC_SIZE, &status, 4);

    cr++;
    if (cr >= hme_get_rx_ring_count(s)) {
        cr = 0;
    }

    hme_set_rx_ring_nr(s, cr);
    
    /* Indicate RX complete */
    uint32_t intstatus = s->sebregs[HME_SEBI_STAT >> 2];
    intstatus |= HME_SEB_STAT_RXTOHOST;
    s->sebregs[HME_SEBI_STAT >> 2] = intstatus;

    hme_update_irq(s);

    return len;
}

static NetClientInfo net_hme_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = hme_can_receive,
    .receive = hme_receive,
    .link_status_changed = hme_set_link_status,
};

static void hme_realize(PCIDevice *pci_dev, Error **errp)
{
    HMEState *s = HME(pci_dev);
    DeviceState *d = DEVICE(pci_dev);
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

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_hme_info, &s->conf,
                          object_get_typename(OBJECT(d)), d->id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
    
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
    
    /* Configure default interrupt mask */
    s->mifregs[HME_MIFI_IMASK >> 2] = 0xffff;
    s->sebregs[HME_SEBI_IMASK >> 2] = 0xff7fffff;
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
