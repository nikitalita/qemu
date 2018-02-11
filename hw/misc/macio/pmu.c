/*
 * QEMU PowerMac PMU device support
 *
 * Copyright (c) 2016 Benjamin Herrenschmidt, IBM Corp.
 * Copyright (c) 2018 Mark Cave-Ayland
 *
 * Based on the CUDA device by:
 *
 * Copyright (c) 2004-2007 Fabrice Bellard
 * Copyright (c) 2007 Jocelyn Mayer
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
#include "hw/ppc/mac.h"
#include "hw/input/adb.h"
#include "hw/misc/mos6522.h"
#include "hw/misc/macio/pmu.h"
#include "qemu/timer.h"
#include "sysemu/sysemu.h"
#include "qemu/cutils.h"
#include "qemu/log.h"

/* XXX: implement all timer modes */

#undef DEBUG_PMU
#undef DEBUG_PMU_AUTOPOLL
#undef DEBUG_PMU_ALL_MMIO
#undef DEBUG_PMU_PROTOCOL
#undef DEBUG_VIA

/* debug PMU packets */
#define DEBUG_PMU_PACKET

#ifdef DEBUG_PMU
#define PMU_DPRINTF(fmt, ...)                                  \
    do { printf("PMU: " fmt , ## __VA_ARGS__); } while (0)
#else
#define PMU_DPRINTF(fmt, ...)
#endif

/* Bits in B data register: all active low */
#define TACK    0x08    /* Transfer request (input) */
#define TREQ    0x10    /* Transfer acknowledge (output) */

/* PMU returns time_t's offset from Jan 1, 1904, not 1970 */
#define RTC_OFFSET                      2082844800

/* XXX FIXME */
#define VIA_TIMER_FREQ (4700000 / 6)

static void via_update_irq(PMUState *s)
{
    MOS6522PMUState *mps = MOS6522_PMU(s->mos6522_pmu);
    MOS6522State *ms = MOS6522(mps);

    bool new_state = !!(ms->ifr & ms->ier & (SR_INT | T1_INT | T2_INT));

    if (new_state != s-> via_irq_state) {
        s-> via_irq_state = new_state;
        qemu_set_irq(s->via_irq, new_state);
    }
}

static void via_set_sr_int(void *opaque)
{
    PMUState *s = opaque;
    MOS6522PMUState *mps = s->mos6522_pmu;
    MOS6522State *ms = MOS6522(mps);
    MOS6522DeviceClass *mdc = MOS6522_DEVICE_GET_CLASS(ms);

    mdc->set_sr_int(ms);
}

static void pmu_update_extirq(PMUState *s)
{
    if ((s->intbits & s->intmask) != 0) {
        MacIOSetGPIO(s->macio, 1, false);
    } else {
        MacIOSetGPIO(s->macio, 1, true);
    }
}

static void pmu_adb_poll(void *opaque)
{
    PMUState *s = opaque;
    int olen;

    if (!(s->intbits & PMU_INT_ADB)) {
        /* XXX Check this */
        olen = adb_poll(&s->adb_bus, s->adb_reply, s->adb_poll_mask);
#ifdef DEBUG_PMU_AUTOPOLL
        PMU_DPRINTF("ADB autopoll, olen=%d\n", olen);
#endif
        if (olen > 0) {
            s->adb_reply_size = olen;
            s->intbits |= PMU_INT_ADB | PMU_INT_ADB_AUTO;
            pmu_update_extirq(s);
        }
    }
    timer_mod(s->adb_poll_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 30);
}

static void pmu_one_sec_timer(void *opaque)
{
    PMUState *s = opaque;

    PMU_DPRINTF("PMU one sec...\n");
    s->intbits |= PMU_INT_TICK;
    pmu_update_extirq(s);
    s->one_sec_target += 1000;
    timer_mod(s->one_sec_timer, s->one_sec_target);
}

static void pmu_cmd_int_ack(PMUState *s,
                            const uint8_t *in_data, uint8_t in_len,
                            uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 0) {
        PMU_DPRINTF("INT_ACK command, invalid len: %d want: 0\n", in_len);
        return;
    }
    /* Make appropriate reply packet */
    if (s->intbits & PMU_INT_ADB) {
        if (!s->adb_reply_size) {
            PMU_DPRINTF("Odd, PMU_INT_ADB set with no reply in buffer\n");
        }
        memcpy(out_data + 1, s->adb_reply, s->adb_reply_size);
        out_data[0] = s->intbits & (PMU_INT_ADB | PMU_INT_ADB_AUTO);
        *out_len = s->adb_reply_size + 1;
        s->intbits &= ~(PMU_INT_ADB | PMU_INT_ADB_AUTO);
        s->adb_reply_size = 0;
    } else {
        out_data[0] = s->intbits;
        s->intbits = 0;
        *out_len = 1;
    }
    pmu_update_extirq(s);
}

static void pmu_cmd_set_int_mask(PMUState *s,
                                 const uint8_t *in_data, uint8_t in_len,
                                 uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 1) {
        PMU_DPRINTF("SET_INT_MASK command, invalid len: %d want: 1\n", in_len);
        return;
    }

    PMU_DPRINTF("Setting PMU int mask to 0x%02x\n", s->intmask);
    s->intmask = in_data[0];
    pmu_update_extirq(s);
}

static void pmu_cmd_set_adb_autopoll(PMUState *s, uint16_t mask)
{
    PMU_DPRINTF("ADB set autopoll, mask=%04x\n", mask);

    if (s->autopoll_mask == mask) {
        return;
    }
    s->autopoll_mask = mask;
    if (mask) {
        timer_mod(s->adb_poll_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 30);
    } else {
        timer_del(s->adb_poll_timer);
    }
}

static void pmu_cmd_adb(PMUState *s,
                        const uint8_t *in_data, uint8_t in_len,
                        uint8_t *out_data, uint8_t *out_len)
{
    int len, adblen;
    uint8_t adb_cmd[255];

    if (in_len < 2) {
        PMU_DPRINTF("ADB PACKET, invalid len: %d want at least 2\n", in_len);
        return;
    }

    *out_len = 0;

    if (!s->has_adb) {
        PMU_DPRINTF("ADB PACKET with no ADB bus !\n");
        return;
    }
    /* Set autopoll is a special form of the command */
    if (in_data[0] == 0 && in_data[1] == 0x86) {
        uint16_t mask = in_data[2];
        mask = (mask << 8) | in_data[3];
        if (in_len != 4) {
                PMU_DPRINTF("ADB Autopoll requires 4 bytes, got %d\n", in_len);
                return;
        }
        pmu_cmd_set_adb_autopoll(s, mask);
        return;
    }
    PMU_DPRINTF("ADB request: "
                "len=%d,cmd=0x%02x,pflags=0x%02x,adblen=%d: %02x %02x...\n",
                in_len, in_data[0], in_data[1],
                in_data[2], in_data[3], in_data[4]);
    *out_len = 0;

    /* Check ADB len */
    adblen = in_data[2];
    if (adblen > (in_len - 3)) {
        PMU_DPRINTF("ADB len is %d > %d (in_len -3)...erroring \n",
                    adblen, in_len - 3);
        len = -1;
    } else if (adblen > 252) {
        PMU_DPRINTF("ADB command too big !\n");
        len = -1;
    } else {
        /* Format command */
        adb_cmd[0] = in_data[0];
        memcpy(&adb_cmd[1], &in_data[3], in_len - 3);
        len = adb_request(&s->adb_bus, s->adb_reply + 2, adb_cmd, in_len - 2);
        PMU_DPRINTF("ADB reply is %d bytes\n", len);
    }
    if (len > 0) {
        /* XXX Check this */
        s->adb_reply_size = len + 2;
        s->adb_reply[0] = 0x01;
        s->adb_reply[1] = len;
    } else {
        /* XXX Check this */
        s->adb_reply_size = 1;
        s->adb_reply[0] = 0x00;
    }
    s->intbits |= PMU_INT_ADB;
    pmu_update_extirq(s);
}

static void pmu_cmd_adb_poll_off(PMUState *s,
                                 const uint8_t *in_data, uint8_t in_len,
                                 uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 0) {
        PMU_DPRINTF("ADB POLL OFF command, invalid len: %d want: 0\n", in_len);
        return;
    }

    if (s->has_adb && s->autopoll_mask) {
        timer_del(s->adb_poll_timer);
        s->autopoll_mask = false;
    }
}

static void pmu_cmd_shutdown(PMUState *s,
                             const uint8_t *in_data, uint8_t in_len,
                             uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 4) {
        PMU_DPRINTF("SHUTDOWN command, invalid len: %d want: 4\n", in_len);
        return;
    }
    *out_len = 1;
    out_data[0] = 0;
    if (in_data[0] != 'M' || in_data[1] != 'A' ||
        in_data[2] != 'T' || in_data[3] != 'T') {
        PMU_DPRINTF("SHUTDOWN command, Bad MATT signature\n");
        return;
    }
    qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
}

static void pmu_cmd_reset(PMUState *s,
                          const uint8_t *in_data, uint8_t in_len,
                          uint8_t *out_data, uint8_t *out_len)
{
    if (in_len != 0) {
        PMU_DPRINTF("RESET command, invalid len: %d want: 0\n", in_len);
        return;
    }

    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void pmu_cmd_get_rtc(PMUState *s,
                            const uint8_t *in_data, uint8_t in_len,
                            uint8_t *out_data, uint8_t *out_len)
{
    uint32_t ti;

    if (in_len != 0) {
        PMU_DPRINTF("GET_RTC command, invalid len: %d want: 0\n", in_len);
        return;
    }

    ti = s->tick_offset + (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                           / NANOSECONDS_PER_SECOND);
    out_data[0] = ti >> 24;
    out_data[1] = ti >> 16;
    out_data[2] = ti >> 8;
    out_data[3] = ti;
    *out_len = 4;
}

static void pmu_cmd_set_rtc(PMUState *s,
                            const uint8_t *in_data, uint8_t in_len,
                            uint8_t *out_data, uint8_t *out_len)
{
    uint32_t ti;

    if (in_len != 4) {
        PMU_DPRINTF("SET_RTC command, invalid len: %d want: 4\n", in_len);
        return;
    }

    ti = (((uint32_t)in_data[0]) << 24) + (((uint32_t)in_data[1]) << 16)
         + (((uint32_t)in_data[2]) << 8) + in_data[3];
    s->tick_offset = ti - (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                           / NANOSECONDS_PER_SECOND);
}

static void pmu_cmd_system_ready(PMUState *s,
                                 const uint8_t *in_data, uint8_t in_len,
                                 uint8_t *out_data, uint8_t *out_len)
{
        /* Do nothing */
}

static void pmu_cmd_get_version(PMUState *s,
                                const uint8_t *in_data, uint8_t in_len,
                                uint8_t *out_data, uint8_t *out_len)
{
    *out_len = 1;
    *out_data = 1; /* ??? Check what Apple does */
}

static void pmu_cmd_power_events(PMUState *s,
                                 const uint8_t *in_data, uint8_t in_len,
                                 uint8_t *out_data, uint8_t *out_len)
{
    if (in_len < 1) {
        PMU_DPRINTF("POWER EVENTS command, invalid len %d, want at least 1\n",
                    in_len);
        return;
    }
    switch(in_data[0]) {
    /* Dummies for now */
    case PMU_PWR_GET_POWERUP_EVENTS:
        *out_len = 2;
        out_data[0] = 0;
        out_data[1] = 0;
        break;
    case PMU_PWR_SET_POWERUP_EVENTS:
    case PMU_PWR_CLR_POWERUP_EVENTS:
        break;
    case PMU_PWR_GET_WAKEUP_EVENTS:
        *out_len = 2;
        out_data[0] = 0;
        out_data[1] = 0;
        break;
    case PMU_PWR_SET_WAKEUP_EVENTS:
    case PMU_PWR_CLR_WAKEUP_EVENTS:
        break;
    default:
        PMU_DPRINTF("POWER EVENTS unknown subcommand 0x%02x\n",
                    in_data[0]);
    }
}

static void pmu_cmd_get_cover(PMUState *s,
                              const uint8_t *in_data, uint8_t in_len,
                              uint8_t *out_data, uint8_t *out_len)
{
    /* Not 100% sure here, will have to check what a real Mac
     * returns other than byte 0 bit 0 is LID closed on laptops
     */
    *out_len = 1;
    *out_data = 0x00;
}

static void pmu_cmd_download_status(PMUState *s,
                                    const uint8_t *in_data, uint8_t in_len,
                                    uint8_t *out_data, uint8_t *out_len)
{
    /* This has to do with PMU firmware updates as far as I can tell.
     *
     * We return 0x62 which is what OpenPMU expects
     */
    *out_len = 1;
    *out_data = 0x62;
}

static void pmu_cmd_read_pmu_ram(PMUState *s,
                                 const uint8_t *in_data, uint8_t in_len,
                                 uint8_t *out_data, uint8_t *out_len)
{
    if (in_len < 3) {
        PMU_DPRINTF("READ_PMU_RAM command, invalid len %d, expected 3\n",
                    in_len);
        return;
    }
    PMU_DPRINTF("Unsupported READ_PMU_RAM, args: %02x %02x %02x\n",
                in_data[0], in_data[1], in_data[2]);
    *out_len = 0;
}

/* description of commands */
typedef struct PMUCmdHandler {
    uint8_t command;
    const char *name;
    void (*handler)(PMUState *s,
                    const uint8_t *in_args, uint8_t in_len,
                    uint8_t *out_args, uint8_t *out_len);
} PMUCmdHandler;

static const PMUCmdHandler PMUCmdHandlers[] = {
    { PMU_INT_ACK, "INT ACK", pmu_cmd_int_ack },
    { PMU_SET_INTR_MASK, "SET INT MASK", pmu_cmd_set_int_mask },
    { PMU_ADB_CMD, "ADB COMMAND", pmu_cmd_adb },
    { PMU_ADB_POLL_OFF, "ADB POLL OFF", pmu_cmd_adb_poll_off },
    { PMU_RESET, "REBOOT", pmu_cmd_reset },
    { PMU_SHUTDOWN, "SHUTDOWN", pmu_cmd_shutdown },
    { PMU_READ_RTC, "GET RTC", pmu_cmd_get_rtc },
    { PMU_SET_RTC, "SET RTC", pmu_cmd_set_rtc },
    { PMU_SYSTEM_READY, "SYSTEM READY", pmu_cmd_system_ready },
    { PMU_GET_VERSION, "GET VERSION", pmu_cmd_get_version },
    { PMU_POWER_EVENTS, "POWER EVENTS", pmu_cmd_power_events },
    { PMU_GET_COVER, "GET_COVER", pmu_cmd_get_cover },
    { PMU_DOWNLOAD_STATUS, "DOWNLOAD STATUS", pmu_cmd_download_status },
    { PMU_READ_PMU_RAM, "READ PMGR RAM", pmu_cmd_read_pmu_ram },
    // .../...
};

static void pmu_dispatch_cmd(PMUState *s)
{
    unsigned int i;

    /* No response by default */
    s->cmd_rsp_sz = 0;

    for (i = 0; i < ARRAY_SIZE(PMUCmdHandlers); i++) {
        const PMUCmdHandler *desc = &PMUCmdHandlers[i];
        if (desc->command != s->cmd) {
            continue;
        }
        PMU_DPRINTF("handling command %s\n", desc->name);
        desc->handler(s, s->cmd_buf, s->cmd_buf_pos,
                      s->cmd_rsp, &s->cmd_rsp_sz);
        if (s->rsplen != -1 && s->rsplen != s->cmd_rsp_sz) {
            PMU_DPRINTF("Qemu internal cmd resp mismatch !\n");
        } else {
#ifdef DEBUG_PMU_PROTOCOL
            PMU_DPRINTF("sending %d resp bytes\n", s->cmd_rsp_sz);
#endif
        }
        return;
    }
    PMU_DPRINTF("Unknown PMU command %02x !\n", s->cmd);

    /* Manufacture fake response with 0's */
    if (s->rsplen == -1) {
            s->cmd_rsp_sz = 0;
    } else {
            s->cmd_rsp_sz = s->rsplen;
            memset(s->cmd_rsp, 0, s->rsplen);
    }
}

static void pmu_update(PMUState *s)
{
    MOS6522PMUState *mps = s->mos6522_pmu;
    MOS6522State *ms = MOS6522(mps);
    
    /* Only react to changes in reg B */
    if (ms->b == s->last_b) {
        return;
    }
    s->last_b = ms->b;

    /* Check the TREQ / TACK state */
    switch (ms->b & (TREQ | TACK)) {
    case TREQ:
        /* This is an ack release, handle it and bail out */
        ms->b |= TACK;
        s->last_b = ms->b;
#ifdef DEBUG_PMU_PROTOCOL
        PMU_DPRINTF("handshake: TREQ high, setting TACK\n");
#endif
        return;
    case TACK:
        /* This is a valid request, handle below */
        break;
    case TREQ | TACK:
        /* This is an idle state */
        return;
    default:
        /* Invalid state, log and ignore */
        PMU_DPRINTF("protocol error ! portB=0x%02x\n", ms->b);
        return;
    }

    /* If we wanted to handle commands asynchronously, this is where
     * we would delay the clearing of TACK until we are ready to send
     * the response
     */

    /* We have a request, handshake TACK so we don't stay in
     * an invalid state. If we were concurrent with the OS we
     * should only do this after we grabbed the SR but that isn't
     * a problem here.
     */
#ifdef DEBUG_PMU_PROTOCOL
    PMU_DPRINTF("TREQ cleared, clearing TACK, state: %d\n", s->cmd_state);
#endif
    ms->b &= ~TACK;
    s->last_b = ms->b;

    /* Act according to state */
    switch(s->cmd_state) {
    case pmu_state_idle:
        if (!(ms->acr & SR_OUT)) {
            PMU_DPRINTF("protocol error ! state idle, ACR reading\n");
            break;
        }
        s->cmd = ms->sr;
        via_set_sr_int(s);
        s->cmdlen = pmu_data_len[s->cmd][0];
        s->rsplen = pmu_data_len[s->cmd][1];
        s->cmd_buf_pos = 0;
        s->cmd_rsp_pos = 0;
        s->cmd_state = pmu_state_cmd;
#ifdef DEBUG_PMU_PROTOCOL
        PMU_DPRINTF("Got command byte %02x, clen=%d,rlen=%d\n",
                    s->cmd, s->cmdlen, s->rsplen);
#endif
        break;
    case pmu_state_cmd:
        if (!(ms->acr & SR_OUT)) {
            PMU_DPRINTF("protocol error ! state cmd, ACR reading\n");
            break;
        }
        if (s->cmdlen == -1) {
#ifdef DEBUG_PMU_PROTOCOL
            PMU_DPRINTF("got cmd length byte: %d\n", ms->sr);
#endif
            s->cmdlen = ms->sr;
            if (s->cmdlen > sizeof(s->cmd_buf)) {
                PMU_DPRINTF("command too big (%d bytes)!\n", s->cmdlen);
            }
        } else if (s->cmd_buf_pos < sizeof(s->cmd_buf)) {
            s->cmd_buf[s->cmd_buf_pos++] = ms->sr;
        }
        via_set_sr_int(s);
        break;
    case pmu_state_rsp:
        if (ms->acr & SR_OUT) {
            PMU_DPRINTF("protocol error ! state resp, ACR writing\n");
            break;
        }
        if (s->rsplen == -1) {
#ifdef DEBUG_PMU_PROTOCOL
            PMU_DPRINTF(" sending length byte: %d\n", s->cmd_rsp_sz);
#endif
            ms->sr = s->cmd_rsp_sz;
            s->rsplen = s->cmd_rsp_sz;
        } else if (s->cmd_rsp_pos < s->cmd_rsp_sz) {
#ifdef DEBUG_PMU_PROTOCOL
            PMU_DPRINTF(" sending byte: %d/%d\n",
                        s->cmd_rsp_pos, s->rsplen);
#endif
            ms->sr = s->cmd_rsp[s->cmd_rsp_pos++];
        }
        via_set_sr_int(s);
        break;
    }

    /* Check for state completion */
    if (s->cmd_state == pmu_state_cmd && s->cmdlen == s->cmd_buf_pos) {
#ifdef DEBUG_PMU_PROTOCOL
        PMU_DPRINTF("Command reception complete, dispatching...\n");
#endif
        pmu_dispatch_cmd(s);
        s->cmd_state = pmu_state_rsp;
    }
    if (s->cmd_state == pmu_state_rsp && s->rsplen == s->cmd_rsp_pos) {
#ifdef DEBUG_PMU_PROTOCOL
        PMU_DPRINTF("Response send complete. IER=%02x\n", ms->ier);
#endif
        s->cmd_state = pmu_state_idle;
    }
}

static uint64_t pmu_read(void *opaque, hwaddr addr, unsigned size)
{
    PMUState *s = opaque;
    MOS6522PMUState *mps = s->mos6522_pmu;
    MOS6522State *ms = MOS6522(mps);

    addr = (addr >> 9) & 0xf;
    return mos6522_read(ms, addr, size);
}

static void pmu_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    PMUState *s = opaque;
    MOS6522PMUState *mps = s->mos6522_pmu;
    MOS6522State *ms = MOS6522(mps);

    addr = (addr >> 9) & 0xf;
    mos6522_write(ms, addr, val, size);
}

static const MemoryRegionOps pmu_mm_ops = {
    .read = pmu_read,
    .write = pmu_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

#if 0
static const VMStateDescription vmstate_cuda = {
    .name = "cuda",
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(last_b, PMUState),
        VMSTATE_INT32(data_in_size, PMUState),
        VMSTATE_INT32(data_in_index, PMUState),
        VMSTATE_INT32(data_out_index, PMUState),
        VMSTATE_UINT8(autopoll, PMUState),
        VMSTATE_UINT8(autopoll_rate_ms, PMUState),
        VMSTATE_UINT16(adb_poll_mask, PMUState),
        VMSTATE_BUFFER(data_in, PMUState),
        VMSTATE_BUFFER(data_out, PMUState),
        VMSTATE_UINT32(tick_offset, PMUState),
        VMSTATE_STRUCT_ARRAY(timers, PMUState, 2, 1,
                             vmstate_cuda_timer, VIATimer),
        VMSTATE_TIMER_PTR(adb_poll_timer, PMUState),
        VMSTATE_TIMER_PTR(sr_delay_timer, PMUState),
        VMSTATE_END_OF_LIST()
    }
};
#endif

static void pmu_reset(DeviceState *dev)
{
    PMUState *s = VIA_PMU(dev);
    MOS6522PMUState *pms = s->mos6522_pmu;
    MOS6522State *ms = MOS6522(pms);
    
    ms->b = s->last_b = TACK | TREQ;

    // XXX OPENBIOS needs to do this ? MacOS 9 needs it
    s->intmask = PMU_INT_ADB | PMU_INT_TICK;
    s->intbits = 0;

    s->cmd_state = pmu_state_idle;
    s->autopoll_mask = 0;
}

static void pmu_realizefn(DeviceState *dev, Error **errp)
{
    PMUState *s = VIA_PMU(dev);
    SysBusDevice *sbd;
    MOS6522State *ms;
    DeviceState *d;
    struct tm tm;

    PMU_DPRINTF("realize, macio=%p\n", s->macio);
    
    d = qdev_create(NULL, TYPE_MOS6522_PMU);
    object_property_set_link(OBJECT(d), OBJECT(s), "pmu", errp);
    qdev_init_nofail(d);
    s->mos6522_pmu = MOS6522_PMU(d);
    
    /* Pass IRQ from 6522 */
    ms = MOS6522(d);
    sbd = SYS_BUS_DEVICE(s);
    sysbus_pass_irq(sbd, SYS_BUS_DEVICE(ms));
    
    qemu_get_timedate(&tm, 0);
    s->tick_offset = (uint32_t)mktimegm(&tm) + RTC_OFFSET;
    s->one_sec_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, pmu_one_sec_timer, s);
    s->one_sec_target = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000;
    timer_mod(s->one_sec_timer, s->one_sec_target);

    if (s->has_adb) {
        PMU_DPRINTF("PMU: Creating ADB bus\n");
        qbus_create_inplace(&s->adb_bus, sizeof(s->adb_bus), TYPE_ADB_BUS,
                            DEVICE(dev), "adb.0");
        s->adb_poll_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, pmu_adb_poll, s);
        s->autopoll_rate_ms = 20;
        s->adb_poll_mask = 0xffff;
    }

    /* GPIO is up by default */
    MacIOSetGPIO(s->macio, 1, true);
}

static void pmu_initfn(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    PMUState *s = VIA_PMU(obj);

    memory_region_init_io(&s->mem, OBJECT(s),
                          &pmu_mm_ops, s, "via-pmu", 0x2000);
    sysbus_init_mmio(d, &s->mem);
    //sysbus_init_irq(d, &s->via_irq);
}

static Property pmu_properties[] = {
    // XXX Add a "has ADB" property
    DEFINE_PROP_UINT64("frequency", PMUState, frequency, 0),
    DEFINE_PROP_PTR("macio", PMUState, macio),
    DEFINE_PROP_BOOL("has_adb", PMUState, has_adb, true),
    DEFINE_PROP_END_OF_LIST()
};

static void pmu_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = pmu_realizefn;
    dc->reset = pmu_reset;
//    dc->vmsd = &vmstate_cuda;
    dc->props = pmu_properties;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
}

static const TypeInfo pmu_type_info = {
    .name = TYPE_VIA_PMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PMUState),
    .instance_init = pmu_initfn,
    .class_init = pmu_class_init,
};

static void mos6522_pmu_portB_write(MOS6522State *s)
{
    MOS6522PMUState *mcs = container_of(s, MOS6522PMUState, parent_obj);
    PMUState *ps = VIA_PMU(mcs->pmu);
    
    if ((s->pcr & 0xe0) == 0x20 || (s->pcr & 0xe0) == 0x60) {
        s->ifr &= ~CB2_INT;
    }
    s->ifr &= ~CB1_INT;
    
    via_update_irq(ps);
    pmu_update(ps);
}

static void mos6522_pmu_portA_write(MOS6522State *s)
{
    MOS6522PMUState *mcs = container_of(s, MOS6522PMUState, parent_obj);
    PMUState *ps = VIA_PMU(mcs->pmu);

    if ((s->pcr & 0x0e) == 0x02 || (s->pcr & 0x0e) == 0x06) {
        s->ifr &= ~CA2_INT;
    }
    s->ifr &= ~CA1_INT;
        
    via_update_irq(ps);
}

static void mos6522_pmu_realize(DeviceState *dev, Error **errp)
{
    MOS6522State *ms = MOS6522(dev);
    MOS6522DeviceClass *mdc = MOS6522_DEVICE_GET_CLASS(ms);

    mdc->parent_realize(dev, errp);

    ms->timers[0].frequency = VIA_TIMER_FREQ;
    ms->timers[1].frequency = (SCALE_US * 6000) / 4700;
}

static void mos6522_pmu_init(Object *obj)
{
    MOS6522PMUState *s = MOS6522_PMU(obj);

    object_property_add_link(obj, "pmu", TYPE_VIA_PMU,
                             (Object **) &s->pmu,
                             qdev_prop_allow_set_link_before_realize,
                             0, NULL);
}

static void mos6522_pmu_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    MOS6522DeviceClass *mdc = MOS6522_DEVICE_CLASS(oc);

    dc->realize = mos6522_pmu_realize;
    mdc->portB_write = mos6522_pmu_portB_write;
    mdc->portA_write = mos6522_pmu_portA_write;
}

static const TypeInfo mos6522_pmu_type_info = {
    .name = TYPE_MOS6522_PMU,
    .parent = TYPE_MOS6522,
    .instance_size = sizeof(MOS6522PMUState),
    .instance_init = mos6522_pmu_init,
    .class_init = mos6522_pmu_class_init,
};

static void pmu_register_types(void)
{
    type_register_static(&pmu_type_info);
    type_register_static(&mos6522_pmu_type_info);
}

type_init(pmu_register_types)
