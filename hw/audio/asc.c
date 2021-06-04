/*
 *  QEMU Apple Sound Chip emulation
 *
 *  Apple Sound Chip (ASC) 344S0063
 *  Enhanced Apple Sound Chip (EASC) 343S1063
 *
 *  Copyright (c) 2012-2018 Laurent Vivier <laurent@vivier.eu>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "audio/audio.h"
#include "hw/audio/asc.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "trace.h"

/*
 * Linux doesn't provide information about ASC, see arch/m68k/mac/macboing.c
 * and arch/m68k/include/asm/mac_asc.h
 *
 * best information is coming from MAME:
 *   https://github.com/mamedev/mame/blob/master/src/devices/sound/asc.h
 *   https://github.com/mamedev/mame/blob/master/src/devices/sound/asc.cpp
 *   Emulation by R. Belmont
 * or MESS:
 *   http://mess.redump.net/mess/driver_info/easc
 *
 *     0x800: VERSION
 *     0x801: MODE
 *            1=FIFO mode,
 *            2=wavetable mode
 *     0x802: CONTROL
 *            bit 0=analog or PWM output,
 *                1=stereo/mono,
 *                7=processing time exceeded
 *     0x803: FIFO MODE
 *            bit 7=clear FIFO,
 *            bit 1="non-ROM companding",
 *            bit 0="ROM companding")
 *     0x804: FIFO IRQ STATUS
 *            bit 0=ch A 1/2 full,
 *                1=ch A full,
 *                2=ch B 1/2 full,
 *                3=ch B full)
 *     0x805: WAVETABLE CONTROL
 *            bits 0-3 wavetables 0-3 start
 *     0x806: VOLUME
 *            bits 2-4 = 3 bit internal ASC volume,
 *            bits 5-7 = volume control sent to Sony sound chip
 *     0x807: CLOCK RATE
 *            0 = Mac 22257 Hz,
 *            1 = undefined,
 *            2 = 22050 Hz,
 *            3 = 44100 Hz
 *     0x80a: PLAY REC A
 *     0x80f: TEST
 *            bits 6-7 = digital test,
 *            bits 4-5 = analog test
 *     0x810: WAVETABLE 0 PHASE
 *            big-endian 9.15 fixed-point, only 24 bits valid
 *     0x814: WAVETABLE 0 INCREMENT
 *            big-endian 9.15 fixed-point, only 24 bits valid
 *     0x818: WAVETABLE 1 PHASE
 *     0x81C: WAVETABLE 1 INCREMENT
 *     0x820: WAVETABLE 2 PHASE
 *     0x824: WAVETABLE 2 INCREMENT
 *     0x828: WAVETABLE 3 PHASE
 *     0x82C: WAVETABLE 3 INCREMENT
 *     0x830: UNKNOWN START
 *            NetBSD writes Wavetable data here (are there more
 *            wavetables/channels than we know about?)
 *     0x857: UNKNOWN END
*/

#define ASC_SIZE           0x2000

#define ASC_FIFO_OFFSET    0x0
#define ASC_FIFO_SIZE      0x800

#define ASC_REG_OFFSET     0x800
#define ASC_REG_SIZE       0x60

#define ASC_EXTREG_OFFSET  0xf00
#define ASC_EXTREG_SIZE    0x40

enum {
    ASC_VERSION     = 0x00,
    ASC_MODE        = 0x01,
    ASC_CONTROL     = 0x02,
    ASC_FIFOMODE    = 0x03,
    ASC_FIFOIRQ     = 0x04,
    ASC_WAVECTRL    = 0x05,
    ASC_VOLUME      = 0x06,
    ASC_CLOCK       = 0x07,
    ASC_PLAYRECA    = 0x0a,
    ASC_TEST        = 0x0f,
    ASC_WAVETABLE   = 0x10
};

static void asc_raise_irq(ASCState *s)
{
    trace_asc_raise_irq();
    qemu_irq_raise(s->irq);
}

static void asc_lower_irq(ASCState *s)
{
    trace_asc_lower_irq();
    qemu_irq_lower(s->irq);
}

static inline bool asc_fifo_half_full_irq_enabled(ASCState *s, int fifo)
{
    int ireg;

    /* ASC half full irq always enabled */
    if (s->type == ASC_TYPE_ASC) {
        return true;
    }

    /* Otherwise check the bit in the extreg */
    ireg = (fifo == 0) ? 0x9 : 0x29;
    if (s->extregs[ireg] & 1) {
        return true;
    }

    return false;
}

static void asc_fifo_put(ASCState *s, int fifo, uint8_t val)
{
    bool fifo_half_irq_enabled = asc_fifo_half_full_irq_enabled(s, fifo);

    switch (fifo) {
    case 0:
        /* FIFO A */
        s->fifo[s->a_wptr++] = val;
        s->a_cnt++;

        if (s->a_cnt <= 0x1ff) {
            s->regs[ASC_FIFOIRQ] |= 1; /* FIFO A Half Full */
            if (s->a_cnt == 0x1ff && fifo_half_irq_enabled) {
                asc_raise_irq(s);
            }
        }
        if (s->a_cnt == 0x3ff) {
            s->regs[ASC_FIFOIRQ] |= 2; /* FIFO A Full */
            asc_raise_irq(s);
        }
        s->a_wptr &= 0x3ff;
        break;

    case 1:
        /* FIFO B */
        s->fifo[s->b_wptr++ + 0x400] = val;
        s->b_cnt++;

        if (s->b_cnt <= 0x1ff) {
            s->regs[ASC_FIFOIRQ] |= 4;
            if (s->b_cnt == 0x1ff && fifo_half_irq_enabled) {
                asc_raise_irq(s);
            }
        }
        if (s->b_cnt == 0x3ff) {
            s->regs[ASC_FIFOIRQ] |= 8; /* FIFO A Full */
            asc_raise_irq(s);
        }
        s->b_wptr &= 0x3ff;
        break;

    default:
        g_assert_not_reached();
    }
}

static bool asc_fifo_get(ASCState *s, int fifo, uint8_t *val)
{
    bool fifo_half_irq_enabled = asc_fifo_half_full_irq_enabled(s, fifo);
    bool res = true;

    switch (fifo) {
    case 0:
        /* FIFO A */
        *val = s->fifo[s->a_rptr];
        if (s->a_cnt) {
            s->a_rptr++;
            s->a_rptr &= 0x3ff;
            s->a_cnt--;
        } else {
            res = false;
        }

        if (s->a_cnt <= 0x1ff) {
            s->regs[ASC_FIFOIRQ] |= 1; /* FIFO A less than half full */
            if (s->a_cnt == 0x1ff && fifo_half_irq_enabled) {
                asc_raise_irq(s);
            }
            if (s->a_cnt == 0) {
                s->regs[ASC_FIFOIRQ] |= 2; /* FIFO A empty */
                asc_raise_irq(s);
            }
        }
        break;

    case 1:
        *val = s->fifo[s->b_rptr + 0x400];
        if (s->b_cnt) {
            s->b_rptr++;
            s->b_rptr &= 0x3ff;
            s->b_cnt--;
        } else {
            res = false;
        }

        if (s->b_cnt <= 0x1ff) {
            s->regs[ASC_FIFOIRQ] |= 4; /* FIFO B less than half full */
            if (s->b_cnt == 0x1ff && fifo_half_irq_enabled) {
                asc_raise_irq(s);
            }
            if (s->b_cnt == 0) {
                s->regs[ASC_FIFOIRQ] |= 8; /* FIFO B empty */
                asc_raise_irq(s);
            }
        }
        break;

    default:
        g_assert_not_reached();
    }

    return res;
}

static inline uint32_t get_phase(ASCState *s, int channel)
{
    return be32_to_cpu(*(uint32_t *)(s->regs + ASC_WAVETABLE + channel * 8));
}

static inline void set_phase(ASCState *s, int channel, uint32_t phase)
{
    *(uint32_t *)(s->regs + ASC_WAVETABLE + channel * 8) = cpu_to_be32(phase);
}

static inline uint32_t get_incr(ASCState *s, int channel)
{
    return be32_to_cpu(*(uint32_t *)(s->regs + ASC_WAVETABLE + 4 +
                                     channel * 8));
}

static inline uint32_t incr_phase(ASCState *s, int channel)
{
    uint32_t incr = get_incr(s, channel);
    uint32_t phase = get_phase(s, channel);

    set_phase(s, channel, phase + incr);

    return get_phase(s, channel);
}

static int generate_fifo(ASCState *s, int maxsamples)
{
    int8_t *buf = s->mixbuf + s->pos;
    int i;

    for (i = 0; i < maxsamples; i++) {
        uint8_t v;
        int8_t left, right;
        bool res;

        res = asc_fifo_get(s, 0, &v);
        if (res) {
            left = v ^ 0x80;
        } else {
            left = 0;
        }

        res = asc_fifo_get(s, 1, &v);
        if (res) {
            right = v ^ 0x80;
        } else {
            right = 0;
        }

        buf[i * 2] = left;
        buf[i * 2 + 1] = right;
    }

    return i;
}

static int generate_wavetable(ASCState *s, int maxsamples)
{
    int control = s->regs[ASC_WAVECTRL];
    int8_t *buf = s->mixbuf + s->pos;
    int channel, i;

    for (i = 0; i < maxsamples; i++) {
        int32_t left, right;
        int8_t sample;

        left = 0;
        right = 0;

        if (control) { /* FIXME: how to use it ? */
            for (channel = 0; channel < 4; channel++) {
                uint32_t phase = incr_phase(s, channel);

                phase = (phase >> 15) & 0x1ff;
                sample = s->fifo[0x200 * channel + phase] ^ 0x80;

                left += sample;
                right += sample;
            }
            buf[i * 2] = left >> 2;
            buf[i * 2 + 1] = right >> 2;
        } else {
            /* FIXME: only works with linux macboing.c */
            uint32_t phase = incr_phase(s, 0);
            phase = (phase >> 15) & 0x7ff;
            sample = s->fifo[phase];
            buf[i * 2] = sample;
            buf[i * 2 + 1] = sample;
        }
    }

    return i;
}

static int write_audio(ASCState *s, int samples)
{
    int net = 0;
    int pos = s->pos;

    while (samples) {
        int nbytes, wbytes, wsampl;

        nbytes = samples << s->shift;
        wbytes = AUD_write(s->voice,
                           s->mixbuf + (pos << (s->shift - 1)),
                           nbytes);
        if (wbytes) {
            wsampl = wbytes >> s->shift;

            samples -= wsampl;
            pos = (pos + wsampl) % s->samples;

            net += wsampl;
        } else {
            break;
        }
    }

    return net;
}

static void asc_out_cb(void *opaque, int free_b)
{
    ASCState *s = opaque;
    int samples, net = 0, to_play, written;

    samples = free_b >> s->shift;
    if (!samples) {
        return;
    }

    to_play = MIN(s->left, samples);
    while (to_play) {
        written = write_audio(s, to_play);

        if (written) {
            s->left -= written;
            samples -= written;
            to_play -= written;
            s->pos = (s->pos + written) % s->samples;
        } else {
            return;
        }
    }

    samples = MIN(samples, s->samples - s->pos);
    if (!samples) {
        return;
    }

    switch (s->regs[ASC_MODE] & 3) {
    case 0: /* Off */
        break;
    case 1: /* FIFO mode */
        samples = generate_fifo(s, samples);
        break;
    case 2: /* Wave table mode */
        samples = generate_wavetable(s, samples);
        break;
    }

    while (samples) {
        written = write_audio(s, samples);

        if (written) {
            net += written;
            samples -= written;
            s->pos = (s->pos + written) % s->samples;
        } else {
            s->left = samples;
            return;
        }
    }
}

static uint64_t asc_fifo_read(void *opaque, hwaddr addr,
                              unsigned size)
{
    ASCState *s = opaque;

    trace_asc_read_fifo(addr, size, s->fifo[addr]);
    return s->fifo[addr];
}

static void asc_fifo_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    ASCState *s = opaque;

    trace_asc_write_fifo(addr, size, value);
    if (s->regs[ASC_MODE] == 1) {
        if (addr < 0x400) {
            /* FIFO A */
            asc_fifo_put(s, 0, value);
        } else {
            /* FIFO B */
            asc_fifo_put(s, 1, value);
        }
    } else {
        s->fifo[addr] = value;
    }
    return;
}

static const MemoryRegionOps asc_fifo_ops = {
    .read = asc_fifo_read,
    .write = asc_fifo_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t asc_read(void *opaque, hwaddr addr,
                         unsigned size)
{
    ASCState *s = opaque;
    uint64_t prev, value;

    switch (addr) {
    case ASC_VERSION:
        switch (s->type) {
        default:
        case ASC_TYPE_ASC:
            value = 0;
            break;
        case ASC_TYPE_V8:
        case ASC_TYPE_EAGLE:
        case ASC_TYPE_SPICE:
        case ASC_TYPE_VASP:
            value = 0xe8;
            break;
        case ASC_TYPE_EASC:
            value = 0xb0;
            break;
        case ASC_TYPE_SONORA:
            value = 0xbc;
            break;
        }
        break;
    case ASC_MODE:
        switch (s->type) {
        case ASC_TYPE_V8:
        case ASC_TYPE_EAGLE:
        case ASC_TYPE_SPICE:
        case ASC_TYPE_VASP:
            value = 1;
            break;
        default:
            value = s->regs[addr];
            break;
        }
        break;
    case ASC_CONTROL:
        switch (s->type) {
        case ASC_TYPE_V8:
        case ASC_TYPE_EAGLE:
        case ASC_TYPE_SPICE:
        case ASC_TYPE_VASP:
            value = 1;
            break;
        default:
            value = s->regs[addr];
            break;
        }
        break;
    case ASC_FIFOIRQ:
        if (s->type == ASC_TYPE_V8) {
            prev = 3;
        } else {
            prev = s->regs[ASC_FIFOIRQ];
        }
        s->regs[ASC_FIFOIRQ] = 0;
        asc_lower_irq(s);
        value = prev;
        break;
    default:
        value = s->regs[addr];
        break;
    }

    trace_asc_read_reg(addr, size, value);
    return value;
}

static void asc_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    ASCState *s = opaque;

    switch (addr) {
    case ASC_MODE:
        value &= 3;
        if (value != s->regs[ASC_MODE]) {
            s->a_rptr = 0;
            s->a_wptr = 0;
            s->a_cnt = 0;
            s->b_rptr = 0;
            s->b_wptr = 0;
            s->b_cnt = 0;
            if (value != 0) {
                AUD_set_active_out(s->voice, 1);
            } else {
                AUD_set_active_out(s->voice, 0);
            }
        }
        break;
    case ASC_FIFOMODE:
        if (value & 0x80) {
            s->a_rptr = 0;
            s->a_wptr = 0;
            s->a_cnt = 0;
            s->b_rptr = 0;
            s->b_wptr = 0;
            s->b_cnt = 0;
        }
        break;
    case ASC_WAVECTRL:
        break;
    }

    trace_asc_write_reg(addr, size, value);
    s->regs[addr] = value;
}

static const MemoryRegionOps asc_regs_ops = {
    .read = asc_read,
    .write = asc_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t asc_ext_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    ASCState *s = opaque;
    uint64_t value;

    value = s->extregs[addr];

    trace_asc_read_extreg(addr, size, value);
    return value;
}

static void asc_ext_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    ASCState *s = opaque;

    trace_asc_write_extreg(addr, size, value);
    s->extregs[addr] = value;
}

static const MemoryRegionOps asc_extregs_ops = {
    .read = asc_ext_read,
    .write = asc_ext_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_BIG_ENDIAN,
};

static int asc_post_load(void *opaque, int version_id)
{
    return 0;
}

static const VMStateDescription vmstate_asc = {
    .name = "apple-sound-chip",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = asc_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void asc_reset(DeviceState *d)
{
    ASCState *s = ASC(d);

    AUD_set_active_out(s->voice, 0);

    memset(s->regs, 0, sizeof(s->regs));
    s->a_wptr = 0;
    s->a_rptr = 0;
    s->a_cnt = 0;
    s->b_wptr = 0;
    s->b_rptr = 0;
    s->b_cnt = 0;

    /* FIFO A and B empty */
    s->regs[ASC_FIFOIRQ] = 0xf;
}

static void asc_unrealize(DeviceState *dev)
{
    ASCState *s = ASC(dev);

    g_free(s->mixbuf);
    g_free(s->fifo);

    AUD_remove_card(&s->card);
}

static void asc_realize(DeviceState *dev, Error **errp)
{
    ASCState *s = ASC(dev);
    struct audsettings as;

    AUD_register_card("Apple Sound Chip", &s->card);

    as.freq = 22257;
    as.nchannels = 2;
    as.fmt = AUDIO_FORMAT_S8;
    as.endianness = AUDIO_HOST_ENDIANNESS;

    s->voice = AUD_open_out(&s->card, s->voice, "asc.out", s, asc_out_cb,
                            &as);
    s->shift = 1;
    s->samples = AUD_get_buffer_size_out(s->voice) >> s->shift;
    s->mixbuf = g_malloc0(s->samples << s->shift);

    s->fifo = g_malloc0(ASC_FIFO_SIZE);

    /* Add easc registers if required */
    if (s->type == ASC_TYPE_EASC) {
        memory_region_init_io(&s->mem_extregs, OBJECT(s), &asc_extregs_ops,
                              s, "asc.extregs", ASC_EXTREG_OFFSET);
        memory_region_add_subregion(&s->asc, ASC_EXTREG_OFFSET,
                                    &s->mem_extregs);
    }
}

static void asc_init(Object *obj)
{
    ASCState *s = ASC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init(&s->asc, OBJECT(obj), "asc", ASC_SIZE);
    memory_region_init_io(&s->mem_fifo, OBJECT(obj), &asc_fifo_ops, s, "asc.fifo",
                          ASC_FIFO_SIZE);
    memory_region_add_subregion(&s->asc, ASC_FIFO_OFFSET, &s->mem_fifo);
    memory_region_init_io(&s->mem_regs, OBJECT(obj), &asc_regs_ops, s, "asc.regs",
                          ASC_REG_SIZE);
    memory_region_add_subregion(&s->asc, ASC_REG_OFFSET, &s->mem_regs);

    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_mmio(sbd, &s->asc);
}

static Property asc_properties[] = {
    DEFINE_AUDIO_PROPERTIES(ASCState, card),
    DEFINE_PROP_UINT8("asctype", ASCState, type, ASC_TYPE_ASC),
    DEFINE_PROP_END_OF_LIST(),
};

static void asc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = asc_realize;
    dc->unrealize = asc_unrealize;
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
    dc->reset = asc_reset;
    dc->vmsd = &vmstate_asc;
    device_class_set_props(dc, asc_properties);
}

static TypeInfo asc_info = {
    .name = TYPE_ASC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ASCState),
    .instance_init = asc_init,
    .class_init = asc_class_init,
};

static void asc_register_types(void)
{
    type_register_static(&asc_info);
}

type_init(asc_register_types)
