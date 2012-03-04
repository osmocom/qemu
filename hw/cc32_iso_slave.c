/*
 * ChipCity CC32RS512 ISO7816 Slave Controller emulation
 *
 * Copyright (C) 2012 Harald Welte <laforge@gnumonks.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw.h"
#include "sysbus.h"
#include "trace.h"
#include "qemu-char.h"
#include "qemu-error.h"

enum iso_slave_reg {
	ISOCON		= 0x00,
	ISOCON1		= 0x04,
	ISOCON2		= 0x08,
	ISOSTS		= 0x0c,
	ISOBRC		= 0x10,
	ISOBUF		= 0x14,
	ISODIO		= 0x18,
	ISOMSK		= 0x1c,
	ISODMACON	= 0x30,
	ISODMASTS	= 0x34,
	ISODMABFAD	= 0x38,
	ISODMABFLEN	= 0x3c,
	ISODMABFPT	= 0x40,
	ISODMAMSK	= 0x44,
	ISOTCON		= 0x50,
	ISOTDAT		= 0x54,
	ISOTRLD		= 0x58,
	ISOTMSK		= 0x5c,
	ISONULL		= 0x60,
};

#define ISOCON_TR	(1 << 5)
#define ISOCON_TACT	(1 << 4)

#define ISOSTS_TBE	(1 << 0)
#define ISOSTS_RBF	(1 << 1)
#define ISOSTS_PE	(1 << 2)
#define ISOSTS_OE	(1 << 3)


#define NUM_REGS	25

struct CC32IsoSlaveState {
	SysBusDevice busdev;
	MemoryRegion iomem;
	CharDriverState *chr;
	qemu_irq irq;

	uint32_t regs[NUM_REGS];
};
typedef struct CC32IsoSlaveState CC32IsoSlaveState;

static void iso_slave_update_irq(CC32IsoSlaveState *s)
{
	unsigned int irq = 0;

	if (s->regs[ISOSTS>>2] & s->regs[ISOMSK>>2])
		irq = 1;

	qemu_set_irq(s->irq, irq);
}

static uint64_t iso_slave_read(void *opaque, target_phys_addr_t addr, unsigned size)
{
	CC32IsoSlaveState *s = opaque;
	uint32_t r;
	uint32_t reg_idx = addr >> 2;

	if (addr & 3) {
		error_report("cc32_iso_slave: unaligned read access");
		return 0;
	}

	switch (addr) {
	case ISOBUF:
		r = s->regs[reg_idx];
		s->regs[ISOSTS>>2] &= ~ISOSTS_RBF;
	default:
		r = s->regs[reg_idx];
		break;
	}

	return r;
}

static void iso_slave_write(void *opaque, target_phys_addr_t addr,
			uint64_t value, unsigned size)
{
	CC32IsoSlaveState *s = opaque;
	uint32_t reg_idx = addr >> 2;
	uint32_t val32 = value;
	uint32_t mask = 0;

	switch (addr) {
	case ISOCON:
		mask = (1 << 5) | (1 << 7);
		break;
	case ISOCON1:
		mask = 0x3F;
		break;
	case ISOCON2:
		mask = (1 << 0) | (1 << 2) | (1 << 7);
		break;
	case ISOSTS:
		if (val32 & (1 << 2)) {
			val32 &= ~(1 << 2);
			mask |= (1 << 2);
		}
		if (val32 & (1 << 3)) {
			val32 &= ~(1 <<3);
			mask |= (1 << 3);
		}
		break;
	case ISOMSK:
		mask = 0x8F;
		break;
	case ISOBRC:
	case ISOBUF:
		/* check if we are in transmitting mode */
		if (s->regs[ISOCON>>2] & ISOCON_TR) {
			uint8_t ch = val32;
			qemu_chr_fe_write(s->chr, &ch, 1);
			s->regs[ISOCON>>2] &= ~ISOCON_TACT;
			mask = 0;
		} else
			mask = 0xFF;
		break;
	case ISODIO:
	case ISODMACON:
		mask = 3;
		break;
	case ISODMASTS:
	case ISODMAMSK:
		mask = 0x07;
		break;
	case ISODMABFAD:
		mask = 0xffffff;
		break;
	case ISODMABFLEN:
		mask = 0x1FF;
		break;
	case ISODMABFPT:
		mask = 0;
		break;
	case ISOTCON:
		mask = 0x0F;
		break;
	case ISOTMSK:
		mask = 1;
		break;
	case ISOTDAT:
	case ISOTRLD:
		mask = 0xFFFF;
		break;
	case ISONULL:
		mask = 0xFF;
		break;
	default:
		mask = 0;
		break;
	}

	s->regs[reg_idx] = (s->regs[reg_idx] & ~mask) | (val32 & mask);

	iso_slave_update_irq(s);
}

static const MemoryRegionOps iso_slave_ops = {
	.read = iso_slave_read,
	.write = iso_slave_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
	.valid = {
		.min_access_size = 4,
		.max_access_size = 4,
	},
};

static void iso_slave_rx(void *opaque, const uint8_t *buf, int size)
{
	CC32IsoSlaveState *s = opaque;

	if (s->regs[ISOSTS>>2] & ISOSTS_RBF)
		s->regs[ISOSTS>>2] |= ISOSTS_OE;
	else
		s->regs[ISOSTS>>2] |= ISOSTS_RBF;

	s->regs[ISOBUF>>2] = *buf;

	iso_slave_update_irq(s);
}

static int iso_slave_can_rx(void *opaque)
{
	CC32IsoSlaveState *s = opaque;

	/* check if we are in receive mode */
	if (s->regs[ISOCON>>2] & ISOCON_TR)
		return 0;

	return 1;
}

static void iso_slave_event(void *opaque, int event)
{
}

static void iso_slave_reset(DeviceState *d)
{
	CC32IsoSlaveState *s = container_of(d, CC32IsoSlaveState, busdev.qdev);
	int i;

	for (i = 0; i < NUM_REGS; i++) {
		switch (i << 2) {
		case ISOCON:
		case ISOSTS:
		case ISOBRC:
		case ISODIO:
		case ISOTMSK:
			s->regs[i] = 0x00000001;
			break;
		case ISOCON1:
		case ISODMAMSK:
			s->regs[i] = 0x00000007;
			break;
		case ISONULL:
			s->regs[i] = 0x00000060;
			break;
		default:
			s->regs[i] = 0;
			break;
		}
	}
}

static int iso_slave_init(SysBusDevice *dev)
{
	CC32IsoSlaveState *s = FROM_SYSBUS(typeof(*s), dev);

	sysbus_init_irq(dev, &s->irq);

	memory_region_init_io(&s->iomem, &iso_slave_ops, s, "iso7816_slave", 0x64);
	sysbus_init_mmio(dev, &s->iomem);

	s->chr = qemu_char_get_next_serial();
	if (s->chr) {
		qemu_chr_add_handlers(s->chr, iso_slave_can_rx, iso_slave_rx, iso_slave_event, s);
	}

	return 0;
}

static const VMStateDescription vmstate_iso_slave = {
	.name = "cc32-iso-slave",
	.version_id = 1,
	.minimum_version_id = 1,
	.minimum_version_id_old = 1,
	.fields = (VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs, CC32IsoSlaveState, NUM_REGS),
		VMSTATE_END_OF_LIST()
	},
};

static void iso_slave_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

	k->init = iso_slave_init;
	dc->reset = iso_slave_reset;
	dc->vmsd = &vmstate_iso_slave;
}

static TypeInfo iso_slave_info = {
	.name		= "cc32-iso-slave",
	.parent		= TYPE_SYS_BUS_DEVICE,
	.instance_size	= sizeof(CC32IsoSlaveState),
	.class_init	= iso_slave_class_init,
};

static void iso_slave_register_types(void)
{
	type_register_static(&iso_slave_info);
}

type_init(iso_slave_register_types);
