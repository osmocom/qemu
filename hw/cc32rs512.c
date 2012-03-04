/*
 * ChipCity CC32RS512 Smart Card emulation
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

#include "sysbus.h"
#include "arm-misc.h"
#include "devices.h"
#include "boards.h"
#include "exec-memory.h"

/***********************************************************************
 * Flash controller
 ***********************************************************************/

enum cc32_flcon_reg {
	FLCON		= 0x00,
	FLSDP1		= 0x04,
	FLSDP2		= 0x08,
	FLSTS		= 0x0C,
	FLBUF		= 0x10,
};

#define NUM_FL_REGS 	((FLBUF>>2) + 4)

enum write_state {
	WRST_NONE,
	WRST_SDP1_55,
	WRST_SDP1_AA,
	WRST_ERASE,
	WRST_PROGRAM,
};

typedef struct cc32_flcon_state {
	SysBusDevice busdev;
	MemoryRegion iomem;
	qemu_irq irq;

	uint32_t regs[NUM_FL_REGS];
	enum write_state wrst;
	uint8_t *storage;
	uint8_t *ram_storage;
} cc32_flcon_state;

static void cc32_flcon_update(cc32_flcon_state *s)
{
	if (s->regs[FLSTS>>2] & 1)
		qemu_set_irq(s->irq, 1);
	else
		qemu_set_irq(s->irq, 0);
}

static uint64_t cc32_flcon_read(void *opaque, target_phys_addr_t offset,
			     unsigned size)
{
	cc32_flcon_state *s = (cc32_flcon_state *)opaque;

	return s->regs[offset>>2];
}

static void cc32_flcon_write(void *opaque, target_phys_addr_t offset,
			uint64_t value, unsigned size)
{
	cc32_flcon_state *s = (cc32_flcon_state *)opaque;

	s->regs[offset>>2] = value;

	switch (offset) {
	case FLSDP1:
		switch (value) {
		case 0x55:
			s->wrst = WRST_SDP1_55;
			break;
		case 0xAA:
			s->wrst = WRST_SDP1_AA;
			break;
		default:
			/* Signal incorrect operation */
			s->regs[FLSTS>>2] |= (1 << 1);
			s->wrst = WRST_NONE;
			break;
		}
		break;
	case FLSDP2:
		if (s->wrst == WRST_SDP1_55 && (value == 0xAA)) {
			s->wrst = WRST_ERASE;
			/* actual ERASE will be performed once the program
			 * writes a 0xFF to any address within the flash page */
		} else if ((s->wrst == WRST_SDP1_AA) && (value == 0x55)) {
			s->wrst = WRST_PROGRAM;
			/* actual PROGRAM will be performed once the program
			 * writes a 0x00 to any address within the flash page */
		} else {
			/* Signal incorrect operation */
			s->regs[FLSTS>>2] |= (1 << 1);
			s->wrst = WRST_NONE;
			break;
		}
		break;
	case FLSTS:
		/* clear bits */
		s->regs[offset>>2] &= ~(value & 3);
		break;
	}

	cc32_flcon_update(s);
}

/* callback for user writes to the flash memory region */
static void cc32_flash_write(void *opaque, target_phys_addr_t offset,
			uint64_t value, unsigned size)
{
	cc32_flcon_state *s = (cc32_flcon_state *)opaque;
	uint32_t epsize = (s->regs[FLCON>>2] >> 3);
	uint32_t page_size, mask;

	if (epsize == 0) {
		page_size = 512;
		mask = 0xfffffe00;
	} else {
		page_size = 256;
		mask = 0xffffff00;
	}
	/* FIXME: check for 0x00 / 0xFF as value */

	switch (s->wrst) {
	case WRST_ERASE:
		fprintf(stderr, "cc32-flcon: Erasing %u at offset 0x%06x\n",
			page_size, offset & mask);
		memset(s->storage + (offset & mask), 0xFF, page_size);
		s->regs[FLSTS>>2] |= 1;
		s->wrst = WRST_NONE;
		break;
	case WRST_PROGRAM:
		fprintf(stderr, "cc32-flcon: Programming %u at offset 0x%06x\n",
			page_size, offset & mask);
		memcpy(s->storage + (offset & mask),
		       s->ram_storage + ((s->regs[FLBUF>>2] & 0xffffff) - 0xC0000),
		       page_size);
		s->regs[FLSTS>>2] |= 1;
		s->wrst = WRST_NONE;
		break;
	default:
		break;
	}

	cc32_flcon_update(s);
}


/* MMIO interface of the FLCON */
static const MemoryRegionOps cc32_flcon_ops = {
	.read = cc32_flcon_read,
	.write = cc32_flcon_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
	.valid = {
		.min_access_size = 4,
		.max_access_size = 4,
	},
};

/* Flash memory writes for memory_region_init_rom_device() */
static const MemoryRegionOps cc32_flash_ops = {
	.write = cc32_flash_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static int cc32_flcon_init(SysBusDevice *dev)
{
	cc32_flcon_state *s = FROM_SYSBUS(cc32_flcon_state, dev);

	sysbus_init_irq(dev, &s->irq);

	memory_region_init_io(&s->iomem, &cc32_flcon_ops, s, "cc32-flcon", NUM_FL_REGS<<2);
	sysbus_init_mmio(dev, &s->iomem);

	return 0;
}

static void cc32_flcon_reset(DeviceState *d)
{
	cc32_flcon_state *s = container_of(d, cc32_flcon_state, busdev.qdev);

	s->regs[FLBUF>>2] = 0xC0000 + (16*1024) - 512;
}

static void cc32_flcon_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = cc32_flcon_init;
	dc->reset = cc32_flcon_reset;
}

static TypeInfo cc32_flcon_info = {
	.name		= "cc32-flcon",
	.parent		= TYPE_SYS_BUS_DEVICE,
	.instance_size	= sizeof(cc32_flcon_state),
	.class_init 	= cc32_flcon_class_init,
};

/***********************************************************************
 * System controller
 ***********************************************************************/

enum cc32_sysc_reg {
	SCCM0		= 0x00,
	SCSYS		= 0x04,
	SCCKOUT		= 0x20,
	SCRSTFLG	= 0x28,
	SCRSTEN		= 0x2C,
	SCSFTRST	= 0x30,
	SCRSTCON0	= 0x34,
	SCRSTCON4	= 0x38,
	SCSLEEP		= 0x3C,
	SCGCON		= 0x40,
	SCINTSTS	= 0x44,
	SCINTEN		= 0x48,
	SCGINT0		= 0x5C,
	SCGLEV		= 0x64,
	SCWUT		= 0x68,
	SCCM4		= 0x7C,
};

#define NUM_REGS	((SCCM4>>2) + 4)

typedef struct cc32_sysc_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;

	uint32_t level;
	uint32_t irq_enabled;

	qemu_irq parent_irq;
	qemu_irq parent_fiq;

} cc32_sysc_state;

#define FIQ_MASK 	(1 << 8)
#define IRQ_MASK	~FIQ_MASK

static void cc32_sysc_update(cc32_sysc_state *s)
{
	uint32_t flags;

	flags = (s->level & s->irq_enabled & IRQ_MASK);
	qemu_set_irq(s->parent_irq, flags != 0);

	flags = (s->level & s->irq_enabled & FIQ_MASK);
	qemu_set_irq(s->parent_fiq, flags != 0);
}

static void cc32_sysc_set_irq(void *opaque, int irq, int level)
{
	cc32_sysc_state *s = (cc32_sysc_state *)opaque;

	if (level)
		s->level |= (1 << irq);
	else
		s->level &= ~(1 << irq);

	cc32_sysc_update(s);
}

static uint64_t cc32_sysc_read(void *opaque, target_phys_addr_t offset,
			     unsigned size)
{
	cc32_sysc_state *s = (cc32_sysc_state *)opaque;

	switch (offset) {
	case SCINTSTS:
		return s->level & s->irq_enabled;
	case SCINTEN:
		return s->irq_enabled;
	}

	return 0;
}

static void cc32_sysc_write(void *opaque, target_phys_addr_t offset,
			uint64_t value, unsigned size)
{
	cc32_sysc_state *s = (cc32_sysc_state *)opaque;
	switch (offset) {
	case SCINTEN:
		s->irq_enabled = value;
		fprintf(stderr, "cc32-sysc: INTEN = 0x%08x\n", s->irq_enabled);
		break;
	}
	cc32_sysc_update(s);
}

static const MemoryRegionOps cc32_sysc_ops = {
	.read = cc32_sysc_read,
	.write = cc32_sysc_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static int cc32_sysc_init(SysBusDevice *dev)
{
	cc32_sysc_state *s = FROM_SYSBUS(cc32_sysc_state, dev);

	qdev_init_gpio_in(&dev->qdev, cc32_sysc_set_irq, 32);
	sysbus_init_irq(dev, &s->parent_irq);
	sysbus_init_irq(dev, &s->parent_fiq);
	memory_region_init_io(&s->iomem, &cc32_sysc_ops, s, "cc32-sysc", NUM_REGS<<2);
	sysbus_init_mmio(dev, &s->iomem);

	return 0;
}

static void cc32_sysc_reset(DeviceState *d)
{
	cc32_sysc_state *s = container_of(d, cc32_sysc_state, busdev.qdev);

	s->irq_enabled = 0xFFFEF7FF;
	/* FIXME */
}

static void cc32_sysc_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = cc32_sysc_init;
	dc->reset = cc32_sysc_reset;
}

static TypeInfo cc32_sysc_info = {
	.name		= "cc32-sysc",
	.parent		= TYPE_SYS_BUS_DEVICE,
	.instance_size	= sizeof(cc32_sysc_state),
	.class_init 	= cc32_sysc_class_init,
};

static void cc32_register_types(void)
{
	type_register_static(&cc32_sysc_info);
	type_register_static(&cc32_flcon_info);
}

type_init(cc32_register_types)


/***********************************************************************
 * SoC core
 ***********************************************************************/


struct arm_boot_info cc32rs512_binfo;

static void cc32rs512_init(ram_addr_t ram_size,
		const char *boot_device,
		const char *kernel_filename, const char *kernel_cmdline,
		const char *initrd_filename, const char *cpu_model)
{
	CPUState *env;
	MemoryRegion *sysmem = get_system_memory();
	MemoryRegion *ram = g_new(MemoryRegion, 1);
	MemoryRegion *rsa_ram = g_new(MemoryRegion, 1);
	MemoryRegion *flash = g_new(MemoryRegion, 1);
	qemu_irq *cpu_irq;
	qemu_irq pic[32];
	DeviceState *dev;
	cc32_flcon_state *fl;
	int i;

	if (!cpu_model)
		cpu_model = "arm926";
	env = cpu_init(cpu_model);
	if (!env) {
		fprintf(stderr, "Unable to find CPU definition\n");
		exit(1);
	}

	memory_region_init_ram(ram, "cc32rs512.ram", 16*1024);
	vmstate_register_ram_global(ram);
	memory_region_add_subregion(sysmem, 0xc0000, ram);

	memory_region_init_ram(rsa_ram, "cc32rs512.rsa_ram", 2*1024);
	vmstate_register_ram_global(rsa_ram);
	memory_region_add_subregion(sysmem, 0xd4000, rsa_ram);

	cc32rs512_binfo.ram_size = ram_size;
	cc32rs512_binfo.kernel_filename = kernel_filename;

	cpu_irq = arm_pic_init_cpu(env);
	dev = sysbus_create_varargs("cc32-sysc", 0x0F0000,
				    cpu_irq[ARM_PIC_CPU_IRQ],
				    cpu_irq[ARM_PIC_CPU_FIQ], NULL);

	for (i = 0; i < 32; i++)
		pic[i] = qdev_get_gpio_in(dev, i);

	sysbus_create_simple("cc32-iso-slave", 0x0F8800, pic[8]);

	dev = sysbus_create_simple("cc32-flcon", 0x0F2000, pic[6]);
	fl = container_of(dev, cc32_flcon_state, busdev.qdev);
	memory_region_init_rom_device(flash, &cc32_flash_ops, fl,
					"cc32rs512.flash", 512*1024);
	vmstate_register_ram_global(flash);
	fl->storage = memory_region_get_ram_ptr(flash);
	fl->ram_storage = memory_region_get_ram_ptr(ram);
	memory_region_add_subregion(sysmem, 0, flash);

	arm_load_kernel(env, &cc32rs512_binfo);
}

static QEMUMachine cc32rs512_machine = {
	.name = "cc32rs512",
	.desc = "Chip City Smart Card (SC100)",
	.init = cc32rs512_init,
	.max_cpus = 1,
};

static void cc32rs512_machine_init(void)
{
	qemu_register_machine(&cc32rs512_machine);
}

machine_init(cc32rs512_machine_init);
