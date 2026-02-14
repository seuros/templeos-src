/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Scott Long
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "opt_thunderbolt.h"

/* PCIe interface for Thunderbolt Native Host Interface */
#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/param.h>
#include <sys/endian.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/stdarg.h>
#include <sys/rman.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pci_private.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <dev/acpica/acpivar.h>

#include <dev/thunderbolt/tb_reg.h>
#include <dev/thunderbolt/nhi_reg.h>
#include <dev/thunderbolt/nhi_var.h>
#include <dev/thunderbolt/tbcfg_reg.h>
#include <dev/thunderbolt/router_var.h>
#include <dev/thunderbolt/tb_debug.h>
#include "tb_if.h"

static int	nhi_pci_probe(device_t);
static int	nhi_pci_attach(device_t);
static int	nhi_pci_detach(device_t);
static int	nhi_pci_suspend(device_t);
static int	nhi_pci_resume(device_t);
static void	nhi_pci_free(struct nhi_softc *);
static int	nhi_pci_allocate_interrupts(struct nhi_softc *);
static int	nhi_acpi_power_on(struct nhi_softc *);
static void	nhi_pci_free_resources(struct nhi_softc *);

static device_method_t nhi_methods[] = {
	DEVMETHOD(device_probe, 	nhi_pci_probe),
	DEVMETHOD(device_attach,	nhi_pci_attach),
	DEVMETHOD(device_detach,	nhi_pci_detach),
	DEVMETHOD(device_suspend,	nhi_pci_suspend),
	DEVMETHOD(device_resume,	nhi_pci_resume),

	DEVMETHOD(tb_find_ufp,		tb_generic_find_ufp),
	DEVMETHOD(tb_get_debug,		tb_generic_get_debug),

	DEVMETHOD_END
};

static driver_t nhi_pci_driver = {
	"nhi",
	nhi_methods,
	sizeof(struct nhi_softc)
};

struct nhi_ident {
	uint16_t	vendor;
	uint16_t	device;
	uint16_t	subvendor;
	uint16_t	subdevice;
	uint8_t		class;
	uint8_t		subclass;
	uint32_t	flags;
	const char	*desc;
} nhi_identifiers[] = {
#define NHI_ANY_CLASS	0xff
	{ VENDOR_INTEL, DEVICE_LR_NHI, 0x2222, 0x1111,
	    PCIC_BASEPERIPH, PCIS_BASEPERIPH_OTHER, NHI_TYPE_FW_CM,
	    "Thunderbolt 1 NHI (Light Ridge)" },
	{ VENDOR_INTEL, DEVICE_CR_4C_NHI, 0x2222, 0x1111,
	    PCIC_BASEPERIPH, PCIS_BASEPERIPH_OTHER, NHI_TYPE_FW_CM,
	    "Thunderbolt 1 NHI (Cactus Ridge 4C)" },
	{ VENDOR_INTEL, DEVICE_FR_2C_NHI, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_FW_CM,
	    "Thunderbolt 2 NHI (Falcon Ridge 2C)" },
	{ VENDOR_INTEL, DEVICE_FR_4C_NHI, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_FW_CM,
	    "Thunderbolt 2 NHI (Falcon Ridge 4C)" },
	{ VENDOR_INTEL, DEVICE_AR_2C_NHI, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_AR,
	    "Thunderbolt 3 NHI (Alpine Ridge 2C)" },
	{ VENDOR_INTEL, DEVICE_AR_DP_B_NHI, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_AR,
	    "Thunderbolt 3 NHI (Alpine Ridge 4C Rev B)" },
	{ VENDOR_INTEL, DEVICE_AR_DP_C_NHI, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_AR,
	    "Thunderbolt 3 NHI (Alpine Ridge 4C Rev C)" },
	{ VENDOR_INTEL, DEVICE_AR_LP_NHI, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_AR,
	    "Thunderbolt 3 NHI (Alpine Ridge LP 2C)" },
	{ VENDOR_INTEL, DEVICE_ICL_NHI_0, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_ICL,
	    "Thunderbolt 3 NHI Port 0 (IceLake)" },
	{ VENDOR_INTEL, DEVICE_ICL_NHI_1, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_ICL,
	    "Thunderbolt 3 NHI Port 1 (IceLake)" },
	{ VENDOR_AMD, DEVICE_PINK_SARDINE_0, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_USB4,
	    "USB4 NHI Port 0 (Pink Sardine)" },
	{ VENDOR_AMD, DEVICE_PINK_SARDINE_1, 0xffff, 0xffff,
	    NHI_ANY_CLASS, NHI_ANY_CLASS, NHI_TYPE_USB4,
	    "USB4 NHI Port 1 (Pink Sardine)" },
#undef NHI_ANY_CLASS
	{ 0, 0, 0, 0, 0, 0, 0, NULL }
};

DRIVER_MODULE_ORDERED(nhi, pci, nhi_pci_driver, NULL, NULL,
    SI_ORDER_ANY);

static struct nhi_ident *
nhi_find_ident(device_t dev)
{
	struct nhi_ident *n;

	for (n = nhi_identifiers; n->vendor != 0; n++) {
		if (n->vendor != pci_get_vendor(dev))
			continue;
		if (n->device != pci_get_device(dev))
			continue;
		if ((n->class != 0xff) &&
		    (n->class != pci_get_class(dev)))
			continue;
		if ((n->subclass != 0xff) &&
		    (n->subclass != pci_get_subclass(dev)))
			continue;
		if ((n->subvendor != 0xffff) &&
		    (n->subvendor != pci_get_subvendor(dev)))
			continue;
		if ((n->subdevice != 0xffff) &&
		    (n->subdevice != pci_get_subdevice(dev)))
			continue;
		return (n);
	}

	return (NULL);
}

static int
nhi_pci_probe(device_t dev)
{
	struct nhi_ident *n;

	if (resource_disabled("tb", 0))
		return (ENXIO);
	/* TB1/TB2/TB3 by device ID */
	if ((n = nhi_find_ident(dev)) != NULL) {
		device_set_desc(dev, n->desc);
		return (BUS_PROBE_DEFAULT);
	}
	/* Generic USB4 NHI by class */
	if ((pci_get_class(dev) == PCIC_SERIALBUS)
	    && (pci_get_subclass(dev) == PCIS_SERIALBUS_USB)
	    && (pci_get_progif(dev) == PCIP_SERIALBUS_USB_USB4)) {
		device_set_desc(dev, "Generic USB4 NHI");
		return (BUS_PROBE_DEFAULT);
	}
	return (ENXIO);
}

/*
 * Call Apple ACPI XRPE method to power on the Thunderbolt controller.
 *
 * On Apple Macs, the Thunderbolt firmware CPU is started by the EFI
 * bootloader via the XRPE ACPI method.  When booting non-macOS, this
 * never happens and the firmware mailbox returns 0xdeadbeaf.
 *
 * XRPE lives on the NHI0 ACPI node.  Arg0: 1=power on, 0=power off.
 * The method is gated by OSDW() which checks OSYS==0x2710 (Darwin).
 * FreeBSD installs Darwin OSI on Apple hardware, so the gate passes.
 */
static int
nhi_acpi_power_on(struct nhi_softc *sc)
{
	ACPI_OBJECT arg;
	ACPI_OBJECT_LIST args;
	ACPI_STATUS status;

	arg.Type = ACPI_TYPE_INTEGER;
	arg.Integer.Value = 1;
	args.Count = 1;
	args.Pointer = &arg;

	/*
	 * Use the absolute ACPI path to invoke XRPE directly.
	 * The NHI PCI endpoint doesn't have a mapped ACPI handle,
	 * but AcpiEvaluateObject accepts fully qualified paths
	 * from the root namespace.
	 *
	 * Known paths on Apple Macs:
	 *   Mac Mini 2011: \_SB.PCI0.RP05.UPSB.DSB0.NHI0.XRPE
	 *   Other models may differ — try common paths.
	 */
	/*
	 * Find XRPE by walking ACPI handles up from the NHI device.
	 * Only evaluate if found — do not use hardcoded paths as they
	 * differ per Mac model and calling XRPE on the wrong hardware
	 * can hang.
	 */
	status = AE_NOT_FOUND;
	for (device_t d = sc->dev; d != NULL; d = device_get_parent(d)) {
		ACPI_HANDLE h = acpi_get_handle(d);
		ACPI_HANDLE xrpe_h;
		if (h == NULL)
			continue;
		if (ACPI_SUCCESS(AcpiGetHandle(h, "XRPE", &xrpe_h))) {
			tb_printf(sc, "Found XRPE on %s, powering on\n",
			    device_get_nameunit(d));
			status = AcpiEvaluateObject(xrpe_h, NULL,
			    &args, NULL);
			break;
		}
	}

	if (ACPI_FAILURE(status)) {
		tb_debug(sc, DBG_INIT, "XRPE not available (%s)\n",
		    AcpiFormatException(status));
		return (ENOENT);
	}

	tb_printf(sc, "XRPE(1) succeeded, waiting for firmware\n");
	pause("tbpwr", hz * 2);

	return (0);
}

static int
nhi_pci_attach(device_t dev)
{
	devclass_t dc;
	bus_dma_template_t t;
	struct nhi_softc *sc;
	struct nhi_ident *n;
	int error = 0;

	sc = device_get_softc(dev);
	bzero(sc, sizeof(*sc));
	sc->dev = dev;
	n = nhi_find_ident(dev);
	sc->hwflags = (n != NULL) ? n->flags : NHI_TYPE_USB4;
	nhi_get_tunables(sc);

	tb_debug(sc, DBG_INIT|DBG_FULL, "busmaster status was %s\n",
	    (pci_read_config(dev, PCIR_COMMAND, 2) & PCIM_CMD_BUSMASTEREN)
	    ? "enabled" : "disabled");
	pci_enable_busmaster(dev);

	sc->ufp = NULL;
	if ((TB_FIND_UFP(dev, &sc->ufp) != 0) || (sc->ufp == NULL)) {
		dc = devclass_find("tbolt");
		if (dc != NULL)
			sc->ufp = devclass_get_device(dc, device_get_unit(dev));
	}
	if (sc->ufp == NULL)
		tb_printf(sc, "Cannot find Upstream Facing Port\n");
	else
		tb_printf(sc, "Upstream Facing Port is %s\n",
		    device_get_nameunit(sc->ufp));

	/* Allocate BAR0 DMA registers */
	sc->regs_rid = PCIR_BAR(0);
	if ((sc->regs_resource = bus_alloc_resource_any(dev,
	    SYS_RES_MEMORY, &sc->regs_rid, RF_ACTIVE)) == NULL) {
		tb_printf(sc, "Cannot allocate PCI registers\n");
		return (ENXIO);
	}
	sc->regs_btag = rman_get_bustag(sc->regs_resource);
	sc->regs_bhandle = rman_get_bushandle(sc->regs_resource);

	/* Allocate parent DMA tag */
	bus_dma_template_init(&t, bus_get_dma_tag(dev));
	if (bus_dma_template_tag(&t, &sc->parent_dmat) != 0) {
		tb_printf(sc, "Cannot allocate parent DMA tag\n");
		nhi_pci_free(sc);
		return (ENOMEM);
	}

	/*
	 * On Apple Macs with FW-CM controllers, call XRPE to power on
	 * the Thunderbolt firmware CPU before nhi_attach touches the
	 * mailbox registers.
	 */
	if (NHI_IS_FW_CM(sc))
		(void)nhi_acpi_power_on(sc);

	error = nhi_pci_allocate_interrupts(sc);
	if (error == 0)
		error = nhi_attach(sc);
	if (error != 0)
		nhi_pci_detach(sc->dev);
	return (error);
}

static int
nhi_pci_detach(device_t dev)
{
	struct nhi_softc *sc;

	sc = device_get_softc(dev);

	nhi_detach(sc);
	nhi_pci_free(sc);

	return (0);
}

static int
nhi_pci_suspend(device_t dev)
{

	return (0);
}

static int
nhi_pci_resume(device_t dev)
{

	return (0);
}

static void
nhi_pci_free(struct nhi_softc *sc)
{

	nhi_pci_free_resources(sc);

	if (sc->parent_dmat != NULL) {
		bus_dma_tag_destroy(sc->parent_dmat);
		sc->parent_dmat = NULL;
	}

	if (sc->regs_resource != NULL) {
		bus_release_resource(sc->dev, SYS_RES_MEMORY,
		    sc->regs_rid, sc->regs_resource);
		sc->regs_resource = NULL;
	}

	return;
}

static int
nhi_pci_allocate_interrupts(struct nhi_softc *sc)
{
	int msgs, error = 0;

	/* Map the Pending Bit Array and Vector Table BARs for MSI-X */
	sc->irq_pba_rid = pci_msix_pba_bar(sc->dev);
	sc->irq_table_rid = pci_msix_table_bar(sc->dev);

	if (sc->irq_pba_rid != -1)
		sc->irq_pba = bus_alloc_resource_any(sc->dev, SYS_RES_MEMORY,
		    &sc->irq_pba_rid, RF_ACTIVE);
	if (sc->irq_table_rid != -1)
		sc->irq_table = bus_alloc_resource_any(sc->dev, SYS_RES_MEMORY,
		    &sc->irq_table_rid, RF_ACTIVE);

	msgs = pci_msix_count(sc->dev);
	tb_debug(sc, DBG_INIT|DBG_INTR|DBG_FULL,
	    "Counted %d MSI-X messages\n", msgs);
	msgs = min(msgs, NHI_MSIX_MAX);
	msgs = max(msgs, 1);
	if (msgs != 0) {
		tb_debug(sc, DBG_INIT|DBG_INTR, "Attempting to allocate %d "
		    "MSI-X interrupts\n", msgs);
		error = pci_alloc_msix(sc->dev, &msgs);
		tb_debug(sc, DBG_INIT|DBG_INTR|DBG_FULL,
		    "pci_alloc_msix return msgs= %d, error= %d\n", msgs, error);
	}

	if ((error != 0) || (msgs <= 0)) {
		tb_printf(sc, "Failed to allocate any interrupts\n");
		msgs = 0;
	}

	sc->msix_count = msgs;
	return (error);
}

void
nhi_pci_free_interrupts(struct nhi_softc *sc)
{
	int i;

	for (i = 0; i < sc->msix_count; i++) {
		bus_teardown_intr(sc->dev, sc->irqs[i], sc->intrhand[i]);
		bus_release_resource(sc->dev, SYS_RES_IRQ, sc->irq_rid[i],
		    sc->irqs[i]);
	}

	pci_release_msi(sc->dev);
}

static void
nhi_pci_free_resources(struct nhi_softc *sc)
{
	if (sc->irq_table != NULL) {
		bus_release_resource(sc->dev, SYS_RES_MEMORY,
		    sc->irq_table_rid, sc->irq_table);
		sc->irq_table = NULL;
	}

	if (sc->irq_pba != NULL) {
		bus_release_resource(sc->dev, SYS_RES_MEMORY,
		    sc->irq_pba_rid, sc->irq_pba);
		sc->irq_pba = NULL;
	}

	if (sc->intr_trackers != NULL)
		free(sc->intr_trackers, M_NHI);
	return;
}

int
nhi_pci_configure_interrupts(struct nhi_softc *sc)
{
	struct nhi_intr_tracker *trkr;
	int rid, i, error;

	nhi_pci_disable_interrupts(sc);

	sc->intr_trackers = malloc(sizeof(struct nhi_intr_tracker) *
	    sc->msix_count, M_NHI, M_ZERO | M_NOWAIT);
	if (sc->intr_trackers == NULL) {
		tb_debug(sc, DBG_INIT, "Cannot allocate intr trackers\n");
		return (ENOMEM);
	}

	for (i = 0; i < sc->msix_count; i++) {
		rid = i + 1;
		trkr = &sc->intr_trackers[i];
		trkr->sc = sc;
		trkr->ring = NULL;
		trkr->vector = i;

		sc->irq_rid[i] = rid;
		sc->irqs[i] = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ,
			&sc->irq_rid[i], RF_ACTIVE);
		if (sc->irqs[i] == NULL) {
			tb_debug(sc, DBG_INIT,
			    "Cannot allocate interrupt RID %d\n",
			    sc->irq_rid[i]);
			break;
		}
		error = bus_setup_intr(sc->dev, sc->irqs[i], INTR_TYPE_BIO |
		    INTR_MPSAFE, NULL, nhi_intr, trkr, &sc->intrhand[i]);
		if (error) {
			tb_debug(sc, DBG_INIT,
			    "cannot setup interrupt RID %d\n", sc->irq_rid[i]);
			break;
		}
	}

	tb_debug(sc, DBG_INIT, "Set up %d interrupts\n", sc->msix_count);

	/* Set the interrupt throttle rate to 128us */
	for (i = 0; i < 16; i ++)
		nhi_write_reg(sc, NHI_ITR0 + i * 4, 0x1f4);

	return (error);
}

#define NHI_SET_INTERRUPT(offset, mask, val)	\
do {					\
	reg = offset / 32;		\
	offset %= 32;			\
	ivr[reg] &= ~(mask << offset);	\
	ivr[reg] |= (val << offset);	\
} while (0)

void
nhi_pci_enable_interrupt(struct nhi_ring_pair *r)
{
	struct nhi_softc *sc = r->sc;
	uint32_t ivr[12];
	u_int offset, reg, max_ivr_regs, max_imr_regs, i, ivr_descs;
	u_int path_count;
	int has_ne;

	tb_debug(sc, DBG_INIT|DBG_INTR, "Enabling interrupts for ring %d\n",
	    r->ring_num);

	path_count = sc->path_count;
	has_ne = (path_count * 3 <= 64);

	/*
	 * Compute the routing between event type and MSI-X vector.
	 * 4 bits per descriptor.
	 * IVR has TX and RX descriptors, and NE on <= 21 path controllers.
	 * Each descriptor is 4 bits.
	 * Number of 32-bit registers depends on descriptor count.
	 */
	ivr_descs = path_count * (has_ne ? 3 : 2);
	max_ivr_regs = (ivr_descs * 4 + 31) / 32;
	if (max_ivr_regs > 12)
		max_ivr_regs = 12;

	for (i = 0; i < max_ivr_regs; i++)
		ivr[i] = nhi_read_reg(sc, NHI_IVR0 + i * 4);

	/* Program TX - ring N gets bit N in IVR space */
	offset = r->ring_num * 4;
	NHI_SET_INTERRUPT(offset, 0x0f, r->ring_num);

	/* Program RX - ring N gets bit (N + path_count) */
	offset = (r->ring_num + path_count) * 4;
	NHI_SET_INTERRUPT(offset, 0x0f, r->ring_num);

	if (has_ne) {
		/* Program Nearly Empty - ring N gets bit (N + 2*path_count) */
		offset = (r->ring_num + 2 * path_count) * 4;
		NHI_SET_INTERRUPT(offset, 0x0f, 0x0f);
	}

	for (i = 0; i < max_ivr_regs; i++)
		nhi_write_reg(sc, NHI_IVR0 + i * 4, ivr[i]);

	tb_debug(sc, DBG_INIT|DBG_INTR|DBG_FULL,
	    "Wrote %d IVR registers\n", max_ivr_regs);

	/*
	 * Interrupt Mask Register, 1 bit per descriptor.
	 * IMR has TX/RX bits; NE bits are available on <= 21 path controllers.
	 */
	max_imr_regs = has_ne ? ((path_count * 3 + 31) / 32) :
	    RING_INTERRUPT_REG_COUNT(path_count);
	if (max_imr_regs > 12)
		max_imr_regs = 12;

	for (i = 0; i < max_imr_regs; i++)
		ivr[i] = nhi_read_reg(sc, NHI_IMR0 + i * 4);

	/* TX - ring N gets bit N */
	offset = r->ring_num;
	NHI_SET_INTERRUPT(offset, 0x01, 1);

	/* RX - ring N gets bit (N + path_count) */
	offset = r->ring_num + path_count;
	NHI_SET_INTERRUPT(offset, 0x01, 1);

	if (has_ne) {
		/* NE - ring N gets bit (N + 2*path_count) */
		offset = r->ring_num + 2 * path_count;
		NHI_SET_INTERRUPT(offset, 0x01, 1);
	}

	for (i = 0; i < max_imr_regs; i++)
		nhi_write_reg(sc, NHI_IMR0 + i * 4, ivr[i]);

	tb_debug(sc, DBG_INIT|DBG_FULL,
	    "Wrote %d IMR registers\n", max_imr_regs);
}

void
nhi_pci_disable_interrupts(struct nhi_softc *sc)
{
	u_int max_imr_regs, max_isr_regs, max_ivr_regs, i, ivr_descs;
	u_int path_count;
	int has_ne;

	tb_debug(sc, DBG_INIT, "Disabling interrupts\n");

	/*
	 * Clear all interrupt and notify registers based on path count.
	 * If the device is not yet initialized, fall back to HOST_CAPS
	 * and clamp to the driver max.
	 */
	path_count = sc->path_count;
	if (path_count == 0)
		path_count = GET_HOST_CAPS_PATHS(nhi_read_reg(sc,
		    NHI_HOST_CAPS));
	if (path_count == 0)
		path_count = NHI_MAX_NUM_RINGS;
	if (path_count > NHI_MAX_NUM_RINGS)
		path_count = NHI_MAX_NUM_RINGS;

	has_ne = (path_count * 3 <= 64);
	max_imr_regs = has_ne ? ((path_count * 3 + 31) / 32) :
	    RING_INTERRUPT_REG_COUNT(path_count);
	max_isr_regs = RING_NOTIFY_REG_COUNT(path_count);
	ivr_descs = path_count * (has_ne ? 3 : 2);
	max_ivr_regs = (ivr_descs * 4 + 31) / 32;
	if (max_ivr_regs > 12)
		max_ivr_regs = 12;

	/* Clear IMR registers */
	for (i = 0; i < max_imr_regs && i < 12; i++)
		nhi_write_reg(sc, NHI_IMR0 + i * 4, 0);

	/* Clear IVR registers */
	for (i = 0; i < max_ivr_regs; i++)
		nhi_write_reg(sc, NHI_IVR0 + i * 4, 0);

	/* Dummy reads to clear pending bits in ISR registers */
	for (i = 0; i < max_isr_regs && i < 12; i++)
		nhi_read_reg(sc, NHI_ISR0 + i * 4);
}
