/*-
 * Copyright (c) 2026 Abdelkader Boudih <freebsd@seuros.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel PMC (Power Management Controller) Core driver
 * Exposes sleep state residency counters and power gating status
 * for debugging S0ix (Modern Standby) power states.
 *
 * Register offsets from Intel 100 Series Chipset Family PCH Datasheet Vol 2
 *
 * NOTE: This driver only supports Sunrise Point (different PCH generations
 * have different register layouts). Some firmware/BIOS implementations
 * lock PMC access - if registers read as 0xFFFFFFFF, access is denied.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/sysctl.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

/*
 * Sunrise Point (SPT) PMC Register Definitions
 * Reference: Intel 100 Series Chipset Family PCH Datasheet Vol 2
 */
#define SPT_PMC_PM_CFG_OFFSET		0x18
#define SPT_PMC_PM_STS_OFFSET		0x1C
#define SPT_PMC_SLP_S0_RES_OFFSET	0x13C
#define SPT_PMC_SLP_S0_RES_STEP		100	/* 100us granularity */
#define SPT_PMC_LTR_IGNORE_OFFSET	0x30C

struct intel_pmc_core_softc {
	int			rid;
	struct resource		*res;
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	uint32_t		slp_s0_step;
	int			access_denied;
};

static const struct pci_device_table intel_pmc_core_devices[] = {
	/* Only Sunrise Point - other generations have different layouts */
	{ PCI_DEV(0x8086, 0x9d21), PCI_DESCR("Sunrise Point-LP PMC")},
	{ PCI_DEV(0x8086, 0xa121), PCI_DESCR("Sunrise Point-H PMC")},
};

static inline uint32_t
pmc_read(struct intel_pmc_core_softc *sc, uint32_t offset)
{
	return (bus_space_read_4(sc->bst, sc->bsh, offset));
}

static int
pmc_core_slp_s0_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_core_softc *sc = oidp->oid_arg1;
	uint64_t val;
	uint64_t usec;

	if (sc->access_denied)
		usec = 0;
	else {
		val = pmc_read(sc, SPT_PMC_SLP_S0_RES_OFFSET);
		usec = val * sc->slp_s0_step;
	}
	return sysctl_handle_64(oidp, &usec, 0, req);
}

static int
pmc_core_ltr_ignore_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_core_softc *sc = oidp->oid_arg1;
	uint32_t val;

	if (sc->access_denied)
		val = 0;
	else
		val = pmc_read(sc, SPT_PMC_LTR_IGNORE_OFFSET);
	return sysctl_handle_32(oidp, &val, 0, req);
}

static int
pmc_core_pm_cfg_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_core_softc *sc = oidp->oid_arg1;
	uint32_t val;

	if (sc->access_denied)
		val = 0;
	else
		val = pmc_read(sc, SPT_PMC_PM_CFG_OFFSET);
	return sysctl_handle_32(oidp, &val, 0, req);
}

static int
pmc_core_pm_sts_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_core_softc *sc = oidp->oid_arg1;
	uint32_t val;

	if (sc->access_denied)
		val = 0;
	else
		val = pmc_read(sc, SPT_PMC_PM_STS_OFFSET);
	return sysctl_handle_32(oidp, &val, 0, req);
}

static int
intel_pmc_core_probe(device_t dev)
{
	const struct pci_device_table *tbl;

	tbl = PCI_MATCH(dev, intel_pmc_core_devices);
	if (tbl == NULL)
		return (ENXIO);
	device_set_desc(dev, tbl->descr);
	return (BUS_PROBE_DEFAULT);
}

static int
intel_pmc_core_attach(device_t dev)
{
	struct intel_pmc_core_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	uint32_t pm_cfg, pm_sts, ltr_ign, slp_s0;

	sc->rid = PCIR_BAR(0);
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate PMC BAR0\n");
		return (ENOMEM);
	}

	sc->bst = rman_get_bustag(sc->res);
	sc->bsh = rman_get_bushandle(sc->res);
	sc->slp_s0_step = SPT_PMC_SLP_S0_RES_STEP;

	/*
	 * Check multiple registers to detect firmware lock.
	 * All reading 0xFFFFFFFF indicates access denied.
	 */
	pm_cfg = pmc_read(sc, SPT_PMC_PM_CFG_OFFSET);
	pm_sts = pmc_read(sc, SPT_PMC_PM_STS_OFFSET);
	ltr_ign = pmc_read(sc, SPT_PMC_LTR_IGNORE_OFFSET);
	slp_s0 = pmc_read(sc, SPT_PMC_SLP_S0_RES_OFFSET);

	if (pm_cfg == 0xFFFFFFFF && pm_sts == 0xFFFFFFFF &&
	    ltr_ign == 0xFFFFFFFF && slp_s0 == 0xFFFFFFFF) {
		sc->access_denied = 1;
		device_printf(dev,
		    "PMC access denied by firmware (S0ix not supported)\n");
	} else
		sc->access_denied = 0;

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "access_denied", CTLFLAG_RD | CTLFLAG_MPSAFE, &sc->access_denied, 0,
	    "Firmware denied access to PMC registers");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "slp_s0_residency", CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, pmc_core_slp_s0_sysctl, "QU",
	    "Cumulative time in any S0ix sub-state (microseconds)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "ltr_ignore", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, pmc_core_ltr_ignore_sysctl, "IU",
	    "LTR ignore mask");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pm_cfg", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, pmc_core_pm_cfg_sysctl, "IU",
	    "Power Management configuration");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pm_sts", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, pmc_core_pm_sts_sysctl, "IU",
	    "Power Management status");

	return (0);
}

static int
intel_pmc_core_detach(device_t dev)
{
	struct intel_pmc_core_softc *sc = device_get_softc(dev);

	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);
	return (0);
}

static device_method_t intel_pmc_core_methods[] = {
	DEVMETHOD(device_probe,		intel_pmc_core_probe),
	DEVMETHOD(device_attach,	intel_pmc_core_attach),
	DEVMETHOD(device_detach,	intel_pmc_core_detach),
	DEVMETHOD_END
};

static driver_t intel_pmc_core_driver = {
	"intel_pmc_core",
	intel_pmc_core_methods,
	sizeof(struct intel_pmc_core_softc)
};

DRIVER_MODULE(intel_pmc_core, pci, intel_pmc_core_driver, 0, 0);
PCI_PNP_INFO(intel_pmc_core_devices);
MODULE_VERSION(intel_pmc_core, 1);
MODULE_DEPEND(intel_pmc_core, pci, 1, 1, 1);
