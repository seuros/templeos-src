/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Abdelkader Boudih <freebsd@seuros.com>
 */

/*
 * intelpmc — Intel PCH Power Management Controller driver
 *
 * Exposes sleep state residency counters and power gating status
 * for debugging S0ix (Modern Standby) power states.
 *
 * Register offsets from Intel 100 Series Chipset Family PCH Datasheet Vol 2
 * Section 4.3.53 describes the SLP_S0_RESIDENCY_COUNTER (offset 0x13C).
 *
 * The SLP_S0_RESIDENCY counter increments (100 us per step) while the PCH
 * SLP_S0 signal is asserted.  SLP_S0 is asserted only when all of the
 * following conditions are met simultaneously:
 *   - CPU in Package C10 (deepest package C-state)
 *   - PCH in low power mode
 *   - ModPhy lanes power-gated
 *   - PLLs idle
 * This represents the deepest achievable S0ix platform state.  A counter
 * that never advances indicates S0ix transitions are not occurring, which
 * is useful for diagnosing Modern Standby power regressions.
 *
 * Ref: https://patchwork.kernel.org/project/platform-driver-x86/patch/\
 *      1462891545-24060-1-git-send-email-rajneesh.bhardwaj@intel.com/
 * Ref: Intel 100 Series Chipset Family PCH Datasheet Vol 2, §4.3.53
 *
 * NOTE: This driver only supports Sunrise Point (different PCH generations
 * have different register layouts). Some firmware/BIOS implementations
 * lock PMC access - if registers read as 0xFFFFFFFF, access is denied.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

/*
 * Sunrise Point (SPT) PMC Register Definitions
 * Reference: Intel 100 Series Chipset Family PCH Datasheet Vol 2
 */
#define SPT_PMC_PM_CFG_OFFSET		0x18
#define SPT_PMC_PM_STS_OFFSET		0x1C
#define SPT_PMC_SLP_S0_RES_OFFSET	0x13C
#define SPT_PMC_SLP_S0_RES_STEP		100	/* 100us per datasheet */
#define SPT_PMC_LTR_IGNORE_OFFSET	0x30C

struct intelpmc_softc {
	int			rid;
	struct resource		*res;
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	int			access_denied;
};

static const struct pci_device_table intelpmc_devices[] = {
	/* Only Sunrise Point - other generations have different layouts */
	{ PCI_DEV(0x8086, 0x9d21), PCI_DESCR("Sunrise Point-LP PMC")},
	{ PCI_DEV(0x8086, 0xa121), PCI_DESCR("Sunrise Point-H PMC")},
};

static inline uint32_t
intelpmc_read(struct intelpmc_softc *sc, uint32_t offset)
{
	return (bus_space_read_4(sc->bst, sc->bsh, offset));
}

static inline uint32_t
intelpmc_read_safe(struct intelpmc_softc *sc, uint32_t offset)
{
	if (sc->access_denied)
		return (0);
	return (intelpmc_read(sc, offset));
}

static int
intelpmc_slp_s0_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intelpmc_softc *sc = oidp->oid_arg1;
	uint64_t val, usec;

	val = intelpmc_read_safe(sc, SPT_PMC_SLP_S0_RES_OFFSET);
	usec = val * SPT_PMC_SLP_S0_RES_STEP;
	return (sysctl_handle_64(oidp, &usec, 0, req));
}

static int
intelpmc_reg32_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intelpmc_softc *sc = oidp->oid_arg1;
	uint32_t val;

	val = intelpmc_read_safe(sc, oidp->oid_arg2);
	return (sysctl_handle_32(oidp, &val, 0, req));
}

static int
intelpmc_probe(device_t dev)
{
	const struct pci_device_table *tbl;

	tbl = PCI_MATCH(dev, intelpmc_devices);
	if (tbl == NULL)
		return (ENXIO);
	device_set_desc(dev, tbl->descr);
	return (BUS_PROBE_DEFAULT);
}

static int
intelpmc_attach(device_t dev)
{
	struct intelpmc_softc *sc = device_get_softc(dev);
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

	/*
	 * Check multiple registers to detect firmware lock.
	 * All reading 0xFFFFFFFF indicates access denied.
	 */
	pm_cfg = intelpmc_read(sc, SPT_PMC_PM_CFG_OFFSET);
	pm_sts = intelpmc_read(sc, SPT_PMC_PM_STS_OFFSET);
	ltr_ign = intelpmc_read(sc, SPT_PMC_LTR_IGNORE_OFFSET);
	slp_s0 = intelpmc_read(sc, SPT_PMC_SLP_S0_RES_OFFSET);

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
	    sc, 0, intelpmc_slp_s0_sysctl, "QU",
	    "Time PCH SLP_S0 signal asserted (PC10+PCH low power+ModPhy gated+PLLs idle), in microseconds");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "ltr_ignore", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, SPT_PMC_LTR_IGNORE_OFFSET, intelpmc_reg32_sysctl, "IU",
	    "LTR ignore mask");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pm_cfg", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, SPT_PMC_PM_CFG_OFFSET, intelpmc_reg32_sysctl, "IU",
	    "Power Management configuration");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pm_sts", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, SPT_PMC_PM_STS_OFFSET, intelpmc_reg32_sysctl, "IU",
	    "Power Management status");

	return (0);
}

static int
intelpmc_detach(device_t dev)
{
	struct intelpmc_softc *sc = device_get_softc(dev);

	bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);
	return (0);
}

static device_method_t intelpmc_methods[] = {
	DEVMETHOD(device_probe,		intelpmc_probe),
	DEVMETHOD(device_attach,	intelpmc_attach),
	DEVMETHOD(device_detach,	intelpmc_detach),
	DEVMETHOD_END
};

static driver_t intelpmc_driver = {
	"intelpmc",
	intelpmc_methods,
	sizeof(struct intelpmc_softc)
};

DRIVER_MODULE(intelpmc, pci, intelpmc_driver, 0, 0);
PCI_PNP_INFO(intelpmc_devices);
MODULE_VERSION(intelpmc, 1);
MODULE_DEPEND(intelpmc, pci, 1, 1, 1);
