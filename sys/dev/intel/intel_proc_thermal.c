/*-
 * Copyright (c) 2026 Abdelkader Boudih <freebsd@seuros.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel Processor Thermal Device driver (Skylake)
 * Exposes RAPL power limits and thermal control via sysctl.
 *
 * Register offsets from Intel Processor Thermal Device documentation.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/sysctl.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/specialreg.h>
#include <sys/rman.h>
#include <machine/resource.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

/*
 * RAPL MMIO Register Offsets (relative to BAR0)
 */
#define RAPL_PKG_POWER_LIMIT		0x59A0
#define RAPL_PKG_POWER_INFO		0x5994

/* Power limit register bits */
#define POWER_LIMIT_ENABLE		(1ULL << 15)
#define POWER_LIMIT2_ENABLE		(1ULL << 47)
#define POWER_LIMIT_LOCK		(1ULL << 63)
#define POWER_LIMIT_MASK		0x7FFF
#define POWER_LIMIT2_SHIFT		32

/* Default power unit divisor (1/8 W) if MSR read fails */
#define POWER_UNIT_DEFAULT		8

struct intel_proc_thermal_softc {
	int			rid;
	struct resource		*res;
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	int			access_denied;
	uint32_t		power_unit_div;
	uint8_t			power_unit_shift;
};

static const struct pci_device_table intel_proc_thermal_devices[] = {
	/* Only Skylake validated - other generations may differ */
	{ PCI_DEV(0x8086, 0x1903), PCI_DESCR("Skylake Processor Thermal")},
};

static inline uint32_t
pth_read32(struct intel_proc_thermal_softc *sc, uint32_t offset)
{
	return (bus_space_read_4(sc->bst, sc->bsh, offset));
}

static inline uint64_t
pth_read64(struct intel_proc_thermal_softc *sc, uint32_t offset)
{
	uint64_t lo, hi;
	lo = bus_space_read_4(sc->bst, sc->bsh, offset);
	hi = bus_space_read_4(sc->bst, sc->bsh, offset + 4);
	return (lo | (hi << 32));
}

static int
proc_thermal_pl1_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_proc_thermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	uint32_t watts_mw;

	if (sc->access_denied) {
		watts_mw = 0;
	} else {
		pl = pth_read64(sc, RAPL_PKG_POWER_LIMIT);
		watts_mw = ((pl & POWER_LIMIT_MASK) * 1000ULL) / sc->power_unit_div;
	}
	return sysctl_handle_32(oidp, &watts_mw, 0, req);
}

static int
proc_thermal_pl2_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_proc_thermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	uint32_t watts_mw;

	if (sc->access_denied) {
		watts_mw = 0;
	} else {
		pl = pth_read64(sc, RAPL_PKG_POWER_LIMIT);
		watts_mw = (((pl >> POWER_LIMIT2_SHIFT) & POWER_LIMIT_MASK) *
		    1000ULL) / sc->power_unit_div;
	}
	return sysctl_handle_32(oidp, &watts_mw, 0, req);
}

static int
proc_thermal_pl1_enable_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_proc_thermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	int enabled;

	if (sc->access_denied) {
		enabled = 0;
	} else {
		pl = pth_read64(sc, RAPL_PKG_POWER_LIMIT);
		enabled = (pl & POWER_LIMIT_ENABLE) ? 1 : 0;
	}
	return sysctl_handle_int(oidp, &enabled, 0, req);
}

static int
proc_thermal_pl2_enable_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_proc_thermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	int enabled;

	if (sc->access_denied) {
		enabled = 0;
	} else {
		pl = pth_read64(sc, RAPL_PKG_POWER_LIMIT);
		enabled = (pl & POWER_LIMIT2_ENABLE) ? 1 : 0;
	}
	return sysctl_handle_int(oidp, &enabled, 0, req);
}

static int
proc_thermal_locked_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_proc_thermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	int locked;

	if (sc->access_denied) {
		locked = 1;
	} else {
		pl = pth_read64(sc, RAPL_PKG_POWER_LIMIT);
		locked = (pl & POWER_LIMIT_LOCK) ? 1 : 0;
	}
	return sysctl_handle_int(oidp, &locked, 0, req);
}

static int
proc_thermal_tdp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_proc_thermal_softc *sc = oidp->oid_arg1;
	uint32_t info;
	uint32_t tdp_mw;

	if (sc->access_denied) {
		tdp_mw = 0;
	} else {
		info = pth_read32(sc, RAPL_PKG_POWER_INFO);
		tdp_mw = ((info & POWER_LIMIT_MASK) * 1000ULL) / sc->power_unit_div;
	}
	return sysctl_handle_32(oidp, &tdp_mw, 0, req);
}

static int
intel_proc_thermal_probe(device_t dev)
{
	const struct pci_device_table *tbl;

	tbl = PCI_MATCH(dev, intel_proc_thermal_devices);
	if (tbl == NULL)
		return (ENXIO);
	device_set_desc(dev, tbl->descr);
	return (BUS_PROBE_DEFAULT);
}

static int
intel_proc_thermal_attach(device_t dev)
{
	struct intel_proc_thermal_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	uint32_t pwr_limit_lo, pwr_limit_hi, pwr_info;
	uint64_t rapl_units;
	uint32_t pu;

	sc->rid = PCIR_BAR(0);
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate BAR0\n");
		return (ENOMEM);
	}

	sc->bst = rman_get_bustag(sc->res);
	sc->bsh = rman_get_bushandle(sc->res);

	/*
	 * Read RAPL power unit from MSR 0x606.
	 * Bits 3:0 = power unit exponent, units are 1/2^PU Watts.
	 * Fallback to 1/8 W (PU=3) if MSR read fails.
	 */
	sc->power_unit_div = POWER_UNIT_DEFAULT;
	sc->power_unit_shift = 3;
	if (rdmsr_safe(MSR_RAPL_POWER_UNIT, &rapl_units) == 0) {
		pu = rapl_units & 0xF;
		if (pu <= 30) {
			sc->power_unit_shift = pu;
			sc->power_unit_div = 1U << pu;
		}
	}

	/*
	 * Check multiple registers to detect firmware lock.
	 * All reading 0xFFFFFFFF indicates access denied.
	 */
	pwr_limit_lo = pth_read32(sc, RAPL_PKG_POWER_LIMIT);
	pwr_limit_hi = pth_read32(sc, RAPL_PKG_POWER_LIMIT + 4);
	pwr_info = pth_read32(sc, RAPL_PKG_POWER_INFO);

	if (pwr_limit_lo == 0xFFFFFFFF && pwr_limit_hi == 0xFFFFFFFF &&
	    pwr_info == 0xFFFFFFFF) {
		sc->access_denied = 1;
		device_printf(dev, "MMIO access denied by firmware\n");
	} else {
		sc->access_denied = 0;
	}

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "access_denied", CTLFLAG_RD | CTLFLAG_MPSAFE, &sc->access_denied, 0,
	    "Firmware denied access to RAPL registers");

	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "power_unit_div", CTLFLAG_RD | CTLFLAG_MPSAFE, &sc->power_unit_div, 0,
	    "Power unit divisor (from MSR 0x606)");

	SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "power_unit_shift", CTLFLAG_RD | CTLFLAG_MPSAFE, &sc->power_unit_shift, 0,
	    "Power unit shift (bits 3:0 of MSR 0x606)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pl1", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, proc_thermal_pl1_sysctl, "IU",
	    "PL1 (long-term) power limit in milliwatts");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pl2", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, proc_thermal_pl2_sysctl, "IU",
	    "PL2 (short-term) power limit in milliwatts");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pl1_enabled", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, proc_thermal_pl1_enable_sysctl, "I",
	    "PL1 power limit enabled");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pl2_enabled", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, proc_thermal_pl2_enable_sysctl, "I",
	    "PL2 power limit enabled");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "locked", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, proc_thermal_locked_sysctl, "I",
	    "Power limits locked by firmware");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "tdp", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, proc_thermal_tdp_sysctl, "IU",
	    "Thermal Design Power in milliwatts");

	return (0);
}

static int
intel_proc_thermal_detach(device_t dev)
{
	struct intel_proc_thermal_softc *sc = device_get_softc(dev);

	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);
	return (0);
}

static device_method_t intel_proc_thermal_methods[] = {
	DEVMETHOD(device_probe,		intel_proc_thermal_probe),
	DEVMETHOD(device_attach,	intel_proc_thermal_attach),
	DEVMETHOD(device_detach,	intel_proc_thermal_detach),
	DEVMETHOD_END
};

static driver_t intel_proc_thermal_driver = {
	"intel_proc_thermal",
	intel_proc_thermal_methods,
	sizeof(struct intel_proc_thermal_softc)
};

DRIVER_MODULE(intel_proc_thermal, pci, intel_proc_thermal_driver, 0, 0);
PCI_PNP_INFO(intel_proc_thermal_devices);
MODULE_VERSION(intel_proc_thermal, 1);
MODULE_DEPEND(intel_proc_thermal, pci, 1, 1, 1);
