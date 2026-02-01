/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Abdelkader Boudih <freebsd@seuros.com>
 */

/*
 * intelthermal — Intel Processor Thermal Device driver
 *
 * Exposes RAPL power limits (PL1/PL2) and TDP via sysctl for
 * Skylake and later processors with B0D4 thermal device.
 *
 * Register offsets from Intel 10th Gen Core Processor Datasheet Vol 2.
 */

#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <machine/specialreg.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

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
#define	POWER_UNIT_DEFAULT		8
#define	RAPL_POWER_UNIT_MASK		0x0F
#define	RAPL_POWER_UNIT_MAX_SHIFT	30
#define	MW_PER_WATT			1000ULL
#define	MMIO_ACCESS_DENIED		UINT32_MAX

struct intelthermal_softc {
	int			rid;
	struct resource		*res;
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	int			access_denied;
	uint32_t		power_unit_div;
	uint8_t			power_unit_shift;
};

static const struct pci_device_table intelthermal_devices[] = {
	/* Only Skylake validated - other generations may differ */
	{ PCI_DEV(0x8086, 0x1903), PCI_DESCR("Skylake Processor Thermal")},
};

static inline uint32_t
intelthermal_read32(struct intelthermal_softc *sc, uint32_t offset)
{
	return (bus_space_read_4(sc->bst, sc->bsh, offset));
}

static inline uint64_t
intelthermal_read64(struct intelthermal_softc *sc, uint32_t offset)
{
	uint64_t lo, hi;
	lo = bus_space_read_4(sc->bst, sc->bsh, offset);
	hi = bus_space_read_4(sc->bst, sc->bsh, offset + 4);
	return (lo | (hi << 32));
}

static int
intelthermal_pl1_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intelthermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	uint32_t watts_mw;

	if (sc->access_denied) {
		watts_mw = 0;
	} else {
		pl = intelthermal_read64(sc, RAPL_PKG_POWER_LIMIT);
		watts_mw = ((pl & POWER_LIMIT_MASK) * MW_PER_WATT) / sc->power_unit_div;
	}
	return (sysctl_handle_32(oidp, &watts_mw, 0, req));
}

static int
intelthermal_pl2_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intelthermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	uint32_t watts_mw;

	if (sc->access_denied) {
		watts_mw = 0;
	} else {
		pl = intelthermal_read64(sc, RAPL_PKG_POWER_LIMIT);
		watts_mw = (((pl >> POWER_LIMIT2_SHIFT) & POWER_LIMIT_MASK) *
		    1000ULL) / sc->power_unit_div;
	}
	return (sysctl_handle_32(oidp, &watts_mw, 0, req));
}

static int
intelthermal_pl1_enable_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intelthermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	int enabled;

	if (sc->access_denied) {
		enabled = 0;
	} else {
		pl = intelthermal_read64(sc, RAPL_PKG_POWER_LIMIT);
		enabled = (pl & POWER_LIMIT_ENABLE) ? 1 : 0;
	}
	return (sysctl_handle_int(oidp, &enabled, 0, req));
}

static int
intelthermal_pl2_enable_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intelthermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	int enabled;

	if (sc->access_denied) {
		enabled = 0;
	} else {
		pl = intelthermal_read64(sc, RAPL_PKG_POWER_LIMIT);
		enabled = (pl & POWER_LIMIT2_ENABLE) ? 1 : 0;
	}
	return (sysctl_handle_int(oidp, &enabled, 0, req));
}

static int
intelthermal_locked_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intelthermal_softc *sc = oidp->oid_arg1;
	uint64_t pl;
	int locked;

	if (sc->access_denied) {
		locked = 1;
	} else {
		pl = intelthermal_read64(sc, RAPL_PKG_POWER_LIMIT);
		locked = (pl & POWER_LIMIT_LOCK) ? 1 : 0;
	}
	return (sysctl_handle_int(oidp, &locked, 0, req));
}

static int
intelthermal_tdp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intelthermal_softc *sc = oidp->oid_arg1;
	uint32_t info;
	uint32_t tdp_mw;

	if (sc->access_denied) {
		tdp_mw = 0;
	} else {
		info = intelthermal_read32(sc, RAPL_PKG_POWER_INFO);
		tdp_mw = ((info & POWER_LIMIT_MASK) * MW_PER_WATT) / sc->power_unit_div;
	}
	return (sysctl_handle_32(oidp, &tdp_mw, 0, req));
}

static int
intelthermal_probe(device_t dev)
{
	const struct pci_device_table *tbl;

	tbl = PCI_MATCH(dev, intelthermal_devices);
	if (tbl == NULL)
		return (ENXIO);
	device_set_desc(dev, tbl->descr);
	return (BUS_PROBE_DEFAULT);
}

static int
intelthermal_attach(device_t dev)
{
	struct intelthermal_softc *sc = device_get_softc(dev);
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
	if (rdmsr_safe(MSR_RAPL_POWER_UNIT, &rapl_units) != 0) {
		device_printf(dev,
		    "MSR_RAPL_POWER_UNIT read failed, using default\n");
	} else {
		pu = rapl_units & RAPL_POWER_UNIT_MASK;
		if (pu <= RAPL_POWER_UNIT_MAX_SHIFT) {
			sc->power_unit_shift = pu;
			sc->power_unit_div = 1U << pu;
		}
	}

	/*
	 * Check multiple registers to detect firmware lock.
	 * All reading 0xFFFFFFFF indicates access denied.
	 */
	pwr_limit_lo = intelthermal_read32(sc, RAPL_PKG_POWER_LIMIT);
	pwr_limit_hi = intelthermal_read32(sc, RAPL_PKG_POWER_LIMIT + 4);
	pwr_info = intelthermal_read32(sc, RAPL_PKG_POWER_INFO);

	if (pwr_limit_lo == MMIO_ACCESS_DENIED &&
	    pwr_limit_hi == MMIO_ACCESS_DENIED &&
	    pwr_info == MMIO_ACCESS_DENIED) {
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
	    sc, 0, intelthermal_pl1_sysctl, "IU",
	    "PL1 (long-term) power limit in milliwatts");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pl2", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, intelthermal_pl2_sysctl, "IU",
	    "PL2 (short-term) power limit in milliwatts");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pl1_enabled", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, intelthermal_pl1_enable_sysctl, "I",
	    "PL1 power limit enabled");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "pl2_enabled", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, intelthermal_pl2_enable_sysctl, "I",
	    "PL2 power limit enabled");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "locked", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, intelthermal_locked_sysctl, "I",
	    "Power limits locked by firmware");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "tdp", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, intelthermal_tdp_sysctl, "IU",
	    "Thermal Design Power in milliwatts");

	return (0);
}

static int
intelthermal_detach(device_t dev)
{
	struct intelthermal_softc *sc = device_get_softc(dev);

	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);
	return (0);
}

static device_method_t intelthermal_methods[] = {
	DEVMETHOD(device_probe,		intelthermal_probe),
	DEVMETHOD(device_attach,	intelthermal_attach),
	DEVMETHOD(device_detach,	intelthermal_detach),
	DEVMETHOD_END
};

static driver_t intelthermal_driver = {
	"intelthermal",
	intelthermal_methods,
	sizeof(struct intelthermal_softc)
};

DRIVER_MODULE(intelthermal, pci, intelthermal_driver, 0, 0);
PCI_PNP_INFO(intelthermal_devices);
MODULE_VERSION(intelthermal, 1);
MODULE_DEPEND(intelthermal, pci, 1, 1, 1);
