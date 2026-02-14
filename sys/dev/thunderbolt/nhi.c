/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
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

/* PCIe interface for Thunderbolt Native Host Interface (nhi) */
#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/taskqueue.h>
#include <sys/gsb_crc32.h>
#include <sys/endian.h>
#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/atomic.h>
#include <machine/stdarg.h>

#include <dev/thunderbolt/nhi_reg.h>
#include <dev/thunderbolt/nhi_var.h>
#include <dev/thunderbolt/tb_reg.h>
#include <dev/thunderbolt/tb_var.h>
#include <dev/thunderbolt/tb_debug.h>
#include <dev/thunderbolt/hcm_var.h>
#include <dev/thunderbolt/tbcfg_reg.h>
#include <dev/thunderbolt/router_var.h>
#include <dev/thunderbolt/tb_dev.h>
#include "tb_if.h"

static int nhi_alloc_ring(struct nhi_softc *, int, int, int,
    struct nhi_ring_pair **);
static void nhi_free_ring(struct nhi_ring_pair *);
static void nhi_free_rings(struct nhi_softc *);
static int nhi_configure_ring(struct nhi_softc *, struct nhi_ring_pair *);
static int nhi_activate_ring(struct nhi_ring_pair *);
static int nhi_deactivate_ring(struct nhi_ring_pair *);
static int nhi_alloc_ring0(struct nhi_softc *);
static void nhi_free_ring0(struct nhi_softc *);
static void nhi_fill_rx_ring(struct nhi_softc *, struct nhi_ring_pair *);
static int nhi_init(struct nhi_softc *);
static void nhi_post_init(void *);
static int nhi_tx_enqueue(struct nhi_ring_pair *, struct nhi_cmd_frame *);
static int nhi_setup_sysctl(struct nhi_softc *);
static int nhi_set_fw_cm_mode(struct nhi_softc *);
static int nhi_valid_mbox_connmode(uint32_t);
static int nhi_wait_fw_cm_ready(struct nhi_softc *);
static int nhi_wait_outmail_ready(struct nhi_softc *, uint32_t *);
static int nhi_firmware_start(struct nhi_softc *);
static int nhi_firmware_reset(struct nhi_softc *);
static int nhi_detect_fw_mode(struct nhi_softc *);

SYSCTL_NODE(_hw, OID_AUTO, nhi, CTLFLAG_RD, 0, "NHI Driver Parameters");

MALLOC_DEFINE(M_NHI, "nhi", "nhi driver memory");

#ifndef NHI_DEBUG_LEVEL
#define NHI_DEBUG_LEVEL 0
#endif

/* 0 = default, 1 = force-on, 2 = force-off */
#ifndef NHI_FORCE_HCM
#define NHI_FORCE_HCM 0
#endif
/*
 * Default firmware connection manager mode is Certified/Any Depth.
 * Relax to ANY_* only if you must allow uncertified devices.
 */
#ifndef NHI_MBOX_CONNMODE
#define NHI_MBOX_CONNMODE INMAILCMD_SETMODE_CERT_TB_ANY_DEPTH
#endif
#define NHI_FW_CM_READY_TIMEOUT_SEC 5

void
nhi_get_tunables(struct nhi_softc *sc)
{
	devclass_t dc;
	device_t ufp;
	char	tmpstr[80], oid[80];
	u_int	val;
	char	*maker;
	bool	is_apple;

	/* Set local defaults */
	sc->debug = NHI_DEBUG_LEVEL;
	sc->max_ring_count = NHI_DEFAULT_NUM_RINGS;
	sc->force_hcm = NHI_FORCE_HCM;
	sc->mbox_connmode = NHI_MBOX_CONNMODE;
	sc->firmware_managed = !NHI_IS_FW_CM(sc);

	is_apple = false;
	maker = kern_getenv("smbios.system.maker");
	if (maker != NULL) {
		if (strcasestr(maker, "Apple") != NULL)
			is_apple = true;
		freeenv(maker);
	}
	if (is_apple)
		sc->firmware_managed = 1;

	/* Inherit setting from the upstream thunderbolt switch node */
	val = TB_GET_DEBUG(sc->dev, &sc->debug);
	if (val != 0) {
		dc = devclass_find("tbolt");
		if (dc != NULL) {
			ufp = devclass_get_device(dc, device_get_unit(sc->dev));
			if (ufp != NULL)
				TB_GET_DEBUG(ufp, &sc->debug);
		} else {
			if (TUNABLE_STR_FETCH("hw.tbolt.debug_level", oid,
			    80) != 0)
				tb_parse_debug(&sc->debug, oid);
		}
	}

	/*
	 * Grab global variables.  Allow nhi debug flags to override
	 * thunderbolt debug flags, if present.
	 */
	bzero(oid, 80);
	if (TUNABLE_STR_FETCH("hw.nhi.debug_level", oid, 80) != 0)
		tb_parse_debug(&sc->debug, oid);
	if (TUNABLE_INT_FETCH("hw.nhi.max_rings", &val) != 0) {
		val = min(val, NHI_MAX_NUM_RINGS);
		sc->max_ring_count = max(val, 1);
	}
	if (TUNABLE_INT_FETCH("hw.nhi.force_hcm", &val) != 0)
		sc->force_hcm = val;
	if (NHI_IS_FW_CM(sc) && !is_apple) {
		if (TUNABLE_INT_FETCH("hw.nhi.firmware_managed", &val) != 0)
			sc->firmware_managed = (val != 0);
	}
	if (TUNABLE_INT_FETCH("hw.nhi.connmode", &val) != 0)
		sc->mbox_connmode = val;

	/* Grab instance variables */
	bzero(oid, 80);
	snprintf(tmpstr, sizeof(tmpstr), "dev.nhi.%d.debug_level",
	    device_get_unit(sc->dev));
	if (TUNABLE_STR_FETCH(tmpstr, oid, 80) != 0)
		tb_parse_debug(&sc->debug, oid);
	snprintf(tmpstr, sizeof(tmpstr), "dev.nhi.%d.max_rings",
	    device_get_unit(sc->dev));
	if (TUNABLE_INT_FETCH(tmpstr, &val) != 0) {
		val = min(val, NHI_MAX_NUM_RINGS);
		sc->max_ring_count = max(val, 1);
	}
	snprintf(tmpstr, sizeof(tmpstr), "dev.nhi.%d.force_hcm",
	    device_get_unit(sc->dev));
	if (TUNABLE_INT_FETCH(tmpstr, &val) != 0)
		sc->force_hcm = val;
	if (NHI_IS_FW_CM(sc) && !is_apple) {
		snprintf(tmpstr, sizeof(tmpstr), "dev.nhi.%d.firmware_managed",
		    device_get_unit(sc->dev));
		if (TUNABLE_INT_FETCH(tmpstr, &val) != 0)
			sc->firmware_managed = (val != 0);
	}
	snprintf(tmpstr, sizeof(tmpstr), "dev.nhi.%d.connmode",
	    device_get_unit(sc->dev));
	if (TUNABLE_INT_FETCH(tmpstr, &val) != 0)
		sc->mbox_connmode = val;

	return;
}

static void
nhi_configure_caps(struct nhi_softc *sc)
{

	if (NHI_IS_USB4(sc) || NHI_IS_FW_CM(sc) ||
	    (sc->force_hcm == NHI_FORCE_HCM_ON))
		sc->caps |= NHI_CAP_HCM;
	if (sc->force_hcm == NHI_FORCE_HCM_OFF)
		sc->caps &= ~NHI_CAP_HCM;
}

struct nhi_cmd_frame *
nhi_alloc_tx_frame(struct nhi_ring_pair *r)
{
	struct nhi_cmd_frame *cmd;

	mtx_lock(&r->mtx);
	cmd = nhi_alloc_tx_frame_locked(r);
	mtx_unlock(&r->mtx);

	return (cmd);
}

void
nhi_free_tx_frame(struct nhi_ring_pair *r, struct nhi_cmd_frame *cmd)
{
	mtx_lock(&r->mtx);
	nhi_free_tx_frame_locked(r, cmd);
	mtx_unlock(&r->mtx);
}

/*
 * Push a command and data dword through the mailbox to the firmware.
 * Response is either good, error, or timeout.  Commands that return data
 * do so by reading OUTMAILDATA.
 */
int
nhi_inmail_cmd(struct nhi_softc *sc, uint32_t cmd, uint32_t data)
{
	uint32_t val;
	u_int error, timeout;

	mtx_lock(&sc->nhi_mtx);
	/*
	 * XXX Should a defer/reschedule happen here, or is it not worth
	 * worrying about?
	 */
	if (sc->hwflags & NHI_MBOX_BUSY) {
		mtx_unlock(&sc->nhi_mtx);
		tb_debug(sc, DBG_MBOX, "Driver busy with mailbox\n");
		return (EBUSY);
	}
	sc->hwflags |= NHI_MBOX_BUSY;

	val = nhi_read_reg(sc, TBT_INMAILCMD);
	tb_debug(sc, DBG_MBOX|DBG_FULL, "Reading INMAILCMD= 0x%08x\n", val);
	if (val & INMAILCMD_ERROR)
		tb_debug(sc, DBG_MBOX, "Error already set in INMAILCMD\n");
	if (val & INMAILCMD_OPREQ) {
		tb_debug(sc, DBG_MBOX,
		    "INMAILCMD request already in progress\n");
		timeout = NHI_MAILBOX_TIMEOUT;
		while (timeout > 0) {
			DELAY(1000);	/* 1ms poll */
			val = nhi_read_reg(sc, TBT_INMAILCMD);
			if ((val & INMAILCMD_OPREQ) == 0)
				break;
			timeout--;
		}
		if (val & INMAILCMD_OPREQ) {
			tb_printf(sc, "Timeout waiting for mailbox\n");
			sc->hwflags &= ~NHI_MBOX_BUSY;
			mtx_unlock(&sc->nhi_mtx);
			return (ETIMEDOUT);
		}
	}

	nhi_write_reg(sc, TBT_INMAILDATA, data);
	val = nhi_read_reg(sc, TBT_INMAILCMD);
	val &= ~(INMAILCMD_CMD_MASK | INMAILCMD_ERROR);
	val |= INMAILCMD_OPREQ | cmd;
	nhi_write_reg(sc, TBT_INMAILCMD, val);

	/* Poll at 1ms intervals, up to NHI_MAILBOX_TIMEOUT ms */
	timeout = NHI_MAILBOX_TIMEOUT;
	while (timeout > 0) {
		DELAY(1000);	/* 1ms */
		val = nhi_read_reg(sc, TBT_INMAILCMD);
		tb_debug(sc, DBG_MBOX|DBG_EXTRA,
		    "Polling INMAILCMD= 0x%08x\n", val);
		if ((val & INMAILCMD_OPREQ) == 0)
			break;
		timeout--;
	}
	sc->hwflags &= ~NHI_MBOX_BUSY;
	mtx_unlock(&sc->nhi_mtx);

	error = 0;
	if (val & INMAILCMD_OPREQ) {
		tb_printf(sc, "Timeout waiting for mailbox\n");
		error = ETIMEDOUT;
	}
	if (val & INMAILCMD_ERROR) {
		tb_printf(sc, "Firmware reports error in mailbox\n");
		error = EINVAL;
	}

	return (error);
}

/*
 * Pull command status and data from the firmware mailbox.
 */
int
nhi_outmail_cmd(struct nhi_softc *sc, uint32_t *val)
{

	if (val == NULL)
		return (EINVAL);
	*val = nhi_read_reg(sc, TBT_OUTMAILCMD);
	return (0);
}

int
nhi_attach(struct nhi_softc *sc)
{
	uint32_t val;
	int error = 0;

	if ((error = nhi_setup_sysctl(sc)) != 0)
		return (error);

	mtx_init(&sc->nhi_mtx, "nhimtx", "NHI Control Mutex", MTX_DEF);

	/*
	 * Get the number of TX/RX paths.  This sizes some of the register
	 * arrays during allocation and initialization.  USB4 spec says that
	 * the max is 21, but older Thunderbolt 1/2 controllers (Light Ridge,
	 * Falcon Ridge) report 32.  Alpine Ridge reports 12.
	 */
	val = GET_HOST_CAPS_PATHS(nhi_read_reg(sc, NHI_HOST_CAPS));
	tb_debug(sc, DBG_INIT|DBG_NOISY, "Total Paths= %d\n", val);
	if (val == 0) {
		tb_printf(sc, "WARN: path count is 0\n");
		return (ENXIO);
	}
	if (val > NHI_MAX_NUM_RINGS) {
		tb_printf(sc, "WARN: path count %d > %d, clamping\n", val,
		    NHI_MAX_NUM_RINGS);
		val = NHI_MAX_NUM_RINGS;
	}
	sc->path_count = val;

	SLIST_INIT(&sc->ring_list);

	error = nhi_pci_configure_interrupts(sc);
	if (error == 0)
		error = nhi_alloc_ring0(sc);
	if (error == 0) {
		nhi_configure_ring(sc, sc->ring0);
		nhi_activate_ring(sc->ring0);
		nhi_fill_rx_ring(sc, sc->ring0);
		/* Give hardware time to stabilize after interrupt enable */
		DELAY(100000); /* 100ms */
	}

	if (error == 0)
		error = tbdev_add_interface(sc);

	if ((error == 0) && (NHI_USE_ICM(sc)))
		tb_printf(sc, "WARN: device uses an internal connection manager\n");
	if ((error == 0) && (NHI_USE_HCM(sc)))
		error = hcm_attach(sc);

	if (error == 0)
		error = nhi_init(sc);

	return (error);
}

int
nhi_detach(struct nhi_softc *sc)
{
	hcm_detach(sc);

	if (sc->root_rsc != NULL)
		tb_router_detach(sc->root_rsc);

	tbdev_remove_domain(sc->uuid);
	tbdev_remove_interface(sc);

	nhi_pci_disable_interrupts(sc);
	nhi_pci_free_interrupts(sc);

	nhi_free_ring0(sc);

	/* XXX Should the rings be marked as !VALID in the descriptors? */
	nhi_free_rings(sc);

	mtx_destroy(&sc->nhi_mtx);

	return (0);
}

static void
nhi_memaddr_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	bus_addr_t *addr;

	addr = arg;
	if (error == 0 && nsegs == 1) {
		*addr = segs[0].ds_addr;
	} else
		*addr = 0;
}

static int
nhi_alloc_ring(struct nhi_softc *sc, int ringnum, int tx_depth, int rx_depth,
    struct nhi_ring_pair **rp)
{
	bus_dma_template_t t;
	bus_addr_t ring_busaddr;
	struct nhi_ring_pair *r;
	int ring_size, error;
	u_int rxring_len, txring_len;
	char *ring;

	if (ringnum >= sc->max_ring_count) {
		tb_debug(sc, DBG_INIT, "Tried to allocate ring number %d\n",
		    ringnum);
		return (EINVAL);
	}

	/* Allocate the ring structure and the RX ring tacker together. */
	rxring_len = rx_depth * sizeof(void *);
	txring_len = tx_depth * sizeof(void *);
	r = malloc(sizeof(struct nhi_ring_pair) + rxring_len + txring_len,
	    M_NHI, M_NOWAIT|M_ZERO);
	if (r == NULL) {
		tb_printf(sc, "ERROR: Cannot allocate ring memory\n");
		return (ENOMEM);
	}

	r->sc = sc;
	TAILQ_INIT(&r->tx_head);
	TAILQ_INIT(&r->rx_head);
	r->ring_num = ringnum;
	r->tx_ring_depth = tx_depth;
	r->tx_ring_mask = tx_depth - 1;
	r->rx_ring_depth = rx_depth;
	r->rx_ring_mask = rx_depth - 1;
	r->rx_pici_reg = NHI_RX_RING_PICI + ringnum * 16;
	r->tx_pici_reg = NHI_TX_RING_PICI + ringnum * 16;
	r->rx_cmd_ring = (struct nhi_cmd_frame **)((uint8_t *)r + sizeof (*r));
	r->tx_cmd_ring = (struct nhi_cmd_frame **)((uint8_t *)r->rx_cmd_ring +
	    rxring_len);

	snprintf(r->name, NHI_RING_NAMELEN, "nhiring%d\n", ringnum);
	mtx_init(&r->mtx, r->name, "NHI Ring Lock", MTX_DEF);
	tb_debug(sc, DBG_INIT | DBG_FULL, "Allocated ring context at %p, "
	    "mutex %p\n", r, &r->mtx);

	/* Allocate the RX and TX buffer descriptor rings */
	ring_size = sizeof(struct nhi_tx_buffer_desc) * r->tx_ring_depth;
	ring_size += sizeof(struct nhi_rx_buffer_desc) * r->rx_ring_depth;
	tb_debug(sc, DBG_INIT | DBG_FULL, "Ring %d ring_size= %d\n",
	    ringnum, ring_size);

	bus_dma_template_init(&t, sc->parent_dmat);
	t.alignment = 4;
	t.maxsize = t.maxsegsize = ring_size;
	t.nsegments = 1;
	if ((error = bus_dma_template_tag(&t, &r->ring_dmat)) != 0) {
		tb_printf(sc, "Cannot allocate ring %d DMA tag: %d\n",
		    ringnum, error);
		goto fail;
	}
	error = bus_dmamem_alloc(r->ring_dmat, (void **)&ring, BUS_DMA_NOWAIT,
	    &r->ring_map);
	if (error != 0) {
		tb_printf(sc, "Cannot allocate ring memory: %d\n", error);
		goto fail;
	}
	r->ring = ring;
	bzero(ring, ring_size);
	error = bus_dmamap_load(r->ring_dmat, r->ring_map, ring, ring_size,
	    nhi_memaddr_cb, &ring_busaddr, 0);
	if (error != 0) {
		tb_printf(sc, "Cannot map ring memory: %d\n", error);
		goto fail;
	}

	r->tx_ring = (union nhi_ring_desc *)(ring);
	r->tx_ring_busaddr = ring_busaddr;
	ring += sizeof(struct nhi_tx_buffer_desc) * r->tx_ring_depth;
	ring_busaddr += sizeof(struct nhi_tx_buffer_desc) * r->tx_ring_depth;

	r->rx_ring = (union nhi_ring_desc *)(ring);
	r->rx_ring_busaddr = ring_busaddr;

	tb_debug(sc, DBG_INIT | DBG_EXTRA, "Ring %d: RX %p [0x%jx] "
	    "TX %p [0x%jx]\n", ringnum, r->tx_ring, r->tx_ring_busaddr,
	    r->rx_ring, r->rx_ring_busaddr);

	*rp = r;
	return (0);
fail:
	if (r->ring != NULL) {
		bus_dmamem_free(r->ring_dmat, r->ring, r->ring_map);
		r->ring = NULL;
	}
	if (r->ring_dmat != NULL) {
		bus_dma_tag_destroy(r->ring_dmat);
		r->ring_dmat = NULL;
	}
	mtx_destroy(&r->mtx);
	free(r, M_NHI);
	return (error);
}

static void
nhi_free_ring(struct nhi_ring_pair *r)
{

	tb_debug(r->sc, DBG_INIT, "Freeing ring %d resources\n", r->ring_num);
	nhi_deactivate_ring(r);

	if (r->tx_ring_busaddr != 0) {
		bus_dmamap_unload(r->ring_dmat, r->ring_map);
		r->tx_ring_busaddr = 0;
	}
	if (r->ring != NULL) {
		bus_dmamem_free(r->ring_dmat, r->ring, r->ring_map);
		r->ring = NULL;
	}
	if (r->ring_dmat != NULL) {
		bus_dma_tag_destroy(r->ring_dmat);
		r->ring_dmat = NULL;
	}
	mtx_destroy(&r->mtx);
}

static void
nhi_free_rings(struct nhi_softc *sc)
{
	struct nhi_ring_pair *r;

	while ((r = SLIST_FIRST(&sc->ring_list)) != NULL) {
		nhi_free_ring(r);
		mtx_lock(&sc->nhi_mtx);
		SLIST_REMOVE_HEAD(&sc->ring_list, ring_link);
		mtx_unlock(&sc->nhi_mtx);
		free(r, M_NHI);
	}

	return;
}

static int
nhi_configure_ring(struct nhi_softc *sc, struct nhi_ring_pair *ring)
{
	bus_addr_t busaddr;
	uint32_t val;
	int idx;

	idx = ring->ring_num * 16;

	/* Program the TX ring address and size */
	busaddr = ring->tx_ring_busaddr;
	nhi_write_reg(sc, NHI_TX_RING_ADDR_LO + idx, busaddr & 0xffffffff);
	nhi_write_reg(sc, NHI_TX_RING_ADDR_HI + idx, busaddr >> 32);
	nhi_write_reg(sc, NHI_TX_RING_SIZE + idx, ring->tx_ring_depth);
	nhi_write_reg(sc, NHI_TX_RING_TABLE_TIMESTAMP + idx, 0x0);
	tb_debug(sc, DBG_INIT, "TX Ring %d TX_RING_SIZE= 0x%x\n",
	    ring->ring_num, ring->tx_ring_depth);

	/* Program the RX ring address and size */
	busaddr = ring->rx_ring_busaddr;
	val = (ring->rx_buffer_size << 16) | ring->rx_ring_depth;
	nhi_write_reg(sc, NHI_RX_RING_ADDR_LO + idx, busaddr & 0xffffffff);
	nhi_write_reg(sc, NHI_RX_RING_ADDR_HI + idx, busaddr >> 32);
	nhi_write_reg(sc, NHI_RX_RING_SIZE + idx, val);
	nhi_write_reg(sc, NHI_RX_RING_TABLE_BASE1 + idx, 0xffffffff);
	tb_debug(sc, DBG_INIT, "RX Ring %d RX_RING_SIZE= 0x%x\n",
	    ring->ring_num, val);

	return (0);
}

static int
nhi_activate_ring(struct nhi_ring_pair *ring)
{
	struct nhi_softc *sc = ring->sc;
	int idx;

	nhi_pci_enable_interrupt(ring);

	idx = ring->ring_num * 32;
	tb_debug(sc, DBG_INIT, "Activating ring %d at idx %d\n",
	    ring->ring_num, idx);
	nhi_write_reg(sc, NHI_TX_RING_TABLE_BASE0 + idx,
	    TX_TABLE_RAW | TX_TABLE_VALID);
	nhi_write_reg(sc, NHI_RX_RING_TABLE_BASE0 + idx,
	    RX_TABLE_RAW | RX_TABLE_VALID);

	return (0);
}

static int
nhi_deactivate_ring(struct nhi_ring_pair *r)
{
	struct nhi_softc *sc = r->sc;
	int idx;

	idx = r->ring_num * 32;
	tb_debug(sc, DBG_INIT, "Deactiving ring %d at idx %d\n",
	    r->ring_num, idx);
	nhi_write_reg(sc, NHI_TX_RING_TABLE_BASE0 + idx, 0);
	nhi_write_reg(sc, NHI_RX_RING_TABLE_BASE0 + idx, 0);

	idx = r->ring_num * 16;
	tb_debug(sc, DBG_INIT, "Setting ring %d sizes to 0\n", r->ring_num);
	nhi_write_reg(sc, NHI_TX_RING_SIZE + idx, 0);
	nhi_write_reg(sc, NHI_RX_RING_SIZE + idx, 0);

	return (0);
}

static int
nhi_alloc_ring0(struct nhi_softc *sc)
{
	bus_addr_t frames_busaddr;
	bus_dma_template_t t;
	struct nhi_intr_tracker *trkr;
	struct nhi_ring_pair *r;
	struct nhi_cmd_frame *cmd;
	char *frames;
	int error, size, i;

	if ((error = nhi_alloc_ring(sc, 0, NHI_RING0_TX_DEPTH,
	    NHI_RING0_RX_DEPTH, &r)) != 0) {
		tb_printf(sc, "Error allocating control ring\n");
		return (error);
	}

	r->rx_buffer_size = NHI_RING0_FRAME_SIZE;/* Control packets are small */

	/* Allocate the RX and TX buffers that are used for Ring0 comms */
	size = r->tx_ring_depth * NHI_RING0_FRAME_SIZE;
	size += r->rx_ring_depth * NHI_RING0_FRAME_SIZE;

	bus_dma_template_init(&t, sc->parent_dmat);
	t.maxsize = t.maxsegsize = size;
	t.nsegments = 1;
	error = bus_dma_template_tag(&t, &sc->ring0_dmat);
	if (error != 0) {
		tb_printf(sc, "Error allocating control ring buffer tag\n");
		goto fail;
	}

	error = bus_dmamem_alloc(sc->ring0_dmat, (void **)&frames, BUS_DMA_NOWAIT,
	    &sc->ring0_map);
	if (error != 0) {
		tb_printf(sc, "Error allocating control ring memory: %d\n",
		    error);
		goto fail;
	}
	sc->ring0_frames = frames;
	bzero(frames, size);
	error = bus_dmamap_load(sc->ring0_dmat, sc->ring0_map, frames, size,
	    nhi_memaddr_cb, &frames_busaddr, 0);
	if (error != 0) {
		tb_printf(sc, "Error mapping control ring memory: %d\n",
		    error);
		goto fail;
	}
	sc->ring0_frames_busaddr = frames_busaddr;

	/* Allocate the driver command trackers */
	sc->ring0_cmds = malloc(sizeof(struct nhi_cmd_frame) *
	    (r->tx_ring_depth + r->rx_ring_depth), M_NHI, M_NOWAIT | M_ZERO);
	if (sc->ring0_cmds == NULL) {
		error = ENOMEM;
		goto fail;
	}

	/* Initialize the RX frames so they can be used */
	mtx_lock(&r->mtx);
	for (i = 0; i < r->rx_ring_depth; i++) {
		cmd = &sc->ring0_cmds[i];
		cmd->data = (uint32_t *)(frames + NHI_RING0_FRAME_SIZE * i);
		cmd->data_busaddr = frames_busaddr + NHI_RING0_FRAME_SIZE * i;
		cmd->flags = CMD_MAPPED;
		cmd->idx = i;
		TAILQ_INSERT_TAIL(&r->rx_head, cmd, cm_link);
	}

	/* Inititalize the TX frames */
	for ( ; i < r->tx_ring_depth + r->rx_ring_depth - 1; i++) {
		cmd = &sc->ring0_cmds[i];
		cmd->data = (uint32_t *)(frames + NHI_RING0_FRAME_SIZE * i);
		cmd->data_busaddr = frames_busaddr + NHI_RING0_FRAME_SIZE * i;
		cmd->flags = CMD_MAPPED;
		cmd->idx = i;
		nhi_free_tx_frame_locked(r, cmd);
	}
	mtx_unlock(&r->mtx);

	/* Do a 1:1 mapping of rings to interrupt vectors. */
	/* XXX Should be abstracted */
	trkr = &sc->intr_trackers[0];
	trkr->ring = r;
	r->tracker = trkr;

	/* XXX Should be an array */
	sc->ring0 = r;
	SLIST_INSERT_HEAD(&sc->ring_list, r, ring_link);

	return (0);
fail:
	if (sc->ring0_cmds != NULL) {
		free(sc->ring0_cmds, M_NHI);
		sc->ring0_cmds = NULL;
	}
	if (sc->ring0_frames_busaddr != 0) {
		bus_dmamap_unload(sc->ring0_dmat, sc->ring0_map);
		sc->ring0_frames_busaddr = 0;
	}
	if (sc->ring0_frames != NULL) {
		bus_dmamem_free(sc->ring0_dmat, sc->ring0_frames,
		    sc->ring0_map);
		sc->ring0_frames = NULL;
	}
	if (sc->ring0_dmat != NULL) {
		bus_dma_tag_destroy(sc->ring0_dmat);
		sc->ring0_dmat = NULL;
	}
	if (r != NULL) {
		nhi_free_ring(r);
		free(r, M_NHI);
	}
	return (error);
}

static void
nhi_free_ring0(struct nhi_softc *sc)
{
	if (sc->ring0_cmds != NULL) {
		free(sc->ring0_cmds, M_NHI);
		sc->ring0_cmds = NULL;
	}

	if (sc->ring0_frames_busaddr != 0) {
		bus_dmamap_unload(sc->ring0_dmat, sc->ring0_map);
		sc->ring0_frames_busaddr = 0;
	}

	if (sc->ring0_frames != NULL) {
		bus_dmamem_free(sc->ring0_dmat, sc->ring0_frames,
		    sc->ring0_map);
		sc->ring0_frames = NULL;
	}

	if (sc->ring0_dmat != NULL)
		bus_dma_tag_destroy(sc->ring0_dmat);

	return;
}

static void
nhi_fill_rx_ring(struct nhi_softc *sc, struct nhi_ring_pair *rp)
{
	struct nhi_cmd_frame *cmd;
	struct nhi_rx_buffer_desc *desc;
	u_int ci;

	/* Assume that we never grow or shrink the ring population */
	rp->rx_ci = ci = 0;
	rp->rx_pi = 0;

	do {
		cmd = TAILQ_FIRST(&rp->rx_head);
		if (cmd == NULL)
			break;
		TAILQ_REMOVE(&rp->rx_head, cmd, cm_link);
		desc = &rp->rx_ring[ci].rx;
		if ((cmd->flags & CMD_MAPPED) == 0)
			panic("Need rx buffer mapping code");

		desc->addr_lo = cmd->data_busaddr & 0xffffffff;
		desc->addr_hi = (cmd->data_busaddr >> 32) & 0xffffffff;
		desc->offset = 0;
		desc->flags = RX_BUFFER_DESC_RS | RX_BUFFER_DESC_IE;
		rp->rx_ci = ci;
		rp->rx_cmd_ring[ci] = cmd;
		tb_debug(sc, DBG_RXQ | DBG_FULL,
		    "Updating ring%d ci= %d cmd= %p, busaddr= 0x%jx\n",
		    rp->ring_num, ci, cmd, cmd->data_busaddr);

		ci = (rp->rx_ci + 1) & rp->rx_ring_mask;
	} while (ci != rp->rx_pi);

	/* Update the CI in one shot */
	tb_debug(sc, DBG_RXQ, "Writing RX CI= %d\n", rp->rx_ci);
	nhi_write_reg(sc, rp->rx_pici_reg, rp->rx_ci);

	return;
}

static int
nhi_init(struct nhi_softc *sc)
{
	tb_route_t root_route = {0x0, 0x0};
	uint32_t val;
	int error;

	tb_debug(sc, DBG_INIT, "Initializing NHI\n");

	/* Set interrupt Auto-ACK */
	val = nhi_read_reg(sc, NHI_DMA_MISC);
	tb_debug(sc, DBG_INIT|DBG_FULL, "Read NHI_DMA_MISC= 0x%08x\n", val);
	val |= DMA_MISC_INT_AUTOCLEAR;
	tb_debug(sc, DBG_INIT, "Setting interrupt auto-ACK, 0x%08x\n", val);
	nhi_write_reg(sc, NHI_DMA_MISC, val);

	/*
	 * Attach the router to the root thunderbolt bridge now that the DMA
	 * channel is configured and ready.
	 * The root router always has a route of 0x0...0, so set it statically
	 * here.
	 */
	if (sc->fw_safe_mode) {
		tb_printf(sc, "Firmware in safe mode, skipping root router attach\n");
		nhi_deactivate_ring(sc->ring0);
		return (0);
	}

	if ((error = tb_router_attach_root(sc, root_route)) != 0)
		tb_printf(sc, "tb_router_attach_root()  error."
		    "  The driver should be loaded at boot\n");

	if (error == 0) {
		sc->ich.ich_func = nhi_post_init;
		sc->ich.ich_arg = sc;
		error = config_intrhook_establish(&sc->ich);
		if (error)
			tb_printf(sc, "Failed to establish config hook\n");
	}

	return (error);
}

static void
nhi_post_init(void *arg)
{
	struct nhi_softc *sc;
	uint8_t *u;
	int error;

	sc = (struct nhi_softc *)arg;
	tb_debug(sc, DBG_INIT | DBG_EXTRA, "nhi_post_init\n");

	bzero(sc->lc_uuid, 16);
	error = tb_config_get_lc_uuid(sc->root_rsc, sc->lc_uuid);
	if (error == 0) {
		u = sc->lc_uuid;
		tb_printf(sc, "Root Router LC UUID: %02x%02x%02x%02x-"
		    "%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x\n",
		    u[15], u[14], u[13], u[12], u[11], u[10], u[9], u[8], u[7],
		    u[6], u[5], u[4], u[3], u[2], u[1], u[0]);
	} else
		tb_printf(sc, "Error finding LC registers: %d\n", error);

	u = sc->uuid;
	tb_printf(sc, "Root Router UUID: %02x%02x%02x%02x-"
	    "%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x\n",
	    u[15], u[14], u[13], u[12], u[11], u[10], u[9], u[8], u[7],
	    u[6], u[5], u[4], u[3], u[2], u[1], u[0]);

	if (NHI_IS_FW_CM(sc)) {
		if (!sc->firmware_managed) {
			error = nhi_set_fw_cm_mode(sc);
			if (error != 0)
				tb_printf(sc,
				    "Failed to set FW CM mode: %d\n",
				    error);
		} else {
			tb_printf(sc, "Firmware-managed CM enabled\n");
		}
	}

	(void)tbdev_add_domain(sc->uuid);

	/* Discover devices already connected at load time. */
	if (NHI_USE_HCM(sc) && sc->hcm != NULL)
		hcm_router_discover(sc->hcm);

	config_intrhook_disestablish(&sc->ich);
}

static int
nhi_tx_enqueue(struct nhi_ring_pair *r, struct nhi_cmd_frame *cmd)
{
	struct nhi_softc *sc;
	struct nhi_tx_buffer_desc *desc;
	uint16_t pi;

	sc = r->sc;

	/* A length of 0 means 4096.  Can't have longer lengths */
	if (cmd->req_len > TX_BUFFER_DESC_LEN_MASK + 1) {
		tb_debug(sc, DBG_TXQ, "Error: TX frame too big\n");
		return (EINVAL);
	}
	cmd->req_len &= TX_BUFFER_DESC_LEN_MASK;

	mtx_lock(&r->mtx);
	desc = &r->tx_ring[r->tx_pi].tx;
	pi = (r->tx_pi + 1) & r->tx_ring_mask;
	if (pi == r->tx_ci) {
		mtx_unlock(&r->mtx);
		return (EBUSY);
	}
	r->tx_cmd_ring[r->tx_pi] = cmd;
	r->tx_pi = pi;

	desc->addr_lo = htole32(cmd->data_busaddr & 0xffffffff);
	desc->addr_hi = htole32(cmd->data_busaddr >> 32);
	desc->eof_len = htole16((cmd->pdf << TX_BUFFER_DESC_EOF_SHIFT) |
	    cmd->req_len);
	desc->flags_sof = cmd->pdf | TX_BUFFER_DESC_IE | TX_BUFFER_DESC_RS;
	desc->offset = 0;
	desc->payload_time = 0;

	tb_debug(sc, DBG_TXQ, "enqueue TXdescIdx= %d cmdidx= %d len= %d, "
	    "busaddr= 0x%jx\n", r->tx_pi, cmd->idx, cmd->req_len,
	    cmd->data_busaddr);

	nhi_write_reg(sc, r->tx_pici_reg, pi << TX_RING_PI_SHIFT | r->tx_ci);
	mtx_unlock(&r->mtx);
	return (0);
}

/*
 * No scheduling happens for now.  Ring0 scheduling is done in the TB
 * layer.
 */
int
nhi_tx_schedule(struct nhi_ring_pair *r, struct nhi_cmd_frame *cmd)
{
	int error;

	error = nhi_tx_enqueue(r, cmd);
	if (error == EBUSY)
		nhi_write_reg(r->sc, r->tx_pici_reg, r->tx_pi << TX_RING_PI_SHIFT | r->tx_ci);
	return (error);
}

int
nhi_tx_synchronous(struct nhi_ring_pair *r, struct nhi_cmd_frame *cmd)
{
	int error, count;

	if ((error = nhi_tx_schedule(r, cmd)) != 0)
		return (error);

	if (cmd->flags & CMD_POLLED) {
		error = 0;
		count = cmd->timeout * 100;

		/* Enter the loop at least once */
		while ((count-- > 0) &&
		    (atomic_load_acq_16(&cmd->flags) & CMD_REQ_COMPLETE) == 0) {
			DELAY(10000);
			rmb();
			nhi_intr(r->tracker);
		}
	} else {
		error = msleep(cmd, &r->mtx, PCATCH, "nhi_tx", cmd->timeout);
		if ((error == 0) &&
		    (atomic_load_acq_16(&cmd->flags) & CMD_REQ_COMPLETE) != 0)
			error = EWOULDBLOCK;
	}

	if ((atomic_load_acq_16(&cmd->flags) & CMD_REQ_COMPLETE) == 0)
		error = ETIMEDOUT;

	tb_debug(r->sc, DBG_TXQ|DBG_FULL, "tx_synchronous done waiting, "
	    "err= %d, TX_COMPLETE= %d\n", error,
	    !!(atomic_load_acq_16(&cmd->flags) & CMD_REQ_COMPLETE));

	if (error == ERESTART) {
		tb_printf(r->sc, "TX command interrupted\n");
	} else if ((error == EWOULDBLOCK) || (error == ETIMEDOUT)) {
		tb_printf(r->sc, "TX command timed out\n");
	} else if (error != 0) {
		tb_printf(r->sc, "TX command failed error= %d\n", error);
	}

	return (error);
}

static int
nhi_tx_complete(struct nhi_ring_pair *r, struct nhi_tx_buffer_desc *desc,
    struct nhi_cmd_frame *cmd)
{
	struct nhi_softc *sc;
	struct nhi_pdf_dispatch *txpdf;
	u_int sof;

	sc = r->sc;
	sof = desc->flags_sof & TX_BUFFER_DESC_SOF_MASK;
	tb_debug(sc, DBG_TXQ, "Recovered TX pdf= %s cmdidx= %d flags= 0x%x\n",
	    tb_get_string(sof, nhi_frame_pdf), cmd->idx, desc->flags_sof);

	if ((desc->flags_sof & TX_BUFFER_DESC_DONE) == 0)
		tb_debug(sc, DBG_TXQ,
		    "warning, TX descriptor DONE flag not set\n");

	atomic_set_short(&cmd->flags, CMD_REQ_COMPLETE);

	txpdf = &r->tracker->txpdf[sof];
	if (txpdf->cb != NULL) {
		tb_debug(sc, DBG_INTR|DBG_TXQ, "Calling PDF TX callback\n");
		txpdf->cb(txpdf->context, (union nhi_ring_desc *)desc, cmd);
		return (0);
	}

	tb_debug(sc, DBG_TXQ, "Unhandled TX complete %s\n",
	    tb_get_string(sof, nhi_frame_pdf));
	nhi_free_tx_frame(r, cmd);

	return (0);
}

static int
nhi_rx_complete(struct nhi_ring_pair *r, struct nhi_rx_post_desc *desc,
    struct nhi_cmd_frame *cmd)
{
	struct nhi_softc *sc;
	struct nhi_pdf_dispatch *rxpdf;
	u_int eof, len;

	sc = r->sc;
	eof = desc->eof_len >> RX_BUFFER_DESC_EOF_SHIFT;
	len = desc->eof_len & RX_BUFFER_DESC_LEN_MASK;
	tb_debug(sc, DBG_INTR|DBG_RXQ,
	    "Recovered RX pdf= %s len= %d cmdidx= %d, busaddr= 0x%jx\n",
	    tb_get_string(eof, nhi_frame_pdf), len, cmd->idx,
	    cmd->data_busaddr);

	rxpdf = &r->tracker->rxpdf[eof];
	if (rxpdf->cb != NULL) {
		tb_debug(sc, DBG_INTR|DBG_RXQ, "Calling PDF RX callback\n");
		rxpdf->cb(rxpdf->context, (union nhi_ring_desc *)desc, cmd);
		return (0);
	}

	/*
	 * PDF_HOTPLUG frames arrive before the router registers its
	 * callback during attach, and continuously in firmware safe
	 * mode where no router attaches at all.  Silently drop them.
	 */
	if (eof != PDF_HOTPLUG)
		tb_debug(sc, DBG_INTR, "Unhandled RX frame %s\n",
		    tb_get_string(eof, nhi_frame_pdf));

	return (0);
}

int
nhi_register_pdf(struct nhi_ring_pair *rp, struct nhi_dispatch *tx,
    struct nhi_dispatch *rx)
{
	struct nhi_intr_tracker *trkr;
	struct nhi_pdf_dispatch *slot;

	KASSERT(rp != NULL, ("ring_pair is null\n"));
	tb_debug(rp->sc, DBG_INTR|DBG_EXTRA, "nhi_register_pdf called\n");

	trkr = rp->tracker;
	if (trkr == NULL) {
		tb_debug(rp->sc, DBG_INTR, "Invalid tracker\n");
		return (EINVAL);
	}

	tb_debug(rp->sc, DBG_INTR|DBG_EXTRA, "Registering TX interrupts\n");
	if (tx != NULL) {
		while (tx->cb != NULL) {
			if ((tx->pdf < 0) || (tx->pdf > 15))
				return (EINVAL);
			slot = &trkr->txpdf[tx->pdf];
			if (slot->cb != NULL) {
				tb_debug(rp->sc, DBG_INTR,
				    "Attempted to register busy callback\n");
				return (EBUSY);
			}
			slot->cb = tx->cb;
			slot->context = tx->context;
			tb_debug(rp->sc, DBG_INTR,
			    "Registered TX callback for PDF %d\n", tx->pdf);
			tx++;
		}
	}

	tb_debug(rp->sc, DBG_INTR|DBG_EXTRA, "Registering RX interrupts\n");
	if (rx != NULL) {
		while (rx->cb != NULL) {
			if ((rx->pdf < 0) || (rx->pdf > 15))
				return (EINVAL);
			slot = &trkr->rxpdf[rx->pdf];
			if (slot->cb != NULL) {
				tb_debug(rp->sc, DBG_INTR,
				    "Attempted to register busy callback\n");
				return (EBUSY);
			}
			slot->cb = rx->cb;
			slot->context = rx->context;
			tb_debug(rp->sc, DBG_INTR,
			    "Registered RX callback for PDF %d\n", rx->pdf);
			rx++;
		}
	}

	return (0);
}

int
nhi_deregister_pdf(struct nhi_ring_pair *rp, struct nhi_dispatch *tx,
    struct nhi_dispatch *rx)
{
	struct nhi_intr_tracker *trkr;
	struct nhi_pdf_dispatch *slot;

	tb_debug(rp->sc, DBG_INTR|DBG_EXTRA, "nhi_register_pdf called\n");

	trkr = rp->tracker;

	if (tx != NULL) {
		while (tx->cb != NULL) {
			if ((tx->pdf < 0) || (tx->pdf > 15))
				return (EINVAL);
			slot = &trkr->txpdf[tx->pdf];
			slot->cb = NULL;
			slot->context = NULL;
			tx++;
		}
	}

	if (rx != NULL) {
		while (rx->cb != NULL) {
			if ((rx->pdf < 0) || (rx->pdf > 15))
				return (EINVAL);
			slot = &trkr->rxpdf[rx->pdf];
			slot->cb = NULL;
			slot->context = NULL;
			rx++;
		}
	}

	return (0);
}

/*
 * The CI and PI indexes are not read from the hardware.  We track them in
 * software, so we know where in the ring to start a scan on an interrupt.
 * All we have to do is check for the appropriate Done bit in the next
 * descriptor, and we know if we have reached the last descriptor that the
 * hardware touched.  This technique saves at least 2 MEMIO reads per
 * interrupt.
 */
void
nhi_intr(void *data)
{
	union nhi_ring_desc *rxd;
	struct nhi_cmd_frame *cmd;
	struct nhi_intr_tracker *trkr = data;
	struct nhi_softc *sc;
	struct nhi_ring_pair *r;
	struct nhi_tx_buffer_desc *txd;
	uint32_t val, old_ci;
	u_int count;

	sc = trkr->sc;

	tb_debug(sc, DBG_INTR|DBG_FULL, "Interrupt @ vector %d\n",
	    trkr->vector);
	if ((r = trkr->ring) == NULL)
		return;

	/*
	 * Process TX completions from the adapter.  Only go through
	 * the ring once to prevent unbounded looping.
	 */
	count = r->tx_ring_depth;
	while (count-- > 0) {
		txd = &r->tx_ring[r->tx_ci].tx;
		if ((txd->flags_sof & TX_BUFFER_DESC_DONE) == 0)
			break;
		cmd = r->tx_cmd_ring[r->tx_ci];
		tb_debug(sc, DBG_INTR|DBG_TXQ|DBG_FULL,
		    "Found tx cmdidx= %d cmd= %p\n", r->tx_ci, cmd);

		/* Pass the completion up the stack */
		nhi_tx_complete(r, txd, cmd);

		/*
		 * Advance to the next item in the ring via the cached
		 * copy of the CI.  Clear the flags so we can detect
		 * a new done condition the next time the ring wraps
		 * around.  Anything higher up the stack that needs this
		 * field should have already copied it.
		 *
		 * XXX is a memory barrier needed?
		 */
		txd->flags_sof = 0;
		r->tx_ci = (r->tx_ci + 1) & r->tx_ring_mask;
	}

	/* Process RX packets from the adapter */
	count = r->rx_ring_depth;
	old_ci = r->rx_ci;

	while (count-- > 0) {
		tb_debug(sc, DBG_INTR|DBG_RXQ|DBG_FULL,
		    "Checking RX descriptor at %d\n", r->rx_pi);

		/* Look up RX descriptor and cmd */
		rxd = &r->rx_ring[r->rx_pi];
		tb_debug(sc, DBG_INTR|DBG_RXQ|DBG_FULL,
		    "rx desc len= 0x%04x flags= 0x%04x\n", rxd->rxpost.eof_len,
		    rxd->rxpost.flags_sof);
		if ((rxd->rxpost.flags_sof & RX_BUFFER_DESC_DONE) == 0)
			break;
		cmd = r->rx_cmd_ring[r->rx_pi];
		tb_debug(sc, DBG_INTR|DBG_RXQ|DBG_FULL,
		    "Found rx cmdidx= %d cmd= %p\n", r->rx_pi, cmd);

		/*
		 * Pass the RX frame up the stack.  RX frames are re-used
		 * in-place, so their contents must be copied before this
		 * function returns.
		 *
		 * XXX Rings other than Ring0 might want to have a different
		 * re-use and re-populate policy
		 */
		nhi_rx_complete(r, &rxd->rxpost, cmd);

		/*
		 * Advance the CI and move forward to the next item in the
		 * ring via our cached copy of the PI.  Clear out the
		 * length field so we can detect a new RX frame when the
		 * ring wraps around.  Reset the flags of the descriptor.
		 */
		rxd->rxpost.eof_len = 0;
		rxd->rx.flags = RX_BUFFER_DESC_RS | RX_BUFFER_DESC_IE;
		r->rx_ci = (r->rx_ci + 1) & r->rx_ring_mask;
		r->rx_pi = (r->rx_pi + 1) & r->rx_ring_mask;
	}

	/*
	 * Tell the firmware about the new RX CI
	 *
	 * XXX There's a chance this will overwrite an update to the PI.
	 * Is that OK?  We keep our own copy of the PI and never read it from
	 * hardware.  However, will overwriting it result in a missed
	 * interrupt?
	 */
	if (r->rx_ci != old_ci) {
		val = r->rx_pi << RX_RING_PI_SHIFT | r->rx_ci;
		tb_debug(sc, DBG_INTR | DBG_RXQ,
		    "Writing new RX PICI= 0x%08x\n", val);
		nhi_write_reg(sc, r->rx_pici_reg, val);
	}
}

static int
nhi_setup_sysctl(struct nhi_softc *sc)
{
	struct sysctl_ctx_list	*ctx = NULL;
	struct sysctl_oid	*tree = NULL;

	ctx = device_get_sysctl_ctx(sc->dev);
	if (ctx != NULL)
		tree = device_get_sysctl_tree(sc->dev);

	/*
	 * Not being able to create sysctls is going to hamper other
	 * parts of the driver.
	 */
	if (tree == NULL) {
		tb_printf(sc, "Error: cannot create sysctl nodes\n");
		return (EINVAL);
	}
	sc->sysctl_tree = tree;
	sc->sysctl_ctx = ctx;

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "debug_level", CTLTYPE_STRING|CTLFLAG_RW|CTLFLAG_MPSAFE,
	    &sc->debug, 0, tb_debug_sysctl, "A", "Thunderbolt debug level");
	SYSCTL_ADD_U16(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "max_rings", CTLFLAG_RD, &sc->max_ring_count, 0,
	    "Max number of rings available");
	SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "force_hcm", CTLFLAG_RD, &sc->force_hcm, 0,
	    "Force on/off the function of the host connection manager");
	SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "connmode", CTLFLAG_RD, &sc->mbox_connmode, 0,
	    "FW CM mode (set via hw.nhi.connmode or dev.nhi.N.connmode)");
	SYSCTL_ADD_U8(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "firmware_managed", CTLFLAG_RD, &sc->firmware_managed, 0,
	    "Firmware-managed connection manager");

	return (0);
}

static int
nhi_valid_mbox_connmode(uint32_t mode)
{

	switch (mode) {
	case INMAILCMD_SETMODE_CERT_TB_1ST_DEPTH:
	case INMAILCMD_SETMODE_ANY_TB_1ST_DEPTH:
	case INMAILCMD_SETMODE_CERT_TB_ANY_DEPTH:
	case INMAILCMD_SETMODE_ANY_TB_ANY_DEPTH:
		return (1);
	default:
		return (0);
	}
}

static int
nhi_set_fw_cm_mode(struct nhi_softc *sc)
{
	uint32_t mode;
	uint32_t out;
	int error;

	mode = sc->mbox_connmode;
	if (!nhi_valid_mbox_connmode(mode)) {
		tb_printf(sc, "Invalid FW CM mode 0x%x, using default\n",
		    mode);
		mode = NHI_MBOX_CONNMODE;
		sc->mbox_connmode = mode;
	}

	/*
	 * Run the full firmware detection and reset flow.
	 * This attempts to start firmware if not running, performs
	 * CIO reset if needed, and detects safe mode.
	 */
	error = nhi_detect_fw_mode(sc);
	if (error != 0) {
		tb_printf(sc,
		    "Cannot enter FW CM mode (%d)\n", error);
		return (error);
	}

	/*
	 * Firmware is in CM mode — wait for it to be ready,
	 * then configure the connection security policy.
	 */
	error = nhi_wait_fw_cm_ready(sc);
	if (error != 0) {
		tb_printf(sc,
		    "FW CM not ready after mode detection (%d)\n", error);
		sc->fw_safe_mode = 1;
		return (error);
	}

	error = nhi_wait_outmail_ready(sc, &out);
	if (error != 0) {
		tb_printf(sc, "Outmailcmd not ready (%d)\n", error);
		sc->fw_safe_mode = 1;
		return (error);
	}

	tb_printf(sc, "Outmailcmd: 0x%08x (%s)\n", out,
	    tb_get_string(out & OUTMAILCMD_OPMODE_MASK,
	    nhi_outmailcmd_opmode));

	tb_printf(sc, "FW CM mode: %s\n",
	    tb_get_string(mode, tb_mbox_connmode));
	error = nhi_inmail_cmd(sc, mode, 0);
	if (error != 0)
		return (error);

	/* Authorize already-connected devices to stop hotplug storm. */
	return (nhi_inmail_cmd(sc, INMAILCMD_SAVE_CONNECTED, 0));
}

int
nhi_ensure_fw_cm_mode(struct nhi_softc *sc)
{
	int error;

	if (sc->fw_cm_set)
		return (0);
	if (sc->fw_safe_mode)
		return (ENODEV);
	error = nhi_set_fw_cm_mode(sc);
	if (error == 0 && !sc->fw_safe_mode)
		sc->fw_cm_set = 1;
	return (error);
}

static int
nhi_wait_fw_cm_ready(struct nhi_softc *sc)
{
	uint32_t val;
	int i;

	for (i = 0; i < NHI_FW_CM_READY_TIMEOUT_SEC * 10; i++) {
		val = nhi_read_reg(sc, TBT_FW_STATUS);
		if (val & FWSTATUS_CM_READY)
			return (0);
		pause("tbtfw", hz / 10);
	}

	tb_printf(sc, "FW CM not ready, FW_STATUS=0x%08x\n", val);
	return (ETIMEDOUT);
}

static int
nhi_wait_outmail_ready(struct nhi_softc *sc, uint32_t *outp)
{
	uint32_t out;
	int i;

	out = 0;
	for (i = 0; i < NHI_FW_CM_READY_TIMEOUT_SEC * 10; i++) {
		out = nhi_read_reg(sc, TBT_OUTMAILCMD);
		if ((out & OUTMAILCMD_STATUS_BUSY) == 0)
			break;
		pause("tbtmb", hz / 10);
	}

	if (outp != NULL)
		*outp = out;

	if (out & OUTMAILCMD_STATUS_BUSY) {
		tb_printf(sc, "Mailbox busy, OUTMAILCMD=0x%08x\n", out);
		return (ETIMEDOUT);
	}

	return (0);
}

/*
 * Firmware initialization and reset functions.
 * Ported from Linux ICM driver (drivers/thunderbolt/icm.c).
 */

/*
 * Read the firmware operation mode from the OUTMAILCMD register.
 * Returns the opmode field (0=safe, 1=auth, 2=endpoint, 3=CM).
 */
static int
nhi_fw_opmode(struct nhi_softc *sc)
{
	uint32_t val;

	val = nhi_read_reg(sc, TBT_OUTMAILCMD);

	/* 0xdeadbeaf or similar garbage means firmware never initialized */
	if (val == 0xdeadbeaf || val == 0xffffffff) {
		tb_printf(sc, "OUTMAILCMD reads 0x%08x, firmware not running\n",
		    val);
		return (-1);
	}

	return ((val & OUTMAILCMD_OPMODE_MASK) >> 8);
}

/*
 * Reset the firmware CIO (Common I/O) block.
 * Sets the CIO_RESET_REQ flag, then toggles the firmware CPU enable bits
 * in the FW_STATUS register.  This is the equivalent of Linux's
 * icm_firmware_reset().
 *
 * On Light Ridge, the CIO reset through PCIe2CIO is not available
 * (no upstream port), so we rely solely on the FW_STATUS register poke.
 */
static int
nhi_firmware_reset(struct nhi_softc *sc)
{
	uint32_t val;

	tb_printf(sc, "Attempting firmware CIO reset\n");

	/* Step 1: Request CIO reset via FW_STATUS register */
	val = nhi_read_reg(sc, TBT_FW_STATUS);
	val |= FWSTATUS_CIO_RESET;
	nhi_write_reg(sc, TBT_FW_STATUS, val);

	/* Step 2: Start the firmware CPU (toggle enable bits) */
	val = nhi_read_reg(sc, TBT_FW_STATUS);
	val |= FWSTATUS_INVERT | FWSTATUS_START;
	nhi_write_reg(sc, TBT_FW_STATUS, val);

	/*
	 * Step 3: Send CIO_RESET command through the input mailbox.
	 * This may timeout if firmware is truly dead, which is fine —
	 * we'll check the result via FW_STATUS afterward.
	 */
	(void)nhi_inmail_cmd(sc, INMAILCMD_CIO_RESET, 0);

	/* Allow firmware time to reset and reinitialize */
	pause("tbcio", hz);

	return (0);
}

/*
 * Wait for firmware NVM authentication to complete.
 * Returns 0 if NVM_AUTH_DONE is set, ETIMEDOUT otherwise.
 */
static int
nhi_wait_nvm_auth(struct nhi_softc *sc)
{
	uint32_t val;
	int i;

	for (i = 0; i < 60; i++) {
		val = nhi_read_reg(sc, TBT_FW_STATUS);
		if (val & FWSTATUS_CM_READY)
			return (0);
		pause("tbnvm", hz / 20);	/* 50ms intervals, 3s total */
	}

	tb_debug(sc, DBG_INIT, "NVM auth timeout, FW_STATUS=0x%08x\n", val);
	return (ETIMEDOUT);
}

/*
 * Attempt to start the firmware and bring it out of safe mode.
 * This is the equivalent of Linux's icm_firmware_start().
 *
 * Sequence:
 * 1. Check if firmware is already running (FWSTATUS_ENABLE set)
 * 2. If not, perform a CIO reset to restart it
 * 3. Wait for NVM authentication to complete
 * 4. Re-check the firmware mode
 */
static int
nhi_firmware_start(struct nhi_softc *sc)
{
	uint32_t val;
	int mode;
	int error;

	/* Check if firmware is already running */
	val = nhi_read_reg(sc, TBT_FW_STATUS);
	tb_printf(sc, "FW_STATUS=0x%08x\n", val);

	/*
	 * If register reads return a garbage sentinel, the firmware
	 * MMIO region is not functional.  No point in poking registers.
	 */
	if (val == 0xdeadbeaf || val == 0xffffffff) {
		tb_printf(sc, "FW_STATUS reads garbage, firmware MMIO dead\n");
		return (ENXIO);
	}

	tb_debug(sc, DBG_INIT, "FW_STATUS:%s%s%s%s%s\n",
	    (val & FWSTATUS_ENABLE) ? " EN" : "",
	    (val & FWSTATUS_INVERT) ? " INV" : "",
	    (val & FWSTATUS_START) ? " CPU" : "",
	    (val & FWSTATUS_CIO_RESET) ? " CIO_RST" : "",
	    (val & FWSTATUS_CM_READY) ? " CM_READY" : "");

	if (val & FWSTATUS_ENABLE) {
		tb_printf(sc, "Firmware already running (FW_STS EN bit)\n");
		return (0);
	}

	/*
	 * FW_STS EN bit not set, but firmware might still be alive
	 * (e.g., warm-booted from macOS).  Check the mailbox before
	 * attempting a destructive CIO reset.
	 */
	mode = nhi_fw_opmode(sc);
	if (mode == 3) {	/* CM mode — firmware is alive */
		tb_printf(sc, "Firmware alive (CM mode), skipping reset\n");
		return (0);
	}
	if (mode >= 0 && mode != 0) {
		tb_printf(sc, "Firmware in mode %d, skipping reset\n", mode);
		return (0);
	}

	/*
	 * Firmware is not running.  Attempt a CIO reset to restart it.
	 * This is our best shot at getting Light Ridge out of safe mode.
	 */
	error = nhi_firmware_reset(sc);
	if (error != 0) {
		tb_printf(sc, "Firmware reset failed: %d\n", error);
		return (error);
	}

	/* Wait for NVM authentication to complete after reset */
	error = nhi_wait_nvm_auth(sc);
	if (error != 0) {
		tb_printf(sc, "NVM auth did not complete after reset\n");
		/* Not fatal — firmware may still be in safe mode */
	}

	/* Re-read the mode after reset attempt */
	mode = nhi_fw_opmode(sc);
	tb_printf(sc, "Post-reset firmware mode: %d (%s)\n", mode,
	    (mode < 0) ? "not running" :
	    (mode == 0) ? "safe" :
	    (mode == 3) ? "CM" : "other");

	return (0);
}

/*
 * Detect the firmware mode and set fw_safe_mode accordingly.
 * If firmware is in safe mode or not running, attempt a reset.
 * This is called early during NHI initialization, before any
 * router or topology operations.
 *
 * Equivalent of Linux's icm_firmware_init() + icm_driver_ready().
 */
static int
nhi_detect_fw_mode(struct nhi_softc *sc)
{
	int error, mode;

	sc->fw_safe_mode = 0;

	/* Try to start firmware if it's not running */
	error = nhi_firmware_start(sc);
	if (error != 0) {
		/*
		 * Firmware MMIO is dead — registers return garbage.
		 * This typically means the Light Ridge NVM is
		 * corrupted or was never programmed by macOS.
		 * Nothing we can do except bail out cleanly.
		 */
		tb_printf(sc, "Firmware startup failed (%d)\n", error);
		sc->fw_safe_mode = 1;
		return (error);
	}

	/* Read the current firmware mode */
	mode = nhi_fw_opmode(sc);

	if (mode < 0) {
		tb_printf(sc, "Firmware not responding, entering safe mode\n");
		sc->fw_safe_mode = 1;
		return (ENODEV);
	}

	switch (mode) {
	case 0:	/* OUTMAILCMD_OPMODE_SAFE >> 8 */
		tb_printf(sc,
		    "Firmware is in safe mode (NVM may need update)\n");
		sc->fw_safe_mode = 1;
		return (ENODEV);
	case 3:	/* OUTMAILCMD_OPMODE_CM_FULL >> 8 */
		tb_printf(sc,
		    "Firmware in CM mode, ready for enumeration\n");
		sc->fw_safe_mode = 0;
		return (0);
	default:
		tb_printf(sc,
		    "Firmware in unexpected mode %d, entering safe mode\n",
		    mode);
		sc->fw_safe_mode = 1;
		return (ENODEV);
	}
}
