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

/*
 * Host Configuration Manager (HCM) for USB4 and later TB3.
 *
 * On TB1/TB2 (NHI_TYPE_FW_CM), the HCM relies on firmware-managed PCIe
 * tunnels established before the driver loads.  Apple EFI sets these up;
 * coreboot requires a payload that initialises the Light Ridge controller.
 * The driver maintains the firmware CM state but does not create tunnels.
 */
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
#include <machine/stdarg.h>

#include <dev/thunderbolt/nhi_reg.h>
#include <dev/thunderbolt/nhi_var.h>
#include <dev/thunderbolt/tb_reg.h>
#include <dev/thunderbolt/tb_var.h>
#include <dev/thunderbolt/tb_debug.h>
#include <dev/thunderbolt/tbcfg_reg.h>
#include <dev/thunderbolt/router_var.h>
#include <dev/thunderbolt/hcm_var.h>

static void hcm_cfg_task(void *, int);
static void hcm_disconnect(struct hcm_softc *);

int
hcm_attach(struct nhi_softc *nsc)
{
	struct hcm_softc *hcm;

	tb_debug(nsc, DBG_HCM|DBG_EXTRA, "hcm_attach called\n");

	hcm = malloc(sizeof(struct hcm_softc), M_THUNDERBOLT, M_NOWAIT|M_ZERO);
	if (hcm == NULL) {
		tb_debug(nsc, DBG_HCM, "Cannot allocate hcm object\n");
		return (ENOMEM);
	}

	hcm->dev = nsc->dev;
	hcm->nsc = nsc;
	nsc->hcm = hcm;

	/*
	 * Set FW CM mode and send SAVE_CONNECTED before interrupt
	 * handlers are registered to pre-authorise connected devices
	 * and prevent a PDF_HOTPLUG storm.
	 */
	if (NHI_IS_FW_CM(nsc))
		nhi_ensure_fw_cm_mode(nsc);

	hcm->taskqueue = taskqueue_create("hcm_event", M_NOWAIT,
	    taskqueue_thread_enqueue, &hcm->taskqueue);
	if (hcm->taskqueue == NULL) {
		nsc->hcm = NULL;
		free(hcm, M_THUNDERBOLT);
		return (ENOMEM);
	}
	taskqueue_start_threads(&hcm->taskqueue, 1, PI_DISK, "tbhcm%d_tq",
	    device_get_unit(nsc->dev));
	TASK_INIT(&hcm->cfg_task, 0, hcm_cfg_task, hcm);

	return (0);
}

int
hcm_detach(struct nhi_softc *nsc)
{
	struct hcm_softc *hcm;

	hcm = nsc->hcm;
	if (hcm == NULL)
		return (0);
	if (hcm->taskqueue) {
		taskqueue_drain(hcm->taskqueue, &hcm->cfg_task);
		taskqueue_free(hcm->taskqueue);
	}
	nsc->hcm = NULL;
	free(hcm, M_THUNDERBOLT);

	return (0);
}

/*
 * Rescan PCI buses behind Thunderbolt bridges.  After the firmware
 * connection manager authorises connected devices, the PCIe tunnels
 * become active and downstream devices (ethernet, FireWire, USB
 * bridge) start responding to config-space reads.  If the TB module
 * was loaded after the initial PCI bus scan, those buses will be
 * empty and need a rescan.
 *
 * Walk from the NHI device up to the parent PCI bus (which hosts all
 * Light Ridge downstream bridges), then rescan every sibling PCI bus.
 */
static void
hcm_pci_rescan(struct hcm_softc *hcm)
{
	device_t nhi_dev, pci_bus, bridge, parent_bus;
	device_t *children;
	int i, nchildren;

	nhi_dev = hcm->dev;

	/*
	 * nhi0 → pci4 → pcib4 → pci3
	 * We want pci3, the bus that hosts all downstream bridges.
	 */
	pci_bus = device_get_parent(nhi_dev);	/* pci4 */
	if (pci_bus == NULL)
		return;
	bridge = device_get_parent(pci_bus);	/* pcib4 */
	if (bridge == NULL)
		return;
	parent_bus = device_get_parent(bridge);	/* pci3 */
	if (parent_bus == NULL)
		return;

	/*
	 * Re-enumerate the bus that owns the downstream bridges first so a
	 * previously deleted tbolt bridge device is recreated on replug.
	 */
	bus_topo_lock();
	if (device_is_attached(parent_bus))
		BUS_RESCAN(parent_bus);
	bus_topo_unlock();

	/* Enumerate all bridges on the parent bus. */
	if (device_get_children(parent_bus, &children, &nchildren) != 0)
		return;

	bus_topo_lock();
	for (i = 0; i < nchildren; i++) {
		device_t child, *grandchildren;
		int j, ngrandchildren;

		/* Skip our own bridge (pcib4 / NHI). */
		if (children[i] == bridge)
			continue;

		/* Find the PCI bus child of each sibling bridge. */
		if (device_get_children(children[i], &grandchildren,
		    &ngrandchildren) != 0)
			continue;

		for (j = 0; j < ngrandchildren; j++) {
			child = grandchildren[j];
			if (device_is_attached(child)) {
				tb_printf(hcm, "rescanning %s\n",
				    device_get_nameunit(child));
				BUS_RESCAN(child);
			}
		}
		free(grandchildren, M_TEMP);
	}
	bus_topo_unlock();
	free(children, M_TEMP);
}

int
hcm_router_discover(struct hcm_softc *hcm)
{

	if (hcm->discovery_pending)
		return (0);
	hcm->discovery_pending = 1;
	taskqueue_enqueue(hcm->taskqueue, &hcm->cfg_task);

	return (0);
}

/*
 * Recursively detach child routers from the adapter tree, then delete the
 * corresponding tbolt PCIe bridge device so downstream devices (e.g. bge0)
 * are removed cleanly before their hardware disappears.
 */
static void
hcm_disconnect(struct hcm_softc *hcm)
{
	struct router_softc *rsc;
	devclass_t dc;
	device_t tbolt;
	u_int i;

	if (!hcm->connected)
		return;
	hcm->connected = 0;

	rsc = hcm->nsc->root_rsc;
	if (rsc == NULL)
		return;

	tb_printf(hcm, "TB disconnect: cleaning up downstream routers\n");

	/* Detach all child routers tracked in the adapter array. */
	if (rsc->adapters != NULL) {
		for (i = 0; i <= rsc->max_adap; i++) {
			if (rsc->adapters[i] != NULL) {
				if (tb_router_detach(rsc->adapters[i]) != 0)
					tb_printf(hcm,
					    "TB disconnect: adapter %u busy, keeping topology entry\n",
					    i);
			}
		}
	}

	/*
	 * Delete the tbolt PCIe bridge device.  This cascades through
	 * bus_generic_detach() to all downstream devices (bge, etc.),
	 * preventing hangs from MII reads on dead hardware.
	 *
	 * bus_topo_lock is required: device_detach() asserts it.
	 */
	dc = devclass_find("tbolt");
	if (dc != NULL) {
		tbolt = devclass_get_device(dc, device_get_unit(hcm->dev));
		if (tbolt != NULL) {
			tb_printf(hcm, "TB disconnect: removing %s\n",
			    device_get_nameunit(tbolt));
			bus_topo_lock();
			device_delete_child(device_get_parent(tbolt), tbolt);
			bus_topo_unlock();
		}
	}
}

static void
hcm_cfg_task(void *arg, int pending)
{
	struct hcm_softc *hcm;
	struct router_softc *rsc;
	struct router_cfg_cap cap;
	struct tb_cfg_router *cfg;
	struct tb_cfg_adapter *adp;
	struct tb_cfg_cap_lane *lane;
	uint32_t *buf;
	uint8_t *u;
	u_int error, i, offset;

	hcm = (struct hcm_softc *)arg;
	hcm->discovery_pending = 0;

	tb_debug(hcm, DBG_HCM|DBG_EXTRA, "hcm_cfg_task called\n");

	/* Ensure FW CM mode is set before first discovery. */
	if (NHI_IS_FW_CM(hcm->nsc))
		nhi_ensure_fw_cm_mode(hcm->nsc);

	if (hcm->nsc->fw_safe_mode) {
		tb_debug(hcm, DBG_HCM,
		    "Firmware in safe mode, skipping discovery\n");
		return;
	}

	buf = malloc(8 * 4, M_THUNDERBOLT, M_NOWAIT|M_ZERO);
	if (buf == NULL) {
		tb_debug(hcm, DBG_HCM, "Cannot alloc memory for discovery\n");
		return;
	}

	rsc = hcm->nsc->root_rsc;
	error = tb_config_router_read(rsc, 0, 5, buf);
	if (error != 0) {
		/* Root router unreachable: cable unplugged. */
		free(buf, M_THUNDERBOLT);
		hcm_disconnect(hcm);
		return;
	}

	cfg = (struct tb_cfg_router *)buf;

	cap.space = TB_CFG_CS_ROUTER;
	cap.adap = 0;
	cap.next_cap = GET_ROUTER_CS_NEXT_CAP(cfg);
	while (cap.next_cap != 0) {
		error = tb_config_next_cap(rsc, &cap);
		if (error != 0)
			break;

		if ((cap.cap_id == TB_CFG_CAP_VSEC) && (cap.vsc_len == 0)) {
			tb_debug(hcm, DBG_HCM, "Router Cap= %d, vsec= %d, "
			    "len= %d, next_cap= %d\n", cap.cap_id,
			    cap.vsc_id, cap.vsec_len, cap.next_cap);
		} else if (cap.cap_id == TB_CFG_CAP_VSC) {
			tb_debug(hcm, DBG_HCM, "Router cap= %d, vsc= %d, "
			    "len= %d, next_cap= %d\n", cap.cap_id,
			    cap.vsc_id, cap.vsc_len, cap.next_cap);
		} else
			tb_debug(hcm, DBG_HCM, "Router cap= %d, "
			    "next_cap= %d\n", cap.cap_id, cap.next_cap);
		if (cap.next_cap > TB_CFG_CAP_OFFSET_MAX)
			cap.next_cap = 0;
	}

	u = (uint8_t *)buf;
	error = tb_config_get_lc_uuid(rsc, u);
	if (error == 0) {
		tb_debug(hcm, DBG_HCM, "Router LC UUID: %02x%02x%02x%02x-"
		    "%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x\n",
		    u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], u[8],
		    u[9], u[10], u[11], u[12], u[13], u[14], u[15]);
	} else
		tb_printf(hcm, "Error finding LC registers: %d\n", error);

	for (i = 1; i <= rsc->max_adap; i++) {
		error = tb_config_adapter_read(rsc, i, 0, 8, buf);
		if (error != 0) {
			tb_debug(hcm, DBG_HCM, "Adapter %d: no adapter\n", i);
			continue;
		}
		adp = (struct tb_cfg_adapter *)buf;
		tb_debug(hcm, DBG_HCM, "Adapter %d: %s, max_counters= 0x%08x,"
		    " adapter_num= %d\n", i,
		    tb_get_string(GET_ADP_CS_TYPE(adp), tb_adapter_type),
		    GET_ADP_CS_MAX_COUNTERS(adp), GET_ADP_CS_ADP_NUM(adp));

		if (GET_ADP_CS_TYPE(adp) != ADP_CS2_LANE)
			continue;

		error = tb_config_find_adapter_cap(rsc, i, TB_CFG_CAP_LANE,
		    &offset);
		if (error)
			continue;

		error = tb_config_adapter_read(rsc, i, offset, 3, buf);
		if (error)
			continue;

		lane = (struct tb_cfg_cap_lane *)buf;
		tb_debug(hcm, DBG_HCM, "Lane Adapter State= %s %s\n",
		    tb_get_string((lane->current_lws & CAP_LANE_STATE_MASK),
		    tb_adapter_state), (lane->targ_lwp & CAP_LANE_DISABLE) ?
		    "disabled" : "enabled");

		if ((lane->current_lws & CAP_LANE_STATE_MASK) ==
		    CAP_LANE_STATE_CL0) {
			tb_route_t newr;

			newr = TB_CHILD_ROUTE(rsc, i);

			tb_printf(hcm, "want to add router at 0x%08x%08x\n",
			    newr.hi, newr.lo);
			error = tb_router_attach(rsc, newr);
			tb_printf(rsc, "tb_router_attach returned %d\n", error);
			if (error == 0)
				hcm->connected = 1;
		} else if (rsc->adapters != NULL && i <= rsc->max_adap &&
		    rsc->adapters[i] != NULL) {
			/* Lane dropped out of CL0: device disconnected. */
			tb_printf(hcm, "TB disconnect on adapter %d\n", i);
			error = tb_router_detach(rsc->adapters[i]);
			if (error != 0)
				tb_printf(hcm,
				    "Failed to detach router on adapter %d: %d\n",
				    i, error);
		}
	}

	free(buf, M_THUNDERBOLT);

	/* Discover existing DP tunnels on firmware-managed controllers. */
	if (hcm->nsc->firmware_managed)
		tb_tunnel_discover_dp(rsc);

	/* Rescan sibling PCI buses for newly-visible devices. */
	if (hcm->connected)
		hcm_pci_rescan(hcm);
}
