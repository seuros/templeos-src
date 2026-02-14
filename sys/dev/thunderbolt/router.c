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

/* Config space access for switches, ports, and devices in TB3 and USB4 */
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
#include <sys/refcount.h>
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
#include <dev/thunderbolt/tbcfg_reg.h>
#include <dev/thunderbolt/router_var.h>
#include <dev/thunderbolt/hcm_var.h>
#include <dev/thunderbolt/tb_debug.h>
#include <dev/thunderbolt/tb_dev.h>

static int router_alloc_cmd(struct router_softc *, struct router_command **);
static void router_free_cmd(struct router_softc *, struct router_command *);
static int _tb_router_attach(struct router_softc *);
static void router_finalize_frame(struct nhi_cmd_frame *, uint16_t, size_t);
static void router_prepare_read(struct router_softc *, struct router_command *,
    int);
static void router_prepare_write(struct router_softc *, struct router_command *,
    int);
static int _tb_config_read(struct router_softc *, u_int, u_int, u_int, u_int,
    uint32_t *, void *, struct router_command **);
static int _tb_config_write(struct router_softc *, u_int, u_int, u_int, u_int,
    uint32_t *, void *, struct router_command **);
static int router_schedule(struct router_softc *, struct router_command *);
static int router_schedule_locked(struct router_softc *,
    struct router_command *);
static nhi_ring_cb_t router_complete_intr;
static nhi_ring_cb_t router_response_intr;
static nhi_ring_cb_t router_notify_intr;
static void router_hotplug_task(void *, int);
static int router_lane_disable(struct router_softc *, u_int, int);
static int router_detach_tree(struct router_softc *);
static int router_detach_child(struct router_softc *, u_int);
static int router_direct_child_hop(struct router_softc *, struct router_softc *,
    uint8_t *);
static void router_hotplug_task_done(struct router_hotplug_task *);
static struct router_hotplug_task *router_hotplug_task_hold_first(
    struct router_softc *);
static void router_sever_topology(struct router_softc *);
static void router_rescan_legacy(struct router_softc *, u_int);
static void router_send_hotplug_ack(struct router_softc *, tb_route_t, u_int,
    int);

#define CFG_DEFAULT_RETRIES	3
#define CFG_DEFAULT_TIMEOUT	2
#define TBCFG_MAX_DWORDS	((NHI_RING0_FRAME_SIZE / 4) - 4)

struct router_hotplug_task {
	struct task		task;
	TAILQ_ENTRY(router_hotplug_task) link;
	struct router_softc	*sc;
	tb_route_t		route;
	u_int			adap;
	volatile u_int		refs;
	int			unplug;
	int			linked;
};

static int
router_direct_child_hop(struct router_softc *parent, struct router_softc *child,
    uint8_t *hopp)
{
	uint64_t parent_rt, child_rt, mask;
	uint8_t hop;

	if (parent == NULL || child == NULL || hopp == NULL)
		return (EINVAL);

	child_rt = TB_ROUTE(child);
	hop = (child_rt >> (parent->depth * 8)) & 0xff;
	if (hop == 0)
		return (EINVAL);

	parent_rt = TB_ROUTE(parent);
	mask = ~((uint64_t)0xff << (parent->depth * 8));
	if ((child_rt & mask) != parent_rt)
		return (EINVAL);

	*hopp = hop;
	return (0);
}

static void
router_hotplug_task_done(struct router_hotplug_task *ht)
{
	struct router_softc *sc;

	sc = ht->sc;
	mtx_lock(&sc->mtx);
	if (ht->linked) {
		TAILQ_REMOVE(&sc->hotplug_tasks, ht, link);
		ht->linked = 0;
		refcount_release(&ht->refs);
	}
	mtx_unlock(&sc->mtx);

	if (refcount_release(&ht->refs))
		free(ht, M_THUNDERBOLT);
}

static struct router_hotplug_task *
router_hotplug_task_hold_first(struct router_softc *sc)
{
	struct router_hotplug_task *ht;

	mtx_lock(&sc->mtx);
	ht = TAILQ_FIRST(&sc->hotplug_tasks);
	if (ht != NULL)
		refcount_acquire(&ht->refs);
	mtx_unlock(&sc->mtx);

	return (ht);
}

static void
router_sever_topology(struct router_softc *sc)
{
	struct router_softc *parent;
	uint8_t hop;

	parent = sc->parent;
	if (parent != NULL && parent->adapters != NULL &&
	    router_direct_child_hop(parent, sc, &hop) == 0 &&
	    hop <= parent->max_adap && parent->adapters[hop] == sc)
		parent->adapters[hop] = NULL;

	sc->parent = NULL;
	if (sc->nsc != NULL && sc->nsc->root_rsc == sc)
		sc->nsc->root_rsc = NULL;
}

static int
router_lookup_device(struct router_softc *sc, tb_route_t route,
    struct router_softc **dev)
{
	struct router_softc *cursor;
	uint64_t search_rt, remainder_rt, this_rt;
	uint8_t hop;

	KASSERT(dev != NULL, ("dev cannot be NULL\n"));

	cursor = tb_config_get_root(sc);
	remainder_rt = search_rt = route.lo | ((uint64_t)route.hi << 32);
	tb_debug(sc, DBG_ROUTER|DBG_EXTRA,
	    "%s: Searching for router 0x%016jx\n", __func__, search_rt);

	while (cursor != NULL) {
		this_rt = TB_ROUTE(cursor);
		tb_debug(sc, DBG_ROUTER|DBG_EXTRA,
		    "Comparing cursor route 0x%016jx\n", this_rt);
		if (this_rt == search_rt)
			break;

		/* Prepare to go to the next hop node in the route */
		hop = remainder_rt & 0xff;
		remainder_rt >>= 8;
		tb_debug(sc, DBG_ROUTER|DBG_EXTRA,
		    "hop= 0x%02x, remainder= 0x%016jx\n", hop, remainder_rt);

		/*
		 * An adapter index of 0x0 is only for the host interface
		 * adapter on the root route.  The only time that
		 * it's valid for searches is when you're looking for the
		 * root route, and that case has already been handled.
		 */
		if (hop == 0) {
			tb_debug(sc, DBG_ROUTER,
			    "End of route chain, route not found\n");
			return (ENOENT);
		}

		if (hop > cursor->max_adap) {
			tb_debug(sc, DBG_ROUTER,
			    "Route hop out of range for parent\n");
			return (EINVAL);
		}

		if (cursor->adapters == NULL) {
			tb_debug(sc, DBG_ROUTER,
			    "Error, router not fully initialized\n");
			return (EINVAL);
		}

		cursor = cursor->adapters[hop];
	}

	if (cursor == NULL)
		return (ENOENT);

	*dev = cursor;
	return (0);
}

static int
router_insert(struct router_softc *sc, struct router_softc *parent)
{
	uint8_t this_hop;
	int error;

	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "router_insert called\n");

	if (parent == NULL) {
		tb_debug(sc, DBG_ROUTER, "Parent cannot be NULL in insert\n");
		return (EINVAL);
	}

	error = router_direct_child_hop(parent, sc, &this_hop);
	if (error != 0) {
		tb_debug(sc, DBG_ROUTER, "Added route 0x%08x%08x is not a "
		    "direct child of the parent route 0x%08x%08x\n",
		    sc->route.hi, sc->route.lo, parent->route.hi,
		    parent->route.lo);
		return (error);
	}

	tb_debug(sc, DBG_ROUTER, "Inserting route 0x%08x%08x with last hop "
	    "of 0x%02x and depth of %d\n", sc->route.hi, sc->route.lo,
	    this_hop, sc->depth);

	if (this_hop > parent->max_adap) {
		tb_debug(sc, DBG_ROUTER|DBG_EXTRA,
		    "Inserted route is out of range of the parent\n");
		return (EINVAL);
	}

	if (parent->adapters[this_hop] != NULL) {
		tb_debug(sc, DBG_ROUTER|DBG_EXTRA,
		    "Inserted route already exists\n");
		return (EEXIST);
	}

	parent->adapters[this_hop] = sc;

	tb_debug(sc, DBG_ROUTER, "Added router 0x%08x%08x to parent "
	    "0x%08x%08x\n", sc->route.hi, sc->route.lo, parent->route.hi,
	    parent->route.lo);
	return (0);
}

static void
router_finalize_frame(struct nhi_cmd_frame *nhicmd, uint16_t pdf, size_t len)
{
	uint32_t *msg;
	int msglen, i;

	KASSERT(nhicmd != NULL, ("nhicmd cannot be NULL\n"));
	KASSERT(len >= 4, ("Invalid short command\n"));
	KASSERT(len % 4 == 0, ("Message must be 32bit padded\n"));

	msg = (uint32_t *)nhicmd->data;
	msglen = (len - 4) / 4;
	for (i = 0; i < msglen; i++)
		msg[i] = htobe32(msg[i]);

	msg[msglen] = htobe32(tb_calc_crc(nhicmd->data, len - 4));

	nhicmd->pdf = pdf;
	nhicmd->req_len = len;
}

static void
router_hotplug_intr(void *context, union nhi_ring_desc *ring,
    struct nhi_cmd_frame *nhicmd)
{
	struct router_softc *sc = context;
	struct router_hotplug_task *ht;
	struct tb_cfg_hotplug hp;
	u_int adap, unplug;

	hp.route.hi = be32toh(nhicmd->data[0]);
	hp.route.lo = be32toh(nhicmd->data[1]);
	hp.adapter_attrs = be32toh(nhicmd->data[2]);
	hp.route.hi &= ~0x80000000;
	adap = hp.adapter_attrs & TB_CFG_ADPT_MASK;
	unplug = hp.adapter_attrs & TB_CFG_UPG_UNPLUG;

	tb_debug(sc, DBG_ROUTER, "Hotplug event route 0x%08x%08x adap %u %s\n",
	    hp.route.hi, hp.route.lo, adap, unplug ? "unplug" : "plug");

	router_send_hotplug_ack(sc, hp.route, adap, (unplug != 0));
	if (atomic_load_acq_int(&sc->sc_dying))
		return;

	ht = malloc(sizeof(*ht), M_THUNDERBOLT, M_NOWAIT | M_ZERO);
	if (ht == NULL) {
		tb_debug(sc, DBG_ROUTER, "Cannot alloc hotplug task\n");
		return;
	}
	TASK_INIT(&ht->task, 0, router_hotplug_task, ht);
	ht->sc = sc;
	ht->route = hp.route;
	ht->adap = adap;
	ht->unplug = (unplug != 0);
	ht->refs = 2;
	ht->linked = 1;
	mtx_lock(&sc->mtx);
	TAILQ_INSERT_TAIL(&sc->hotplug_tasks, ht, link);
	mtx_unlock(&sc->mtx);
	taskqueue_enqueue(taskqueue_thread, &ht->task);
}

static int
router_register_interrupts(struct router_softc *sc)
{
	struct nhi_dispatch tx[] = { { PDF_READ, router_complete_intr, sc },
				     { PDF_WRITE, router_complete_intr, sc },
				     { 0, NULL, NULL } };
	struct nhi_dispatch rx[] = { { PDF_READ, router_response_intr, sc },
				     { PDF_WRITE, router_response_intr, sc },
				     { PDF_NOTIFY, router_notify_intr, sc },
				     { PDF_HOTPLUG, router_hotplug_intr, sc },
				     { 0, NULL, NULL } };

	return (nhi_register_pdf(sc->ring0, tx, rx));
}

static void
router_deregister_interrupts(struct router_softc *sc)
{
	struct nhi_dispatch tx[] = { { PDF_READ, router_complete_intr, sc },
				     { PDF_WRITE, router_complete_intr, sc },
				     { 0, NULL, NULL } };
	struct nhi_dispatch rx[] = { { PDF_READ, router_response_intr, sc },
				     { PDF_WRITE, router_response_intr, sc },
				     { PDF_NOTIFY, router_notify_intr, sc },
				     { PDF_HOTPLUG, router_hotplug_intr, sc },
				     { 0, NULL, NULL } };

	nhi_deregister_pdf(sc->ring0, tx, rx);
}

static int
router_lane_disable(struct router_softc *sc, u_int adap, int disable)
{
	struct tb_cfg_cap_lane *lane;
	uint32_t buf[3];
	u_int offset;
	int error;
	uint16_t targ;

	error = tb_config_find_adapter_cap(sc, adap, TB_CFG_CAP_LANE, &offset);
	if (error != 0)
		return (error);

	error = tb_config_adapter_read(sc, adap, offset, 3, buf);
	if (error != 0)
		return (error);

	lane = (struct tb_cfg_cap_lane *)buf;
	targ = lane->targ_lwp;
	if (disable)
		targ |= CAP_LANE_DISABLE;
	else
		targ &= ~CAP_LANE_DISABLE;
	if (targ == lane->targ_lwp)
		return (0);
	lane->targ_lwp = targ;

	return (tb_config_adapter_write(sc, adap, offset, 3, buf));
}

static int
router_detach_tree(struct router_softc *sc)
{
	return (tb_router_detach(sc));
}

static int
router_detach_child(struct router_softc *sc, u_int adap)
{
	struct router_softc *child;
	int error;

	if (sc == NULL || sc->adapters == NULL)
		return (EINVAL);
	if (adap == 0 || adap > sc->max_adap)
		return (EINVAL);

	child = sc->adapters[adap];
	if (child == NULL)
		return (ENOENT);

	error = router_detach_tree(child);

	return (error);
}

static void
router_rescan_legacy(struct router_softc *sc, u_int adap)
{
	struct tb_cfg_adapter *adp;
	struct tb_cfg_cap_lane *lane;
	tb_route_t newr;
	uint32_t buf[8];
	u_int offset;
	int error;

	if (sc == NULL || sc->adapters == NULL)
		return;
	if (adap == 0 || adap > sc->max_adap)
		return;

	error = tb_config_adapter_read(sc, adap, 0, 8, buf);
	if (error != 0)
		return;
	adp = (struct tb_cfg_adapter *)buf;
	if (GET_ADP_CS_TYPE(adp) != ADP_CS2_LANE)
		return;

	error = tb_config_find_adapter_cap(sc, adap, TB_CFG_CAP_LANE,
	    &offset);
	if (error != 0)
		return;
	error = tb_config_adapter_read(sc, adap, offset, 3, buf);
	if (error != 0)
		return;

	lane = (struct tb_cfg_cap_lane *)buf;
	if ((lane->current_lws & CAP_LANE_STATE_MASK) != CAP_LANE_STATE_CL0)
		return;

	if (sc->adapters[adap] != NULL)
		return;

	newr = TB_CHILD_ROUTE(sc, adap);
	tb_debug(sc, DBG_ROUTER,
	    "Legacy rescan: add router at 0x%08x%08x (adap %u)\n",
	    newr.hi, newr.lo, adap);
	error = tb_router_attach(sc, newr);
	if (error != 0)
		tb_debug(sc, DBG_ROUTER,
		    "Legacy rescan attach failed: %d\n", error);
}

static void
router_hotplug_task(void *arg, int pending __unused)
{
	struct router_hotplug_task *ht;
	struct router_softc *rsc;
	int error;

	ht = (struct router_hotplug_task *)arg;
	rsc = NULL;
	error = router_lookup_device(ht->sc, ht->route, &rsc);
	if (error != 0 || rsc == NULL) {
		router_hotplug_task_done(ht);
		return;
	}

	if (ht->unplug) {
		error = router_detach_child(rsc, ht->adap);
		if (error != 0 && error != ENOENT && error != EINVAL)
			tb_debug(rsc, DBG_ROUTER,
			    "Failed to detach child adap %u: %d\n",
			    ht->adap, error);
		error = router_lane_disable(rsc, ht->adap, 1);
		if (error != 0)
			tb_debug(rsc, DBG_ROUTER,
			    "Failed to disable lane adap %u: %d\n",
			    ht->adap, error);
	} else {
		error = router_lane_disable(rsc, ht->adap, 0);
		if (error != 0)
			tb_debug(rsc, DBG_ROUTER,
			    "Failed to enable lane adap %u: %d\n",
			    ht->adap, error);
		if (rsc->nsc->hcm != NULL)
			hcm_router_discover(rsc->nsc->hcm);
		else
			router_rescan_legacy(rsc, ht->adap);
	}

	router_hotplug_task_done(ht);
}

int
tb_router_attach(struct router_softc *parent, tb_route_t route)
{
	struct router_softc *sc;
	int error;

	tb_debug(parent, DBG_ROUTER|DBG_EXTRA, "tb_router_attach called\n");

	sc = malloc(sizeof(*sc), M_THUNDERBOLT, M_ZERO|M_NOWAIT);
	if (sc == NULL) {
		tb_debug(parent, DBG_ROUTER, "Cannot allocate root router\n");
		return (ENOMEM);
	}

	sc->dev = parent->dev;
	sc->debug = parent->debug;
	sc->ring0 = parent->ring0;
	sc->route = route;
	sc->nsc = parent->nsc;
	sc->parent = parent;
	sc->depth = parent->depth + 1;

	mtx_init(&sc->mtx, "tbcfg", "Thunderbolt Router Config", MTX_DEF);
	TAILQ_INIT(&sc->cmd_queue);
	TAILQ_INIT(&sc->hotplug_tasks);

	error = router_insert(sc, parent);
	if (error != 0) {
		tb_debug(parent, DBG_ROUTER,
		    "Failed to insert child router: %d\n", error);
		mtx_destroy(&sc->mtx);
		free(sc, M_THUNDERBOLT);
		return (error);
	}

	/*
	 * Insert the child before the first config transaction so responses for
	 * its route can be matched during initial attach.
	 */
	error = _tb_router_attach(sc);
	if (error != 0) {
		tb_debug(parent, DBG_ROUTER,
		    "Failed to attach child router: %d\n", error);
		tb_router_detach(sc);
		return (error);
	}

	if (sc->depth != parent->depth + 1) {
		tb_debug(parent, DBG_ROUTER,
		    "Child router depth mismatch: expected %u got %u\n",
		    parent->depth + 1, sc->depth);
		tb_router_detach(sc);
		return (EINVAL);
	}

	tbdev_add_router(sc);
	return (0);
}

int
tb_router_attach_root(struct nhi_softc *nsc, tb_route_t route)
{
	struct router_softc *sc;
	int error;

	tb_debug(nsc, DBG_ROUTER|DBG_EXTRA, "tb_router_attach_root called\n");

	sc = malloc(sizeof(*sc), M_THUNDERBOLT, M_ZERO|M_NOWAIT);
	if (sc == NULL) {
		tb_debug(nsc, DBG_ROUTER, "Cannot allocate root router\n");
		return (ENOMEM);
	}

	sc->dev = nsc->dev;
	sc->debug = nsc->debug;
	sc->ring0 = nsc->ring0;
	sc->route = route;
	sc->nsc = nsc;
	sc->parent = NULL;

	mtx_init(&sc->mtx, "tbcfg", "Thunderbolt Router Config", MTX_DEF);
	TAILQ_INIT(&sc->cmd_queue);
	TAILQ_INIT(&sc->hotplug_tasks);

	/*
	 * This router is semi-virtual and represents the router that's part
	 * of the NHI DMA engine.  Commands can't be issued to the topology
	 * until the NHI is initialized and this router is initialized, so
	 * there's no point in registering router interrupts earlier than this,
	 * even if other routers are found first.
	 */
	tb_config_set_root(sc);
	error = router_register_interrupts(sc);
	if (error) {
		tb_router_detach(sc);
		return (error);
	}

	error = _tb_router_attach(sc);
	if (error) {
		tb_router_detach(sc);
		return (error);
	}

	bcopy((uint8_t *)sc->uuid, nsc->uuid, 16);
	tbdev_add_router(sc);
	return (0);
}

static int
_tb_router_attach(struct router_softc *sc)
{
	struct tb_cfg_router *cfg;
	uint32_t *buf;
	int error, up;

	buf = malloc(9 * 4, M_THUNDERBOLT, M_NOWAIT|M_ZERO);
	if (buf == NULL)
		return (ENOMEM);

	error = tb_config_router_read_polled(sc, 0, 9, buf);
	if (error != 0) {
		free(buf, M_THUNDERBOLT);
		return (error);
	}

	cfg = (struct tb_cfg_router *)buf;
	up = GET_ROUTER_CS_UPSTREAM_ADAP(cfg);
	sc->max_adap = GET_ROUTER_CS_MAX_ADAP(cfg);
	sc->depth = GET_ROUTER_CS_DEPTH(cfg);
	sc->uuid[0] = cfg->uuid_lo;
	sc->uuid[1] = cfg->uuid_hi;
	sc->uuid[2] = 0xffffffff;
	sc->uuid[3] = 0xffffffff;
	tb_debug(sc, DBG_ROUTER, "Router upstream_port= %d, max_port= %d, "
	    "depth= %d\n", up, sc->max_adap, sc->depth);
	free(buf, M_THUNDERBOLT);

	/* Downstream adapters are indexed in the array allocated here. */
	sc->max_adap = MIN(sc->max_adap, ROUTER_CS1_MAX_ADAPTERS);
	sc->adapters = malloc((1 + sc->max_adap) * sizeof(void *),
	    M_THUNDERBOLT, M_NOWAIT|M_ZERO);
	if (sc->adapters == NULL) {
		tb_debug(sc, DBG_ROUTER,
		    "Cannot allocate downstream adapter memory\n");
		return (ENOMEM);
	}
	SLIST_INIT(&sc->tunnel_list);

	tb_debug(sc, DBG_ROUTER, "Router created, route 0x%08x%08x\n",
	    sc->route.hi, sc->route.lo);

	return (0);
}

int
tb_router_detach(struct router_softc *sc)
{
	struct router_hotplug_task *ht;
	struct router_softc *child;
	int error;
	u_int i;
	int is_root;

	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "tb_router_deattach called\n");

	is_root = (sc->ring0 != NULL && sc->nsc != NULL &&
	    sc->nsc->root_rsc == sc);
	atomic_store_rel_int(&sc->sc_dying, 1);

	/*
	 * Pending hotplug tasks only hold a root-router pointer.  Once detach
	 * starts, new hotplug tasks are suppressed by sc_dying; drain the ones
	 * already queued before tearing down children.
	 */
	if (is_root) {
		while ((ht = router_hotplug_task_hold_first(sc)) != NULL) {
			taskqueue_drain(taskqueue_thread, &ht->task);
			if (refcount_release(&ht->refs))
				free(ht, M_THUNDERBOLT);
		}
	}

	if (sc->inflight_cmd != NULL || TAILQ_FIRST(&sc->cmd_queue) != NULL) {
		atomic_store_rel_int(&sc->sc_dying, 0);
		return (EBUSY);
	}

	if (sc->adapters != NULL) {
		for (i = 1; i <= sc->max_adap; i++) {
			child = sc->adapters[i];
			if (child == NULL)
				continue;
			error = tb_router_detach(child);
			if (error != 0) {
				atomic_store_rel_int(&sc->sc_dying, 0);
				return (error);
			}
		}
	}

	if (sc->inflight_cmd != NULL || TAILQ_FIRST(&sc->cmd_queue) != NULL) {
		atomic_store_rel_int(&sc->sc_dying, 0);
		return (EBUSY);
	}

	/*
	 * Only deregister ring callbacks for the root router.
	 * The PDF handlers are global per-ring — child router detach
	 * must not tear down the root router's dispatch table.
	 */
	if (is_root)
		router_deregister_interrupts(sc);

	router_sever_topology(sc);

	mtx_destroy(&sc->mtx);

	tb_tunnel_free_all(sc);

	if (sc->adapters != NULL)
		free(sc->adapters, M_THUNDERBOLT);

	tbdev_remove_router(sc);
	free(sc, M_THUNDERBOLT);

	return (0);
}

/*
 * Free all discovered tunnels on a router.
 */
void
tb_tunnel_free_all(struct router_softc *sc)
{
	struct tb_tunnel *tn;

	while ((tn = SLIST_FIRST(&sc->tunnel_list)) != NULL) {
		SLIST_REMOVE_HEAD(&sc->tunnel_list, link);
		free(tn, M_THUNDERBOLT);
	}
}

/*
 * Walk the hop chain starting at (adap, hopid) on the given router.
 * Fills path->hops[] and sets path->nhops.
 * Returns 0 on success, error if a hop read fails or the chain is broken.
 */
static int
tb_path_discover(struct router_softc *sc, u_int adap, u_int hopid,
    struct tb_path *path)
{
	struct tb_cfg_hop hop;
	struct tb_path_hop *ph;
	int error, i;

	path->nhops = 0;
	path->activated = 0;

	for (i = 0; i < TB_PATH_MAX_HOPS; i++) {
		error = tb_config_path_read(sc, adap, hopid, 1,
		    (uint32_t *)&hop);
		if (error != 0) {
			tb_debug(sc, DBG_ROUTER,
			    "Path read failed adap %u hop %u: %d\n",
			    adap, hopid, error);
			return (error);
		}

		if (!hop.enable)
			break;

		ph = &path->hops[i];
		ph->rsc = sc;
		ph->in_port = adap;
		ph->in_hopid = hopid;
		ph->out_port = hop.out_port;
		ph->next_hopid = hop.next_hop;
		ph->hop = hop;
		path->nhops++;

		tb_debug(sc, DBG_ROUTER,
		    "  hop %d: adap %u hopid %u -> out_port %u "
		    "next_hop %u credits %u\n",
		    i, adap, hopid, hop.out_port,
		    hop.next_hop, hop.initial_credits);

		/*
		 * Follow to the next switch.  The out_port on this switch
		 * connects to the remote switch's ingress adapter.  For
		 * single-switch tunnels (TB1/TB2 root), the path terminates
		 * at the out_port adapter directly.
		 */
		if (hop.out_port == 0 || hop.next_hop == 0)
			break;

		/*
		 * For multi-switch paths, we'd need to look up the
		 * downstream router at out_port and continue the walk
		 * there.  For now, single-switch paths only.
		 */
		if (sc->adapters != NULL &&
		    hop.out_port <= sc->max_adap &&
		    sc->adapters[hop.out_port] != NULL) {
			sc = sc->adapters[hop.out_port];
			adap = hop.out_port;
			hopid = hop.next_hop;
		} else {
			break;
		}
	}

	if (path->nhops > 0)
		path->activated = 1;

	return (0);
}

/*
 * Check if a DP adapter is enabled by reading its config space.
 * Returns non-zero if Video Enable or AUX Enable is set.
 */
static int
tb_dp_port_is_enabled(struct router_softc *sc, u_int adap, u_int cap_offset)
{
	uint32_t buf[2];
	int error;

	error = tb_config_adapter_read(sc, adap, cap_offset + ADP_DP_CS_0,
	    2, buf);
	if (error != 0)
		return (0);

	return ((buf[0] & (ADP_DP_CS_0_VE | ADP_DP_CS_0_AE)) != 0);
}

/*
 * Check if HPD (Hot Plug Detect) is active on a DP OUT adapter.
 */
static int __unused
tb_dp_port_hpd_is_active(struct router_softc *sc, u_int adap, u_int cap_offset)
{
	uint32_t val;
	int error;

	error = tb_config_adapter_read(sc, adap, cap_offset + ADP_DP_CS_2,
	    1, &val);
	if (error != 0)
		return (0);

	return ((val & ADP_DP_CS_2_HPD) != 0);
}

/*
 * Discover existing DP tunnels on a router by scanning DP IN adapters.
 * For each enabled DP IN, read the video/aux hop IDs and trace the paths.
 */
int
tb_tunnel_discover_dp(struct router_softc *sc)
{
	struct tb_tunnel *tn;
	uint32_t buf[8], dp_cs[4];
	u_int i, adp_type, cap_offset;
	u_int video_hopid, aux_tx_hopid, aux_rx_hopid;
	int error, found;

	found = 0;

	for (i = 1; i <= sc->max_adap; i++) {
		error = tb_config_adapter_read(sc, i, 0, 8, buf);
		if (error != 0)
			continue;

		adp_type = ((struct tb_cfg_adapter *)buf)->adp_cs2 &
		    ADP_CS2_TYPE_MASK;

		if (adp_type != ADP_CS2_DP_IN)
			continue;

		/* Find the DP adapter capability */
		error = tb_config_find_adapter_cap(sc, i,
		    TB_CFG_CAP_ADP, &cap_offset);
		if (error != 0) {
			tb_debug(sc, DBG_ROUTER,
			    "DP IN adap %u: no adapter cap (%d)\n",
			    i, error);
			continue;
		}

		/* Read DP config registers */
		if (!tb_dp_port_is_enabled(sc, i, cap_offset)) {
			tb_debug(sc, DBG_ROUTER,
			    "DP IN adap %u: not enabled\n", i);
			continue;
		}

		error = tb_config_adapter_read(sc, i,
		    cap_offset + ADP_DP_CS_0, 4, dp_cs);
		if (error != 0)
			continue;

		video_hopid = (dp_cs[0] & ADP_DP_CS_0_VIDEO_HOPID_MASK) >>
		    ADP_DP_CS_0_VIDEO_HOPID_SHIFT;
		aux_tx_hopid = dp_cs[1] & ADP_DP_CS_1_AUX_TX_HOPID_MASK;
		aux_rx_hopid = (dp_cs[1] & ADP_DP_CS_1_AUX_RX_HOPID_MASK) >>
		    ADP_DP_CS_1_AUX_RX_HOPID_SHIFT;

		tb_printf(sc,
		    "DP IN adap %u: video_hop=%u aux_tx_hop=%u "
		    "aux_rx_hop=%u\n", i, video_hopid,
		    aux_tx_hopid, aux_rx_hopid);

		if (video_hopid == 0) {
			tb_debug(sc, DBG_ROUTER,
			    "DP IN adap %u: no video HopID\n", i);
			continue;
		}

		tn = malloc(sizeof(*tn), M_THUNDERBOLT, M_NOWAIT | M_ZERO);
		if (tn == NULL)
			continue;

		tn->type = TB_TUNNEL_DP;
		tn->src_rsc = sc;
		tn->src_adap = i;
		tn->npaths = 0;

		/* Discover video path */
		error = tb_path_discover(sc, i, video_hopid,
		    &tn->paths[0]);
		if (error == 0 && tn->paths[0].nhops > 0) {
			tn->npaths++;
			tn->dst_adap = tn->paths[0].hops[
			    tn->paths[0].nhops - 1].out_port;
			tn->dst_rsc = tn->paths[0].hops[
			    tn->paths[0].nhops - 1].rsc;
		}

		/* Discover AUX TX path */
		if (aux_tx_hopid != 0) {
			error = tb_path_discover(sc, i, aux_tx_hopid,
			    &tn->paths[1]);
			if (error == 0 && tn->paths[1].nhops > 0)
				tn->npaths++;
		}

		/* Discover AUX RX path (reverse direction) */
		if (aux_rx_hopid != 0 && tn->dst_adap != 0) {
			error = tb_path_discover(sc, tn->dst_adap,
			    aux_rx_hopid, &tn->paths[2]);
			if (error == 0 && tn->paths[2].nhops > 0)
				tn->npaths++;
		}

		if (tn->npaths > 0) {
			tb_printf(sc,
			    "Discovered DP tunnel: adap %u -> adap %u "
			    "(%d paths)\n", tn->src_adap, tn->dst_adap,
			    tn->npaths);
			SLIST_INSERT_HEAD(&sc->tunnel_list, tn, link);
			found++;
		} else {
			tb_debug(sc, DBG_ROUTER,
			    "DP IN adap %u: no active paths\n", i);
			free(tn, M_THUNDERBOLT);
		}
	}

	return (found);
}

static void
router_clear_cmd_context(struct router_command *cmd)
{

	if (cmd != NULL && cmd->nhicmd != NULL)
		cmd->nhicmd->context = NULL;
}

static void
router_abandon_cmd_locked(struct router_softc *sc, struct router_command *cmd)
{

	KASSERT(mtx_owned(&sc->mtx),
	    ("router_abandon_cmd_locked without sc->mtx"));

	if (cmd == NULL)
		return;

	router_clear_cmd_context(cmd);
	if (sc->inflight_cmd == cmd)
		sc->inflight_cmd = NULL;
}

static void
router_get_config_cb(struct router_softc *sc, struct router_command *cmd,
    void *arg)
{
	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "router_get_config_cb called\n");

	/*
	 * Only do the copy if the command didn't have a notify event thrown.
	 * These events serve as asynchronous exception signals, which is
	 * cumbersome.
	 */
	if (cmd->ev == 0 && cmd->callback_arg != NULL && cmd->dwlen != 0)
		bcopy((uint8_t *)cmd->resp_buffer,
		    (uint8_t *)cmd->callback_arg, cmd->dwlen * 4);

	mtx_lock(&sc->mtx);
	router_abandon_cmd_locked(sc, cmd);

	if ((cmd->flags & RCMD_POLLED) == 0)
		wakeup(cmd);
	else
		cmd->flags |= RCMD_POLL_COMPLETE;

	router_schedule_locked(sc, NULL);
	mtx_unlock(&sc->mtx);
}

int
tb_config_read(struct router_softc *sc, u_int space, u_int adapter,
    u_int offset, u_int dwlen, uint32_t *buf)
{
	struct router_command *cmd;
	int error, retries;

	if ((error = _tb_config_read(sc, space, adapter, offset, dwlen, buf,
	    router_get_config_cb, &cmd)) != 0)
		return (error);

	retries = cmd->retries;
	mtx_lock(&sc->mtx);
	while (retries-- >= 0) {
		error = router_schedule_locked(sc, cmd);
		if (error)
			break;

		error = msleep(cmd, &sc->mtx, 0, "tbtcfg", cmd->timeout * hz);
		if (error != EWOULDBLOCK)
			break;
		router_abandon_cmd_locked(sc, cmd);
		tb_debug(sc, DBG_ROUTER, "Config command timed out, retries=%d\n", retries);
	}

	if (cmd->ev != 0)
		error = EINVAL;
	router_free_cmd(sc, cmd);
	mtx_unlock(&sc->mtx);
	return (error);
}

int
tb_config_read_polled(struct router_softc *sc, u_int space, u_int adapter,
    u_int offset, u_int dwlen, uint32_t *buf)
{
	struct router_command *cmd;
	int error, retries, timeout;

	if ((error = _tb_config_read(sc, space, adapter, offset, dwlen, buf,
	    router_get_config_cb, &cmd)) != 0)
		return (error);

	retries = cmd->retries;
	cmd->flags |= RCMD_POLLED;
	timeout = cmd->timeout * 1000000;

	mtx_lock(&sc->mtx);
	while (retries-- >= 0) {
		error = router_schedule_locked(sc, cmd);
		if (error)
			break;
		mtx_unlock(&sc->mtx);

		while (timeout > 0) {
			DELAY(100 * 1000);
			if ((cmd->flags & RCMD_POLL_COMPLETE) != 0)
				break;
			timeout -= 100000;
		}

		mtx_lock(&sc->mtx);
		if ((cmd->flags & RCMD_POLL_COMPLETE) == 0) {
			error = ETIMEDOUT;
			router_abandon_cmd_locked(sc, cmd);
			tb_debug(sc, DBG_ROUTER, "Config command timed out, retries=%d\n", retries);
			continue;
		} else
			break;
	}

	if (cmd->ev != 0)
		error = EINVAL;
	router_free_cmd(sc, cmd);
	mtx_unlock(&sc->mtx);
	return (error);
}

int
tb_config_read_async(struct router_softc *sc, u_int space, u_int adapter,
    u_int offset, u_int dwlen, uint32_t *buf, void *cb)
{
	struct router_command *cmd;
	int error;

	if ((error = _tb_config_read(sc, space, adapter, offset, dwlen, buf,
	    cb, &cmd)) != 0)
		return (error);

	error = router_schedule(sc, cmd);

	return (error);
}

static int
_tb_config_read(struct router_softc *sc, u_int space, u_int adapter,
    u_int offset, u_int dwlen, uint32_t *buf, void *cb,
    struct router_command **rcmd)
{
	struct router_command *cmd;
	struct tb_cfg_read *msg;
	int error;

	if ((error = router_alloc_cmd(sc, &cmd)) != 0)
		return (error);

	msg = router_get_frame_data(cmd);
	bzero(msg, sizeof(*msg));
	msg->route.hi = sc->route.hi;
	msg->route.lo = sc->route.lo;
	msg->addr_attrs = TB_CONFIG_ADDR(0, space, adapter, dwlen, offset);
	cmd->callback = cb;
	cmd->callback_arg = buf;
	cmd->dwlen = dwlen;
	router_prepare_read(sc, cmd, sizeof(*msg));

	if (rcmd != NULL)
		*rcmd = cmd;

	return (0);
}

static int
_tb_config_write(struct router_softc *sc, u_int space, u_int adapter,
    u_int offset, u_int dwlen, uint32_t *buf, void *cb,
    struct router_command **rcmd)
{
	struct router_command *cmd;
	struct tb_cfg_write *msg;
	size_t len;
	int error;

	if (dwlen > TBCFG_MAX_DWORDS)
		return (EINVAL);
	if (dwlen != 0 && buf == NULL)
		return (EINVAL);

	if ((error = router_alloc_cmd(sc, &cmd)) != 0)
		return (error);

	len = sizeof(*msg) + (dwlen * sizeof(uint32_t)) + sizeof(uint32_t);
	if (len > NHI_RING0_FRAME_SIZE) {
		router_free_cmd(sc, cmd);
		return (EMSGSIZE);
	}

	msg = router_get_frame_data(cmd);
	bzero(msg, len);
	msg->route.hi = sc->route.hi;
	msg->route.lo = sc->route.lo;
	msg->addr_attrs = TB_CONFIG_ADDR(0, space, adapter, dwlen, offset);
	if (dwlen != 0)
		bcopy(buf, msg->data, dwlen * sizeof(uint32_t));

	cmd->callback = cb;
	cmd->callback_arg = NULL;
	cmd->dwlen = 0;
	router_prepare_write(sc, cmd, len);

	if (rcmd != NULL)
		*rcmd = cmd;

	return (0);
}

int
tb_config_write(struct router_softc *sc, u_int space, u_int adapter,
    u_int offset, u_int dwlen, uint32_t *buf)
{
	struct router_command *cmd;
	int error, retries;

	if ((error = _tb_config_write(sc, space, adapter, offset, dwlen, buf,
	    router_get_config_cb, &cmd)) != 0)
		return (error);

	retries = cmd->retries;
	mtx_lock(&sc->mtx);
	while (retries-- >= 0) {
		error = router_schedule_locked(sc, cmd);
		if (error)
			break;

		error = msleep(cmd, &sc->mtx, 0, "tbtcfgw", cmd->timeout * hz);
		if (error != EWOULDBLOCK)
			break;
		router_abandon_cmd_locked(sc, cmd);
		tb_debug(sc, DBG_ROUTER, "Config write timed out, retries=%d\n",
		    retries);
	}

	if (cmd->ev != 0)
		error = EINVAL;
	router_free_cmd(sc, cmd);
	mtx_unlock(&sc->mtx);
	return (error);
}

static int
router_alloc_cmd(struct router_softc *sc, struct router_command **rcmd)
{
	struct router_command *cmd;

	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "router_alloc_cmd\n");

	cmd = malloc(sizeof(*cmd), M_THUNDERBOLT, M_ZERO|M_NOWAIT);
	if (cmd == NULL) {
		tb_debug(sc, DBG_ROUTER, "Cannot allocate cmd/response\n");
		return (ENOMEM);
	}

	cmd->nhicmd = nhi_alloc_tx_frame(sc->ring0);
	if (cmd->nhicmd == NULL) {
		tb_debug(sc, DBG_ROUTER, "Cannot allocate command frame\n");
		free(cmd, M_THUNDERBOLT);
		return (EBUSY);
	}

	cmd->sc = sc;
	*rcmd = cmd;
	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "Allocated command with index %d\n",
	    cmd->nhicmd->idx);

	return (0);
}

static void
router_free_cmd(struct router_softc *sc, struct router_command *cmd)
{

	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "router_free_cmd\n");

	if (cmd == NULL)
		return;

	if (cmd->nhicmd != NULL) {
		tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "Freeing nhi command %d\n",
		    cmd->nhicmd->idx);
		cmd->nhicmd->context = NULL;
		nhi_free_tx_frame(sc->ring0, cmd->nhicmd);
	}
	free(cmd, M_THUNDERBOLT);

	return;
}

static void
router_prepare_read(struct router_softc *sc, struct router_command *cmd,
    int len)
{
	struct nhi_cmd_frame *nhicmd;

	KASSERT(cmd != NULL, ("cmd cannot be NULL\n"));

	nhicmd = cmd->nhicmd;
	router_finalize_frame(nhicmd, PDF_READ, len);

	nhicmd->timeout = NHI_CMD_TIMEOUT;
	nhicmd->retries = 0;
	nhicmd->resp_buffer = (uint32_t *)cmd->resp_buffer;
	nhicmd->resp_len = (cmd->dwlen + 3) * 4;
	nhicmd->context = cmd;

	cmd->retries = CFG_DEFAULT_RETRIES;
	cmd->timeout = CFG_DEFAULT_TIMEOUT;

	return;
}

static void
router_prepare_write(struct router_softc *sc, struct router_command *cmd,
    int len)
{
	struct nhi_cmd_frame *nhicmd;

	KASSERT(cmd != NULL, ("cmd cannot be NULL\n"));

	nhicmd = cmd->nhicmd;
	router_finalize_frame(nhicmd, PDF_WRITE, len);

	nhicmd->timeout = NHI_CMD_TIMEOUT;
	nhicmd->retries = 0;
	nhicmd->resp_buffer = (uint32_t *)cmd->resp_buffer;
	nhicmd->resp_len = sizeof(struct tb_cfg_write_resp);
	nhicmd->context = cmd;

	cmd->retries = CFG_DEFAULT_RETRIES;
	cmd->timeout = CFG_DEFAULT_TIMEOUT;
}

static int
router_schedule(struct router_softc *sc, struct router_command *cmd)
{
	int error;

	mtx_lock(&sc->mtx);
	error = router_schedule_locked(sc, cmd);
	mtx_unlock(&sc->mtx);

	return(error);
}

static int
router_schedule_locked(struct router_softc *sc, struct router_command *cmd)
{
	struct nhi_cmd_frame *nhicmd;
	int error;

	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "router_schedule\n");
	if (atomic_load_acq_int(&sc->sc_dying))
		return (ESHUTDOWN);

	if (cmd != NULL)
		TAILQ_INSERT_TAIL(&sc->cmd_queue, cmd, link);

	while ((sc->inflight_cmd == NULL) &&
	    ((cmd = TAILQ_FIRST(&sc->cmd_queue)) != NULL)) {

		TAILQ_REMOVE(&sc->cmd_queue, cmd, link);
		nhicmd = cmd->nhicmd;
		nhicmd->context = cmd;	/* re-arm for completion callback */
		tb_debug(sc, DBG_ROUTER|DBG_EXTRA,
		    "Scheduling command with index %d\n", nhicmd->idx);
		sc->inflight_cmd = cmd;
		if ((error = nhi_tx_schedule(sc->ring0, nhicmd)) != 0) {
			tb_debug(sc, DBG_ROUTER, "nhi ring error "
			    "%d\n", error);
			router_abandon_cmd_locked(sc, cmd);
			if (error == EBUSY) {
				TAILQ_INSERT_HEAD(&sc->cmd_queue, cmd, link);
				error = 0;
			}
			break;
		}
	}

	return (error);
}

static void
router_send_hotplug_ack(struct router_softc *sc, tb_route_t route, u_int adap,
    int unplug)
{
	struct nhi_cmd_frame *nhicmd;
	struct tb_cfg_notify *ack;
	size_t len;
	uint32_t event;
	int error;

	nhicmd = nhi_alloc_tx_frame(sc->ring0);
	if (nhicmd == NULL) {
		tb_debug(sc, DBG_ROUTER, "Cannot allocate hotplug ack frame\n");
		return;
	}

	len = sizeof(*ack);
	ack = (struct tb_cfg_notify *)nhicmd->data;
	bzero(ack, len);
	ack->route.hi = route.hi;
	ack->route.lo = route.lo;
	event = TB_CFG_HP_ACK |
	    (adap << TB_CFG_EVENT_ADAPTER_SHIFT) |
	    (unplug ? TB_CFG_PG_UNPLUG : TB_CFG_PG_PLUG);
	ack->event_adap = event;

	router_finalize_frame(nhicmd, PDF_NOTIFY, len);

	nhicmd->timeout = NHI_CMD_TIMEOUT;
	nhicmd->retries = 0;
	nhicmd->resp_len = 0;
	nhicmd->context = NULL;

	error = nhi_tx_schedule(sc->ring0, nhicmd);
	if (error != 0)
		nhi_free_tx_frame(sc->ring0, nhicmd);
}

static void
router_complete_intr(void *context, union nhi_ring_desc *ring,
    struct nhi_cmd_frame *nhicmd)
{
	struct router_softc *sc;
	struct router_command *cmd;

	KASSERT(nhicmd != NULL, ("nhicmd cannot be NULL\n"));

	/* Guard against stale/freed command frames */
	cmd = (struct router_command *)(nhicmd->context);
	if (cmd == NULL)
		return;
	sc = cmd->sc;
	if (sc == NULL)
		return;

	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "router_complete_intr called\n");

	if (atomic_load_acq_16(&nhicmd->flags) & CMD_RESP_COMPLETE) {
		cmd->callback(sc, cmd, cmd->callback_arg);
	}

	return;
}

static void
router_response_intr(void *context, union nhi_ring_desc *ring, struct nhi_cmd_frame *nhicmd)
{
	struct router_softc *sc, *dev;
	struct tb_cfg_read_resp *read;
	struct tb_cfg_write_resp *write;
	struct router_command *cmd;
	tb_route_t route;
	u_int error, i, eof, len;
	uint32_t attrs;

	KASSERT(context != NULL, ("context cannot be NULL\n"));

	sc = (struct router_softc *)context;
	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "router_response_intr called\n");

	eof = ring->rxpost.eof_len >> RX_BUFFER_DESC_EOF_SHIFT;

	if (eof == PDF_WRITE) {
		write = (struct tb_cfg_write_resp *)nhicmd->data;
		route.hi = be32toh(write->route.hi);
		route.lo = be32toh(write->route.lo);
	} else {
		read = (struct tb_cfg_read_resp *)nhicmd->data;
		route.hi = be32toh(read->route.hi);
		route.lo = be32toh(read->route.lo);
		attrs = be32toh(read->addr_attrs);
		len = (attrs & TB_CFG_SIZE_MASK) >> TB_CFG_SIZE_SHIFT;
	}

	/* XXX Is this a problem? */
	if ((route.hi & 0x80000000) == 0)
		tb_debug(sc, DBG_ROUTER, "Invalid route\n");
	route.hi &= ~0x80000000;

	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "Looking up route 0x%08x%08x\n",
	    route.hi, route.lo);

	error = router_lookup_device(sc, route, &dev);
	if (error != 0 || dev == NULL) {
		tb_debug(sc, DBG_ROUTER, "Cannot find device, error= %d\n",
		    error);
		return;
	}

	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "Found device %s route 0x%08x%08x, "
	    "inflight_cmd= %p\n", device_get_nameunit(dev->dev), dev->route.hi,
	    dev->route.lo, dev->inflight_cmd);

	cmd = dev->inflight_cmd;
	if (cmd == NULL) {
		tb_debug(dev, DBG_ROUTER, "Null inflight cmd\n");
		return;
	}

	if (eof == PDF_READ) {
		for (i = 0; i < len; i++)
			cmd->nhicmd->resp_buffer[i] = be32toh(read->data[i]);
	}

	atomic_set_short(&cmd->nhicmd->flags, CMD_RESP_COMPLETE);
	if (atomic_load_acq_16(&cmd->nhicmd->flags) & CMD_REQ_COMPLETE) {
		tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "TX_COMPLETE set\n");
		cmd->callback(dev, cmd, cmd->callback_arg);
	}

	return;
}

static void
router_notify_intr(void *context, union nhi_ring_desc *ring, struct nhi_cmd_frame *nhicmd)
{
	struct router_softc *sc;
	struct router_command *cmd;
	struct tb_cfg_notify event;
	u_int ev, adap;

	KASSERT(context != NULL, ("context cannot be NULL\n"));

	sc = (struct router_softc *)context;
	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "router_notify_intr called\n");

	event.route.hi = be32toh(nhicmd->data[0]);
	event.route.lo = be32toh(nhicmd->data[1]);
	event.event_adap = be32toh(nhicmd->data[2]);

	ev = GET_NOTIFY_EVENT(&event);
	adap = GET_NOTIFY_ADAPTER(&event);

	tb_debug(sc, DBG_ROUTER, "Event route 0x%08x%08x adap %d code %s\n",
	    event.route.hi, event.route.lo, adap,
	    tb_get_string(ev, tb_notify_event));

	switch (ev) {
	case TB_CFG_ERR_CONN:
	case TB_CFG_ERR_LINK:
	case TB_CFG_ERR_ADDR:
	case TB_CFG_ERR_ADP:
	case TB_CFG_ERR_ENUM:
	case TB_CFG_ERR_NUA:
	case TB_CFG_ERR_LEN:
	case TB_CFG_ERR_HEC:
	case TB_CFG_ERR_FC:
	case TB_CFG_ERR_LOCK:
	case TB_CFG_HP_ACK:
	case TB_CFG_DP_BW:
		if (sc->inflight_cmd != NULL) {
			cmd = sc->inflight_cmd;
			cmd->ev = ev;
			cmd->callback(sc, cmd, cmd->callback_arg);
		}
		break;
	case TB_CFG_ERR_PLUG:
		if (sc->inflight_cmd != NULL) {
			cmd = sc->inflight_cmd;
			cmd->ev = ev;
			cmd->callback(sc, cmd, cmd->callback_arg);
		}
		if (sc->nsc->hcm != NULL)
			hcm_router_discover(sc->nsc->hcm);
		break;
	default:
		break;
	}
	return;
}

int
tb_config_next_cap(struct router_softc *sc, struct router_cfg_cap *cap)
{
	union tb_cfg_cap *tbcap;
	uint32_t *buf;
	uint16_t current;
	int error;

	KASSERT(cap != NULL, ("cap cannot be NULL\n"));
	KASSERT(cap->next_cap != 0, ("next_cap cannot be 0\n"));

	buf = malloc(sizeof(*tbcap), M_THUNDERBOLT, M_NOWAIT|M_ZERO);

	current = cap->next_cap;
	error = tb_config_read(sc, cap->space, cap->adap, current, 1, buf);
	if (error)
		return (error);

	tbcap = (union tb_cfg_cap *)buf;
	cap->cap_id = tbcap->hdr.cap_id;
	cap->next_cap = tbcap->hdr.next_cap;
	cap->current_cap = current;

	if ((cap->space != TB_CFG_CS_ROUTER) &&
	    (tbcap->hdr.cap_id != TB_CFG_CAP_VSC)) {
		free(buf, M_THUNDERBOLT);
		return (0);
	}

	tb_config_read(sc, cap->space, cap->adap, current, 2, buf);
	if (error) {
		free(buf, M_THUNDERBOLT);
		return (error);
	}

	cap->vsc_id = tbcap->vsc.vsc_id;
	cap->vsc_len = tbcap->vsc.len;
	if (tbcap->vsc.len == 0) {
		cap->next_cap = tbcap->vsec.vsec_next_cap;
		cap->vsec_len = tbcap->vsec.vsec_len;
	}

	free(buf, M_THUNDERBOLT);
	return (0);
}

int
tb_config_find_cap(struct router_softc *sc, struct router_cfg_cap *cap)
{
	u_int cap_id, vsc_id;
	int error;

	tb_debug(sc, DBG_ROUTER|DBG_EXTRA, "tb_config_find_cap called\n");

	cap_id = cap->cap_id;
	vsc_id = cap->vsc_id;

	cap->cap_id = cap->vsc_id = 0;
	while ((cap->cap_id != cap_id) || (cap->vsc_id != vsc_id)) {
		tb_debug(sc, DBG_ROUTER|DBG_EXTRA,
		    "Looking for cap %d at offset %d\n", cap->cap_id,
		    cap->next_cap);
		if ((cap->next_cap == 0) ||
		    (cap->next_cap > TB_CFG_CAP_OFFSET_MAX))
			return (EINVAL);
		error = tb_config_next_cap(sc, cap);
		if (error)
			break;
	}

	return (0);
}

int
tb_config_find_router_cap(struct router_softc *sc, u_int cap, u_int vsc, u_int *offset)
{
	struct router_cfg_cap rcap;
	struct tb_cfg_router *cfg;
	uint32_t *buf;
	int error;

	buf = malloc(8 * 4, M_THUNDERBOLT, M_NOWAIT|M_ZERO);
	if (buf == NULL)
		return (ENOMEM);

	error = tb_config_router_read(sc, 0, 5, buf);
	if (error != 0) {
		free(buf, M_THUNDERBOLT);
		return (error);
	}

	cfg = (struct tb_cfg_router *)buf;
	rcap.space = TB_CFG_CS_ROUTER;
	rcap.adap = 0;
	rcap.next_cap = GET_ROUTER_CS_NEXT_CAP(cfg);
	rcap.cap_id = cap;
	rcap.vsc_id = vsc;
	error = tb_config_find_cap(sc, &rcap);
	if (error == 0)
		*offset = rcap.current_cap;

	free(buf, M_THUNDERBOLT);
	return (error);
}

int
tb_config_find_router_vsc(struct router_softc *sc, u_int cap, u_int *offset)
{

	return (tb_config_find_router_cap(sc, TB_CFG_CAP_VSC, cap, offset));
}

int
tb_config_find_router_vsec(struct router_softc *sc, u_int cap, u_int *offset)
{

	return (tb_config_find_router_cap(sc, TB_CFG_CAP_VSEC, cap, offset));
}

int
tb_config_find_adapter_cap(struct router_softc *sc, u_int adap, u_int cap, u_int *offset)
{
	struct router_cfg_cap rcap;
	struct tb_cfg_adapter *cfg;
	uint32_t *buf;
	int error;

	buf = malloc(8 * 4, M_THUNDERBOLT, M_NOWAIT|M_ZERO);
	if (buf == NULL)
		return (ENOMEM);

	error = tb_config_adapter_read(sc, adap, 0, 8, buf);
	if (error != 0) {
		free(buf, M_THUNDERBOLT);
		return (error);
	}

	cfg = (struct tb_cfg_adapter *)buf;
	rcap.space = TB_CFG_CS_ADAPTER;
	rcap.adap = adap;
	rcap.next_cap = GET_ADP_CS_NEXT_CAP(cfg);
	rcap.cap_id = cap;
	rcap.vsc_id = 0;
	error = tb_config_find_cap(sc, &rcap);
	if (error == 0)
		*offset = rcap.current_cap;

	free(buf, M_THUNDERBOLT);
	return (error);
}

int
tb_config_get_lc_uuid(struct router_softc *rsc, uint8_t *uuid)
{
	u_int error, offset;
	uint32_t buf[8];

	bzero(buf, sizeof(buf));

	error = tb_config_find_router_vsec(rsc, TB_CFG_VSEC_LC, &offset);
	if (error != 0) {
		tb_debug(rsc, DBG_ROUTER, "Error finding LC registers: %d\n",
		    error);
		return (error);
	}

	error = tb_config_router_read(rsc, offset + TB_LC_UUID, 4, buf);
	if (error != 0) {
		tb_debug(rsc, DBG_ROUTER, "Error fetching UUID: %d\n", error);
		return (error);
	}

	bcopy(buf, uuid, 16);
	return (0);
}
