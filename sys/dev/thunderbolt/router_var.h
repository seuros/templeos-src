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
 *
 * $FreeBSD$
 */

#ifndef _ROUTER_VAR_H
#define _ROUTER_VAR_H

struct router_softc;
struct router_command;
struct router_topo;
struct tb_tunnel;
struct router_hotplug_task;

/*
 * A single hop in a tunnel path.
 */
struct tb_path_hop {
	struct router_softc	*rsc;		/* switch containing this hop */
	u_int			in_port;	/* ingress adapter */
	u_int			in_hopid;	/* ingress HopID */
	u_int			out_port;	/* egress adapter (from hop reg) */
	u_int			next_hopid;	/* HopID on next switch ingress */
	struct tb_cfg_hop	hop;		/* raw hop register contents */
};

#define TB_PATH_MAX_HOPS	16

struct tb_path {
	struct tb_path_hop	hops[TB_PATH_MAX_HOPS];
	int			nhops;
	int			activated;	/* non-zero if path is live */
};

enum tb_tunnel_type {
	TB_TUNNEL_DP = 0,
	TB_TUNNEL_PCIE,
	TB_TUNNEL_USB3,
};

/*
 * A discovered tunnel.  DP tunnels have 3 paths (video, aux_tx, aux_rx).
 * PCIe tunnels have 2 paths (down, up).
 */
#define TB_TUNNEL_MAX_PATHS	3

struct tb_tunnel {
	SLIST_ENTRY(tb_tunnel)	link;
	enum tb_tunnel_type	type;
	struct router_softc	*src_rsc;	/* source router */
	struct router_softc	*dst_rsc;	/* destination router */
	u_int			src_adap;	/* source adapter number */
	u_int			dst_adap;	/* destination adapter number */
	struct tb_path		paths[TB_TUNNEL_MAX_PATHS];
	int			npaths;
};

typedef void (*router_callback_t)(struct router_softc *,
    struct router_command *, void *);

struct router_command {
	TAILQ_ENTRY(router_command) link;
	struct router_softc	*sc;
	struct nhi_cmd_frame	*nhicmd;
	u_int			flags;
#define RCMD_POLLED		(1 << 0)
#define RCMD_POLL_COMPLETE	(1 << 1)
	int			resp_len;
	router_callback_t	callback;
	void			*callback_arg;
	u_int			dwlen;
	u_int			timeout;
	int			retries;
	u_int			ev;
	uint8_t			resp_buffer[NHI_RING0_FRAME_SIZE];
};

struct router_softc {
	TAILQ_ENTRY(router_softc) link;
	u_int			debug;
	tb_route_t		route;
	device_t		dev;
	struct nhi_softc	*nsc;
	struct router_softc	*parent;

	struct mtx		mtx;
	struct nhi_ring_pair	*ring0;
	TAILQ_HEAD(,router_command) cmd_queue;
	TAILQ_HEAD(, router_hotplug_task) hotplug_tasks;

	struct router_command	*inflight_cmd;

	uint8_t			depth;
	uint8_t			max_adap;

	struct router_softc	**adapters;

	uint32_t		uuid[4];
	volatile int		sc_dying;	/* set during detach */

	SLIST_HEAD(, tb_tunnel)	tunnel_list;	/* discovered tunnels */
};

struct router_cfg_cap {
	uint16_t	current_cap;
	uint16_t	next_cap;
	uint32_t	space;
	uint8_t		adap;
	uint8_t		cap_id;
	uint8_t		vsc_id;
	uint8_t		vsc_len;
	uint16_t	vsec_len;
};

int tb_router_attach(struct router_softc *, tb_route_t);
int tb_router_attach_root(struct nhi_softc *, tb_route_t);
int tb_router_detach(struct router_softc *);
int tb_config_read(struct router_softc *, u_int, u_int, u_int, u_int,
    uint32_t *);
int tb_config_read_polled(struct router_softc *, u_int, u_int, u_int, u_int,
    uint32_t *);
int tb_config_read_async(struct router_softc *, u_int, u_int, u_int, u_int,
    uint32_t *, void *);
int tb_config_write(struct router_softc *, u_int, u_int, u_int, u_int,
    uint32_t *);
int tb_config_next_cap(struct router_softc *, struct router_cfg_cap *);
int tb_config_find_cap(struct router_softc *, struct router_cfg_cap *);
int tb_config_find_router_cap(struct router_softc *, u_int, u_int, u_int *);
int tb_config_find_router_vsc(struct router_softc *, u_int, u_int *);
int tb_config_find_router_vsec(struct router_softc *, u_int, u_int *);
int tb_config_find_adapter_cap(struct router_softc *, u_int, u_int, u_int *);
int tb_config_get_lc_uuid(struct router_softc *, uint8_t *);
int tb_tunnel_discover_dp(struct router_softc *);
void tb_tunnel_free_all(struct router_softc *);

#define TB_CONFIG_ADDR(seq, space, adapter, dwlen, offset) \
    ((seq << TB_CFG_SEQ_SHIFT) | space | \
    (adapter << TB_CFG_ADAPTER_SHIFT) | (dwlen << TB_CFG_SIZE_SHIFT) | \
    (offset & TB_CFG_ADDR_MASK))

#define TB_ROUTE(router) \
	    ((uint64_t)(router)->route.hi << 32) | (router)->route.lo

static __inline tb_route_t
TB_CHILD_ROUTE(struct router_softc *router, u_int adap)
{
	tb_route_t route;
	uint64_t child;

	child = TB_ROUTE(router) | ((uint64_t)adap << (router->depth * 8));
	route.hi = child >> 32;
	route.lo = child & 0xffffffffU;

	return (route);
}

static __inline void *
router_get_frame_data(struct router_command *cmd)
{
	return ((void *)cmd->nhicmd->data);
}

/*
 * Read the Router config space for the router referred to in the softc.
 * addr - The dword offset in the config space
 * dwlen - The number of dwords
 * buf - must be large enough to hold the number of dwords requested.
 */
static __inline int
tb_config_router_read(struct router_softc *sc, u_int addr, u_int dwlen,
    uint32_t *buf)
{
	return (tb_config_read(sc, TB_CFG_CS_ROUTER, 0, addr, dwlen, buf));
}

static __inline int
tb_config_router_read_polled(struct router_softc *sc, u_int addr, u_int dwlen,
    uint32_t *buf)
{
	return (tb_config_read_polled(sc, TB_CFG_CS_ROUTER, 0, addr, dwlen, buf));
}

/*
 * Write the Router config space for the router referred to in the softc.
 * addr - The dword offset in the config space
 * dwlen - The number of dwords
 * buf - must be large enough to hold the number of dwords requested.
 */
static __inline int
tb_config_router_write(struct router_softc *sc, u_int addr, u_int dwlen,
    uint32_t *buf)
{
	return (tb_config_write(sc, TB_CFG_CS_ROUTER, 0, addr, dwlen, buf));
}

/*
 * Read the Adapter config space for the router referred to in the softc.
 * adap - Adapter number
 * addr - The dword offset in the config space
 * dwlen - The number of dwords
 * buf - must be large enough to hold the number of dwords requested.
 */
static __inline int
tb_config_adapter_read(struct router_softc *sc, u_int adap, u_int addr,
    u_int dwlen, uint32_t *buf)
{
	return (tb_config_read(sc, TB_CFG_CS_ADAPTER, adap, addr, dwlen, buf));
}

/*
 * Read the Adapter config space for the router referred to in the softc.
 * adap - Adapter number
 * addr - The dword offset in the config space
 * dwlen - The number of dwords
 * buf - must be large enough to hold the number of dwords requested.
 */
static __inline int
tb_config_adapter_write(struct router_softc *sc, u_int adap, u_int addr,
    u_int dwlen, uint32_t *buf)
{
	return (tb_config_write(sc, TB_CFG_CS_ADAPTER, adap, addr, dwlen, buf));
}

/*
 * Read the Path config space for the router referred to in the softc.
 * adap - Adapter number
 * hopid - HopID of the path
 * len - The number of adjacent paths
 * buf - must be large enough to hold the number of dwords requested.
 */
static __inline int
tb_config_path_read(struct router_softc *sc, u_int adap, u_int hopid,
    u_int num, uint32_t *buf)
{
	return (tb_config_read(sc, TB_CFG_CS_PATH, adap, hopid * 2,
	    num * 2, buf));
}

/*
 * Write the Path config space for the router referred to in the softc.
 * adap - Adapter number
 * hopid - HopID of the path
 * len - The number of adjacent paths
 * buf - must be large enough to hold the number of dwords requested.
 */
static __inline int
tb_config_path_write(struct router_softc *sc, u_int adap, u_int hopid,
    u_int num, uint32_t *buf)
{
	return (tb_config_write(sc, TB_CFG_CS_PATH, adap, hopid * 2,
	    num * 2, buf));
}

/*
 * Read the Counters config space for the router referred to in the softc.
 * Counters come in sets of 3 dwords.
 * adap - Adapter number
 * set - The counter set index
 * num - The number of adjacent counter sets to read
 * buf - must be large enough to hold the number of dwords requested.
 */
static __inline int
tb_config_counters_read(struct router_softc *sc, u_int adap, u_int set,
    u_int num, uint32_t *buf)
{
	return (tb_config_read(sc, TB_CFG_CS_COUNTERS, adap, set * 3,
	    num * 3, buf));
}

static __inline void
tb_config_set_root(struct router_softc *sc)
{
	sc->nsc->root_rsc = sc;
}

static __inline void *
tb_config_get_root(struct router_softc *sc)
{
	return (sc->nsc->root_rsc);
}

#endif /* _ROUTER_VAR_H */
