/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Abdelkader Boudih <freebsd@seuros.com>
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

/*
 * /dev/cbmem — ioctl interface for CBMEM entry enumeration and read
 *
 * Provides structured access to coreboot's CBMEM entries without
 * requiring /dev/mem. Entries can be listed (CBMEM_IOC_LIST) or
 * read by ID (CBMEM_IOC_READ).
 */

#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/ioccom.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/uio.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>

#include <dev/coreboot/coreboot.h>

static d_open_t		coreboot_cbmem_open;
static d_close_t	coreboot_cbmem_close;
static d_ioctl_t	coreboot_cbmem_ioctl;

static struct cdevsw coreboot_cbmem_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	coreboot_cbmem_open,
	.d_close =	coreboot_cbmem_close,
	.d_ioctl =	coreboot_cbmem_ioctl,
	.d_name =	"cbmem",
};

static int
coreboot_cbmem_open(struct cdev *dev, int oflags, int devtype,
    struct thread *td)
{
	struct coreboot_softc *sc;

	sc = dev->si_drv1;
	if (sc == NULL)
		return (ENXIO);

	return (0);
}

static int
coreboot_cbmem_close(struct cdev *dev, int fflag, int devtype,
    struct thread *td)
{
	return (0);
}

/*
 * Find a CBMEM entry by its ID.
 */
static struct cbmem_entry_info *
cbmem_find_entry(struct coreboot_softc *sc, uint32_t id)
{
	uint32_t i;

	for (i = 0; i < sc->cbmem_count; i++) {
		if (sc->cbmem_entries[i].id == id)
			return (&sc->cbmem_entries[i]);
	}
	return (NULL);
}

static int
coreboot_cbmem_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct coreboot_softc *sc;
	int error;

	sc = dev->si_drv1;
	if (sc == NULL)
		return (ENXIO);

	switch (cmd) {
	case CBMEM_IOC_LIST: {
		struct cbmem_list *list = (struct cbmem_list *)data;

		list->count = sc->cbmem_count;
		memcpy(list->entries, sc->cbmem_entries,
		    sc->cbmem_count * sizeof(struct cbmem_entry_info));
		error = 0;
		break;
	}

	case CBMEM_IOC_READ: {
		struct cbmem_read_req *req = (struct cbmem_read_req *)data;
		struct cbmem_entry_info *info;
		void *mapped;
		vm_size_t map_size;
		uint64_t paddr;

		info = cbmem_find_entry(sc, req->id);
		if (info == NULL) {
			error = ENOENT;
			break;
		}

		if (req->offset >= info->size) {
			error = EINVAL;
			break;
		}

		if (req->size > info->size - req->offset)
			req->size = info->size - req->offset;

		if (req->size == 0) {
			error = 0;
			break;
		}

		if (info->address > UINT64_MAX - req->offset) {
			error = EINVAL;
			break;
		}
		paddr = info->address + req->offset;
		map_size = req->size;
		mapped = pmap_mapbios((vm_paddr_t)paddr, map_size);
		if (mapped == NULL) {
			error = ENOMEM;
			break;
		}

		error = copyout(mapped, req->buffer, req->size);

		pmap_unmapbios(mapped, map_size);
		break;
	}

	default:
		error = ENOTTY;
		break;
	}

	return (error);
}

int
coreboot_cbmem_create(struct coreboot_softc *sc)
{
	struct make_dev_args args;
	int error;

	if (sc->cbmem_count == 0)
		return (ENXIO);

	make_dev_args_init(&args);
	args.mda_devsw = &coreboot_cbmem_cdevsw;
	args.mda_uid = UID_ROOT;
	args.mda_gid = GID_WHEEL;
	args.mda_mode = 0440;
	args.mda_si_drv1 = sc;
	error = make_dev_s(&args, &sc->cbmem_cdev, "cbmem");
	if (error != 0)
		return (error);

	return (0);
}

void
coreboot_cbmem_destroy(struct coreboot_softc *sc)
{
	if (sc->cbmem_cdev != NULL) {
		destroy_dev(sc->cbmem_cdev);
		sc->cbmem_cdev = NULL;
	}
}
