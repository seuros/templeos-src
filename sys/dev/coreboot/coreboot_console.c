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
 * /dev/coreboot_console — read-only access to the CBMEM firmware console
 *
 * The CBMEM console is a ring buffer written by coreboot during boot.
 * Bit 31 of the cursor field indicates overflow (the buffer has wrapped).
 * When overflow is set, data starts at cursor and wraps around.
 * When not set, data starts at offset 0 and ends at cursor.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/uio.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>

#include "coreboot.h"

static d_open_t		coreboot_console_open;
static d_close_t	coreboot_console_close;
static d_read_t		coreboot_console_read;

static struct cdevsw coreboot_console_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	coreboot_console_open,
	.d_close =	coreboot_console_close,
	.d_read =	coreboot_console_read,
	.d_name =	"coreboot_console",
};

static int
coreboot_console_open(struct cdev *dev, int oflags, int devtype,
    struct thread *td)
{
	struct coreboot_softc *sc;

	sc = dev->si_drv1;
	if (sc == NULL || sc->console_vaddr == NULL)
		return (ENXIO);

	return (0);
}

static int
coreboot_console_close(struct cdev *dev, int fflag, int devtype,
    struct thread *td)
{
	return (0);
}

/*
 * Read the console ring buffer.
 *
 * The ring buffer has a cursor that may have wrapped. If the OVERFLOW
 * bit (bit 31) is set, the ring has wrapped and data goes from
 * body[cursor..size-1] then body[0..cursor-1]. Otherwise it's just
 * body[0..cursor-1].
 *
 * We linearize the buffer into a contiguous view for the uiomove.
 */
static int
coreboot_console_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct coreboot_softc *sc;
	struct cbmem_console *cons;
	uint32_t cursor, flags, buf_size;
	uint32_t data_start, data_len;
	off_t offset;
	size_t todo;
	int error;

	sc = dev->si_drv1;
	if (sc == NULL || sc->console_vaddr == NULL)
		return (ENXIO);

	cons = sc->console_vaddr;
	cursor = cons->cursor & CBMEM_CONSOLE_CURSOR_MASK;
	flags = cons->cursor & ~CBMEM_CONSOLE_CURSOR_MASK;
	buf_size = sc->console_data_size;

	if (cursor > buf_size)
		cursor = 0;

	if (flags & CBMEM_CONSOLE_OVERFLOW) {
		/*
		 * Ring has wrapped. Logical order:
		 *   segment 1: body[cursor .. buf_size-1]  (oldest data)
		 *   segment 2: body[0 .. cursor-1]         (newest data)
		 * Total length = buf_size
		 */
		data_len = buf_size;
		offset = uio->uio_offset;

		if (offset >= data_len)
			return (0);

		while (uio->uio_resid > 0 && offset < data_len) {
			uint32_t seg_start, seg_len;
			uint32_t seg1_len = buf_size - cursor;

			if (offset < seg1_len) {
				seg_start = cursor + offset;
				seg_len = seg1_len - offset;
			} else {
				seg_start = offset - seg1_len;
				seg_len = cursor - seg_start;
			}

			todo = MIN(uio->uio_resid, seg_len);
			error = uiomove(cons->body + seg_start, todo, uio);
			if (error != 0)
				return (error);
			offset += todo;
		}
	} else {
		/*
		 * No overflow — data is linear: body[0 .. cursor-1]
		 */
		data_start = 0;
		data_len = cursor;
		offset = uio->uio_offset;

		if (offset >= data_len)
			return (0);

		todo = MIN(uio->uio_resid, data_len - offset);
		error = uiomove(cons->body + data_start + offset, todo, uio);
		if (error != 0)
			return (error);
	}

	return (0);
}

int
coreboot_console_create(struct coreboot_softc *sc)
{
	struct cbmem_console *tmp;
	vm_size_t map_size;
	uint32_t cons_size;

	if (!sc->has_console || sc->console_paddr == 0)
		return (ENXIO);

	tmp = (struct cbmem_console *)pmap_mapbios(sc->console_paddr,
	    sizeof(struct cbmem_console));
	if (tmp == NULL) {
		device_printf(sc->dev,
		    "unable to map CBMEM console header\n");
		return (ENOMEM);
	}

	cons_size = tmp->size;
	pmap_unmapbios(tmp, sizeof(struct cbmem_console));
	if (cons_size == 0 || cons_size > CB_MAX_CONSOLE_BYTES) {
		device_printf(sc->dev, "invalid CBMEM console size: %u\n",
		    cons_size);
		return (EINVAL);
	}
	map_size = sizeof(struct cbmem_console) + cons_size;

	sc->console_vaddr = (struct cbmem_console *)pmap_mapbios(
	    sc->console_paddr, map_size);
	if (sc->console_vaddr == NULL) {
		device_printf(sc->dev,
		    "unable to map CBMEM console (%zu bytes)\n",
		    (size_t)map_size);
		return (ENOMEM);
	}
	sc->console_size = map_size;
	sc->console_data_size = cons_size;

	sc->console_cdev = make_dev(&coreboot_console_cdevsw, 0,
	    UID_ROOT, GID_WHEEL, 0440, "coreboot_console");
	if (sc->console_cdev == NULL) {
		pmap_unmapbios(sc->console_vaddr, sc->console_size);
		sc->console_vaddr = NULL;
		sc->console_size = 0;
		sc->console_data_size = 0;
		return (ENOMEM);
	}
	sc->console_cdev->si_drv1 = sc;

	device_printf(sc->dev,
	    "CBMEM console: %u bytes, cursor at %u%s\n",
	    sc->console_vaddr->size,
	    sc->console_vaddr->cursor & CBMEM_CONSOLE_CURSOR_MASK,
	    (sc->console_vaddr->cursor & CBMEM_CONSOLE_OVERFLOW) ?
	    " (wrapped)" : "");

	return (0);
}

void
coreboot_console_destroy(struct coreboot_softc *sc)
{
	if (sc->console_cdev != NULL) {
		destroy_dev(sc->console_cdev);
		sc->console_cdev = NULL;
	}
}
