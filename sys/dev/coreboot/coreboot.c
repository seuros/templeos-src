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
 * coreboot(4) - FreeBSD driver for coreboot firmware tables
 *
 * Discovers the coreboot table by scanning low memory for the "LBIO"
 * signature, follows CB_TAG_FORWARD to the high-memory table, and
 * exposes firmware information through sysctl(9) and character devices.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/sysctl.h>
#include <sys/rman.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "coreboot.h"

static struct coreboot_softc *coreboot_sc;

/*
 * Debug verbosity control, non-zero enables extra output.
 * Tunable via loader.conf: hw.coreboot.debug=1
 * Runtime: sysctl hw.coreboot.debug=1
 * Registered dynamically under hw.coreboot in
 * coreboot_register_sysctls().
 */
static int coreboot_debug = 0;
TUNABLE_INT("hw.coreboot.debug", &coreboot_debug);

struct coreboot_softc *
coreboot_get_softc(void)
{
	return (coreboot_sc);
}

static void	coreboot_identify(driver_t *, device_t);
static int	coreboot_probe(device_t);
static int	coreboot_attach(device_t);
static int	coreboot_detach(device_t);
static int	coreboot_modevent(module_t, int, void *);

/*
 * Scan a physical memory region for the "LBIO" signature.
 * Returns the physical address of the header, or 0 if not found.
 */
static vm_paddr_t
coreboot_scan_region(vm_paddr_t start, vm_paddr_t end)
{
	vm_paddr_t addr;
	void *va;
	struct cb_header *hdr;

	for (addr = start; addr < end; addr += CB_SCAN_LOW_STEP) {
		va = pmap_mapbios(addr, sizeof(struct cb_header));
		if (va == NULL)
			continue;

		hdr = (struct cb_header *)va;
		if (memcmp(hdr->signature, CB_HEADER_SIGNATURE,
		    CB_HEADER_SIG_LEN) == 0) {
			pmap_unmapbios(va, sizeof(struct cb_header));
			return (addr);
		}
		pmap_unmapbios(va, sizeof(struct cb_header));
	}
	return (0);
}

/*
 * Validate length fields in the header before using them for mappings
 * and pointer arithmetic.
 */
static int
coreboot_sanitize_header(const struct cb_header *hdr, vm_size_t *map_size)
{
	uint64_t total;

	if (hdr->header_bytes < sizeof(*hdr) ||
	    hdr->header_bytes > CB_MAX_HEADER_BYTES)
		return (EINVAL);
	if ((hdr->header_bytes % CB_TABLE_ALIGN) != 0)
		return (EINVAL);
	if (hdr->table_bytes > CB_MAX_TABLE_BYTES)
		return (EINVAL);
	if ((hdr->table_bytes % CB_TABLE_ALIGN) != 0)
		return (EINVAL);

	total = (uint64_t)hdr->header_bytes + (uint64_t)hdr->table_bytes;
	if (total > CB_MAX_TABLE_MAP_BYTES)
		return (EINVAL);

	*map_size = (vm_size_t)total;
	return (0);
}

/*
 * Validate the coreboot header checksum.
 * Returns 0 on success, non-zero on failure.
 */
static int
coreboot_validate_header(struct cb_header *hdr, vm_size_t mapped_len)
{
	uint16_t cksum;

	if (hdr->header_bytes > mapped_len)
		return (EINVAL);

	cksum = cb_checksum(hdr, hdr->header_bytes);
	if (cksum != 0)
		return (EINVAL);

	return (0);
}

/*
 * Validate checksum for the table payload.
 */
static int
coreboot_validate_table(struct cb_header *hdr, vm_size_t mapped_len)
{
	const uint8_t *table;
	uint16_t cksum;

	if (hdr->table_bytes == 0)
		return (0);

	if ((uint64_t)hdr->header_bytes + (uint64_t)hdr->table_bytes >
	    mapped_len)
		return (EINVAL);
	if (hdr->table_checksum > UINT16_MAX)
		return (EINVAL);

	table = (const uint8_t *)hdr + hdr->header_bytes;
	cksum = cb_checksum(table, hdr->table_bytes);
	if (cksum != (uint16_t)hdr->table_checksum)
		return (EINVAL);

	return (0);
}

static void
coreboot_copy_bounded_string(const char *src, size_t maxlen, char *dst,
    size_t dstlen)
{
	size_t slen;

	if (dstlen == 0)
		return;

	slen = strnlen(src, maxlen);
	if (slen >= dstlen)
		slen = dstlen - 1;
	memcpy(dst, src, slen);
	dst[slen] = '\0';
}

/*
 * Copy a coreboot string record into a destination buffer.
 */
static void
coreboot_copy_string(const struct cb_string *rec, char *dst, size_t dstlen)
{
	size_t slen;

	slen = rec->size - sizeof(struct cb_record);
	if (slen >= dstlen)
		slen = dstlen - 1;
	memcpy(dst, rec->string, slen);
	dst[slen] = '\0';

	/* Strip trailing whitespace/nulls */
	while (slen > 0 && (dst[slen - 1] == '\0' || dst[slen - 1] == ' ' ||
	    dst[slen - 1] == '\n'))
		dst[--slen] = '\0';
}

/*
 * Extract mainboard vendor and part number from the strings field.
 */
static void
coreboot_parse_mainboard(struct coreboot_softc *sc,
    const struct cb_mainboard *mb)
{
	const char *strings = (const char *)mb->strings;
	size_t total = mb->size - offsetof(struct cb_mainboard, strings);
	uint8_t vendor_off, part_off;

	vendor_off = mb->vendor_idx;
	part_off = mb->part_idx;

	if (vendor_off < total)
		coreboot_copy_bounded_string(strings + vendor_off,
		    total - vendor_off, sc->mb_vendor, sizeof(sc->mb_vendor));
	if (part_off < total)
		coreboot_copy_bounded_string(strings + part_off,
		    total - part_off, sc->mb_part, sizeof(sc->mb_part));
}

/*
 * Parse all records in the coreboot table and populate softc.
 */
static void
coreboot_parse_table(struct coreboot_softc *sc, struct cb_header *hdr)
{
	uint8_t *entry;
	uint8_t *table_end;
	struct cb_record *rec;

	entry = (uint8_t *)hdr + hdr->header_bytes;
	table_end = entry + hdr->table_bytes;

	while ((size_t)(table_end - entry) >= sizeof(struct cb_record)) {
		size_t rec_size;

		rec = (struct cb_record *)entry;
		rec_size = rec->size;

		if (rec_size < sizeof(struct cb_record))
			break;
		if (rec_size > (size_t)(table_end - entry))
			break;

		switch (rec->tag) {
		case CB_TAG_VERSION:
			coreboot_copy_string((struct cb_string *)rec,
			    sc->version, sizeof(sc->version));
			break;

		case CB_TAG_EXTRA_VERSION:
			coreboot_copy_string((struct cb_string *)rec,
			    sc->extra_version, sizeof(sc->extra_version));
			break;

		case CB_TAG_BUILD:
			coreboot_copy_string((struct cb_string *)rec,
			    sc->build, sizeof(sc->build));
			break;

		case CB_TAG_COMPILE_TIME:
			coreboot_copy_string((struct cb_string *)rec,
			    sc->compile_time, sizeof(sc->compile_time));
			break;

		case CB_TAG_COMPILER:
			coreboot_copy_string((struct cb_string *)rec,
			    sc->compiler, sizeof(sc->compiler));
			break;

		case CB_TAG_PLATFORM_BLOB_VERSION:
			coreboot_copy_string((struct cb_string *)rec,
			    sc->platform_blob_version,
			    sizeof(sc->platform_blob_version));
			break;

		case CB_TAG_SERIALNO:
			coreboot_copy_string((struct cb_string *)rec,
			    sc->serialno, sizeof(sc->serialno));
			break;

		case CB_TAG_VERSION_TIMESTAMP: {
			struct cb_version_timestamp *ts =
			    (struct cb_version_timestamp *)rec;

			if (rec_size < sizeof(*ts))
				break;
			sc->version_timestamp = ts->timestamp;
			sc->has_version_timestamp = 1;
			break;
		}

		case CB_TAG_MAINBOARD:
			if (rec_size < offsetof(struct cb_mainboard, strings))
				break;
			coreboot_parse_mainboard(sc,
			    (struct cb_mainboard *)rec);
			break;

		case CB_TAG_SERIAL: {
			struct cb_serial *ser = (struct cb_serial *)rec;

			if (rec_size < sizeof(*ser))
				break;
			sc->serial_baseaddr = ser->baseaddr;
			sc->serial_baud = ser->baud;
			sc->serial_regwidth = ser->regwidth;
			sc->has_serial = 1;
			break;
		}

		case CB_TAG_TSC_INFO: {
			struct cb_tsc_info *tsc = (struct cb_tsc_info *)rec;

			if (rec_size < sizeof(*tsc))
				break;
			sc->tsc_freq_khz = tsc->freq_khz;
			sc->has_tsc_info = 1;
			break;
		}

		case CB_TAG_PCIE: {
			struct cb_pcie *pcie = (struct cb_pcie *)rec;

			if (rec_size < sizeof(*pcie))
				break;
			sc->pcie_ctrl_base = pcie->ctrl_base;
			sc->has_pcie = 1;
			break;
		}

		case CB_TAG_BOOT_MEDIA_PARAMS: {
			struct cb_boot_media_params *bmp =
			    (struct cb_boot_media_params *)rec;

			if (rec_size < sizeof(*bmp))
				break;
			sc->fmap_offset = bmp->fmap_offset;
			sc->cbfs_offset = bmp->cbfs_offset;
			sc->cbfs_size = bmp->cbfs_size;
			sc->boot_media_size = bmp->boot_media_size;
			sc->has_boot_media = 1;
			break;
		}

		case CB_TAG_MMC_INFO: {
			struct cb_mmc_info *mmc = (struct cb_mmc_info *)rec;

			if (rec_size < sizeof(*mmc))
				break;
			sc->mmc_early_cmd1_status = mmc->early_cmd1_status;
			sc->has_mmc_info = 1;
			break;
		}

		case CB_TAG_CBMEM_CONSOLE: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->console_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_console = 1;
			break;
		}

		case CB_TAG_CBMEM_ENTRY: {
			struct cb_cbmem_entry *ent =
			    (struct cb_cbmem_entry *)rec;

			if (rec_size < sizeof(*ent))
				break;
			if (sc->cbmem_count < CB_MAX_CBMEM_ENTRIES) {
				struct cbmem_entry_info *info =
				    &sc->cbmem_entries[sc->cbmem_count];
				info->id = ent->id;
				info->address = ent->address;
				info->size = ent->entry_size;
				strlcpy(info->name, cbmem_id_to_name(ent->id),
				    sizeof(info->name));
				sc->cbmem_count++;
			}
			break;
		}

		case CB_TAG_BOARD_CONFIG: {
			struct cb_board_config *bc =
			    (struct cb_board_config *)rec;

			if (rec_size < sizeof(*bc))
				break;
			sc->fw_config = bc->fw_config;
			sc->board_id = bc->board_id;
			sc->ram_code = bc->ram_code;
			sc->sku_id = bc->sku_id;
			sc->has_board_config = 1;
			break;
		}

		case CB_TAG_MAC_ADDRS: {
			struct cb_macs *macs = (struct cb_macs *)rec;
			uint32_t i, count;

			if (rec_size < sizeof(*macs))
				break;
			count = macs->count;
			if (count > CB_MAX_MAC_ADDRS)
				count = CB_MAX_MAC_ADDRS;
			if (rec_size < sizeof(*macs) +
			    count * sizeof(struct cb_mac_address))
				break;
			for (i = 0; i < count; i++)
				sc->macs[i] = macs->entries[i];
			sc->mac_count = count;
			break;
		}

		case CB_TAG_ACPI_RSDP: {
			struct cb_acpi_rsdp *rsdp =
			    (struct cb_acpi_rsdp *)rec;

			if (rec_size < sizeof(*rsdp))
				break;
			sc->acpi_rsdp = rsdp->rsdp_pointer;
			sc->has_acpi_rsdp = 1;
			break;
		}

		case CB_TAG_SPI_FLASH: {
			struct cb_spi_flash *spi =
			    (struct cb_spi_flash *)rec;

			if (rec_size < sizeof(*spi))
				break;
			sc->spi_flash_size = spi->flash_size;
			sc->spi_sector_size = spi->sector_size;
			sc->spi_erase_cmd = spi->erase_cmd;
			sc->spi_flags = spi->flags;
			sc->has_spi_flash = 1;
			break;
		}

		case CB_TAG_CONSOLE: {
			struct cb_console *con = (struct cb_console *)rec;

			if (rec_size < sizeof(*con))
				break;
			sc->console_type = con->type;
			sc->has_console_type = 1;
			break;
		}

		case CB_TAG_FRAMEBUFFER: {
			struct cb_framebuffer *fb =
			    (struct cb_framebuffer *)rec;

			if (rec_size < CB_FRAMEBUFFER_MIN_SIZE)
				break;
			sc->fb_addr = fb->physical_address;
			sc->fb_x_res = fb->x_resolution;
			sc->fb_y_res = fb->y_resolution;
			sc->fb_stride = fb->bytes_per_line;
			sc->fb_bpp = fb->bits_per_pixel;
			sc->has_framebuffer = 1;
			break;
		}

		case CB_TAG_GPIO: {
			struct cb_gpios *gpios = (struct cb_gpios *)rec;
			uint32_t i, count;

			if (rec_size < sizeof(*gpios))
				break;
			count = gpios->count;
			if (count > CB_MAX_GPIOS)
				count = CB_MAX_GPIOS;
			if (rec_size < sizeof(*gpios) +
			    count * sizeof(struct cb_gpio))
				break;
			for (i = 0; i < count; i++)
				sc->gpios[i] = gpios->entries[i];
			sc->gpio_count = count;
			break;
		}

		case CB_TAG_TPM_PPI_HANDOFF: {
			struct cb_tpm_ppi *tpm = (struct cb_tpm_ppi *)rec;

			if (rec_size < sizeof(*tpm))
				break;
			sc->tpm_ppi_addr = tpm->ppi_address;
			sc->tpm_version = tpm->tpm_version;
			sc->has_tpm = 1;
			break;
		}

		case CB_TAG_TIMESTAMPS: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->timestamps_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_timestamps = 1;
			break;
		}

		case CB_TAG_ACPI_GNVS: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->acpi_gnvs_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_acpi_gnvs = 1;
			break;
		}

		case CB_TAG_ACPI_CNVS: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->acpi_cnvs_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_acpi_cnvs = 1;
			break;
		}

		case CB_TAG_VPD: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->vpd_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_vpd = 1;
			break;
		}

		case CB_TAG_WIFI_CALIBRATION: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->wifi_cal_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_wifi_cal = 1;
			break;
		}

		case CB_TAG_FMAP: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->fmap_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_fmap = 1;
			break;
		}

		case CB_TAG_VBOOT_WORKBUF: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->vboot_workbuf_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_vboot_workbuf = 1;
			break;
		}

		case CB_TAG_TYPE_C_INFO: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->type_c_info_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_type_c_info = 1;
			break;
		}

		case CB_TAG_ROOT_BRIDGE_INFO: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->root_bridge_info_paddr =
			    (vm_paddr_t)ref->cbmem_addr;
			sc->has_root_bridge_info = 1;
			break;
		}

		case CB_TAG_TPM_CB_LOG: {
			struct cb_cbmem_ref *ref = (struct cb_cbmem_ref *)rec;

			if (rec_size < sizeof(*ref))
				break;
			sc->tpm_log_paddr = (vm_paddr_t)ref->cbmem_addr;
			sc->has_tpm_log = 1;
			break;
		}

		case CB_TAG_SMMSTOREV2: {
			struct cb_smmstorev2 *smm =
			    (struct cb_smmstorev2 *)rec;

			if (rec_size < CB_SMMSTOREV2_BASE_SIZE)
				break;
			sc->smmstore_num_blocks = smm->num_blocks;
			sc->smmstore_block_size = smm->block_size;
			sc->smmstore_com_buffer = smm->com_buffer;
			sc->smmstore_apm_cmd = smm->apm_cmd;
			/* 64-bit mmap_addr only present in newer coreboot */
			if (rec_size >= sizeof(*smm))
				sc->smmstore_mmap_addr = smm->mmap_addr;
			else
				sc->smmstore_mmap_addr =
				    (uint64_t)smm->mmap_addr_lo;
			sc->has_smmstore = 1;
			break;
		}

		default:
			break;
		}

		entry += rec_size;
	}
}

/*
 * Register the sysctl tree under hw.coreboot.*
 */
static void
coreboot_register_sysctls(struct coreboot_softc *sc)
{
	struct sysctl_oid *oid_mb, *oid_serial, *oid_cbmem;
	struct sysctl_oid *oid_entry;
	struct sysctl_oid *oid_board, *oid_spi, *oid_fb, *oid_gpio;
	struct sysctl_oid *oid_boot, *oid_cbrefs;
	struct sysctl_oid *oid_tpm, *oid_smmstore;
	char numstr[8];
	uint32_t i;

	sysctl_ctx_init(&sc->sysctl_ctx);
	sc->sysctl_tree = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
	    SYSCTL_STATIC_CHILDREN(_hw), OID_AUTO, "coreboot",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, 0, "coreboot firmware information");

	if (sc->sysctl_tree == NULL)
		return;

	SYSCTL_ADD_INT(&sc->sysctl_ctx,
	    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "debug",
	    CTLFLAG_RW, &coreboot_debug, 0,
	    "Enable verbose coreboot diagnostics");

	if (sc->version[0] != '\0')
		SYSCTL_ADD_STRING(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "version",
		    CTLFLAG_RD, sc->version, 0, "Firmware version");

	if (sc->build[0] != '\0')
		SYSCTL_ADD_STRING(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "build",
		    CTLFLAG_RD, sc->build, 0, "Build date");

	if (sc->compile_time[0] != '\0')
		SYSCTL_ADD_STRING(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "compile_time",
		    CTLFLAG_RD, sc->compile_time, 0, "Firmware compile time");

	if (sc->compiler[0] != '\0')
		SYSCTL_ADD_STRING(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "compiler",
		    CTLFLAG_RD, sc->compiler, 0, "Compiler info");

	if (sc->extra_version[0] != '\0')
		SYSCTL_ADD_STRING(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "extra_version", CTLFLAG_RD, sc->extra_version, 0,
		    "Extra version info");

	if (sc->serialno[0] != '\0')
		SYSCTL_ADD_STRING(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "serialno",
		    CTLFLAG_RD, sc->serialno, 0, "Serial number");

	if (sc->platform_blob_version[0] != '\0')
		SYSCTL_ADD_STRING(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "platform_blob_version", CTLFLAG_RD,
		    sc->platform_blob_version, 0,
		    "Platform blob version");

	if (sc->has_version_timestamp)
		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "version_timestamp", CTLFLAG_RD, &sc->version_timestamp, 0,
		    "Firmware version timestamp");

	SYSCTL_ADD_U64(&sc->sysctl_ctx, SYSCTL_CHILDREN(sc->sysctl_tree),
	    OID_AUTO, "table_addr", CTLFLAG_RD,
	    (uint64_t *)&sc->table_paddr, 0,
	    "Physical address of coreboot table");

	SYSCTL_ADD_ULONG(&sc->sysctl_ctx, SYSCTL_CHILDREN(sc->sysctl_tree),
	    OID_AUTO, "table_size", CTLFLAG_RD,
	    (unsigned long *)&sc->table_size,
	    "Total coreboot table size");

	if (sc->mb_vendor[0] != '\0' || sc->mb_part[0] != '\0') {
		oid_mb = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "mainboard",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0, "Mainboard information");

		if (sc->mb_vendor[0] != '\0')
			SYSCTL_ADD_STRING(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_mb), OID_AUTO, "vendor",
			    CTLFLAG_RD, sc->mb_vendor, 0, "Board vendor");

		if (sc->mb_part[0] != '\0')
			SYSCTL_ADD_STRING(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_mb), OID_AUTO, "part",
			    CTLFLAG_RD, sc->mb_part, 0, "Board part number");
	}

	if (sc->has_serial) {
		oid_serial = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "serial",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0, "Serial port");

		SYSCTL_ADD_U32(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid_serial),
		    OID_AUTO, "baseaddr", CTLFLAG_RD, &sc->serial_baseaddr, 0,
		    "Base address");

		SYSCTL_ADD_U32(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid_serial),
		    OID_AUTO, "baud", CTLFLAG_RD, &sc->serial_baud, 0,
		    "Baud rate");

		SYSCTL_ADD_U32(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid_serial),
		    OID_AUTO, "regwidth", CTLFLAG_RD, &sc->serial_regwidth, 0,
		    "Register width");
	}

	if (sc->has_tsc_info)
		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "tsc_freq_khz",
		    CTLFLAG_RD, &sc->tsc_freq_khz, 0,
		    "TSC frequency in kHz");

	if (sc->has_pcie)
		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "pcie_ctrl_base", CTLFLAG_RD, &sc->pcie_ctrl_base, 0,
		    "PCIe controller base address");

	if (sc->cbmem_count > 0) {
		oid_cbmem = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "cbmem",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0, "CBMEM entries");

		for (i = 0; i < sc->cbmem_count; i++) {
			struct cbmem_entry_info *info = &sc->cbmem_entries[i];

			snprintf(numstr, sizeof(numstr), "%u", i);
			oid_entry = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_cbmem), OID_AUTO, numstr,
			    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
			    "CBMEM entry");

			SYSCTL_ADD_STRING(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_entry), OID_AUTO, "name",
			    CTLFLAG_RD, info->name, 0, "Entry name");

			SYSCTL_ADD_U32(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_entry), OID_AUTO, "id",
			    CTLFLAG_RD, &info->id, 0, "Entry ID (hex)");

			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_entry), OID_AUTO, "address",
			    CTLFLAG_RD, (uint64_t *)&info->address, 0,
			    "Physical address");

			SYSCTL_ADD_U32(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_entry), OID_AUTO, "size",
			    CTLFLAG_RD, &info->size, 0, "Entry size");
		}
	}

	if (sc->has_board_config) {
		oid_board = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "board",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "Board identification");

		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_board), OID_AUTO, "fw_config",
		    CTLFLAG_RD, &sc->fw_config, 0,
		    "Firmware configuration bitmask");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_board), OID_AUTO, "board_id",
		    CTLFLAG_RD, &sc->board_id, 0,
		    "Board ID");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_board), OID_AUTO, "ram_code",
		    CTLFLAG_RD, &sc->ram_code, 0,
		    "RAM code");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_board), OID_AUTO, "sku_id",
		    CTLFLAG_RD, &sc->sku_id, 0,
		    "SKU ID");
	}

	if (sc->mac_count > 0) {
		struct sysctl_oid *oid_mac;

		oid_mac = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "mac",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "Factory MAC addresses");

		for (i = 0; i < sc->mac_count; i++) {
			uint8_t *m = sc->macs[i].mac_addr;

			snprintf(numstr, sizeof(numstr), "%u", i);
			snprintf(sc->mac_strs[i], sizeof(sc->mac_strs[i]),
			    "%02x:%02x:%02x:%02x:%02x:%02x",
			    m[0], m[1], m[2], m[3], m[4], m[5]);

			SYSCTL_ADD_STRING(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_mac), OID_AUTO, numstr,
			    CTLFLAG_RD, sc->mac_strs[i], 0,
			    "MAC address");
		}
	}

	if (sc->has_acpi_rsdp)
		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "acpi_rsdp",
		    CTLFLAG_RD, &sc->acpi_rsdp, 0,
		    "ACPI RSDP physical address");

	if (sc->has_boot_media) {
		oid_boot = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "boot_media",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "Boot media parameters");

		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_boot), OID_AUTO, "fmap_offset",
		    CTLFLAG_RD, &sc->fmap_offset, 0,
		    "FMAP offset from boot media start");

		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_boot), OID_AUTO, "cbfs_offset",
		    CTLFLAG_RD, &sc->cbfs_offset, 0,
		    "CBFS offset from boot media start");

		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_boot), OID_AUTO, "cbfs_size",
		    CTLFLAG_RD, &sc->cbfs_size, 0,
		    "CBFS size in bytes");

		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_boot), OID_AUTO, "size",
		    CTLFLAG_RD, &sc->boot_media_size, 0,
		    "Boot media size in bytes");
	}

	if (sc->has_mmc_info)
		SYSCTL_ADD_S32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "mmc_early_cmd1_status", CTLFLAG_RD,
		    &sc->mmc_early_cmd1_status, 0,
		    "Early eMMC CMD1 status");

	if (sc->has_spi_flash) {
		oid_spi = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "spi_flash",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "SPI flash parameters");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_spi), OID_AUTO, "size",
		    CTLFLAG_RD, &sc->spi_flash_size, 0,
		    "Flash size in bytes");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_spi), OID_AUTO, "sector_size",
		    CTLFLAG_RD, &sc->spi_sector_size, 0,
		    "Sector size in bytes");

		SYSCTL_ADD_U8(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_spi), OID_AUTO, "erase_cmd",
		    CTLFLAG_RD, &sc->spi_erase_cmd, 0,
		    "Erase command byte");
	}

	if (sc->has_console_type)
		SYSCTL_ADD_U16(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "console_type", CTLFLAG_RD, &sc->console_type, 0,
		    "Firmware console type");

	if (sc->has_framebuffer) {
		oid_fb = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "framebuffer",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "Framebuffer information");

		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_fb), OID_AUTO, "addr",
		    CTLFLAG_RD, &sc->fb_addr, 0,
		    "Physical address");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_fb), OID_AUTO, "x_res",
		    CTLFLAG_RD, &sc->fb_x_res, 0,
		    "Horizontal resolution");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_fb), OID_AUTO, "y_res",
		    CTLFLAG_RD, &sc->fb_y_res, 0,
		    "Vertical resolution");

		SYSCTL_ADD_U8(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_fb), OID_AUTO, "bpp",
		    CTLFLAG_RD, &sc->fb_bpp, 0,
		    "Bits per pixel");
	}

	if (sc->gpio_count > 0) {
		struct sysctl_oid *oid_pin;

		oid_gpio = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "gpio",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "GPIO pin states");

		for (i = 0; i < sc->gpio_count; i++) {
			struct cb_gpio *g = &sc->gpios[i];

			snprintf(numstr, sizeof(numstr), "%u", i);
			oid_pin = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_gpio), OID_AUTO, numstr,
			    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
			    "GPIO pin");

			/* Ensure name is NUL-terminated */
			g->name[sizeof(g->name) - 1] = '\0';
			SYSCTL_ADD_STRING(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_pin), OID_AUTO, "name",
			    CTLFLAG_RD, g->name, 0, "Pin name");

			SYSCTL_ADD_U32(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_pin), OID_AUTO, "port",
			    CTLFLAG_RD, &g->port, 0, "Port number");

			SYSCTL_ADD_U32(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_pin), OID_AUTO, "value",
			    CTLFLAG_RD, &g->value, 0, "Pin value");

			SYSCTL_ADD_U32(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_pin), OID_AUTO, "polarity",
			    CTLFLAG_RD, &g->polarity, 0, "Pin polarity");
		}
	}

	if (sc->has_tpm) {
		oid_tpm = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "tpm",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "TPM information");

		SYSCTL_ADD_U8(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_tpm), OID_AUTO, "version",
		    CTLFLAG_RD, &sc->tpm_version, 0,
		    "TPM version (1=1.2, 2=2.0)");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_tpm), OID_AUTO, "ppi_addr",
		    CTLFLAG_RD, &sc->tpm_ppi_addr, 0,
		    "PPI address");

		if (sc->has_tpm_log)
			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_tpm), OID_AUTO, "cblog_addr",
			    CTLFLAG_RD, (uint64_t *)&sc->tpm_log_paddr, 0,
			    "TPM event log physical address");
	} else if (sc->has_tpm_log) {
		/* TPM log without PPI - create tpm node just for the log */
		oid_tpm = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "tpm",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "TPM information");

		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_tpm), OID_AUTO, "cblog_addr",
		    CTLFLAG_RD, (uint64_t *)&sc->tpm_log_paddr, 0,
		    "TPM event log physical address");
	}

	if (sc->has_smmstore) {
		oid_smmstore = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "smmstore",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "SMMSTORE v2 configuration");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_smmstore), OID_AUTO, "num_blocks",
		    CTLFLAG_RD, &sc->smmstore_num_blocks, 0,
		    "Number of blocks");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_smmstore), OID_AUTO, "block_size",
		    CTLFLAG_RD, &sc->smmstore_block_size, 0,
		    "Block size in bytes");

		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_smmstore), OID_AUTO, "mmap_addr",
		    CTLFLAG_RD, &sc->smmstore_mmap_addr, 0,
		    "Memory-mapped address");

		SYSCTL_ADD_U32(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_smmstore), OID_AUTO, "com_buffer",
		    CTLFLAG_RD, &sc->smmstore_com_buffer, 0,
		    "Communication buffer address");

		SYSCTL_ADD_U8(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(oid_smmstore), OID_AUTO, "apm_cmd",
		    CTLFLAG_RD, &sc->smmstore_apm_cmd, 0,
		    "APM command byte");
		}

	if (sc->has_acpi_gnvs || sc->has_acpi_cnvs || sc->has_vpd ||
	    sc->has_wifi_cal || sc->has_fmap || sc->has_vboot_workbuf ||
	    sc->has_type_c_info || sc->has_root_bridge_info) {
		oid_cbrefs = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO, "cbmem_refs",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
		    "Additional CBMEM reference addresses");

		if (sc->has_acpi_gnvs)
			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_cbrefs), OID_AUTO, "acpi_gnvs",
			    CTLFLAG_RD, (uint64_t *)&sc->acpi_gnvs_paddr, 0,
			    "ACPI GNVS CBMEM physical address");
		if (sc->has_acpi_cnvs)
			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_cbrefs), OID_AUTO, "acpi_cnvs",
			    CTLFLAG_RD, (uint64_t *)&sc->acpi_cnvs_paddr, 0,
			    "ACPI CNVS CBMEM physical address");
		if (sc->has_vpd)
			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_cbrefs), OID_AUTO, "vpd",
			    CTLFLAG_RD, (uint64_t *)&sc->vpd_paddr, 0,
			    "VPD CBMEM physical address");
		if (sc->has_wifi_cal)
			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_cbrefs), OID_AUTO,
			    "wifi_calibration", CTLFLAG_RD,
			    (uint64_t *)&sc->wifi_cal_paddr, 0,
			    "WiFi calibration CBMEM physical address");
		if (sc->has_fmap)
			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_cbrefs), OID_AUTO, "fmap",
			    CTLFLAG_RD, (uint64_t *)&sc->fmap_paddr, 0,
			    "FMAP CBMEM physical address");
		if (sc->has_vboot_workbuf)
			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_cbrefs), OID_AUTO,
			    "vboot_workbuf", CTLFLAG_RD,
			    (uint64_t *)&sc->vboot_workbuf_paddr, 0,
			    "Vboot work buffer CBMEM physical address");
		if (sc->has_type_c_info)
			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_cbrefs), OID_AUTO,
			    "type_c_info", CTLFLAG_RD,
			    (uint64_t *)&sc->type_c_info_paddr, 0,
			    "Type-C info CBMEM physical address");
		if (sc->has_root_bridge_info)
			SYSCTL_ADD_U64(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(oid_cbrefs), OID_AUTO,
			    "root_bridge_info", CTLFLAG_RD,
			    (uint64_t *)&sc->root_bridge_info_paddr, 0,
			    "Root bridge info CBMEM physical address");
	}

	if (sc->has_timestamps) {
		SYSCTL_ADD_U64(&sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    "timestamps_addr", CTLFLAG_RD,
		    (uint64_t *)&sc->timestamps_paddr, 0,
		    "Timestamps CBMEM physical address");

		coreboot_timestamps_register(sc, sc->sysctl_tree);
	}
}

/*
 * Identify: scan low memory for "LBIO" signature and register a child
 */
static void
coreboot_identify(driver_t *driver, device_t parent)
{
	vm_paddr_t low_addr, real_addr;
	struct cb_header *hdr;
	uint8_t *entry, *table_end;
	struct cb_record *rec;
	device_t child;
	void *va;
	vm_size_t map_size;
	int error;

	if (!device_is_alive(parent))
		return;

	if (device_find_child(parent, "coreboot", -1) != NULL)
		return;

	low_addr = coreboot_scan_region(CB_SCAN_LOW_START, CB_SCAN_LOW_END);
	if (low_addr == 0)
		return;

	va = pmap_mapbios(low_addr, sizeof(struct cb_header));
	if (va == NULL)
		return;

	hdr = (struct cb_header *)va;
	if (memcmp(hdr->signature, CB_HEADER_SIGNATURE,
	    CB_HEADER_SIG_LEN) != 0) {
		pmap_unmapbios(va, sizeof(struct cb_header));
		return;
	}
	error = coreboot_sanitize_header(hdr, &map_size);
	pmap_unmapbios(va, sizeof(struct cb_header));
	if (error != 0)
		return;

	va = pmap_mapbios(low_addr, map_size);
	if (va == NULL)
		return;

	hdr = (struct cb_header *)va;
	if (memcmp(hdr->signature, CB_HEADER_SIGNATURE,
	    CB_HEADER_SIG_LEN) != 0 ||
	    coreboot_validate_header(hdr, map_size) != 0 ||
	    coreboot_validate_table(hdr, map_size) != 0) {
		pmap_unmapbios(va, map_size);
		return;
	}

	/* Look for CB_TAG_FORWARD to find the real table in high memory */
	real_addr = low_addr;
	entry = (uint8_t *)hdr + hdr->header_bytes;
	table_end = entry + hdr->table_bytes;
	while ((size_t)(table_end - entry) >= sizeof(struct cb_record)) {
		size_t rec_size;

		rec = (struct cb_record *)entry;
		rec_size = rec->size;
		if (rec_size < sizeof(struct cb_record))
			break;
		if (rec_size > (size_t)(table_end - entry))
			break;
		if (rec->tag == CB_TAG_FORWARD) {
			if (rec_size >= sizeof(struct cb_forward)) {
				struct cb_forward *fwd;

				fwd = (struct cb_forward *)entry;
				real_addr = (vm_paddr_t)fwd->forward;
			}
			break;
		}
		entry += rec_size;
	}
	pmap_unmapbios(va, map_size);

	child = BUS_ADD_CHILD(parent, 5, "coreboot", DEVICE_UNIT_ANY);
	if (child == NULL)
		return;
	device_set_driver(child, driver);

	bus_set_resource(child, SYS_RES_MEMORY, 0, real_addr, PAGE_SIZE);
	device_set_desc(child, "coreboot firmware table");
}

/*
 * Probe: validate the coreboot header at the discovered address
 */
static int
coreboot_probe(device_t dev)
{
	vm_paddr_t pa;
	void *va;
	struct cb_header *hdr;
	int error;
	vm_size_t map_size;

	pa = bus_get_resource_start(dev, SYS_RES_MEMORY, 0);
	if (pa == 0)
		return (ENXIO);

	va = pmap_mapbios(pa, sizeof(struct cb_header));
	if (va == NULL) {
		device_printf(dev, "unable to map coreboot header\n");
		return (ENOMEM);
	}

	hdr = (struct cb_header *)va;
	if (memcmp(hdr->signature, CB_HEADER_SIGNATURE,
	    CB_HEADER_SIG_LEN) != 0) {
		device_printf(dev, "bad coreboot signature at %#jx\n",
		    (uintmax_t)pa);
		pmap_unmapbios(va, sizeof(struct cb_header));
		return (ENXIO);
	}

	error = coreboot_sanitize_header(hdr, &map_size);
	pmap_unmapbios(va, sizeof(struct cb_header));
	if (error != 0) {
		device_printf(dev, "invalid coreboot header lengths\n");
		return (ENXIO);
	}

	va = pmap_mapbios(pa, map_size);
	if (va == NULL) {
		device_printf(dev, "unable to map coreboot table\n");
		return (ENOMEM);
	}
	hdr = (struct cb_header *)va;

	error = coreboot_validate_header(hdr, map_size);
	if (error != 0) {
		device_printf(dev, "coreboot header checksum failed\n");
		pmap_unmapbios(va, map_size);
		return (ENXIO);
	}
	error = coreboot_validate_table(hdr, map_size);
	if (error != 0) {
		device_printf(dev, "coreboot table checksum failed\n");
		pmap_unmapbios(va, map_size);
		return (ENXIO);
	}

	pmap_unmapbios(va, map_size);
	return (BUS_PROBE_SPECIFIC);
}

/*
 * Attach: map the full table, parse records, register sysctls and cdevs
 */
static int
coreboot_attach(device_t dev)
{
	struct coreboot_softc *sc;
	struct cb_header *hdr;
	vm_paddr_t pa;
	void *va;
	vm_size_t map_size;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	pa = bus_get_resource_start(dev, SYS_RES_MEMORY, 0);

	va = pmap_mapbios(pa, sizeof(struct cb_header));
	if (va == NULL) {
		device_printf(dev, "unable to map coreboot header\n");
		return (ENOMEM);
	}

	hdr = (struct cb_header *)va;
	if (memcmp(hdr->signature, CB_HEADER_SIGNATURE,
	    CB_HEADER_SIG_LEN) != 0) {
		device_printf(dev, "bad coreboot signature at %#jx\n",
		    (uintmax_t)pa);
		pmap_unmapbios(va, sizeof(struct cb_header));
		return (ENXIO);
	}
	error = coreboot_sanitize_header(hdr, &map_size);
	pmap_unmapbios(va, sizeof(struct cb_header));
	if (error != 0) {
		device_printf(dev, "invalid coreboot header lengths\n");
		return (ENXIO);
	}

	va = pmap_mapbios(pa, map_size);
	if (va == NULL) {
		device_printf(dev, "unable to map coreboot table (%zu bytes)\n",
		    (size_t)map_size);
		return (ENOMEM);
	}

	sc->table_paddr = pa;
	sc->table_size = map_size;
	sc->table_vaddr = va;

	hdr = (struct cb_header *)va;
	if (memcmp(hdr->signature, CB_HEADER_SIGNATURE,
	    CB_HEADER_SIG_LEN) != 0 ||
	    coreboot_validate_header(hdr, map_size) != 0 ||
	    coreboot_validate_table(hdr, map_size) != 0) {
		device_printf(dev, "coreboot table validation failed\n");
		pmap_unmapbios(va, map_size);
		sc->table_vaddr = NULL;
		return (ENXIO);
	}

	device_printf(dev, "coreboot table at %#jx (%u entries, %u bytes)\n",
	    (uintmax_t)pa, hdr->table_entries, hdr->table_bytes);

	coreboot_parse_table(sc, hdr);

	if (sc->version[0] != '\0')
		device_printf(dev, "firmware: %s\n", sc->version);
	if (sc->mb_vendor[0] != '\0')
		device_printf(dev, "mainboard: %s %s\n", sc->mb_vendor,
		    sc->mb_part);
	if (sc->has_console)
		device_printf(dev, "CBMEM console at %#jx\n",
		    (uintmax_t)sc->console_paddr);
	device_printf(dev, "CBMEM entries: %u\n", sc->cbmem_count);
	if (sc->has_board_config)
		device_printf(dev,
		    "board: id=%u sku=%u fw_config=%#jx\n",
		    sc->board_id, sc->sku_id,
		    (uintmax_t)sc->fw_config);
	if (sc->mac_count > 0)
		device_printf(dev, "factory MAC addresses: %u\n",
		    sc->mac_count);
	if (sc->has_acpi_rsdp)
		device_printf(dev, "ACPI RSDP at %#jx\n",
		    (uintmax_t)sc->acpi_rsdp);
	if (sc->has_pcie)
		device_printf(dev, "PCIe controller at %#jx\n",
		    (uintmax_t)sc->pcie_ctrl_base);
	if (sc->has_boot_media)
		device_printf(dev, "boot media: %#jx bytes, CBFS %#jx+%#jx\n",
		    (uintmax_t)sc->boot_media_size,
		    (uintmax_t)sc->cbfs_offset, (uintmax_t)sc->cbfs_size);
	if (sc->has_mmc_info)
		device_printf(dev, "MMC early CMD1 status: %d\n",
		    sc->mmc_early_cmd1_status);

	if (bootverbose) {
		if (sc->has_spi_flash)
			device_printf(dev,
			    "SPI flash: %u bytes, sector %u, erase %#x\n",
			    sc->spi_flash_size, sc->spi_sector_size,
			    sc->spi_erase_cmd);
		if (sc->has_console_type)
			device_printf(dev, "console type: %u\n",
			    sc->console_type);
		if (sc->has_framebuffer)
			device_printf(dev,
			    "framebuffer: %ux%u@%ubpp at %#jx\n",
			    sc->fb_x_res, sc->fb_y_res, sc->fb_bpp,
			    (uintmax_t)sc->fb_addr);
		if (sc->gpio_count > 0)
			device_printf(dev, "GPIO pins: %u\n",
			    sc->gpio_count);
		if (sc->has_tpm)
			device_printf(dev, "TPM %u.%u PPI at %#x\n",
			    sc->tpm_version == 2 ? 2 : 1,
			    sc->tpm_version == 2 ? 0 : 2,
			    sc->tpm_ppi_addr);
	}

	if (coreboot_debug) {
		if (sc->has_smmstore)
			device_printf(dev,
			    "SMMSTORE v2: %u blocks x %u bytes, "
			    "apm_cmd=%#x\n",
			    sc->smmstore_num_blocks,
			    sc->smmstore_block_size,
			    sc->smmstore_apm_cmd);
		if (sc->has_timestamps)
			device_printf(dev, "timestamps at %#jx\n",
			    (uintmax_t)sc->timestamps_paddr);
		if (sc->has_tpm_log)
			device_printf(dev, "TPM CB log at %#jx\n",
			    (uintmax_t)sc->tpm_log_paddr);
		if (sc->has_fmap)
			device_printf(dev, "FMAP at %#jx\n",
			    (uintmax_t)sc->fmap_paddr);
	}

	coreboot_register_sysctls(sc);

	coreboot_sc = sc;

	if (sc->has_console) {
		error = coreboot_console_create(sc);
		if (error != 0)
			device_printf(dev,
			    "failed to create /dev/coreboot_console (%d)\n",
			    error);
	}
	if (sc->cbmem_count > 0) {
		error = coreboot_cbmem_create(sc);
		if (error != 0)
			device_printf(dev, "failed to create /dev/cbmem (%d)\n",
			    error);
	}

	return (0);
}

/*
 * Detach: unmap table, destroy cdevs and sysctls
 */
static int
coreboot_detach(device_t dev)
{
	struct coreboot_softc *sc;

	sc = device_get_softc(dev);

	coreboot_cbmem_destroy(sc);
	coreboot_console_destroy(sc);

	coreboot_sc = NULL;

	sysctl_ctx_free(&sc->sysctl_ctx);

	if (sc->table_vaddr != NULL) {
		pmap_unmapbios(sc->table_vaddr, sc->table_size);
		sc->table_vaddr = NULL;
	}

	if (sc->console_vaddr != NULL) {
		pmap_unmapbios(sc->console_vaddr, sc->console_size);
		sc->console_vaddr = NULL;
		sc->console_size = 0;
		sc->console_data_size = 0;
	}

	return (0);
}

static int
coreboot_modevent(module_t mod, int what, void *arg)
{
	device_t *devs;
	int count, i;

	switch (what) {
	case MOD_LOAD:
		break;
	case MOD_UNLOAD:
		devclass_get_devices(devclass_find("coreboot"), &devs, &count);
		for (i = 0; i < count; i++)
			device_delete_child(device_get_parent(devs[i]),
			    devs[i]);
		free(devs, M_TEMP);
		break;
	default:
		break;
	}

	return (0);
}

static device_method_t coreboot_methods[] = {
	DEVMETHOD(device_identify,	coreboot_identify),
	DEVMETHOD(device_probe,		coreboot_probe),
	DEVMETHOD(device_attach,	coreboot_attach),
	DEVMETHOD(device_detach,	coreboot_detach),
	DEVMETHOD_END
};

static driver_t coreboot_driver = {
	"coreboot",
	coreboot_methods,
	sizeof(struct coreboot_softc),
};

DRIVER_MODULE(coreboot, nexus, coreboot_driver, coreboot_modevent, NULL);
MODULE_VERSION(coreboot, 1);
