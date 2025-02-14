// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020-2022 Loongson Technology Corporation Limited
 *
 * Derived from MIPS:
 * Copyright (C) 1995 Linus Torvalds
 * Copyright (C) 1995 Waldorf Electronics
 * Copyright (C) 1994, 95, 96, 97, 98, 99, 2000, 01, 02, 03  Ralf Baechle
 * Copyright (C) 1996 Stoned Elipot
 * Copyright (C) 1999 Silicon Graphics, Inc.
 * Copyright (C) 2000, 2001, 2002, 2007	 Maciej W. Rozycki
 */
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/cpu.h>
#include <linux/dmi.h>
#include <linux/efi.h>
#include <linux/export.h>
#include <linux/memblock.h>
#include <linux/initrd.h>
#include <linux/ioport.h>
#include <linux/kexec.h>
#include <linux/crash_dump.h>
#include <linux/root_dev.h>
#include <linux/console.h>
#include <linux/pfn.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/device.h>
#include <linux/dma-map-ops.h>
#include <linux/libfdt.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/suspend.h>
#include <linux/swiotlb.h>

#include <asm/addrspace.h>
#include <asm/alternative.h>
#include <asm/bootinfo.h>
#include <asm/cache.h>
#include <asm/cpu.h>
#include <asm/dma.h>
#include <asm/efi.h>
#include <asm/loongson.h>
#include <asm/numa.h>
#include <asm/pgalloc.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/time.h>
#include <asm/unwind.h>

#include <boot_param.h>

#ifndef CONFIG_BUILTIN_DTB_NAME
#define CONFIG_BUILTIN_DTB_NAME ""
#endif

#define SMBIOS_BIOSSIZE_OFFSET		0x09
#define SMBIOS_BIOSEXTERN_OFFSET	0x13
#define SMBIOS_FREQLOW_OFFSET		0x16
#define SMBIOS_FREQHIGH_OFFSET		0x17
#define SMBIOS_FREQLOW_MASK		0xFF
#define SMBIOS_CORE_PACKAGE_OFFSET	0x23
#define SMBIOS_THREAD_PACKAGE_OFFSET	0x25
#define LOONGSON_EFI_ENABLE		(1 << 3)

unsigned long fw_arg0, fw_arg1, fw_arg2;
DEFINE_PER_CPU(unsigned long, kernelsp);
struct cpuinfo_loongarch cpu_data[NR_CPUS] __read_mostly;

EXPORT_SYMBOL(cpu_data);

struct loongson_board_info b_info;
static const char dmi_empty_string[] = "        ";

/*
 * Setup information
 *
 * These are initialized so they are in the .data section
 */
char init_command_line[COMMAND_LINE_SIZE] __initdata;
char board_name_desc[] = "board_name="; // 这是 command line 里面的内容 一定要有=
char bp_start_desc[] = "bp_start="; // 这是 command line 里面的内容 一定要有=

static int num_standard_resources;
static struct resource *standard_resources;

static struct resource code_resource = { .name = "Kernel code", };
static struct resource data_resource = { .name = "Kernel data", };
static struct resource bss_resource  = { .name = "Kernel bss", };

//////////////////////////////////////
/////for bootloader mem map parse/////
//////////////////////////////////////
static unsigned long long bp_start;
struct loongsonlist_mem_map *loongson_mem_map;
static u8 __init ext_listhdr_checksum(u8 *buffer, u32 length);
static int __init parse_mem(struct _extention_list_hdr *head);
static void __init memmap_bootlaoder_parse(void);
static int __init parse_extlist(struct boot_params *bp);
static int __init bp_start_match(void);

const char *get_system_type(void)
{
	return "generic-loongson-machine";
}

void __init arch_cpu_finalize_init(void)
{
	alternative_instructions();
}

static const char *dmi_string_parse(const struct dmi_header *dm, u8 s)
{
	const u8 *bp = ((u8 *) dm) + dm->length;

	if (s) {
		s--;
		while (s > 0 && *bp) {
			bp += strlen(bp) + 1;
			s--;
		}

		if (*bp != 0) {
			size_t len = strlen(bp)+1;
			size_t cmp_len = len > 8 ? 8 : len;

			if (!memcmp(bp, dmi_empty_string, cmp_len))
				return dmi_empty_string;

			return bp;
		}
	}

	return "";
}

static void __init parse_cpu_table(const struct dmi_header *dm)
{
	long freq_temp = 0;
	char *dmi_data = (char *)dm;

	freq_temp = ((*(dmi_data + SMBIOS_FREQHIGH_OFFSET) << 8) +
			((*(dmi_data + SMBIOS_FREQLOW_OFFSET)) & SMBIOS_FREQLOW_MASK));
	cpu_clock_freq = freq_temp * 1000000;

	loongson_sysconf.cpuname = (void *)dmi_string_parse(dm, dmi_data[16]);
	loongson_sysconf.cores_per_package = *(dmi_data + SMBIOS_THREAD_PACKAGE_OFFSET);

	pr_info("CpuClock = %llu\n", cpu_clock_freq);
}

static void __init parse_bios_table(const struct dmi_header *dm)
{
	char *dmi_data = (char *)dm;

	b_info.bios_size = (*(dmi_data + SMBIOS_BIOSSIZE_OFFSET) + 1) << 6;
}

static void __init find_tokens(const struct dmi_header *dm, void *dummy)
{
	switch (dm->type) {
	case 0x0: /* Extern BIOS */
		parse_bios_table(dm);
		break;
	case 0x4: /* Calling interface */
		parse_cpu_table(dm);
		break;
	}
}
static void __init smbios_parse(void)
{
	b_info.bios_vendor = (void *)dmi_get_system_info(DMI_BIOS_VENDOR);
	b_info.bios_version = (void *)dmi_get_system_info(DMI_BIOS_VERSION);
	b_info.bios_release_date = (void *)dmi_get_system_info(DMI_BIOS_DATE);
	b_info.board_vendor = (void *)dmi_get_system_info(DMI_BOARD_VENDOR);
	b_info.board_name = (void *)dmi_get_system_info(DMI_BOARD_NAME);
	dmi_walk(find_tokens, NULL);
}

#ifdef CONFIG_ARCH_WRITECOMBINE
bool wc_enabled = true;
#else
bool wc_enabled = false;
#endif

EXPORT_SYMBOL(wc_enabled);

static int __init setup_writecombine(char *p)
{
	if (!strcmp(p, "on"))
		wc_enabled = true;
	else if (!strcmp(p, "off"))
		wc_enabled = false;
	else
		pr_warn("Unknown writecombine setting \"%s\".\n", p);

	return 0;
}
early_param("writecombine", setup_writecombine);

static int usermem __initdata;

static int __init early_parse_mem(char *p)
{
	phys_addr_t start, size;

	if (!p) {
		pr_err("mem parameter is empty, do nothing\n");
		return -EINVAL;
	}

	/*
	 * If a user specifies memory size, we
	 * blow away any automatically generated
	 * size.
	 */
	if (usermem == 0) {
		usermem = 1;
		memblock_remove(memblock_start_of_DRAM(),
			memblock_end_of_DRAM() - memblock_start_of_DRAM());
	}
	start = 0;
	size = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);
	else {
		pr_err("Invalid format!\n");
		return -EINVAL;
	}

	if (!IS_ENABLED(CONFIG_NUMA))
		memblock_add(start, size);
	else
		memblock_add_node(start, size, pa_to_nid(start), MEMBLOCK_NONE);

	return 0;
}
early_param("mem", early_parse_mem);

static void __init arch_reserve_vmcore(void)
{
#ifdef CONFIG_PROC_VMCORE
	u64 i;
	phys_addr_t start, end;

	if (!is_kdump_kernel())
		return;

	if (!elfcorehdr_size) {
		for_each_mem_range(i, &start, &end) {
			if (elfcorehdr_addr >= start && elfcorehdr_addr < end) {
				/*
				 * Reserve from the elf core header to the end of
				 * the memory segment, that should all be kdump
				 * reserved memory.
				 */
				elfcorehdr_size = end - elfcorehdr_addr;
				break;
			}
		}
	}

	if (memblock_is_region_reserved(elfcorehdr_addr, elfcorehdr_size)) {
		pr_warn("elfcorehdr is overlapped\n");
		return;
	}

	memblock_reserve(elfcorehdr_addr, elfcorehdr_size);

	pr_info("Reserving %llu KiB of memory at 0x%llx for elfcorehdr\n",
		elfcorehdr_size >> 10, elfcorehdr_addr);
#endif
}

static void __init arch_reserve_crashkernel(void)
{
	int ret;
	unsigned long long low_size = 0;
	unsigned long long crash_base, crash_size;
	char *cmdline = boot_command_line;
	bool high = false;

	if (!IS_ENABLED(CONFIG_CRASH_RESERVE))
		return;

	ret = parse_crashkernel(cmdline, memblock_phys_mem_size(),
				&crash_size, &crash_base, &low_size, &high);
	if (ret)
		return;

	reserve_crashkernel_generic(cmdline, crash_size, crash_base, low_size, high);
}

#ifdef CONFIG_DTB_MATCH_BY_BOARD_NAME
static void* __init get_fdt_by_board_name(void)
{
	void *fdt = NULL;
	char* board_name;
	char temp[128]; // 不想申请空间 应该不会有这么长的名字吧
	board_name = strstr(boot_command_line, board_name_desc);
	if (board_name) {
		int i;
		memset(temp, 0, 128);
		board_name += strlen(board_name_desc); // 跳过 = 和前面的字段
		for (i = 0; i < 127; ++i) {
			if (board_name[i] == 0 || board_name[i] == ' ')
				break;
			temp[i] = board_name[i];
		}
		board_name = temp;
	}
	if (board_name) {
#ifdef CONFIG_LOONGSON_2K500
		int i;
		if (!strncmp(board_name, "LS2K500-HL-MB", 13))
			fdt = &__dtb_ls2k500_hl_mb_begin;
		else if (!strncmp(board_name, "LS2K500-MINI-DP", 15))
			fdt = &__dtb_ls2k500_mini_dp_begin;
		else if (!strncmp(board_name, "LS2K500-DAYU400-MB", 17))
			fdt = &__dtb_ls2k500_dayu400_mb_begin;
		else if (!strncmp(board_name, "LS2K500-MODI-HCT", 16))
			fdt = &__dtb_ls2k500_modi_hct_begin;
		else if (!strncmp(board_name, "LS2K500-ZHENGTAI-PCS1800-V10", 28))
			fdt = &__dtb_ls2k500_zhengtai_pcs1800_v10_begin;
		else if (!strncmp(board_name, "LS2K500-JIAOQIAN-V10", 20))
			fdt = &__dtb_ls2k500_jiaoqian_v10_begin;
		else if (!strncmp(board_name, "LS2K500-ZJJZ", 12))
			fdt = &__dtb_ls2k500_zjjz_begin;
		else
			fdt = &__dtb_ls2k500_mini_dp_begin;

		// for (i = 0; i < NR_CPUS; ++i)
		// 	if (check_cpu_full_name_invaild(i))
		// 		__cpu_full_name[i] = cpu_ls2k500_name;
#elif defined(CONFIG_LOONGSON_2K1000)
		if (!strncmp(board_name, "LS2K1000-JL-MB", 14)) {
			if (!strncmp(board_name, "LS2K1000-JL-MB-MU", 17))
				fdt = &__dtb_ls2k1000_jl_mb_mu_begin;
			else if (!strncmp(board_name, "LS2K1000-JL-MB-NODVO", 20))
				fdt = &__dtb_ls2k1000_jl_mb_nodvo_begin;
			else
				fdt = &__dtb_ls2k1000_jl_mb_begin;
		} else if (!strncmp(board_name, "LS2K1000-DP", 11)) {
		#ifdef CONFIG_SND_LS1X_SOC_I2S
			fdt = &__dtb_ls2k1000_dp_i2s_begin;
		#else
			if (!strncmp(board_name, "LS2K1000-DP-TEST", 16)) {
				fdt = &__dtb_ls2k1000_dp_test_begin;
			} else if (!strncmp(board_name, "LS2K1000-DP-FACTORY", 19)){
				fdt = &__dtb_ls2k1000_dp_factory_begin;
			} else {
				fdt = &__dtb_ls2k1000_dp_begin;
			}
		#endif
		} else if (!strncmp(board_name, "HAC_MB_REVC", 11)) {
			fdt = &__dtb_ksec_hac_mb_begin;
		} else if (!strncmp(board_name, "GBKPDM0-V10", 11)) {
			fdt = &__dtb_gbkpdm0_v10_begin;
		} else if (!strncmp(board_name, "GBKPDM0-LITONG", 14)) {
			fdt = &__dtb_gbkpdm0_litong_begin;
		} else if (!strncmp(board_name, "LS2K1000-ChuangLong-MB", 22)) {
			fdt = &__dtb_ls2k1000_cl_mb_begin;
		} else if (!strncmp(board_name, "C4G-MB001", 9)) {
			fdt = &__dtb_ls2k1000_c4g_mb001_begin;
		} else if (!strncmp(board_name, "ZNWLIDZD-LITONG", 15)) {
			fdt = &__dtb_ls2k1000_znwlidzd_litong_begin;
		} else {
			fdt = &__dtb_ls2k1000_dp_begin;
		}
#elif defined(CONFIG_LOONGSON_2P500)
		if (!strncmp(board_name, "LS2P500-EVB", 11))
			fdt = &__dtb_ls2p500_evb_begin;
		else if (!strncmp(board_name, "LS2P500-RSJ", 11))
			fdt = &__dtb_ls2p500_rsj_mb_v10_begin;
		else if (!strncmp(board_name, "LS2P500-GBKPJM0", 15))
			fdt = &__dtb_ls2p500_gbkpjm0_v10_begin;
		else
			fdt = &__dtb_ls2p500_evb_begin;
#elif defined(CONFIG_LOONGSON_2K300)
		if (!strncmp(board_name, "LS2K300-MINI-DP", 15))
			fdt = &__dtb_ls2k300_mini_dp_begin;
		else if (!strncmp(board_name, "LS2K300-PAI", 11))
			fdt = &__dtb_ls2k300_vanguard_pi_begin;
		else if (!strncmp(board_name, "LS2K300-TEWEI", 13))
			fdt = &__dtb_ls2k300_tewei_begin;
		else
			fdt = &__dtb_ls2k300_mini_dp_begin;
#endif
	}
	return fdt;
}
#endif

static void __init fdt_setup(void)
{
#ifdef CONFIG_OF_EARLY_FLATTREE
	void *fdt_pointer;

	/* ACPI-based systems do not require parsing fdt */
	if (acpi_os_get_root_pointer())
		return;

	fdt_pointer = NULL;

#ifdef CONFIG_DTB_MATCH_BY_BOARD_NAME
	fdt_pointer = efi_fdt_pointer(); /* Fallback to firmware dtb */
	if (!fdt_pointer)
		fdt_pointer = get_fdt_by_board_name();
#else
	/* Prefer to use built-in dtb, checking its legality first. */
	if (!fdt_check_header(__dtb_start) && strcmp(CONFIG_BUILTIN_DTB_NAME, ""))
		fdt_pointer = __dtb_start;
	else
		fdt_pointer = efi_fdt_pointer(); /* Fallback to firmware dtb */
#endif

	if (!fdt_pointer || fdt_check_header(fdt_pointer))
		return;

	early_init_dt_scan(fdt_pointer);
	early_init_fdt_reserve_self();

	max_low_pfn = PFN_PHYS(memblock_end_of_DRAM());
#endif
}

static void __init bootcmdline_init(char **cmdline_p)
{
	/*
	 * If CONFIG_CMDLINE_FORCE is enabled then initializing the command line
	 * is trivial - we simply use the built-in command line unconditionally &
	 * unmodified.
	 */
	if (IS_ENABLED(CONFIG_CMDLINE_FORCE)) {
		strscpy(boot_command_line, CONFIG_CMDLINE, COMMAND_LINE_SIZE);
		goto out;
	}

#ifdef CONFIG_OF_FLATTREE
	/*
	 * If CONFIG_CMDLINE_BOOTLOADER is enabled and we are in FDT-based system,
	 * the boot_command_line will be overwritten by early_init_dt_scan_chosen().
	 * So we need to append init_command_line (the original copy of boot_command_line)
	 * to boot_command_line.
	 */
	if (initial_boot_params) {
		if (boot_command_line[0])
			strlcat(boot_command_line, " ", COMMAND_LINE_SIZE);

		if (!strstr(boot_command_line, init_command_line))
			strlcat(boot_command_line, init_command_line, COMMAND_LINE_SIZE);

		goto out;
	}
#endif

	/*
	 * Append built-in command line to the bootloader command line if
	 * CONFIG_CMDLINE_EXTEND is enabled.
	 */
	if (IS_ENABLED(CONFIG_CMDLINE_EXTEND) && CONFIG_CMDLINE[0]) {
		strlcat(boot_command_line, " ", COMMAND_LINE_SIZE);
		strlcat(boot_command_line, CONFIG_CMDLINE, COMMAND_LINE_SIZE);
	}

	/*
	 * Use built-in command line if the bootloader command line is empty.
	 */
	if (IS_ENABLED(CONFIG_CMDLINE_BOOTLOADER) && !boot_command_line[0])
		strscpy(boot_command_line, CONFIG_CMDLINE, COMMAND_LINE_SIZE);

out:
	*cmdline_p = boot_command_line;
}

void __init platform_init(void)
{
	arch_reserve_vmcore();
	arch_reserve_crashkernel();

#ifdef CONFIG_ACPI
	acpi_table_upgrade();
	acpi_gbl_use_default_register_widths = false;
	acpi_boot_table_init();
#endif

	early_init_fdt_scan_reserved_mem();
	unflatten_and_copy_device_tree();

#ifdef CONFIG_NUMA
	init_numa_memory();
#endif
	dmi_setup();
	smbios_parse();
	pr_info("The BIOS Version: %s\n", b_info.bios_version);

	efi_runtime_init();
}

static void __init check_kernel_sections_mem(void)
{
	phys_addr_t start = __pa_symbol(&_text);
	phys_addr_t size = __pa_symbol(&_end) - start;

	if (!memblock_is_region_memory(start, size)) {
		pr_info("Kernel sections are not in the memory maps\n");
		memblock_add(start, size);
	}
}

/*
 * arch_mem_init - initialize memory management subsystem
 */
static void __init arch_mem_init(char **cmdline_p)
{
	if (usermem)
		pr_info("User-defined physical RAM map overwrite\n");

	check_kernel_sections_mem();

	/*
	 * In order to reduce the possibility of kernel panic when failed to
	 * get IO TLB memory under CONFIG_SWIOTLB, it is better to allocate
	 * low memory as small as possible before swiotlb_init(), so make
	 * sparse_init() using top-down allocation.
	 */
	memblock_set_bottom_up(false);
	sparse_init();
	memblock_set_bottom_up(true);

	swiotlb_init(true, SWIOTLB_VERBOSE);

	dma_contiguous_reserve(PFN_PHYS(max_low_pfn));

	/* Reserve for hibernation. */
	register_nosave_region(PFN_DOWN(__pa_symbol(&__nosave_begin)),
				   PFN_UP(__pa_symbol(&__nosave_end)));

	memblock_dump_all();

	early_memtest(PFN_PHYS(ARCH_PFN_OFFSET), PFN_PHYS(max_low_pfn));
}

static void __init resource_init(void)
{
	long i = 0;
	size_t res_size;
	struct resource *res;
	struct memblock_region *region;

	code_resource.start = __pa_symbol(&_text);
	code_resource.end = __pa_symbol(&_etext) - 1;
	data_resource.start = __pa_symbol(&_etext);
	data_resource.end = __pa_symbol(&_edata) - 1;
	bss_resource.start = __pa_symbol(&__bss_start);
	bss_resource.end = __pa_symbol(&__bss_stop) - 1;

	num_standard_resources = memblock.memory.cnt;
	res_size = num_standard_resources * sizeof(*standard_resources);
	standard_resources = memblock_alloc(res_size, SMP_CACHE_BYTES);

	for_each_mem_region(region) {
		res = &standard_resources[i++];
		if (!memblock_is_nomap(region)) {
			res->name  = "System RAM";
			res->flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;
			res->start = __pfn_to_phys(memblock_region_memory_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_memory_end_pfn(region)) - 1;
		} else {
			res->name  = "Reserved";
			res->flags = IORESOURCE_MEM;
			res->start = __pfn_to_phys(memblock_region_reserved_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_reserved_end_pfn(region)) - 1;
		}

		request_resource(&iomem_resource, res);

		/*
		 *  We don't know which RAM region contains kernel data,
		 *  so we try it repeatedly and let the resource manager
		 *  test it.
		 */
		request_resource(res, &code_resource);
		request_resource(res, &data_resource);
		request_resource(res, &bss_resource);
	}
}

static int __init add_legacy_isa_io(struct fwnode_handle *fwnode,
				resource_size_t hw_start, resource_size_t size)
{
	int ret = 0;
	unsigned long vaddr;
	struct logic_pio_hwaddr *range;

	range = kzalloc(sizeof(*range), GFP_ATOMIC);
	if (!range)
		return -ENOMEM;

	range->fwnode = fwnode;
	range->size = size = round_up(size, PAGE_SIZE);
	range->hw_start = hw_start;
	range->flags = LOGIC_PIO_CPU_MMIO;

	ret = logic_pio_register_range(range);
	if (ret) {
		kfree(range);
		return ret;
	}

	/* Legacy ISA must placed at the start of PCI_IOBASE */
	if (range->io_start != 0) {
		logic_pio_unregister_range(range);
		kfree(range);
		return -EINVAL;
	}

	vaddr = (unsigned long)(PCI_IOBASE + range->io_start);
	vmap_page_range(vaddr, vaddr + size, hw_start, pgprot_device(PAGE_KERNEL));

	return 0;
}

static __init int arch_reserve_pio_range(void)
{
	struct device_node *np;

	for_each_node_by_name(np, "isa") {
		struct of_range range;
		struct of_range_parser parser;

		pr_info("ISA Bridge: %pOF\n", np);

		if (of_range_parser_init(&parser, np)) {
			pr_info("Failed to parse resources.\n");
			of_node_put(np);
			break;
		}

		for_each_of_range(&parser, &range) {
			switch (range.flags & IORESOURCE_TYPE_BITS) {
			case IORESOURCE_IO:
				pr_info(" IO 0x%016llx..0x%016llx  ->  0x%016llx\n",
					range.cpu_addr,
					range.cpu_addr + range.size - 1,
					range.bus_addr);
				if (add_legacy_isa_io(&np->fwnode, range.cpu_addr, range.size))
					pr_warn("Failed to reserve legacy IO in Logic PIO\n");
				break;
			case IORESOURCE_MEM:
				pr_info(" MEM 0x%016llx..0x%016llx  ->  0x%016llx\n",
					range.cpu_addr,
					range.cpu_addr + range.size - 1,
					range.bus_addr);
				break;
			}
		}
	}

	return 0;
}
arch_initcall(arch_reserve_pio_range);

static int __init reserve_memblock_reserved_regions(void)
{
	u64 i, j;

	for (i = 0; i < num_standard_resources; ++i) {
		struct resource *mem = &standard_resources[i];
		phys_addr_t r_start, r_end, mem_size = resource_size(mem);

		if (!memblock_is_region_reserved(mem->start, mem_size))
			continue;

		for_each_reserved_mem_range(j, &r_start, &r_end) {
			resource_size_t start, end;

			start = max(PFN_PHYS(PFN_DOWN(r_start)), mem->start);
			end = min(PFN_PHYS(PFN_UP(r_end)) - 1, mem->end);

			if (start > mem->end || end < mem->start)
				continue;

			reserve_region_with_split(mem, start, end, "Reserved");
		}
	}

	return 0;
}
arch_initcall(reserve_memblock_reserved_regions);

#ifdef CONFIG_SMP
static void __init prefill_possible_map(void)
{
	int i, possible;

	possible = num_processors + disabled_cpus;
	if (possible > nr_cpu_ids)
		possible = nr_cpu_ids;

	pr_info("SMP: Allowing %d CPUs, %d hotplug CPUs\n",
			possible, max((possible - num_processors), 0));

	for (i = 0; i < possible; i++)
		set_cpu_possible(i, true);
	for (; i < NR_CPUS; i++) {
		set_cpu_present(i, false);
		set_cpu_possible(i, false);
	}

	set_nr_cpu_ids(possible);
}
#endif

#ifdef CONFIG_BLK_DEV_INITRD
static int __init rd_start_early(char *p)
{
	phys_initrd_start = __pa(memparse(p, &p));
	return 0;
}
early_param("rd_start", rd_start_early);

static int __init rd_size_early(char *p)
{
	phys_initrd_size = memparse(p, &p);

	return 0;
}
early_param("rd_size", rd_size_early);
#endif

static int __init bp_start_early(char *p)
{
	bp_start = __pa(memparse(p, &p)); // 这里拿到的是物理地址
	bp_start = TO_CACHE(bp_start); // 再转换成当前可以访问的地址
	return 0;
}
early_param("bp_start", bp_start_early);

void __init setup_arch(char **cmdline_p)
{
	cpu_probe();
	unwind_init();

	init_environ();
	efi_init();
	if (!bp_start_match() && bp_start)
		parse_extlist((struct boot_params*)bp_start);

	fdt_setup();
	memblock_init();
	pagetable_init();
	bootcmdline_init(cmdline_p);
	parse_early_param();
	reserve_initrd_mem();

	platform_init();
	arch_mem_init(cmdline_p);

	resource_init();
	jump_label_init(); /* Initialise the static keys for paravirtualization */

#ifdef CONFIG_SMP
	plat_smp_setup();
	prefill_possible_map();
#endif

	paging_init();

#ifdef CONFIG_KASAN
	kasan_init();
#endif
}

//////////////////////////////////////
/////for bootloader mem map parse/////
//////////////////////////////////////
static __init u8 ext_listhdr_checksum(u8 *buffer, u32 length)
{
	u8 sum = 0;
	u8 *end = buffer + length;

	while (buffer < end) {
		sum = (u8)(sum + *(buffer++));
	}

	return (sum);
}

static __init int parse_mem(struct _extention_list_hdr *head)
{
	struct loongsonlist_mem_map_legacy *ptr;
	static struct loongsonlist_mem_map mem_map;
	int i;

	loongson_mem_map = (struct loongsonlist_mem_map *)head;

	if (ext_listhdr_checksum((u8 *)loongson_mem_map, head->length)) {
		printk("mem checksum error\n");
		return -EPERM;
	}

	ptr = (struct loongsonlist_mem_map_legacy *)head;

	pr_info("convert legacy mem map to new mem map.\n");
	memcpy(&mem_map, ptr, sizeof(mem_map.header));
	mem_map.map_count = ptr->map_count;
	for (i = 0; i < ptr->map_count; i++) {
		mem_map.map[i].mem_type = ptr->map[i].mem_type;
		mem_map.map[i].mem_start = ptr->map[i].mem_start;
		mem_map.map[i].mem_size = ptr->map[i].mem_size;
		pr_info("bootloader memmap block %d type : %d start : %.llx size : %.llx\n",
					i, mem_map.map[i].mem_type, mem_map.map[i].mem_start, mem_map.map[i].mem_size);
	}
	loongson_mem_map = &mem_map;
	return 0;
}

static __init void memmap_bootlaoder_parse(void)
{
	int i;
	u32 mem_type;
	u64 mem_start, mem_end, mem_size;
	/* Parse memory information */
	for (i = 0; i < loongson_mem_map->map_count; i++) {
		mem_type = loongson_mem_map->map[i].mem_type;
		mem_start = loongson_mem_map->map[i].mem_start;
		mem_size = loongson_mem_map->map[i].mem_size;
		mem_end = mem_start + mem_size;

		switch (mem_type) {
		case ADDRESS_TYPE_SYSRAM:
			memblock_add(mem_start, mem_size);
			if (max_low_pfn < (mem_end >> PAGE_SHIFT))
				max_low_pfn = mem_end >> PAGE_SHIFT;
			break;
		}
	}
}

static __init int parse_extlist(struct boot_params *bp)
{
	unsigned long next_offset;
	struct _extention_list_hdr *fhead;

	// 原本有几种转换版本，但是这边就这一个
	fhead = (struct _extention_list_hdr *)bp->extlist_offset;

	if (fhead == NULL) {
		printk("the ext struct is empty!\n");
		return -1;
	}

	do {
		if (memcmp(&(fhead->signature), LOONGSON_MEM_SIGNATURE, 3) == 0) {
			if (parse_mem(fhead) != 0) {
				printk("parse mem failed\n");
				return -EPERM;
			}
			memmap_bootlaoder_parse();
		}

		fhead = (struct _extention_list_hdr *)fhead->next_offset;
		next_offset = (unsigned long)fhead;

	} while (next_offset);

	return 0;
}

static int __init bp_start_match(void)
{
	char* bp_start_value;
	char temp[128]; // 不想申请空间 应该不会有这么长的值吧

	bp_start = 0;
	bp_start_value = strstr(boot_command_line, bp_start_desc);
	if (bp_start_value) {
		int i;
		memset(temp, 0, 128);
		bp_start_value += strlen(bp_start_desc); // 跳过 = 和前面的字段
		for (i = 0; i < 127; ++i) {
			if (bp_start_value[i] == 0 || bp_start_value[i] == ' ')
				break;
			temp[i] = bp_start_value[i];
		}
		bp_start_value = temp;
	} else
		return 1;
	return bp_start_early(bp_start_value);
}

//////////////////////////////////////
/////for bootloader mem map parse/////
//////////////////////////////////////
