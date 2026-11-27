// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Simon Andreas Frimann Lund <os@safl.dk>

/**
 * A controller is notified of shutdown before it is disabled
 * ==========================================================
 *
 * Opens a controller on a BAR0 in host memory that a thread plays the controller behind, and
 * expects nvme_controller_shutdown_via_bar0() to write CC.SHN and wait for CSTS.SHST, and the
 * three close functions to do so before they clear CC.EN.
 *
 * No device is needed: the registers are host memory.
 */
#define _UPCIE_WITH_NVME
#include <upcie/upcie.h>
#include <pthread.h>

#define CAP_TO 1 ///< CAP.TO in units of 500 ms
#define SHST_DELAY_US 1000
#define POLL_US 50

/**
 * The controller behind BAR0: answers CC.EN with CSTS.RDY and CC.SHN with CSTS.SHST
 *
 * The thread returns right after it writes CSTS.RDY=0 for CC.EN going 1 to 0, so a close may
 * munmap BAR0 once it has read CSTS.RDY=0.
 */
struct emu {
	uint8_t *bar0;
	uint32_t cc;         ///< CC as last seen; main sets it before the thread starts
	int answer_shn;      ///< Whether CC.SHN=normal is answered with CSTS.SHST=complete
	int nwrites;         ///< CC transitions seen
	int shn_seq;         ///< Transition number of the CC.SHN write, 0 if not seen
	uint32_t shn_cc;     ///< CC as written when CC.SHN became normal
	int disable_seq;     ///< Transition number of the CC.EN=0 write, 0 if not seen
	uint32_t disable_cc; ///< CC as written when CC.EN went 1 to 0
};

static void
emu_csts_write(struct emu *emu, uint32_t csts)
{
	mmio_write32(emu->bar0, NVME_REG_CSTS, csts);
}

static void *
emu_run(void *arg)
{
	struct emu *emu = arg;
	uint32_t prev = emu->cc;

	for (;;) {
		uint32_t cc = nvme_mmio_cc_read(emu->bar0);

		if (cc == prev) {
			usleep(POLL_US);
			continue;
		}
		emu->nwrites++;

		if (!nvme_reg_cc_get_en(prev) && nvme_reg_cc_get_en(cc)) {
			emu_csts_write(emu, 0x1);
		}

		if (nvme_reg_cc_get_shn(prev) == NVME_CC_SHN_NONE &&
		    nvme_reg_cc_get_shn(cc) == NVME_CC_SHN_NORMAL) {
			emu->shn_seq = emu->nwrites;
			emu->shn_cc = cc;
			if (emu->answer_shn && nvme_reg_cc_get_en(cc)) {
				usleep(SHST_DELAY_US);
				emu_csts_write(emu, 0x1 | (NVME_CSTS_SHST_COMPLETE << 2));
			}
		}

		if (nvme_reg_cc_get_en(prev) && !nvme_reg_cc_get_en(cc)) {
			emu->disable_seq = emu->nwrites;
			emu->disable_cc = cc;
			emu_csts_write(emu, 0x0);
			return NULL;
		}

		prev = cc;
	}
}

static uint8_t *
bar0_map(void)
{
	uint8_t *bar0 =
		mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

	assert(bar0 != MAP_FAILED);
	// CSTS reads 0, so the reset finds the controller not ready at once
	*(volatile uint64_t *)bar0 = (uint64_t)CAP_TO << 24;

	return bar0;
}

/**
 * Map a BAR0, start the controller behind it and bring it to CC.EN=1, CSTS.RDY=1
 */
static void
setup(struct nvme_controller *ctrlr, struct emu *emu, pthread_t *thread, int answer_shn)
{
	uint64_t cap;
	int err;

	memset(ctrlr, 0, sizeof(*ctrlr));
	memset(emu, 0, sizeof(*emu));
	emu->bar0 = bar0_map();
	emu->cc = nvme_mmio_cc_read(emu->bar0);
	emu->answer_shn = answer_shn;

	err = pthread_create(thread, NULL, emu_run, emu);
	if (err) {
		printf("FAILED: pthread_create(); err(%d)\n", err);
		exit(1);
	}

	err = nvme_controller_reset_via_bar0(ctrlr, emu->bar0, &cap);
	if (err) {
		printf("FAILED: nvme_controller_reset_via_bar0(); err(%d)\n", err);
		exit(1);
	}
	err = nvme_controller_enable_via_bar0(ctrlr, emu->bar0, cap);
	if (err) {
		printf("FAILED: nvme_controller_enable_via_bar0(); err(%d)\n", err);
		exit(1);
	}
}

/**
 * Disable the controller, which ends the thread, then join it and unmap BAR0
 */
static void
teardown(struct nvme_controller *ctrlr, struct emu *emu, pthread_t thread)
{
	int err;

	nvme_mmio_cc_disable(emu->bar0);
	err = nvme_mmio_csts_wait_until_not_ready(emu->bar0, ctrlr->base.timeout_ms);
	if (err) {
		printf("FAILED: teardown: nvme_mmio_csts_wait_until_not_ready(); err(%d)\n", err);
		exit(1);
	}

	pthread_join(thread, NULL);
	munmap(emu->bar0, 0x1000);
}

static int
test_shutdown_normal(void)
{
	struct nvme_controller ctrlr;
	struct emu emu;
	pthread_t thread;
	uint32_t cc, csts;
	int err;

	setup(&ctrlr, &emu, &thread, 1);

	err = nvme_controller_shutdown_via_bar0(&ctrlr, emu.bar0);
	cc = nvme_mmio_cc_read(emu.bar0);
	csts = nvme_mmio_csts_read(emu.bar0);

	teardown(&ctrlr, &emu, thread);

	if (err || nvme_reg_cc_get_shn(cc) != NVME_CC_SHN_NORMAL || !nvme_reg_cc_get_en(cc) ||
	    nvme_reg_csts_get_shst(csts) != NVME_CSTS_SHST_COMPLETE) {
		printf("FAILED: shutdown of an enabled controller; err(%d) cc(0x%08" PRIx32
		       ") csts(0x%08" PRIx32 ")\n",
		       err, cc, csts);
		return 1;
	}

	printf("PASSED: shutdown of an enabled controller; cc(0x%08" PRIx32 ") csts(0x%08" PRIx32
	       ")\n",
	       cc, csts);
	return 0;
}

static int
test_shutdown_unanswered(void)
{
	struct nvme_controller ctrlr;
	struct emu emu;
	pthread_t thread;
	uint32_t before, after;
	int err;

	setup(&ctrlr, &emu, &thread, 0);
	ctrlr.shutdown_timeout_ms = 50;

	before = nvme_mmio_cc_read(emu.bar0);
	err = nvme_controller_shutdown_via_bar0(&ctrlr, emu.bar0);
	after = nvme_mmio_cc_read(emu.bar0);

	teardown(&ctrlr, &emu, thread);

	if (err != -ETIMEDOUT || after != nvme_reg_cc_set_shn(before, NVME_CC_SHN_NORMAL)) {
		printf("FAILED: shutdown of a controller that does not answer; err(%d) "
		       "cc(0x%08" PRIx32 " -> 0x%08" PRIx32 ")\n",
		       err, before, after);
		return 1;
	}

	printf("PASSED: shutdown of a controller that does not answer times out; cc(0x%08" PRIx32
	       ")\n",
	       after);
	return 0;
}

static int
test_shutdown_disabled(void)
{
	struct nvme_controller ctrlr;
	struct emu emu;
	pthread_t thread;
	uint32_t before, after;
	int err;

	setup(&ctrlr, &emu, &thread, 1);

	nvme_mmio_cc_disable(emu.bar0);
	err = nvme_mmio_csts_wait_until_not_ready(emu.bar0, ctrlr.base.timeout_ms);
	assert(!err);
	pthread_join(thread, NULL);

	before = nvme_mmio_cc_read(emu.bar0);
	err = nvme_controller_shutdown_via_bar0(&ctrlr, emu.bar0);
	after = nvme_mmio_cc_read(emu.bar0);
	munmap(emu.bar0, 0x1000);

	if (err || after != before) {
		printf("FAILED: shutdown of a disabled controller; err(%d) cc(0x%08" PRIx32
		       " -> 0x%08" PRIx32 ")\n",
		       err, before, after);
		return 1;
	}

	printf("PASSED: shutdown of a disabled controller writes nothing; cc(0x%08" PRIx32 ")\n",
	       after);
	return 0;
}

static int
test_shutdown_complete(void)
{
	struct nvme_controller ctrlr;
	struct emu emu;
	pthread_t thread;
	uint32_t before, after;
	int err;

	setup(&ctrlr, &emu, &thread, 1);

	nvme_mmio_cc_shutdown(emu.bar0, NVME_CC_SHN_NORMAL);
	err = nvme_mmio_csts_wait_until_shutdown_complete(emu.bar0, ctrlr.shutdown_timeout_ms);
	assert(!err);

	before = nvme_mmio_cc_read(emu.bar0);
	err = nvme_controller_shutdown_via_bar0(&ctrlr, emu.bar0);
	after = nvme_mmio_cc_read(emu.bar0);

	teardown(&ctrlr, &emu, thread);

	if (err || after != before) {
		printf("FAILED: shutdown of a controller whose shutdown completed; err(%d) "
		       "cc(0x%08" PRIx32 " -> 0x%08" PRIx32 ")\n",
		       err, before, after);
		return 1;
	}

	printf("PASSED: shutdown of a controller whose shutdown completed writes nothing; "
	       "cc(0x%08" PRIx32 ")\n",
	       after);
	return 0;
}

static int
test_shutdown_timeout_zero(void)
{
	struct nvme_controller ctrlr;
	struct emu emu;
	pthread_t thread;
	uint32_t cc_before, cc_after, csts_before, csts_after;
	int err;

	setup(&ctrlr, &emu, &thread, 1);
	ctrlr.shutdown_timeout_ms = 0;

	cc_before = nvme_mmio_cc_read(emu.bar0);
	csts_before = nvme_mmio_csts_read(emu.bar0);
	err = nvme_controller_shutdown_via_bar0(&ctrlr, emu.bar0);
	usleep(SHST_DELAY_US * 2);
	cc_after = nvme_mmio_cc_read(emu.bar0);
	csts_after = nvme_mmio_csts_read(emu.bar0);

	teardown(&ctrlr, &emu, thread);

	if (err != -EINVAL || cc_after != cc_before || csts_after != csts_before) {
		printf("FAILED: shutdown with shutdown_timeout_ms of 0; err(%d) cc(0x%08" PRIx32
		       " -> 0x%08" PRIx32 ") csts(0x%08" PRIx32 " -> 0x%08" PRIx32 ")\n",
		       err, cc_before, cc_after, csts_before, csts_after);
		return 1;
	}

	printf("PASSED: shutdown with shutdown_timeout_ms of 0 writes nothing; cc(0x%08" PRIx32
	       ") csts(0x%08" PRIx32 ")\n",
	       cc_after, csts_after);
	return 0;
}

/**
 * Check what the controller saw from a close: CC.SHN=normal with CC.EN=1 first, CC.EN=0 with
 * CC.SHN=0 after, and that the close returned expected_err
 */
static int
check_close(const char *name, struct emu *emu, int err, int expected_err)
{
	printf("  %s: shn write #%d cc(0x%08" PRIx32 "), en=0 write #%d cc(0x%08" PRIx32
	       "), err(%d)\n",
	       name, emu->shn_seq, emu->shn_cc, emu->disable_seq, emu->disable_cc, err);

	if (err != expected_err) {
		printf("FAILED: %s returned err(%d), expected err(%d)\n", name, err, expected_err);
		return 1;
	}
	if (!emu->shn_seq || !emu->disable_seq || emu->shn_seq >= emu->disable_seq) {
		printf("FAILED: %s did not write CC.SHN before CC.EN=0\n", name);
		return 1;
	}
	if (!nvme_reg_cc_get_en(emu->shn_cc)) {
		printf("FAILED: %s wrote CC.SHN with CC.EN=0\n", name);
		return 1;
	}
	if (nvme_reg_cc_get_shn(emu->disable_cc) != NVME_CC_SHN_NONE) {
		printf("FAILED: %s wrote CC.EN=0 with CC.SHN still set\n", name);
		return 1;
	}

	printf("PASSED: %s notifies the controller of shutdown before disabling it\n", name);
	return 0;
}

static int
test_close_uio(void)
{
	struct nvme_dmamem_uio_ctx ctx = {0};
	struct nvme_controller ctrlr;
	struct emu emu;
	pthread_t thread;
	int err;

	setup(&ctrlr, &emu, &thread, 1);

	// pci_func_close() munmaps the region and close()s the fd; close(-1) fails with EBADF,
	// which it ignores
	ctx.bar0_mapped = 1;
	ctrlr.func.bars[0].region = emu.bar0;
	ctrlr.func.bars[0].size = 0x1000;
	ctrlr.func.bars[0].fd = -1;

	err = nvme_controller_close_dmamem_uio(&ctrlr, &ctx, NULL);
	pthread_join(thread, NULL);

	return check_close("nvme_controller_close_dmamem_uio", &emu, err, 0);
}

static int
test_close_vfio(void)
{
	struct nvme_dmamem_vfio_ctx ctx;
	struct nvme_controller ctrlr;
	struct emu emu;
	pthread_t thread;
	int err;

	setup(&ctrlr, &emu, &thread, 1);

	nvme_dmamem_vfio_ctx_init(&ctx);
	ctx.bar0 = emu.bar0;
	ctx.bar0_size = 0x1000;

	err = nvme_controller_close_dmamem_vfio(&ctrlr, &ctx, NULL);
	pthread_join(thread, NULL);

	return check_close("nvme_controller_close_dmamem_vfio", &emu, err, 0);
}

/**
 * A close whose controller never reports CSTS.SHST=complete still clears CC.EN, and returns
 * -ETIMEDOUT as nothing after the shutdown fails
 */
static int
test_close_vfio_unanswered(void)
{
	struct nvme_dmamem_vfio_ctx ctx;
	struct nvme_controller ctrlr;
	struct emu emu;
	pthread_t thread;
	int err;

	setup(&ctrlr, &emu, &thread, 0);
	ctrlr.shutdown_timeout_ms = 50;

	nvme_dmamem_vfio_ctx_init(&ctx);
	ctx.bar0 = emu.bar0;
	ctx.bar0_size = 0x1000;

	err = nvme_controller_close_dmamem_vfio(&ctrlr, &ctx, NULL);
	pthread_join(thread, NULL);

	return check_close("nvme_controller_close_dmamem_vfio (unanswered)", &emu, err,
			   -ETIMEDOUT);
}

static int
test_close_type1(void)
{
	struct nvme_dmamem_type1_ctx ctx;
	struct nvme_controller ctrlr;
	struct emu emu;
	pthread_t thread;
	int err;

	setup(&ctrlr, &emu, &thread, 1);

	nvme_dmamem_type1_ctx_init(&ctx);
	ctx.bar0 = emu.bar0;
	ctx.bar0_size = 0x1000;

	err = nvme_controller_close_dmamem_type1(&ctrlr, &ctx, NULL);
	pthread_join(thread, NULL);

	return check_close("nvme_controller_close_dmamem_type1", &emu, err, 0);
}

int
main(void)
{
	int failed = 0;

	// An assert() aborts before buffered output reaches a pipe; keep the lines before it
	setvbuf(stdout, NULL, _IOLBF, 0);

	failed += test_shutdown_normal();
	failed += test_shutdown_unanswered();
	failed += test_shutdown_disabled();
	failed += test_shutdown_complete();
	failed += test_shutdown_timeout_zero();
	failed += test_close_uio();
	failed += test_close_vfio();
	failed += test_close_vfio_unanswered();
	failed += test_close_type1();

	return failed ? 1 : 0;
}
