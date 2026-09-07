/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * FIELD_PREP/FIELD_GET rest on MT_CTZ, which replaced __builtin_ctz so the
 * expression stays constant-foldable on compilers that lack that builtin.
 * This checks the replacement against the builtin over every mask the driver
 * can form - all 32 single-bit masks and all 528 contiguous GENMASK(h, l)
 * ranges - rather than over the handful the code happens to use today.
 *
 * It also asserts that MT_CTZ is usable where the builtin was: in a static
 * initialiser. That is the property that ruled out MSVC's _BitScanForward,
 * and a runtime-only check would not notice its loss.
 */
#include <stdio.h>
#include "internal.h"

/* If MT_CTZ ever stops being a constant expression, this fails to compile. */
static const uint32_t constant_folded[] = {
	FIELD_PREP(GENMASK(15, 0), 0x2004),
	FIELD_PREP(GENMASK(25, 20), 0x3f),
	FIELD_PREP(BIT(26), 1),
	FIELD_PREP(GENMASK(19, 18), 1),
};

int main(void)
{
	unsigned checked = 0;
	int fails = 0;

	for (unsigned high = 0; high < 32; high++) {
		for (unsigned low = 0; low <= high; low++) {
			uint32_t mask = GENMASK(high, low);
			unsigned want = (unsigned)__builtin_ctz(mask);
			unsigned got = MT_CTZ(mask);

			checked++;
			if (got != want) {
				printf("  FAIL MT_CTZ(0x%08x) want %u got %u\n",
				       mask, want, got);
				if (++fails > 8) return 1;
			}
		}
	}

	/* Round-trip: what FIELD_PREP writes, FIELD_GET must read back. */
	for (unsigned high = 0; high < 32; high++) {
		for (unsigned low = 0; low <= high; low++) {
			uint32_t mask = GENMASK(high, low);
			uint32_t width = high - low + 1;
			uint32_t value = (width >= 32 ? 0xffffffffu
			                              : (1u << width) - 1u) & 0xa5a5a5a5u;

			if (FIELD_GET(mask, FIELD_PREP(mask, value)) != value) {
				printf("  FAIL round-trip mask 0x%08x value 0x%08x\n",
				       mask, value);
				if (++fails > 8) return 1;
			}
		}
	}

	if (constant_folded[0] != 0x2004u || constant_folded[2] != 0x04000000u) {
		printf("  FAIL static initialiser values\n");
		fails++;
	}

	printf("field_macros: %u masks checked, %s\n", checked,
	       fails ? "FAIL" : "PASS");
	return fails ? 1 : 0;
}
