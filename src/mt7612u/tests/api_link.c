/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Link check for the public API. It includes only <mt7612u/mt7612u.h> - no
 * internal header - and takes the address of every function the header
 * declares, so the link fails if any of them is declared without a
 * definition. It touches no hardware and is never run; building it is the
 * whole test.
 *
 * This exists because the header once declared nine functions that had no
 * definition anywhere, which made the "standalone library" claim false while
 * the bring-up tool still built and ran - the tool calls internals directly.
 */
#include <mt7612u/mt7612u.h>
#include <stdio.h>

static void *const api[] = {
	(void *)mt7612u_open,
	(void *)mt7612u_open_handle,
	(void *)mt7612u_close,
	(void *)mt7612u_keep_detached,
	(void *)mt7612u_set_channel,
	(void *)mt7612u_set_txpower,
	(void *)mt7612u_set_chainmask,
	(void *)mt7612u_start,
	(void *)mt7612u_stop,
	(void *)mt7612u_tx,
	(void *)mt7612u_rx_start,
	(void *)mt7612u_rx_stop,
	(void *)mt7612u_set_monitor_rx,
	(void *)mt7612u_send_packet,
	(void *)mt7612u_send_packets,
	(void *)mt7612u_set_ack_responder,
	(void *)mt7612u_clear_ack_responder,
	(void *)mt7612u_get_stats,
	(void *)mt7612u_link_stats_start,
	(void *)mt7612u_link_stats,
	(void *)mt7612u_read_tsf,
	(void *)mt7612u_write_tsf,
	(void *)mt7612u_get_caps,
	(void *)mt7612u_asic_version,
	(void *)mt7612u_mac_addr,
};

int main(void)
{
	size_t n = sizeof api / sizeof api[0];

	for (size_t i = 0; i < n; i++)
		if (!api[i]) return 1;
	printf("api_link: %zu public entry points resolved\n", n);
	return 0;
}
