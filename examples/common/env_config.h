#pragma once

/* DEVOURER_* environment-variable interface of the example binaries.
 *
 * The library is configured through devourer::DeviceConfig (see
 * src/DeviceConfig.h) and runtime setters on IRtlDevice; it reads no env vars.
 * The demos — and the test scripts driving them — speak env vars, and this
 * translator is where that mapping lives: every library-level DEVOURER_* var
 * becomes a DeviceConfig field (devourer_config_from_env). Demo-local vars
 * (DEVOURER_PID, DEVOURER_CHANNEL, ...) stay in each demo's own code. */

#include "DeviceConfig.h"
#include "TxMode.h"
#include "logger.h"

/* Every DeviceConfig-backed DEVOURER_* var -> a populated DeviceConfig.
 * See env_config.cpp for the full mapping table. */
devourer::DeviceConfig devourer_config_from_env();

/* DEVOURER_TX_RATE parsed to a TxMode (unset -> the 6M-legacy default). */
devourer::TxMode devourer_tx_mode_from_env();

/* Logging env -> Logger configuration (docs/logging.md). Call once at
 * main() start, before any threads. These configure the Logger object, not
 * DeviceConfig — the library itself still reads no env:
 *   DEVOURER_LOG_LEVEL=trace|debug|info|warn|error|silent
 *       diagnostic verbosity on stderr (default debug).
 *   DEVOURER_EVENTS=stdout|stderr|off
 *       destination of the JSONL machine event stream (default stdout).
 *   DEVOURER_EVENT_FLUSH=0
 *       drop the per-event fflush (max-rate benches; default flush-per-line
 *       so piped consumers never stall on libc buffering). */
void apply_logging_env(Logger &logger);
