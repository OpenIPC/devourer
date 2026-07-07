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

/* Every DeviceConfig-backed DEVOURER_* var -> a populated DeviceConfig.
 * See env_config.cpp for the full mapping table. */
devourer::DeviceConfig devourer_config_from_env();

/* DEVOURER_TX_RATE parsed to a TxMode (unset -> the 6M-legacy default). */
devourer::TxMode devourer_tx_mode_from_env();
