# nRF9161 DECT NR+ TDMA Bridge

> **Note:** This repository contains a partial implementation of a larger system. It is designed to run on an nRF9161 DK and must be connected to its companion board, a Teensy 4.1 (running the `teensy_eth_spi_bridge` firmware), to function correctly. Two nRF9161 DKs are needed, one master and one slave.

## Overview

This nRF Connect SDK project is the DECT NR+ radio node of a bidirectional Ethernet-to-SPI bridge. Each board takes Ethernet frames from its Teensy over SPI, sends them to the peer board over a fixed TDMA schedule on the DECT NR+ PHY, and hands frames received from the peer back to the Teensy. Both boards run the same image; only the role differs.

* **SPI:** The nRF is the SPI slave and raises **REQ** when it has a frame for the Teensy. Each transfer is a length exchange followed by a data exchange, each gated by **RDY**. The frame format is in [src/protocol.h](src/protocol.h).
* **Payload:** One Ethernet frame is sent per burst, so the payload is limited to the transport block minus the 30-byte header: 377 bytes at `CONFIG_MCS=4` and 5 subslots. Larger frames are dropped and counted as `oversize`.
* **Time base:** The master outputs a 1 PPS reference on COEX1. All nRF timestamps are nanoseconds since the PPS edge.
* **Diagnostics:** A statistics block is logged every 5 s on the VCOM UART.

### TDMA schedule

Master and slave each send one burst per cycle, half a cycle apart, and open an RX window around the peer's burst. The master fixes the grid at boot; the slave acquires it from the master's bursts and re-acquires after losing sync.

The schedule is defined by these constants at the top of [src/dect.c](src/dect.c):

| Constant | Meaning |
| :--- | :--- |
| `CYCLE_SUBSLOTS` | Cycle length in subslots (16 = 3.33 ms) |
| `SUBSLOT_INDEX` | Burst length, `SUBSLOT_INDEX + 1` subslots. Together with `CONFIG_MCS` it sets the payload size |
| `GUARD` | RX window margin on each side of the peer's burst |
| `SLAVE_TX_OFFSET` | Slave TX offset within the cycle (half a cycle) |
| `TX_LEAD_TICKS` | How long before a burst the TX is submitted. Found on hardware, read the comment before changing it |
| `MAX_COAST_CYCLES` | Cycles without a master burst before the slave re-acquires |
| `SYNC_ACCEPT_TICKS` | The slave rejects a sync measurement further than this from its prediction |

Build asserts check that the MCS and burst combination exists and that the modem has enough time to switch between RX and TX.

## Configuration

| Option | Description |
| :--- | :--- |
| `CONFIG_TDMA_ROLE_MASTER` / `CONFIG_TDMA_ROLE_SLAVE` | Role of the image. The master is the default; [slave.conf](slave.conf) selects the slave. |
| `CONFIG_CARRIER` | DECT NR+ carrier, no default. Set by [overlay-eu.conf](overlay-eu.conf) and [overlay-us.conf](overlay-us.conf). |
| `CONFIG_NETWORK_ID` | Network ID. Default `91`. |
| `CONFIG_MCS` | Modulation and coding scheme. [prj.conf](prj.conf) uses `4`. |
| `CONFIG_TX_POWER` | Transmission power index, `0` to `13`. Default `13`. |

## Hardware Setup

Connect each nRF9161 DK to its Teensy 4.1 using the following pinout. Ensure that both boards share a common ground.

| Signal | nRF9161 DK Pin | Teensy 4.1 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **RDY** | P0.08 | 14 | Data Ready interrupt (nRF → Teensy) |
| **REQ** | P0.09 | 41 | Data Request interrupt (nRF → Teensy) |
| **PPS** | COEX1 | 25 | 1 PPS output of the master nRF; the master's Teensy uses it as its time reference |
| **CSN** | P0.13 | 10 | Chip Select |
| **MOSI** | P0.11 | 11 | Master Out Slave In |
| **MISO** | P0.12 | 12 | Master In Slave Out |
| **SCK** | P0.10 | 13 | Serial Clock |
| **GND** | GND | GND | **Required** |

## Console

Logs are printed over the VCOM UART of the DK.

## Building and Flashing

Built with nRF Connect SDK v3.4.0 for `nrf9161dk/nrf9161/ns`, using the nRF Connect for VS Code extension. It requires the DECT NR+ PHY modem firmware from Nordic Semiconductor.

Create two build configurations (sysbuild enabled) from the same source, one per role:

| Build | Directory | Kconfig fragments |
| :--- | :--- | :--- |
| Master | `build_m` | `overlay-eu.conf` |
| Slave | `build_s` | `overlay-eu.conf`, `slave.conf` |

Leave the base configuration files and devicetree overlays empty: `prj.conf` and [boards/nrf9161dk_nrf9161_ns.overlay](boards/nrf9161dk_nrf9161_ns.overlay) are picked up automatically.

The equivalent command line:

```bash
west build -p -d build_m -b nrf9161dk/nrf9161/ns -- -DEXTRA_CONF_FILE="overlay-eu.conf"
west build -p -d build_s -b nrf9161dk/nrf9161/ns -- -DEXTRA_CONF_FILE="overlay-eu.conf;slave.conf"
```

Flash with `west flash -d build_m` or `-d build_s`, or from the extension. Power up the master first, then the slave.

## Testing

1. Flash one board as the master and one as the slave, and open a serial terminal on each.
2. The slave logs `acquiring...`, then `locked on master` and `acquired, starting grid`.
3. Connect the Teensys and send frames that fit the payload limit.
4. Check the statistics block of each board:
   * `submit_err` and `last_err` stay at zero.
   * On the slave, `outliers` does not grow and `coast` stays low.
   * `oversize`, `len_bad`, `nobuf`, `qfull`, `len_err`, `xfer_err` and `hdr_mm` do not grow.
