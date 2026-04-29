# nRF9161dk 5G Dect NR+ Node

> **Note:** This repository contains a partial implementation of a larger system. It is specifically designed to run on an nRF9161dk and must be connected to its companion board, the Teensy 4.1 (running the `teensy_eth_spi_bridge` firmware), to function correctly.

## Overview
This Zephyr RTOS project serves as the 5G/WiFi gateway node in a high-speed, bidirectional SPI-to-Network bridge. The end goal is to route data from an Ethernet network seamlessly over a cellular/wireless connection using a SPI bridge.

## Current Status

* **SPI Implementation:** Utilizes non-blocking DMA for high-throughput SPI transfers. Bidirectional communication is coordinated using a dedicated Data Ready (DRDY) interrupt line to signal transmission intent.
* **Functionality:** Inputting up to 32 characters into the console will send them to the Teensy41 over SPI

## Hardware Setup
Connect the nRF9161 DK to the Teensy 4.1 using the following pinout. Ensure that both boards share a common ground.

| Signal | nRF9161 DK Pin | Teensy 4.1 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **DRDY** | P0.09 | 14 | Data Ready Interrupt |
| **CS** | P0.13 | 10 | Chip Select |
| **MOSI** | P0.11 | 11 | Master Out Slave In |
| **MISO** | P0.12 | 12 | Master In Slave Out |
| **SCK** | P0.10 | 13 | Serial Clock |
| **GND** | GND | GND | **Required** |

## Console Setup
Baud rate: 115200 \
Line ending: None

## Building and Flashing
To build and flash this firmware using Zephyr's `west` tool:

```bash
# Build for nRF9161 DK
west build -p always -b nrf9161dk/nrf9161/ns

# Flash to the board
west flash