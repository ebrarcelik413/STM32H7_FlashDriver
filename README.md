\# STM32H7 Flash Driver



This repository contains the embedded software architecture developed during my internship at DASAL Aerospace using the STM32H747 microcontroller.



The project evolved from a basic internal Flash driver into a modular embedded software architecture containing:



\- Flash Driver

\- UART Driver

\- Protocol Parser

\- AppMain Orchestrator



\## Architecture



\### Flash Driver



The Flash Driver manages the STM32H747 internal Flash memory.



Key points:



\- STM32H747 internal Flash

\- Bank 2, Sector 7

\- Storage start address: `0x081E0000`

\- Sector size: 128 KB

\- 32-byte FlashWord programming

\- Alignment and ECC constraints

\- Interrupt-based erase/program flow

\- Non-blocking state-machine based operation



\### UART Driver



The UART Driver provides asynchronous communication functionality.



Features include:



\- Interrupt-based UART reception

\- DMA-based UART transmission

\- Polling transmission/reception support

\- TX/RX busy state management

\- HAL callback integration



\### Protocol Parser



The Protocol Parser processes incoming UART packets and separates communication logic from application-level behavior.



The protocol structure is based on:



```text

Header | Command ID | Length | Data | Checksum

