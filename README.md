# PicoDev
A minimalistic development device for PSX

## What does it do?
PicoDev has two stages of operation.
- Booty: Uses a specially crafted stream of bytes to load and run a payload in place of an external rom(pio cheat cart), using a minimal interface: 8 data pins, chip select, read, and reset. This project is built on the work of Nicolas "Pixel" Noble who came up with the Booty concept, and danhans42 who's previous work on booty was used as a reference for designing my implementation here.
- Comms: The device roughly simulates the operation of the FTDI 232h in CPU FIFO mode, allowing communication with the PSX over USB.

## Why?
The project is intended as an entry level device for Playstation development, using a minimal set of components.

## How to get started

## Creating a payload
You can create a payload using [ps1-packer](https://github.com/grumpycoders/pcsx-redux/tree/main/tools/ps1-packer). A pre-compiled version of this tool is included in releases of [PCSX-Redux](https://github.com/grumpycoders/pcsx-redux?tab=readme-ov-file#where).

## Compiling
Compiled using the [Raspberry Pi Pico extension for VS Code](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico), SDK V2.2.0

## Where to get help
If you need help getting started, or run into a problem, come by the psx.dev discord server: https://discord.gg/QByKPpH
