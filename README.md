# GB mini
A Game Boy emulator on the Pokemon mini.

## Overview
GB mini is a port of the [Peanut GB Emulator](https://github.com/deltabeard/Peanut-GB) to run "on" the Pokemon mini handheld via the [PM2040](https://github.com/zwenergy/PM2040) flash cart.
The emulator actually does **not** on the Pokemon mini itself, but on the RP2040 microcontroller which is on the PM2040 flash cart.

## Button Mapping
The D-Pad and A/B buttons are mapped directly.
The C-Button of the PM is mapped to the Start button of the GB.
Currently the Select button is not mapped to any PM button.

## Save Games
Save games are not automatically permanently saved.
To save the current SRAM content to the PM2040's Flash (which is automatically restored during the boot) **hold the C-Button for roughly 3 seconds.**

## Rumble
The rumble function of games with the MBC5 mapper are emulated using the Pokemon mini's own rumble function.

## Audio
Currently there is no sound emulation.

## Video
The video is downscaled by 0.5x.
Additionally, display rows are cut off from the top and bottom to make it fit on the PM's display.

## Usage
You need a [PM2040](https://github.com/zwenergy/PM2040)-based flash cart.
Download the latest ROM-less base FW UF2-file from the releases tab.
Use the [online patcher tool](https://zwenergy.github.io/PM2040ROMPatch/) to inject a GB ROM.
Hold the BOOT-button of the PM2040 cart and drag'n'drop the patched UF2 file onto the cart.
