# GB mini
A Game Boy emulator on the Pokemon mini.

## Overview
GB mini is a port of the [Peanut GB Emulator](https://github.com/deltabeard/Peanut-GB) to run "on" the Pokemon mini handheld via the [PM2040](https://github.com/zwenergy/PM2040) flash cart.
The emulator actually does **not** on the Pokemon mini itself, but on the RP2040 microcontroller which is on the PM2040 flash cart.

## Button Mapping
The D-Pad and A/B buttons are mapped directly.
The C-Button of the PM is mapped to the Start button of the GB.
The Game Boy Select button is mapped to the Power button of the Pokemon mini.

For turning off the Pokemon mini hold the Power button for roughly 2 seconds.

## Save Games
Save games are not automatically permanently saved.
To save the current SRAM content to the PM2040's Flash (which is automatically restored during the boot) **hold the C-Button for roughly 3 seconds.**

## Rumble
There are two pre-compiled versions, one with rumble support and one without.
No dynamic differentiation is done between the mappers. 
The rumble function of games with the MBC5 mapper are emulated using the Pokemon mini's own rumble function.
For most games you should use the pre-compiled variant without rumble, as otherwise the rumble might be activated while not being intended.

## Audio
For audio emulation the [minigb_apu](https://github.com/deltabeard/Peanut-GB/tree/master/examples/sdl2/minigb_apu) emulation from the Peanut emulator is used.
On the Pokemon mini side, the audio samples are played back back as 5b audio samples.

## Video
The video is downscaled by 0.5x.
Additionally, display rows are cut off from the top and bottom to make it fit on the PM's display.

## Usage
You need a [PM2040](https://github.com/zwenergy/PM2040)-based flash cart.
Download the latest ROM-less base FW UF2-file from the releases tab.
Use the [online patcher tool](https://zwenergy.github.io/PM2040ROMPatch/) to inject a GB ROM.
Hold the BOOT-button of the PM2040 cart and drag'n'drop the patched UF2 file onto the cart.
