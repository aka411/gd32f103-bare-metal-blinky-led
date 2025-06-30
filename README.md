# gd32f103-bare-metal-blinky-led
A simple bare metal led blinky 
What it Does
At its core, this project simply blinks an onboard LED. On most common GD32F103 development boards (such as the popular "Blue Pill" style boards), this LED is typically connected to GPIO Port Pin. The LED will repeatedly turn on and off with a visible delay, demonstrating basic output control.

## Why Bare-Metal?
The choice of a bare-metal approach is deliberate and offers several key advantages for learning and specific applications:

Deep Understanding: By directly manipulating registers,I have gained an in-depth understanding of how the microcontroller's peripherals (like GPIOs and the Reset and Clock Control - RCC) are configured and operated at their lowest level.

Efficiency: Bare-metal code often results in a smaller memory footprint and faster execution times compared to solutions that include larger abstraction layers, as there's no overhead from additional software layers.

Debugging Skills: This approach has enabled me to have stronger debugging skills to trace problems down to the hardware register level.
