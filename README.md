# Bare-Metal Blinky LED for GD32F103C8T6

A simple yet powerful project that demonstrates direct hardware control on the GD32F103C8T6 microcontroller. This project implements a classic "blinky" LED program by directly manipulating the microcontroller's GPIO and RCC registers, completely bypassing a hardware abstraction layer (HAL).

This bare-metal approach offers a direct path to understanding the low-level mechanics of the GD32F103C8T6. A key part of this project involved configuring the Reset and Clock Control (RCC) peripheral to operate the microcontroller at a custom speed of 104 MHz, allowing me to gain an in-depth understanding of the MCU's clock tree.
![Blinking LED](blinky_led.gif)


## GD32F103C8T6 Microcontroller Specifications
* **Core & Architecture**
    * **Core:** Cortex®-M3
    * **Series:** GD32F103
    * **Package:** LQFP48
    * **Max Speed:** 108 MHz
    * **Operating Speed:** 104 MHz (Custom configured)

* **Memory**
    * **Flash:** 64 KB
    * **SRAM:** 20 KB

* **Peripherals**
    * **I/O Pins:** up to 37
    * **Timers:** 3x 16-bit GPTM, 1x 16-bit Advanced TM, 1x 24-bit SysTick
    * **Watchdogs:** 2
    * **RTC:** 1
    * **Communication:**
        * **USART:** 3
        * **I2C:** 2
        * **SPI:** 2
        * **CAN:** 1 (CAN 2.0B)
        * **USB:** 1 (2.0 Full-speed)
    * **ADC:** 2 units (10 channels, 12-bit)

 ## Resources

* **[GD32F103xx Datasheet](https://www.gigadevice.com.cn/Public/Uploads/uploadfile/files/20250315/GD32F103xxDatasheet_Rev3.1.pdf)**: Provides electrical characteristics, pin assignments,memory map, and a high-level overview of the MCU.
* **[GD32F10x User Manual](https://www.gigadevice.com.cn/Public/Uploads/uploadfile/files/20250317/GD32F10x_User_Manual_Rev2.9.pdf)**: A comprehensive reference manual for all peripherals, including detailed register descriptions for the RCC and GPIO modules.
