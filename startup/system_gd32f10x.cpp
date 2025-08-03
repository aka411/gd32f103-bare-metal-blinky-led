
/*******************************************************************************
  * Copyright (C) 2025 Akash Joshi
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  * SOFTWARE.
  *******************************************************************************/







/**
  * @file    system_gd32f10x.c
  * @brief   CMSIS Device System Source File for GD32F10x devices
  *
  * This file is a simplified example of system_gd32f10x.c.
  * It demonstrates typical clock and Flash configuration
  * done by SystemInit() during microcontroller startup.
  *
  * This version assumes SystemInit() is called *before* .data/.bss initialization,
  *
  */

#include "includes.h"

#ifdef __cplusplus
extern "C" {
#endif
// Define our target system clock frequency and common oscillator values.
// GD32F103C8T6 commonly runs at 108MHz. We'll set for 104MHz.
#define SYSTEM_CLOCK_108MHZ      (108000000UL)
#define HSI_VALUE                (8000000UL) // Default HSI oscillator value (8MHz)
#define HXTAL_VALUE                (8000000UL) // Assuming an 8MHz external crystal on the board

/* This is the SystemCoreClock variable defined by CMSIS.
   SystemInit() typically calculates and sets this value. */
uint32_t SystemCoreClock = HSI_VALUE; // Start with HSI default, will be updated by SystemInit()



void softDelay(const uint32_t time)
{
  volatile uint32_t count = 0;
  while(count < time)
  {
    count++;
  }
}




/**
 * @brief  Setup the microcontroller system.
 * Initialize the system clock, configure the Flash settings,

 * This function is called automatically at the beginning of program execution.
 * @param  None
 * @retval None
 */
void SystemInit(void)
{

    // 1. Reset the RCU (Reset and Clock Control) configuration to default states
    //    This ensures a clean slate, particularly after a software reset.
    //    It often enables HSI and resets other clock settings.
    RCU->CTL = (uint32_t)0x00000083U; // HSI ON, HSITRIM[4:0] = 16 (default reset value)
    RCU->CFG0 = (uint32_t)0x00000000U; // Reset CFG0
    RCU->INT = (uint32_t)0x009F0000U; // Disable all clock interrupts and clear all pending bits
    RCU->APB2RST = (uint32_t)0x00000000U; // No APB2 peripheral resets
    RCU->APB1RST = (uint32_t)0x00000000U; // No APB1 peripheral resets
    RCU->AHBEN = (uint32_t)0x00000014U; // Enable Flash and SRAM clock (default reset value)
    RCU->APB2EN = (uint32_t)0x00000000U; // Disable APB2 peripherals
    RCU->APB1EN = (uint32_t)0x00000000U; // Disable APB1 peripherals



    // 2. Enable HXTAL (High-Speed External Oscillator)
    //    8MHz crystalin my GD32F103C8T6 board.
    const uint32_t RCU_CTL_HXTAL_EN =(0x01U<<16U) ;
    const uint32_t RCU_CTL_HXTAL_EN_MASK = (0x01U<<16U);

    RCU->CTL &=  ~(RCU_CTL_HXTAL_EN_MASK );
    RCU->CTL |= RCU_CTL_HXTAL_EN; // 8 MHz



    // 3. Wait for HXTAL to be ready (stable)
    //    A timeout loop prevents hanging if HXTAL fails to start.
    volatile uint32_t timeout_counter = 0;

    const uint32_t  RCU_CTL_HXTAL_STB =(0x01U<<17U);
    const uint32_t HXTAL_STARTUP_TIMEOUT = 0x5000U; // A reasonable timeout value

    while (!(RCU->CTL & RCU_CTL_HXTAL_STB) && (timeout_counter < HXTAL_STARTUP_TIMEOUT))
    {
        timeout_counter++;
    }

    // Check if HXTAL started. If not, return
    if (!(RCU->CTL & RCU_CTL_HXTAL_STB))
    {
        // PLL failed  to become system clock . System will remain on HSI (8MHz).
        // In a real application, you might add error handling here (e.g., blink an LED, assert).
        return; // Or go into an infinite loop / error state.
    }












// 4. Set parameters for PLL
//Point PLL to its Source



//17 PREDV0 PREDV0 division factor

//0: PREDV0 input source clock not divided

const uint32_t RCU_CFG0_PREDV_MASK = (0x01U<<17U);
const uint32_t RCU_CFG0_PREDV0_DIV_0 = (0x00U<<17U);

RCU->CFG0 &= ~(RCU_CFG0_PREDV_MASK);
RCU->CFG0 |= RCU_CFG0_PREDV0_DIV_0 ;

    // 5. Configure AHB (HCLK), APB1 (PCLK1), and APB2 (PCLK2) prescalers
    //    Clear existing prescaler settings first.

  //  27 PLLMF[4] Bit 4 of PLLMF
//               see bits 21:18 of RCU_CFG0

  //  21:18 PLLMF[3:0] The PLL clock multiplication factor
  //  16 PLLSEL PLL clock source selection
const uint32_t RCU_CFG0_PLL_SOURCE_HXTAL = (0x01U<<16U); // HXTAL is 8MHz
//Note: The PLL output frequency must not exceed 108 MHz
const uint32_t RCU_CFG0_PLL_MUL_FACTOR_13 = (0x000BU<<18U);

const uint32_t RCU_CFG0_PLL_SOURCE_HXTAL_MASK = (0x01U<<16U);
//Note: The PLL output frequency must not exceed 108 MHz
const uint32_t RCU_CFG0_PLL_MUL_FACTOR_MASK =((0x01U<<27U)|(0x0FU<<18U));


// 8MHz x 13 = 104 MHz;

RCU->CFG0 &= ~(RCU_CFG0_PLL_MUL_FACTOR_MASK  | RCU_CFG0_PLL_SOURCE_HXTAL_MASK);

RCU->CFG0 |= ( RCU_CFG0_PLL_MUL_FACTOR_13 | RCU_CFG0_PLL_SOURCE_HXTAL) ;


//5.Enable PLL and wait till PLL stablises

//bit fields
//24  PLLEN
//25 PLLSTB

const uint32_t RCU_CTL_PLL_EN = (0x01U<<24U);
const uint32_t RCU_CTL_PLL_EN_MASK = (0x01U<<24U);

//Enable PLL

RCU->CTL &= ~(RCU_CTL_PLL_EN_MASK);
RCU->CTL |= RCU_CTL_PLL_EN;

const uint32_t  RCU_CTL_PLL_STB = (0x01U<<25U);
//Now wait for PLL to stablise
// 8. Wait for PLL to be ready (locked and stable)

timeout_counter = 0; // Reset timeout counter
while (!(RCU->CTL & RCU_CTL_PLL_STB) && (timeout_counter < HXTAL_STARTUP_TIMEOUT))
{
    timeout_counter++;
}

// Check if PLL started. If not, system will remain on HSI (8MHz).
if (!(RCU->CTL & RCU_CTL_PLL_STB))
{
    return; // Or error handling.
}





//6. set frequency of the buses , AHB, APB1 , APB2


//13:11 APB2PSC[2:0] APB2 prescaler selection

//10:8 APB1PSC[2:0] APB1 prescaler selection
//7:4  AHBPSC[3:0] AHB prescaler selection

// AHB MAX Freq = 108 MHz.
// APB1 MAX Freq =  60 MHz.
// APB2 MAX Freq =  Not specified.
//Note: The PLL output frequency must not exceed 108 MHz
      const uint32_t  RCU_CFG0_AHB_PRE_SEL_1 = (0U<<4U); // 104 MHz

      const uint32_t  RCU_CFG0_APB1_PRE_SEL_2 =( 0x04U<< 8U); // 104MHz/2 = 52 MHz //100: (CK_AHB / 2) selected
      const uint32_t  RCU_CFG0_APB2_PRE_SEL_2= ( 0x04U<< 11U);//   104MHz/2 = 52 MHz  //100: (CK_AHB / 2) selected

      const uint32_t RCU_CFG0_AHB_PRE_MASK =(0x0FU << 4U);
      const uint32_t RCU_CFG0_APB1_PRE_MASK = (0x07U << 8U);
      const uint32_t RCU_CFG0_APB2_PRE_MASK = (0x07U << 11U);

    RCU->CFG0 &= ~(RCU_CFG0_AHB_PRE_MASK | RCU_CFG0_APB1_PRE_MASK | RCU_CFG0_APB2_PRE_MASK);




    RCU->CFG0 |= (RCU_CFG0_AHB_PRE_SEL_1 |     //  104 MHz (for CPU, SRAM, Flash)
                  RCU_CFG0_APB1_PRE_SEL_2 |    // PCLK1 = HCLK / 2 = 52 MHz (for lower-speed peripherals like UART, I2C, USB, max 54MHz)
                  RCU_CFG0_APB2_PRE_SEL_2);    // PCLK2 = HCLK / 2 = 52 MHz (for higher-speed peripherals like GPIO, ADC, max 108MHz)





  // 7. Configure Flash Latency (Wait States) and Prefetch Buffer
 //    Flash needs more wait states for higher clock frequencies to prevent errors.
//    For 108MHz SYSCLK, 3 wait states are required for GD32F10x.
//    FMC->WS_WSCFG_3WS corresponds to 3 wait states (bits 0-2 of FMC_WS register).


//8.Enable configured wait states

//0 WSEN
//FMC wait state enable register
//This bit is set and reset by software. This bit also protected by the FMC_KEYx
//register. It is necessary to writing 0x45670123 and 0xCDEF89AB to the FMC_KEYx
//register.





FMC->KEY0 = (uint32_t)0x45670123U; // Unlock Bank0 (and related registers)
FMC->KEY0 = (uint32_t)0xCDEF89ABU; // Unlock Bank0 (and related registers)







const uint32_t FMC_CTL0_LK_LOCK = (0x01U<<7U);

softDelay(500);//very important to wait before checking lock state

if(FMC->CTL0 & FMC_CTL0_LK_LOCK)
{
  return; // failed to unlock
}


const uint32_t FMC_WS_MASK = (0x07U<<0U);
const uint32_t FMC_WS_2WS = (0x02U<<0U);


FMC->WS &= ~(FMC_WS_MASK);
FMC->WS |=  FMC_WS_2WS;//dont forget to enable it



const uint32_t FMC_WSEN_EN = (0x01U<<0U);
const uint32_t FMC_WSEN_EN_MASK = (0x01U<<0U);

FMC->WSEN &= ~(FMC_WSEN_EN_MASK);
FMC->WSEN |= FMC_WSEN_EN;

softDelay(500);
//check if enabled
/*
if(!(FMC->WSEN & (0x01U<<0U)))
{
  return;
}
/*
const  uint32_t  isolated_bits2 = (FMC->WS & 0x07U);
if( isolated_bits2 == 0x00U)
{
  return;
}
*/



// 7 LK
const uint32_t FMC_CTL0_LK_EN = (0x01U<<7U);

FMC->CTL0 |=  FMC_CTL0_LK_EN;//lock it
//FMC->CTL1 |=  FMC_CTL0_LK_EN; // lock it back





//9. set CK_SYS source as PLL

//3:2  SCSS[1:0]
//1:0  SCS[1:0]

//10: Select CK_PLL as the CK_SYS source

const uint32_t RCU_CFG0_SCS_MASK = (0x03U<<0U);
const uint32_t RCU_CFG0_SCS_PLL =  (0x02U<<0U);

    //    Clear existing system clock source selection.
    RCU->CFG0 &= ~(RCU_CFG0_SCS_MASK);

    // Set system clock source to PLL.
    RCU->CFG0 |= RCU_CFG0_SCS_PLL;




    // 10. Wait for the System Clock to actually switch to PLL
    const  uint32_t RCU_CFG0_SCSS_PLL = (0x02U << 2U);//status
    timeout_counter = 0; // Reset timeout counter
    while (!(RCU->CFG0 & RCU_CFG0_SCSS_PLL) && (timeout_counter < HXTAL_STARTUP_TIMEOUT))
    {
        timeout_counter++;
    }


if(!(RCU->CFG0 & RCU_CFG0_SCSS_PLL))
{
  return ; //PLL not set as system clock not set
}




    // 11. Update SystemCoreClock variable (CMSIS standard)
     //we need to update SystemCoreClock variable  but after bss and data section initialization



}

#ifdef __cplusplus
}
#endif
