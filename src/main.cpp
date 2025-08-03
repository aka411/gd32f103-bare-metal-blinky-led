#include "includes.h" //my custom driver include
#include "utils.h"



int main ()
{

  /* 1. Enable GPIOA Bus Clock */

//  2 PAEN  GPIO port A clock enable
//This bit is set and reset by software.
//0: Disabled GPIO port A clock
//1: Enabled GPIO port A clock

//The gpio A port is on APB2, so we are going to enable APB2 bus clock
//we turn it on by setting the 2nd bit in APB2EN register in RCU
const uint32_t RCU_APB2EN_GPIOA_EN = (0x01U<<2U);
RCU->APB2EN |= RCU_APB2EN_GPIOA_EN; // Enabled GPIO port A Clock


GPIO_TypeDef* const GPIOA = (GPIO_TypeDef*)(GPIOA_BASE);//casting GPIO A base address as a pointer for type GPIO_TypeDef

//set Pin 0 of Port A as push-pull  in Output mode(50MHz)
GPIOA->CTL0  &= ~(GPIO_CTL0_PIN_0_CB_MASK|GPIO_CTL0_PIN_0_MD_MASK );
GPIOA->CTL0 |= (GPIO_CTL0_PIN_0_CB_GPIO_PUSH_PULL| GPIO_CTL0_PIN_0_MD_OUTPUT_50HZ  );


const uint32_t GPIO_OCTL_PIN_0_SET= (0x01<<0U);

while(1) //loop forever till something breaks
{
  //turn on or off pin 0
GPIOA->OCTL ^= GPIO_OCTL_PIN_0_SET;

softDelay(1000000);//wait for sometime

}


return -1;
}
