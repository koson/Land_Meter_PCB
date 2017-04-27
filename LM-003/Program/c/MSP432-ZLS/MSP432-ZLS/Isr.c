
/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Project Includes */
#include "common.h"

extern uint8_t TXData[];        // Pointer to TX data
extern uint8_t RXData[];
extern uint8_t TXByteCtr;
extern uint8_t TXByteXfer;
extern uint8_t RXByteCtr;


uint8_t switch_toggle=FALSE;


/* GPIO ISR */
void gpio_isr(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1,status);

    if (GPIO_PIN1 & status)
    {
        MAP_GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        MAP_Timer32_setCount(TIMER32_0_MODULE,1280000);
        MAP_Timer32_enableInterrupt(TIMER32_0_MODULE);
        MAP_Timer32_startTimer(TIMER32_0_MODULE, true);
    }

    switch_toggle=TRUE;

}

/* Timer32 ISR */
void timer32_isr(void)
{
    MAP_Timer32_clearInterruptFlag(TIMER32_0_MODULE);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
}

// I2C interrupt service routine
void eUSCIB1IsrHandler(void)
{
    if (UCB1IFG & UCNACKIFG)
    {
    	UCB1IFG &= ~ UCNACKIFG;
        UCB1CTL1 |= UCTXSTT;                  // I2C start condition
    }
    if (UCB1IFG & UCTXIFG0)
    {
    	UCB1IFG &= ~ UCTXIFG0;
        if (TXByteCtr < TXByteXfer)                                // Check TX byte counter
        {
            UCB1TXBUF = TXData[TXByteCtr];            // Load TX buffer
            TXByteCtr++;                              // Decrement TX byte counter
        }
        else
        {
            UCB1CTLW0 |= UCTXSTP;                     // I2C stop condition
            UCB1IFG &= ~UCTXIFG;                      // Clear USCI_B1 TX int flag
            SCB_SCR &= ~SCB_SCR_SLEEPONEXIT;          // Wake up on exit from ISR
        }
    }

    if (UCB1IFG & UCRXIFG0)
    {
    	UCB1IFG &= ~ UCRXIFG0;
    	SCB_SCR &= ~SCB_SCR_SLEEPONEXIT;      // Wake up on exit from ISR
        RXData[RXByteCtr++] = UCB1RXBUF;                   // Get RX data
    }

    if (UCB1IFG & UCBCNTIFG)
    {
    	UCB1IFG &= ~ UCBCNTIFG;
    	SCB_SCR |= SCB_SCR_SLEEPONEXIT;     // Don't wake up on exit from ISR
        P1OUT ^= BIT0;                        // Toggle LED on P1.0
    }

}


void euscia3_isr(void)
{
    uint32_t status = SPI_getEnabledInterruptStatus(EUSCI_A3_MODULE);

    SPI_clearInterruptFlag(EUSCI_A3_MODULE, status);
    if(status & EUSCI_A_SPI_RECEIVE_INTERRUPT)
    {
        /* USCI_A3 TX buffer ready? */
        while (!(SPI_getInterruptStatus(EUSCI_A3_MODULE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)));
    }
}

