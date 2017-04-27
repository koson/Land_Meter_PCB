
/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Project Includes */
#include "version.h"
#include "common.h"
#include "battery.h"
#include "ad9838.h"

extern uint8_t switch_toggle;


uint8_t TXData[8];        // Pointer to TX data
uint8_t RXData[8];
uint8_t TXByteCtr=0;
uint8_t TXByteXfer=0;
uint8_t RXByteCtr=0;

/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 115200 baud rate.
 */
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        26,                                      // BRDIV = 26
        0,                                       // UCxBRF = 0
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION  // Low Frequency Mode
};

/*
 * USCIA0 Uart interrupt handler.
 */
void EusciA0_ISR(void)
{
    int receiveByte = UCA0RXBUF;

    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Echo back. */
    EUSCI_A_UART_transmitData(EUSCI_A0_MODULE, receiveByte);

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
}



void I2C_Init(void)
{
    // Configure GPIO
    P1OUT &= ~BIT0;                         // Clear P1.0 output latch
    P1DIR |= BIT0;                          // For LED

    // Configure Pins for I2C, P6.5 SCL, P6.4 SCA
    P6SEL0 |= BIT4 | BIT5;                  // I2C pins

    // Configure USCI_B1 for I2C mode
    UCB1CTLW0 |= UCSWRST;                   // Software reset enabled
    UCB1CTLW0 |= UCMODE_3 | UCMST | UCSYNC; // I2C mode, Master mode, sync
    UCB1CTLW0 &= ~UCSSEL_3; 					// I2C mode, CLK SOURCE SMCLK
    UCB1CTLW0 |= UCSSEL_3; 					// I2C mode, CLK SOURCE SMCLK
    UCB1CTLW1 |= UCASTP_2;                  // Automatic stop generated
                                            // after UCB1TBCNT is reached
    UCB1I2CSA = NL2020_ADR;              	// configure slave address
    UCB1BRW = 0x001E;                       // baudrate = SMCLK /30 = 100KHz
}

void USCI_I2C_READ(int cmd, int num)
{
    volatile uint32_t i;

    TXByteXfer = 0;
    TXByteCtr = 0;                                    // Load TX byte counter
    RXByteCtr = 0;
    UCB1TBCNT = num;                     		  // number of bytes to be received

    __enable_interrupt();
    NVIC_ISER0 = 1 << ((INT_EUSCIB1 - 16) & 31); // Enable eUSCIB1 interrupt in NVIC module

    UCB1CTLW0 &=~ UCSWRST;                            // clear reset register
    UCB1IE = UCTXIE0 | UCNACKIE | UCBCNTIE;

    SCB_SCR |= SCB_SCR_SLEEPONEXIT;                   // Don't wake up on exit from ISR

    TXData[TXByteXfer++] = cmd;

    while (UCB1CTLW0 & UCTXSTP);                      // Ensure stop condition got sent
    UCB1CTLW0 |= UCTR | UCTXSTT;                      // I2C TX, start condition

	SCB_SCR |= SCB_SCR_SLEEPONEXIT;     // Don't wake up on exit from ISR

    NVIC_ISER0 = 1 << ((INT_EUSCIB1 - 16) & 31); // Enable eUSCIB1 interrupt in NVIC module

    // Configure USCI_B1 for I2C mode
    UCB1CTLW0 &= ~UCTR;                     // I2C RX, start condition
    UCB1CTLW0 &= ~UCSWRST;
    UCB1IE |= UCRXIE | UCNACKIE | UCBCNTIE;

	while (UCB1CTLW0 & UCTXSTP);         // Ensure stop condition got sent
	UCB1CTLW0 |= UCTXSTT;                // I2C start condition

	while (RXByteCtr < num);			//wait....
}




/**
  * @brief  Translate two bytes into an integer
  * @param
  * @retval The calculation results
  */
unsigned int transBytes2Int(unsigned char msb, unsigned char lsb)
{
    unsigned int tmp;

    tmp = ((msb << 8) & 0xFF00);
    return ((unsigned int)(tmp + lsb) & 0x0000FFFF);
}




void read_batter_and_display(void)
{
    int16_t  AverageCurrent, version;

	/* Read spec infor */
	/*
	 * 	Revision 0...3 4 bit binary value 0001 - Version 1.0 and 1.1
								all other values reserved
		Version  4...7 4 bit binary value 0001 – Version 1.0
										   0010 – Version 1.1
										   0011 - Version 1.1 with optional PEC support
												all other values reserved
		VScale   8...11 4 bit binary value 0 - 3 (multiplies voltages* by 10 ^ VScale)
		IPScale 12...15 4 bit binary value 0 - 3 (multiplies currents* and capacities by 10 ^ IPScale)
	 */

	USCI_I2C_READ(NL2020CMD_SPEC_INFO, 2);
	version = transBytes2Int(RXData[1], RXData[0]);
	printf("Revision: 0x%X,", version & 0xF);
	if ((version & 0xF) == 1)
		printf(" (=> 1.0 or 1.1)");
	else
		printf(" => Unknown");

	printf(";    Version: 0x%X,", (version >> 4) & 0xF);
	if ((version & 0x30) == 0x30)
		printf(" =>1.1 with optional PEC support");
	else if ((version & 0x30) == 0x20)
		printf(" =>1.1 ");
	else if ((version & 0x30) == 0x10)
		printf(" =>1.0");
	else
		printf(" Unknown");

	printf("\r\n");

	/* Read temperature (units = 0.1K) */
	USCI_I2C_READ(NL2020CMD_TEMP, 2);

	/* Convert K to Celsius degree */
	printf("Current Temperature  :%d C\r\n", (transBytes2Int(RXData[1], RXData[0])) / 10 - 273);

	/* Read voltage (units = mV) */
	USCI_I2C_READ(NL2020CMD_VOLT, 2);
	printf("Current Voltage  :%dmV\r\n", transBytes2Int(RXData[1], RXData[0]));

	/* Read AverageCurrent (units = mA) */
	USCI_I2C_READ(NL2020CMD_AVG_CURRENT, 2);
	printf("AverageCurrent  :%dmA\r\n", transBytes2Int(RXData[1], RXData[0]));

	/* Read state of charge (units = %) */
	USCI_I2C_READ(NL2020CMD_RELATIVE_STATE_OF_CHARGE, 2);
	printf("State of Charge :%d%s\r\n", transBytes2Int(RXData[1], RXData[0]), "%");

	/* Read DesignCapacity (units = mAH) */
	USCI_I2C_READ(NL2020CMD_DESIGN_CAPACITY, 2);
	printf("DesignCapacity :%dmAH\r\n", transBytes2Int(RXData[1], RXData[0]));

	/* Read RemainingCapacity (units = mAH) */
	USCI_I2C_READ(NL2020CMD_REMAIN_CAPACITY, 2);
	printf("RemainingCapacity :%dmAH\r\n\r\n", transBytes2Int(RXData[1], RXData[0]));

	/* Read AverageCurrent (units = mA) */
	USCI_I2C_READ(NL2020CMD_AVG_CURRENT, 2);
	AverageCurrent = transBytes2Int(RXData[1], RXData[0]);

	if (AverageCurrent > 0) {
		printf("The battery is charging!\r\n");
	} else {
		printf("The battery is discharging!\r\n");
	}

}


void SystemInit(void)
{
    MAP_WDT_A_holdTimer();										 /* Stop WDT */

    MAP_Interrupt_disableMaster();

    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Setting our MCLK to 48MHz */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_16 );		//MLCK = MCLK/16 = 3MHz
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_16 );		//HSMLCK = MCLK/16 = 3MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_16 );		//SMLCK = MCLK/16 = 3MHz

    /* Configuring GPIO */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    /* Configuring Timer32 to 128000 (1s) of MCLK in periodic mode */
    MAP_Timer32_initModule(TIMER32_0_MODULE, TIMER32_PRESCALER_1, TIMER32_32BIT,
            TIMER32_PERIODIC_MODE);

    /* Selecting P1.2 and P1.3 in UART mode. */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_MODULE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_MODULE);

    I2C_Init();

    MAP_Interrupt_enableInterrupt(INT_PORT1);
    MAP_Interrupt_enableInterrupt(INT_T32_INT1);

    UART_enableInterrupt(EUSCI_A0_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);
	Interrupt_enableMaster();

	Setup_SPI();

}

int main(void)
{

	unsigned int freq=AD9838_OUTPUT_FREQ_MIN;

	SystemInit();

	printf("******************************************************************\r\n");
	printf("      ZLS MSP432, Version %d.%d.%d\r\n",VERSION_MAJOR,VERSION_MINOR,VERSION_RELEASE);
	printf("******************************************************************\r\n");
	printf("\r\n");

	////////   READ BATTERY   //////////////////////////////////////////////
	read_batter_and_display();

	////////   SINE WAVE SETUP AND START //////////////////////////////////////////////
	startCPI(freq, AD9838_OUTPUT_PHASE);

    /* Main while loop */
	while(1)
	{
		if(switch_toggle)
		{
			switch_toggle=FALSE;
			read_batter_and_display();

			if(freq < AD9838_OUTPUT_FREQ_MAX)
			{
				freq += AD9838_OUTPUT_FREQ_INCR;
			}
			else
			{
				freq = AD9838_OUTPUT_FREQ_MIN;
			}
			startCPI(freq, AD9838_OUTPUT_PHASE);

			printf("\r\nPress switch 1 for new reading\r\n");

		}
		MAP_PCM_gotoLPM0();
	}
}

