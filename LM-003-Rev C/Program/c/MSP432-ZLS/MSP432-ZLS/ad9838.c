
/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Project Includes */
#include "ad9838.h"

/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig =
{
		EUSCI_A_SPI_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
		48000000,									// SMCLK = 48MHz
		1000000,										// SPICLK = 1MHz
        EUSCI_A_SPI_MSB_FIRST,                     // MSB First
        EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,    // Phase
//		EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
//        EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW, // High polarity
		EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,
        EUSCI_A_SPI_3PIN                           // 3Wire SPI Mode
};


void startCPI(uint32_t freq, uint32_t phase)
{
	uint8_t TXCFreqCmd2[AD9838_FREQ_CMD1_LEN]  		= {0x4F, 0x5C};
	uint8_t TXCFreqCmd1[AD9838_FREQ_CMD2_LEN]  		= {0x40, 0x0A};
	uint8_t TXPhaseCmd[AD9838_PHASE_CMD_LEN]  		= {0xC0, 0x72};
	uint8_t TXCtrlCmd[AD9838_CTRL_CMD_LEN]  		= {0x21, 0x00};
	uint8_t TXCExitCmd[AD9838_EXIT_CMD_LEN]  		= {0x20, 0x00};
	uint32_t freq_reg;
	uint32_t phase_reg;
	uint8_t i;

	//Calculate Frequency Register in decimal:
	//FreqReg = ROUND((Output_Freq * 2^28) / (MCLK_Freq))  //2^28=0x10000000
	//FreqReg = 167772 or 0x28F5C for 10KHz
	//    Freq_EN (01) + LSB Frequency Register: 4F 5C
	//    Freq_EN (01) + MSB Frequency Register: 40 0A
	freq_reg = (uint32_t) ((((double) freq * AD9838_2E28) / AD9838_MCLK) + 0.5);

	//Calculate Phase Register in decimal:
	//PhaseReg = ROUND(Phase_Deg * 4096) / (360)
	//PhaseReg = 114 or 0x72 for 10 deg
	//    Phase_EN (11) + Phase Register: C0 72
	phase_reg = (uint32_t) ((((double) phase * AD9838_PHASE_MULT) / AD9838_PHASE_DIV) + 0.5);

	sendSPIdata(TXCExitCmd, AD9838_EXIT_CMD_LEN);
	__delay_cycles(DELAY);

	sendSPIdata(TXCtrlCmd, AD9838_CTRL_CMD_LEN);
	__delay_cycles(DELAY);

	i=0;
	TXCFreqCmd2[i++] = (AD9838_FREQ_EN | (freq_reg & AD9838_FREQ_BIT_MASK)) >> 8;				//MSB
	TXCFreqCmd2[i++] = (AD9838_FREQ_EN | (freq_reg & AD9838_FREQ_BIT_MASK)) & 0xFF;				//LSB
	sendSPIdata(TXCFreqCmd2, AD9838_FREQ_CMD2_LEN);
	__delay_cycles(DELAY);

	i=0;
	TXCFreqCmd1[i++] = (AD9838_FREQ_EN | (freq_reg >> (AD9838_FREQ_BIT_SIZE >> 1))) >> 8;		//MSB
	TXCFreqCmd1[i++] = (AD9838_FREQ_EN | (freq_reg >> (AD9838_FREQ_BIT_SIZE >> 1))) & 0xFF;		//LSB
	sendSPIdata(TXCFreqCmd1, AD9838_FREQ_CMD1_LEN);
	__delay_cycles(DELAY);

	i=0;
	TXPhaseCmd[i++] = (AD9838_PHASE_EN | (phase_reg & AD9838_PHASE_BIT_MASK)) >> 8;				//MSB
	TXPhaseCmd[i++] = (AD9838_PHASE_EN | (phase_reg & AD9838_PHASE_BIT_MASK)) & 0xFF;			//LSB
	sendSPIdata(TXPhaseCmd, AD9838_PHASE_CMD_LEN);
	__delay_cycles(DELAY);

	sendSPIdata(TXCExitCmd, AD9838_EXIT_CMD_LEN);
	__delay_cycles(DELAY);

	printf("\r\nFreq: %d Hz, Phase: %d degree\r\n\r\n", freq, phase);

}

void sendSPIdata(uint8_t TXDataArray[], uint8_t Length)
{
	uint8_t TXData;
	uint8_t x;

	/* Polling to see if the TX buffer is ready */
	while (!(SPI_getInterruptStatus(EUSCI_A3_MODULE,EUSCI_A_SPI_TRANSMIT_INTERRUPT)));

	for (x = 0; x < Length; x++)
	{
	   TXData = TXDataArray[x];
	   MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN4);
	   /* Transmitting data to slave */
	   SPI_transmitData(EUSCI_A3_MODULE, TXData);
	}
	while(SPI_isBusy(EUSCI_A3_MODULE));
	__delay_cycles(DELAY);

	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN4);
}


void Setup_SPI(void)
{

//	 *             |             P9.4|-> FSYN (CUCA3STE)
//	 *             |             P9.7|-> Data Out (UCA3SIMO)
//	 *             |             P9.5|-> Serial Clock Out (UCA3CLK)

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P9, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P9,
            GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring GPIO outputs for AD9838 */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN4);


    /* Configuring SPI in 3wire master mode */
    SPI_initMaster(EUSCI_A3_MODULE, &spiMasterConfig);

    /* Enable SPI module */
    SPI_enableModule(EUSCI_A3_MODULE);

    /* Enabling interrupts */
    Interrupt_enableInterrupt(INT_EUSCIA3);
    Interrupt_enableSleepOnIsrExit();

    /* Delaying waiting for the module to initialize */
	   __delay_cycles(DELAY);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN4);

}


