#ifndef __AD9838
#define __AD9838

#define AD9838_CTRL_CMD_LEN				2
#define AD9838_FREQ_CMD1_LEN			2
#define AD9838_FREQ_CMD2_LEN			2
#define AD9838_PHASE_CMD_LEN			2
#define AD9838_EXIT_CMD_LEN				2

#define AD9838_FREQ_EN					0x4000
#define AD9838_PHASE_EN					0xC000

#define AD9838_OUTPUT_FREQ				10000
#define AD9838_OUTPUT_FREQ_MIN			10000
#define AD9838_OUTPUT_FREQ_MAX			200000
#define AD9838_OUTPUT_FREQ_INCR			10000

#define AD9838_OUTPUT_PHASE				0
#define AD9838_2E28						0x10000000
#define AD9838_MCLK						16000000
#define AD9838_PHASE_MULT				4096
#define AD9838_PHASE_DIV				360

#define AD9838_FREQ_BIT_SIZE			28
#define AD9838_FREQ_BIT_MASK			0x3FFF
#define AD9838_PHASE_BIT_MASK			0x3FFF

#define DELAY							1200


void sendSPIdata(uint8_t TXDataArray[], uint8_t Length);
void startCPI(uint32_t freq, uint32_t phase);
void Setup_SPI(void);

#endif
