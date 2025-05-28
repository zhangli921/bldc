/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Marcos Chaparro	mchaparro@powerdesigns.ca
	Copyright 2022 Jakub Tomczak

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "enc_icmhm.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "spi_bb.h"
#include "timer.h"

#include <math.h>
#include <string.h>

#undef HW_SPI_DEV
//#define MT6816_NO_MAGNET_ERROR_MASK		0x0002

static volatile int32_t st_val_previous = 0; //zhang li
static volatile int32_t mt_val_accumulate = 0; //zhang li

static ICMHM_config_t *s_cfg;
static void mhm_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint8_t datasize);
// ic-MHM
static void mhm_activate(const uint8_t *active_vector, uint8_t vector_size);
static void mhm_postition_read(uint8_t *data_rx, uint8_t datasize);
static void mhm_register_read_continuous(uint8_t addr, uint8_t *data_rx, uint8_t datasize);
static void mhm_register_write_continuous(uint8_t addr, const uint8_t *data_tx, uint8_t datasize);
static void mhm_read_status(uint8_t stat[4]);
static void mhm_write_instruction(const uint8_t inst[3]);

bool enc_icmhm_init(ICMHM_config_t *cfg) {
	if (cfg->spi_dev == NULL) {
		return false;
	}

	memset(&cfg->state, 0, sizeof(ICMHM_state));

#ifdef HW_SPI_DEV
	palSetPadMode(cfg->sck_gpio, cfg->sck_pin,
			PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(cfg->miso_gpio, cfg->miso_pin,
			PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(cfg->nss_gpio, cfg->nss_pin,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(cfg->mosi_gpio, cfg->mosi_pin,
			PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST);

	//spiStart(cfg->spi_dev, &(cfg->hw_spi_cfg));
	SPIDriver *spip = cfg->spi_dev;
	/* SPI setup and enable.*/
	rccEnableSPI1(TRUE);
	spip->spi->CR1  = 0;
	spip->spi->CR1  = spip->config->cr1 | SPI_CR1_MSTR | SPI_CR1_SSM |
						SPI_CR1_SSI;
	spip->spi->CR2  = SPI_CR2_SSOE;
	spip->spi->CR1 |= SPI_CR1_SPE;
#else
	palSetPadMode(cfg->miso_gpio, cfg->miso_pin, PAL_MODE_INPUT);
	palSetPadMode(cfg->sck_gpio, cfg->sck_pin, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(cfg->nss_gpio, cfg->nss_pin, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// Set MOSI to 1
	palSetPadMode(cfg->mosi_gpio, cfg->mosi_pin, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(cfg->mosi_gpio, cfg->mosi_pin);
#endif

	cfg->state.spi_error_rate = 0.0;
	cfg->state.encoder_no_magnet_error_rate = 0.0;

	s_cfg = cfg;
	if (1) {
		static uint8_t data_rx[24];

		data_rx[0] = 0x83;
		mhm_activate(data_rx, 1);
	}

	return true;
}

void enc_icmhm_deinit(ICMHM_config_t *cfg) {
	if (cfg->spi_dev == NULL) {
		return;
	}

	palSetPadMode(cfg->miso_gpio, cfg->miso_pin, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(cfg->sck_gpio, cfg->sck_pin, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(cfg->nss_gpio, cfg->nss_pin, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(cfg->mosi_gpio, cfg->mosi_pin, PAL_MODE_INPUT_PULLUP);

#ifdef HW_SPI_DEV
	//spiStop(cfg->spi_dev);
	SPIDriver *spip = cfg->spi_dev;
	spip->spi->CR1  = 0;
	spip->spi->CR2  = 0;
	rccEnableSPI1(FALSE);
#endif

	cfg->state.last_enc_angle = 0.0;
	cfg->state.spi_error_rate = 0.0;
}

static uint16_t spi_polled_exchange(SPIDriver *spip, uint16_t frame) {

  spip->spi->DR = frame;
  while ((spip->spi->SR & SPI_SR_RXNE) == 0)
    ;
  return spip->spi->DR;
}

#define SPI_BEGIN()		spi_bb_delay(); palClearPad(s_cfg->nss_gpio, s_cfg->nss_pin); spi_bb_delay();
#define SPI_END()		spi_bb_delay(); palSetPad(s_cfg->nss_gpio, s_cfg->nss_pin); spi_bb_delay();

void enc_icmhm_routine(ICMHM_config_t *cfg) {
	float timestep = timer_seconds_elapsed_since(cfg->state.last_update_time);
	if (timestep > 1.0) {
		timestep = 1.0;
	}
	cfg->state.last_update_time = timer_time_now();

	{
		uint16_t pos;
		static const uint8_t abs_spi_dma_tx_[6] = {0xA6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    	uint8_t abs_spi_dma_rx_[6];

		memset(abs_spi_dma_rx_, 0, sizeof(abs_spi_dma_rx_));
		SPI_BEGIN();
#ifdef HW_SPI_DEV
		abs_spi_dma_rx_[0] = spi_polled_exchange(&HW_SPI_DEV, abs_spi_dma_tx_[0]) & 0xFF;
		abs_spi_dma_rx_[1] = spi_polled_exchange(&HW_SPI_DEV, abs_spi_dma_tx_[1]) & 0xFF;
		abs_spi_dma_rx_[2] = spi_polled_exchange(&HW_SPI_DEV, abs_spi_dma_tx_[2]) & 0xFF;
		abs_spi_dma_rx_[3] = spi_polled_exchange(&HW_SPI_DEV, abs_spi_dma_tx_[3]) & 0xFF;
		abs_spi_dma_rx_[4] = spi_polled_exchange(&HW_SPI_DEV, abs_spi_dma_tx_[4]) & 0xFF;
		abs_spi_dma_rx_[5] = spi_polled_exchange(&HW_SPI_DEV, abs_spi_dma_tx_[5]) & 0xFF;
#else
		mhm_spi_transfer(abs_spi_dma_tx_, abs_spi_dma_rx_, 6);
#endif
		SPI_END();
		spi_bb_delay();

		//commands_printf("@MHM %02x %02x %02x %02x %02x %02x\r\n", abs_spi_dma_rx_[0], abs_spi_dma_rx_[1],
		//	abs_spi_dma_rx_[2], abs_spi_dma_rx_[3], abs_spi_dma_rx_[4], abs_spi_dma_rx_[5]);

		cfg->state.mt_val = ((abs_spi_dma_rx_[1] << 16) | (abs_spi_dma_rx_[2] << 8) | (abs_spi_dma_rx_[3] << 0));
		if (cfg->state.mt_val > 8388608) {
			cfg->state.mt_val -= 16777216;			
		}	

		uint16_t angle = (abs_spi_dma_rx_[4] << 8) | (abs_spi_dma_rx_[5] << 0);
		angle >>= 4;
		pos = angle;
		cfg->state.spi_val = pos;

		cfg->state.last_enc_angle = cfg->state.mt_val * 360.00 + ((float)pos * 360.0) / 4096.0;
		UTILS_LP_FAST(cfg->state.spi_error_rate, 0.0, timestep);
		UTILS_LP_FAST(cfg->state.encoder_no_magnet_error_rate, 0.0, timestep);

		//测试代码开始 ******************

		if(st_val_previous < 500 && pos > 3596) {
			mt_val_accumulate--;
		}else if(st_val_previous > 3596 && pos < 500) {
			mt_val_accumulate++;
		}
		st_val_previous = pos;
		cfg->state.last_enc_angle = mt_val_accumulate * 360 + ((float)pos * 360.0) / 4096.0;
		
		//测试代码结束 ******************
	}
}


/* iC-MHM opcodes */
enum MHM_OPCODES {
	MHM_OPCODE_ACTIVATE = 0xB0,
	MHM_OPCODE_POSITION_READ = 0xA6,
	MHM_OPCODE_REGISTER_READ_CONTINUOUS = 0x8A,
	MHM_OPCODE_REGISTER_WRITE_CONTINUOUS = 0xCF,
	MHM_OPCODE_READ_STATUS = 0x9C,
	MHM_OPCODE_WRITE_INSTRUCTION = 0xD9,
	MHM_OPCODE_REGISTER_READ_SINGLE = 0x97,
	MHM_OPCODE_REGISTER_WRITE_SINGLE = 0xD2,
	MHM_OPCODE_READ_REGISTER_STATUS_DATA = 0xAD
};

static uint8_t buf_tx[48];
static uint8_t buf_rx[48];
static uint16_t bufsize = 0;

static void spi_icmhm_cs_delay(void) {
	__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	//__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();
	__NOP();
}

static void mhm_delay_ms(int ms) {
	for (int i = 0;i < ms*10000; i++) {
		spi_icmhm_cs_delay();
	}
}

static void mhm_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint8_t datasize) {
	SPI_BEGIN();
	//spi_icmhm_cs_delay();
	//spi_icmhm_cs_delay();
	spi_icmhm_cs_delay();
#ifdef HW_SPI_DEV
	spiPolledExchange(&HW_SPI_DEV, data_tx[0]);
#else
	{
		uint8_t i, j , k, val_t, val_r;
		for (i=0; i<datasize; i++) {
			val_t = data_tx[i];
			val_r = 0x00;
			for (j=8; j>0; j--) {
				k = (val_t & (1<<(j-1)));
				spi_bb_delay();
				//spi_bb_delay();
				//rt_pin_write(PIN_M0_MOSI, k?PIN_HIGH:PIN_LOW);
				palWritePad(s_cfg->mosi_gpio, s_cfg->mosi_pin, k?1:0);
				spi_bb_delay();
				//spi_bb_delay();
				//rt_pin_write(PIN_M0_SCK, PIN_HIGH);
				palWritePad(s_cfg->sck_gpio, s_cfg->sck_pin, 1);
				spi_bb_delay();
				//spi_bb_delay();
				
				//k = rt_pin_read(PIN_M0_MISO);
				k = 0;
				int samples = 0;
				samples += palReadPad(s_cfg->miso_gpio, s_cfg->miso_pin);
				__NOP();
				samples += palReadPad(s_cfg->miso_gpio, s_cfg->miso_pin);
				__NOP();
				samples += palReadPad(s_cfg->miso_gpio, s_cfg->miso_pin);
				__NOP();
				samples += palReadPad(s_cfg->miso_gpio, s_cfg->miso_pin);
				__NOP();
				samples += palReadPad(s_cfg->miso_gpio, s_cfg->miso_pin);
				if (samples > 2) {
					k = 1;
				}

				//spi_bb_delay();
				spi_bb_delay();
				//rt_pin_write(PIN_M0_SCK, PIN_LOW);
				palWritePad(s_cfg->sck_gpio, s_cfg->sck_pin, 0);
				val_r |= k ? (1<<(j-1)) : 0x00;
			}
			data_rx[i] = val_r;
		}
	}
#endif
	SPI_END();
	//spi_icmhm_cs_delay();
	//spi_icmhm_cs_delay();
	spi_icmhm_cs_delay();
	spi_bb_delay();
}

/**
 * @brief This function is used to turn the register and sensor data channels in the iC-MHM on and off individually.
 *
 * @note Register communication and sensor data channel are activated by default after startup.
 *
 * @param active_vector is a pointer to a buffer containing the ractive/pactive vector to be transmitted.
 * @param vector_size is the length of the transmitted vector in byte.
 * @retval None
 */
static void mhm_activate(const uint8_t *active_vector, uint8_t vector_size) {
    bufsize = vector_size + 1;
    buf_tx[0] = MHM_OPCODE_ACTIVATE;

    for (uint8_t i = 0; i < vector_size; ++i) {
        buf_tx[i + 1] = active_vector[i];
    }

    memset(buf_rx, 0, sizeof(buf_rx));
    mhm_spi_transfer(buf_tx, buf_rx, bufsize);
    mhm_delay_ms(100);
}

/**
 * @brief This function is used to read the absolute position data from the iC-MHM.
 *
 * @note The number of multiturn bytes in the position data is determined by parameter MHM_RESO_MT.
 * @note Position data is latched on the first rising edge of SCLK when NCS is low.
 *
 * @param data_rx is a pointer to a buffer the received data is stored.
 * @param datasize is the number of full bytes according to the configured position data format.
 * @retval None
 */
static void mhm_postition_read(uint8_t *data_rx, uint8_t datasize) {
    bufsize = datasize + 1;
    buf_tx[0] = MHM_OPCODE_POSITION_READ;

    for (uint16_t i = 1; i < bufsize; i++) {
        buf_tx[i] = 0x00;
    }

    memset(buf_rx, 0, sizeof(buf_rx));
    mhm_spi_transfer(buf_tx, buf_rx, bufsize);
    mhm_delay_ms(100);
    
    for (uint16_t i = 0; i < datasize; i++) {
        data_rx[i] = buf_rx[i + 1];
    }
}

/**
 * @brief This function is used to read data from a contiguous block of one or more RAM addresses starting at a specified address.
 *
 * @param addr is the address of the first register to start reading from.
 * @param data_rx is a pointer to a buffer the received data is stored.
 * @param datasize is the number of consecutive registers to be read.
 * @retval None
 */
static void mhm_register_read_continuous(uint8_t addr, uint8_t *data_rx, uint8_t datasize) {
	bufsize = datasize + 2;
	buf_tx[0] = MHM_OPCODE_REGISTER_READ_CONTINUOUS;
	buf_tx[1] = addr;

	for (uint16_t i = 2; i < bufsize; i++) {
		buf_tx[i] = 0x00;
	}

    memset(buf_rx, 0, sizeof(buf_rx));
	mhm_spi_transfer(buf_tx, buf_rx, bufsize);
    mhm_delay_ms(100);
    
	for (uint8_t i = 0; i < datasize; i++) {
		data_rx[i] = buf_rx[i + 2];
	}
}

/**
 * @brief This function is used to write data to a contiguous block of one or more RAM addresses starting at a specified address.
 *
 * @param addr is the address of the first register to start writing to.
 * @param data_tx is a pointer to a buffer the transmitted data is stored.
 * @param datasize is the number of consecutive registers to be written.
 * @retval None
 */
static void mhm_register_write_continuous(uint8_t addr, const uint8_t *data_tx, uint8_t datasize) {
	bufsize = datasize + 2;
	buf_tx[0] = MHM_OPCODE_REGISTER_WRITE_CONTINUOUS;
	buf_tx[1] = addr;

	for (uint16_t i = 0; i < datasize; i++) {
		buf_tx[i + 2] = data_tx[i];
	}

    memset(buf_rx, 0, sizeof(buf_rx));
	mhm_spi_transfer(buf_tx, buf_rx, bufsize);
    mhm_delay_ms(100);
}

/**
 * @brief This function is used to directly read the iC-MHM status registers at address 0x70 - 0x73.
 *
 * @param stat is a pointer to a 4 byte buffer the received status register data is stored.
 * @retval None
 */
static void mhm_read_status(uint8_t stat[4]) {
    bufsize = 5;
    buf_tx[0] = MHM_OPCODE_READ_STATUS;
    buf_tx[1] = 0;
    buf_tx[2] = 0;
    buf_tx[3] = 0;
    buf_tx[4] = 0;

    memset(buf_rx, 0, sizeof(buf_rx));
    mhm_spi_transfer(buf_tx, buf_rx, bufsize);
    mhm_delay_ms(100);

    for (uint8_t i = 0; i < 4; i++) {
        stat[i] = buf_rx[i + 1];
    }
}

/**
 * @brief This function is used to write values directly to the iC-MHM instruction registers at address 0x74 - 0x76.
 *
 * @param inst is a pointer to a 3 byte buffer the transmitted instruction register data is stored.
 * @retval None
 */
static void mhm_write_instruction(const uint8_t inst[3]) {
	bufsize = 4;
	buf_tx[0] = MHM_OPCODE_WRITE_INSTRUCTION;
	buf_tx[1] = inst[0];
    buf_tx[2] = inst[1];
    buf_tx[3] = inst[2];

    memset(buf_rx, 0, sizeof(buf_rx));
	mhm_spi_transfer(buf_tx, buf_rx, bufsize);
	mhm_delay_ms(100);
}
