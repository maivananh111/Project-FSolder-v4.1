/*
 * valconfig.h
 *
 *  Created on: May 10, 2023
 *      Author: anh
 */

#ifndef VALCONFIG_H_
#define VALCONFIG_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "periph/dma.h"
#include "periph/spi.h"
#include "periph/tim.h"

/**
 * SPI2 and DMA1_Channel5 for TFT LCD.
 */
#define TFT_SPI spi2
#define TFT_DMA dma1_channel5
dma_config_t dma1_channel5_conf = {
	.channel = DMA_Channel5,
	.direction = DMA_MEM_TO_PERIPH,
	.mode = DMA_MODE_CIRCULAR,
	.datasize = DMA_DATASIZE_8BIT,
	.interruptselect = DMA_TRANSFER_COMPLETE_INTERRUPT,
	.channelpriority = DMA_CHANNEL_PRIORITY_VERYHIGH,
	.interruptpriority = 5,
};

spi_config_t spi2_conf = {
	.mode = SPI_HALFDUPLEX_MASTER,
	.control = SPI_DMA_CONTROL,
	.clockdivision = SPI_CLOCKDIVISION_2,
	.clkport = GPIOB,
	.clkpin = 13,
	.mosiport = GPIOB,
	.mosipin = 15,
	.txdma = dma1_channel5,
};

/**
 * TIM1 for encoder.
 */
#define ENC_TIM tim1
tim_config_t tim_enc_base_conf = {
	.prescaler = 1,
	.reload = 0xFFFF,
	.interrupt = TIM_INTERRUPT_ENABLE,
	.interruptpriority = 6,
};
tim_encoder_t tim_enc_conf = {
	.mode = TIM_ENCODER_MODE3,
	.encA_ch1_port = GPIOA,
	.encA_ch1_pin = 8,
	.encB_ch2_port = GPIOA,
	.encB_ch2_pin = 9,
	.encA_ch1_edge = TIM_FALLING_EDGE,
	.encB_ch2_edge = TIM_FALLING_EDGE,
};

/**
 * TIM2 channel2 for hot air heat wire PWM.
 */
#define HA_TIM tim2
tim_config_t tim_ha_base_conf = {
	.prescaler = 72,
	.reload = 1000,
};
tim_pwm_t tim_ha_pwm_conf = {
	.port = GPIOB,
	.pin = 3,
	.preload = TIM_PRELOAD_ENABLE,
};

/**
 * TIM3 channel 1 for hot air heat wire PWM.
 */
#define SD_TIM tim3
tim_config_t tim_sd_base_conf = {
	.prescaler = 72,
	.reload = 1000,
};
tim_pwm_t tim_sd_pwm_conf = {
	.port = GPIOB,
	.pin = 4,
	.preload = TIM_PRELOAD_ENABLE,
};

#ifdef __cplusplus
}
#endif

#endif /* VALCONFIG_H_ */
