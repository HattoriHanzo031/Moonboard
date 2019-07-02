/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup i2s_example_main main.c
 * @{
 * @ingroup i2s_example
 *
 * @brief I2S Example Application main file.
 *
 * This file contains the source code for a sample application using I2S.
 */

#include <stdio.h>
#include "nrf_drv_i2s.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Delay time between consecutive I2S transfers performed in the main loop
// (in milliseconds).
#define PAUSE_TIME          2000
// Number of blocks of data to be contained in each transfer.
#define BLOCKS_TO_TRANSFER  20

#define COLOR_BRIGHTNESS 	0x01
#define COLOR_NONE			0x00000000
#define COLOR_GREEN			COLOR_BRIGHTNESS << 0
#define COLOR_RED			COLOR_BRIGHTNESS << 8
#define COLOR_BLUE			COLOR_BRIGHTNESS << 16

#define RESET_BITS 6
#define NUM_LEDS (18*11)
#define I2S_DATA_BLOCK_WORDS    ((NUM_LEDS*3) + RESET_BITS)

static uint32_t m_buffer_tx[I2S_DATA_BLOCK_WORDS];
static uint32_t m_blocks_transferred;

static inline
void set_led(uint8_t _led, uint32_t _color);
void rgb_to_i2s(uint32_t _rgb, uint32_t* _i2s);

static void data_handler(nrf_drv_i2s_buffers_t const * p_released,
                         uint32_t                      status)
{
	//static uint32_t tmp = 1;
    // 'nrf_drv_i2s_next_buffers_set' is called directly from the handler
    // each time next buffers are requested, so data corruption is not
    // expected.
    ASSERT(p_released);

    // When the handler is called after the transfer has been stopped
    // (no next buffers are needed, only the used buffers are to be
    // released), there is nothing to do.
    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)) {
        return;
    }

	//rgb_to_i2s(tmp++, (uint32_t*)p_released->p_tx_buffer);
    APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(p_released));
	m_blocks_transferred++;
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    app_error_save_and_stop(id, pc, info);
}

static inline
void set_led(uint8_t _led, uint32_t _color)
{
	rgb_to_i2s(_color, &(m_buffer_tx[(_led-1)*3]));
}

void rgb_to_i2s(uint32_t _rgb, uint32_t* _i2s)
{
	int i;
	uint32_t mask = 0x000F0F0F;
	uint32_t tmp;

	_i2s[0] = 0;
	_i2s[1] = 0;
	_i2s[2] = 0;

	tmp = _rgb & (~mask);
	mask = _rgb & mask;
	_rgb = (tmp >> 4) | (mask << 4);

	mask = 1;
	for (i=0; i<24; i++) {
		_i2s[i/8u] += ((_rgb & mask) ? 0xe : 0x8) << ((i%8u)*4);
		mask = mask << 1;
	}
}

int main(void)
{
    uint32_t err_code = NRF_SUCCESS;
	uint32_t* buffer = m_buffer_tx;
	int i;

    bsp_board_init(BSP_INIT_NONE);

    nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;
    // In Master mode the MCK frequency and the MCK/LRCK ratio should be
    // set properly in order to achieve desired audio sample rate (which
    // is equivalent to the LRCK frequency).
    config.sdin_pin  = NRF_DRV_I2S_PIN_NOT_USED;
    config.sdout_pin = I2S_SDOUT_PIN;
	config.mck_pin   = NRF_DRV_I2S_PIN_NOT_USED;
    config.mck_setup = NRF_I2S_MCK_32MDIV10;
    config.ratio     = NRF_I2S_RATIO_32X;
    config.channels  = NRF_I2S_CHANNELS_STEREO;
    err_code = nrf_drv_i2s_init(&config, data_handler);
    APP_ERROR_CHECK(err_code);
#if 0
	for (i=0; i<NUM_LEDS; i++) {
		uint32_t rgb = 0x00000001 << ((i%3)*8);
		buffer+=3;
		rgb_to_i2s(rgb, buffer);
	}
#else
	for (i=1; i<=NUM_LEDS; i++) {
		set_led(i, COLOR_NONE);
	}
#endif

	buffer=&(m_buffer_tx[(NUM_LEDS)*3]);
	for (i=0; i<RESET_BITS; i++)
		*buffer++ = 0x00000000;

	set_led(1, COLOR_RED);
	set_led(3, COLOR_BLUE);
	set_led(5, COLOR_GREEN);

    for (;;) {
        nrf_drv_i2s_buffers_t const initial_buffers = {
            .p_tx_buffer = m_buffer_tx,
            .p_rx_buffer = NULL,
        };

        err_code = nrf_drv_i2s_start(&initial_buffers, I2S_DATA_BLOCK_WORDS, 0);
        APP_ERROR_CHECK(err_code);

        do {
            // Wait for an event.
            __WFE();
            // Clear the event register.
            __SEV();
            __WFE();

        } while (m_blocks_transferred < BLOCKS_TO_TRANSFER);

        nrf_drv_i2s_stop();

        nrf_delay_ms(PAUSE_TIME);
    }
}

/** @} */
