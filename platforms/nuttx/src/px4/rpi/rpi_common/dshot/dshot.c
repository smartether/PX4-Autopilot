/****************************************************************************
 *
 * Copyright (C) 2019, 2021 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#if (0)
//Do nothing. IP DMA V1 MCUs are not supported.
#else

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_pwm_output.h>


#define MOTOR_PWM_BIT_1				14u
#define MOTOR_PWM_BIT_0				7u
#define DSHOT_TIMERS				MAX_IO_TIMERS
#define MOTORS_NUMBER				DIRECT_PWM_OUTPUT_CHANNELS
#define ONE_MOTOR_DATA_SIZE			16u
#define ONE_MOTOR_BUFF_SIZE			17u
#define ALL_MOTORS_BUF_SIZE			(MOTORS_NUMBER * ONE_MOTOR_BUFF_SIZE)
#define DSHOT_THROTTLE_POSITION		5u
#define DSHOT_TELEMETRY_POSITION	4u
#define NIBBLES_SIZE 				4u
#define DSHOT_NUMBER_OF_NIBBLES		3u
#define DSHOT_END_OF_STREAM 		16u
#define MAX_NUM_CHANNELS_PER_TIMER	4u // CCR1-CCR4

#define DSHOT_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | \
		       DMA_SCR_DIR_M2P | DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

typedef struct dshot_handler_t {
	bool			init;
	DMA_HANDLE		dma_handle;
	uint32_t		dma_size;
} dshot_handler_t;

#if defined(CONFIG_ARMV7M_DCACHE)
#  define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#  define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#else
#define DMA_ALIGN_UP(n) (n)
#endif
#define DSHOT_BURST_BUFFER_SIZE(motors_number) (DMA_ALIGN_UP(sizeof(uint32_t)*ONE_MOTOR_BUFF_SIZE*motors_number))

static dshot_handler_t dshot_handler[DSHOT_TIMERS] = {};
static uint8_t dshot_burst_buffer_array[DSHOT_TIMERS * DSHOT_BURST_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER)]
px4_cache_aligned_data() = {};
// static uint32_t *dshot_burst_buffer[DSHOT_TIMERS] = {};

int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq)
{
    for(int i=0;i<4;i++) {
        if((channel_mask & 1u << i) == 0) {
            channel_mask |= 1u << i;
        }
    }
    return channel_mask;
}

void up_dshot_trigger(void)
{
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
void dshot_motor_data_set(unsigned motor_number, uint16_t throttle, bool telemetry)
{
}

int up_dshot_arm(bool armed)
{
	return 0;
}

#endif
