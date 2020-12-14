/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Northern Mechatronics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <am_bsp.h>
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "application.h"
#include "console_task.h"
#include "gnss.h"
#include "lorawan.h"
#include "task_message.h"

#define APPLICATION_CLOCK_SOURCE AM_HAL_CTIMER_LFRC_32HZ
#define APPLICATION_TIMER_PERIOD 32
#define APPLICATION_TIMER_SEGMENT AM_HAL_CTIMER_TIMERA
#define APPLICATION_TIMER_NUMBER 0
#define APPLICATION_TIMER_INT AM_HAL_CTIMER_INT_TIMERA0

#define APPLICATION_REPORT_PERIOD 20
#define APPLICATION_QUEUE_SIZE 10

TaskHandle_t application_task_handle;

static bool bReportCoordinates;

static void application_button0_handler(void)
{
    bReportCoordinates ^= 1;

    if (bReportCoordinates) {
        am_hal_gpio_state_write(AM_BSP_GPIO_LED1, AM_HAL_GPIO_OUTPUT_SET);
        am_hal_gpio_state_write(AM_BSP_GPIO_LED2, AM_HAL_GPIO_OUTPUT_TOGGLE);
        uint32_t ui32Period =
            APPLICATION_REPORT_PERIOD * APPLICATION_TIMER_PERIOD;
        am_hal_ctimer_period_set(APPLICATION_TIMER_NUMBER,
                                 APPLICATION_TIMER_SEGMENT, ui32Period,
                                 (ui32Period >> 1));
        am_hal_ctimer_start(APPLICATION_TIMER_NUMBER,
                            APPLICATION_TIMER_SEGMENT);
    } else {
        am_hal_ctimer_stop(APPLICATION_TIMER_NUMBER, APPLICATION_TIMER_SEGMENT);
        am_hal_gpio_state_write(AM_BSP_GPIO_LED2, AM_HAL_GPIO_OUTPUT_CLEAR);
        am_hal_gpio_state_write(AM_BSP_GPIO_LED1, AM_HAL_GPIO_OUTPUT_CLEAR);
    }
}

static void application_setup()
{
    bReportCoordinates = false;
    am_hal_gpio_state_write(AM_BSP_GPIO_LED1, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_interrupt_register(AM_BSP_GPIO_BUTTON0,
                                   application_button0_handler);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON0, g_AM_BSP_GPIO_BUTTON0);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
    NVIC_EnableIRQ(GPIO_IRQn);
}

static void application_timer_isr()
{
    am_hal_ctimer_int_clear(APPLICATION_TIMER_INT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED2, AM_HAL_GPIO_OUTPUT_TOGGLE);

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    LmAppData.Port = LM_APPLICATION_PORT;
    memcpy(&psLmDataBuffer[0], &gfLatitude, sizeof(float));
    memcpy(&psLmDataBuffer[4], &gfLongitude, sizeof(float));
    LmAppData.BufferSize = 2 * sizeof(float);
    LmAppData.Buffer = psLmDataBuffer;

    task_message_t TaskMessage;
    TaskMessage.ui32Event = SEND;
    TaskMessage.psContent = &LmAppData;
    xQueueSendFromISR(LoRaWANTaskQueue, &TaskMessage,
                      &xHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void application_reporting_timer_setup()
{
    am_hal_ctimer_config_t application_timer = {
        0,
        (AM_HAL_CTIMER_FN_REPEAT | AM_HAL_CTIMER_INT_ENABLE |
         APPLICATION_CLOCK_SOURCE),
        0,
    };

    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);
    am_hal_ctimer_clear(APPLICATION_TIMER_NUMBER, APPLICATION_TIMER_SEGMENT);
    am_hal_ctimer_config(APPLICATION_TIMER_NUMBER, &application_timer);

    am_hal_ctimer_int_register(APPLICATION_TIMER_INT, application_timer_isr);
    am_hal_ctimer_int_clear(APPLICATION_TIMER_INT);
    NVIC_SetPriority(CTIMER_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    am_hal_ctimer_int_enable(APPLICATION_TIMER_INT);
    NVIC_EnableIRQ(CTIMER_IRQn);
}

void application_task(void *pvParameters)
{
    application_setup();
    application_reporting_timer_setup();

    task_message_t task_message;
    while (1) {
        am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_TOGGLE);
        vTaskDelay(500);
    }
}
