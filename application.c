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
#include <string.h>

#include <am_bsp.h>
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "application.h"

TaskHandle_t application_task_handle;

static uint8_t gsGnssUartBuffer[1024];

static void *gsGnssComm;
static am_hal_uart_config_t gsGnssUartConfig = {
    // this is the default baudrate on the C099-F9P
    .ui32BaudRate = 921600,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
    .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    //
    // Set TX and RX FIFOs to interrupt at half-full.
    //
    .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2),

    //
    // Setup the UART to use a RX buffer.  Data transfer from the
    // UART hardware FIFO to the buffer will occur in
    // am_hal_uart_interrupt_service.  The application will retrieve
    // the received data using am_hal_uart_transfer in the main loop.
    //
    .pui8TxBuffer = 0,
    .ui32TxBufferSize = 0,
    .pui8RxBuffer = gsGnssUartBuffer,
    .ui32RxBufferSize = 1024,
};

static am_hal_gpio_pincfg_t GNSS_UART_TX = {
    .uFuncSel = AM_HAL_PIN_24_UART1TX,
    .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA};

static am_hal_gpio_pincfg_t GNSS_UART_RX = {
    .uFuncSel = AM_HAL_PIN_25_UART1RX,
};

static void gnss_uart_setup()
{
    am_hal_uart_initialize(1, &gsGnssComm);
    am_hal_uart_power_control(gsGnssComm, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(gsGnssComm, &gsGnssUartConfig);

    am_hal_gpio_pinconfig(24, GNSS_UART_TX);
    am_hal_gpio_pinconfig(25, GNSS_UART_RX);

    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + 1),
                     NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + 1));
}

void application_task(void *pvParameters)
{
    gnss_uart_setup();

    uint8_t state = 0;
    while (1) {
        uint8_t ui8Buffer[128];
        uint32_t ui32BytesRead = 0;
        am_hal_uart_transfer_t sUartRead = {
            .ui32Direction = AM_HAL_UART_READ,
            .pui8Data = ui8Buffer,
            .ui32NumBytes = 128,
            .ui32TimeoutMs = 0,
            .pui32BytesTransferred = &ui32BytesRead,
        };
        am_hal_uart_transfer(gsGnssComm, &sUartRead);

        if (ui32BytesRead > 0) {
            for (int i = 0; i < ui32BytesRead; i++) {
                switch (state) {
                case 0:
                    if (ui8Buffer[i] == '$')
                        state = 1;
                    break;
                case 1:
                    if (ui8Buffer[i] == 'G')
                        state = 2;
                    break;
                case 2:
                    // Don't care as GLL could be reported by
                    // GPS, GLONASS, BEIDOU, Gallileo, or
                    // multi-constellation fused solution
                    // if (ui8Buffer[i] == 'N')
                    //     state = 3;
                    state = 3;
                    break;
                case 3:
                    if (ui8Buffer[i] == 'G')
                        state = 4;
                    break;
                case 4:
                    if (ui8Buffer[i] == 'L')
                        state = 5;
                    break;
                case 5:
                    if (ui8Buffer[i] == 'L')
                        state = 6;
                    break;
                case 6:
                    if (ui8Buffer[i] == ',')
                        state = 7;
                    break;
                case 7:
                    if (ui8Buffer[i] == '\n')
                        state = 0;
                    else
                        nm_console_write(&ui8Buffer[i], 1);
                    break;
                }
            }
        }
    }
}

void am_uart1_isr()
{
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(gsGnssComm, &ui32Status, true);
    am_hal_uart_interrupt_clear(gsGnssComm, ui32Status);
    am_hal_uart_interrupt_service(gsGnssComm, ui32Status, &ui32Idle);
}
