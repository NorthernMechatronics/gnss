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
#include <stdlib.h>

#include <am_bsp.h>
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "console_task.h"
#include "gnss.h"
#include "gnss_cli.h"

TaskHandle_t gnss_task_handle;

float gfLatitude = 0.0;
float gfLongitude = 0.0;
static uint8_t gsGnssResultBuffer[128];

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

static void gnss_application_setup()
{
    am_hal_uart_initialize(1, &gsGnssComm);
    am_hal_uart_power_control(gsGnssComm, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(gsGnssComm, &gsGnssUartConfig);

    am_hal_gpio_pinconfig(24, GNSS_UART_TX);
    am_hal_gpio_pinconfig(25, GNSS_UART_RX);

    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + 1),
                     NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + 1));

    am_util_stdio_printf_init((am_util_stdio_print_char_t)nm_console_print);
}

static size_t gnss_cmd_fsm(char *cmd, uint8_t dont_care, char ch, size_t current_state)
{
	if ((cmd[current_state] == ch) || (cmd[current_state] == dont_care))
		return current_state + 1;

	return 0;
}

static void gnss_parse_gll()
{
	char buffer[32];

	memcpy(buffer, &gsGnssResultBuffer[0], 2);
	buffer[2] = 0;
	int lat_deg = atoi(buffer);

	memcpy(buffer, &gsGnssResultBuffer[2], 8);
	buffer[8] = 0;
	float lat_min = (float)atof(buffer);

	memcpy(buffer, &gsGnssResultBuffer[11], 1);

	float lat = (lat_deg + lat_min / 60.0) * (buffer[0] == 'N' ? 1.0 : -1.0);

	memcpy(buffer, &gsGnssResultBuffer[13], 3);
	buffer[3] = 0;
	int lon_deg = atoi(buffer);

	memcpy(buffer, &gsGnssResultBuffer[16], 8);
	buffer[8] = 0;
	float lon_min = (float)atof(buffer);

	memcpy(buffer, &gsGnssResultBuffer[25], 1);

	float lon = (lon_deg + lon_min / 60.0) * (buffer[0] == 'E' ? 1.0 : -1.0);
/*
	memcpy(buffer, &gsGnssResultBuffer[27], 2);
	buffer[3] = 0;
	int hour = atoi(buffer);

	memcpy(buffer, &gsGnssResultBuffer[29], 2);
	buffer[3] = 0;
	int min = atoi(buffer);

	memcpy(buffer, &gsGnssResultBuffer[31], 4);
	buffer[5] = 0;
	double sec = atof(buffer);


	am_util_stdio_printf("\r%02d:%02d:%-04.1f  %3.9f, %3.9f\r", hour, min, sec, lat, lon);
*/
	taskENTER_CRITICAL();
	gfLatitude = lat;
	gfLongitude = lon;
	taskEXIT_CRITICAL();
}

void gnss_task(void *pvParameters)
{
    FreeRTOS_CLIRegisterCommand(&GnssCommandDefinition);

    am_util_stdio_printf("\r\n\r\nZED-F9P State Machine Started\r\n\r\n");
    nm_console_print_prompt();

    uint8_t ui8Buffer[32];
    uint32_t ui32BytesRead = 0;
    am_hal_uart_transfer_t sUartRead = {
        .ui32Direction = AM_HAL_UART_READ,
        .pui8Data = ui8Buffer,
        .ui32NumBytes = 32,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &ui32BytesRead,
    };

    char cmd[] = "$GxGLL,";
    size_t cmdlen = strlen(cmd);
    uint8_t state = 0;
    uint32_t result_index = 0;

    gnss_application_setup();
    while (1) {
        am_hal_uart_transfer(gsGnssComm, &sUartRead);

        if (ui32BytesRead > 0) {
            for (int i = 0; i < ui32BytesRead; i++) {
            	if (state == cmdlen)
            	{
                    if (ui8Buffer[i] == '\n')
                    {
                        state = 0;
                        gnss_parse_gll(gsGnssResultBuffer);
                        memset(gsGnssResultBuffer, 0, 128);
                    }
                    else
                    {
                    	gsGnssResultBuffer[result_index++] = ui8Buffer[i];
                    }
            	}
            	else
            	{
            		state = gnss_cmd_fsm(cmd, 'x', ui8Buffer[i], state);
            		if (state == cmdlen)
            		{
            			result_index = 0;
            		}
            	}
            }
        }
        taskYIELD();
    }
}

void am_uart1_isr()
{
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(gsGnssComm, &ui32Status, true);
    am_hal_uart_interrupt_clear(gsGnssComm, ui32Status);
    am_hal_uart_interrupt_service(gsGnssComm, ui32Status, &ui32Idle);
}
