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
#include <FreeRTOS_CLI.h>
#include <queue.h>

#include <LmHandler.h>
#include <LmHandlerMsgDisplay.h>
#include <LmhpCompliance.h>
#include <NvmDataMgmt.h>
#include <board.h>
#include <timer.h>

#include "console_task.h"
#include "gnss.h"
#include "gnss_cli.h"
#include "lorawan.h"
#include "task_message.h"

portBASE_TYPE prvGnssCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                             const char *pcCommandString);

CLI_Command_Definition_t GnssCommandDefinition = {
    (const char *const) "gnss",
    (const char *const) "gnss:\tGnss Application Framework.\r\n",
    prvGnssCommand, -1};

void prvGnssHelpSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                           const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);

    if (pcParameterString == NULL) {
        strcat(pcWriteBuffer, "usage: gnss [command] [<args>]\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer, "Supported commands are:\r\n");
        strcat(pcWriteBuffer, "  get   display last known coordinates\r\n");
        strcat(pcWriteBuffer, "  send  send last known coordinates\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer,
               "See 'gnss help [command] for the details of each command.\r\n");
    } else if (strncmp(pcParameterString, "get", 3) == 0) {
        strcat(pcWriteBuffer, "usage: gnss get\r\n");
        strcat(pcWriteBuffer, "\r\n");
    } else if (strncmp(pcParameterString, "send", 3) == 0) {
        strcat(pcWriteBuffer, "usage: gnss send\r\n");
        strcat(pcWriteBuffer, "\r\n");
    }
}

void prvGnssGetSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                          const char *pcCommandString)
{
    const char *pcParameterString;
    am_util_stdio_sprintf(pcWriteBuffer,
                          "Last Known Position: %03.9f, %03.9f\r\n", gfLatitude,
                          gfLongitude);
}

void prvGnssSendSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                           const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    LmAppData.Port = LM_APPLICATION_PORT;
    memcpy(&psLmDataBuffer[0], &gfLatitude, sizeof(float));
    memcpy(&psLmDataBuffer[4], &gfLongitude, sizeof(float));

    LmAppData.BufferSize = 2 * sizeof(float);
    LmAppData.Buffer = psLmDataBuffer;

    task_message_t TaskMessage;
    TaskMessage.ui32Event = SEND;
    TaskMessage.psContent = &LmAppData;
    xQueueSend(LoRaWANTaskQueue, &TaskMessage, portMAX_DELAY);
}

portBASE_TYPE prvGnssCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                             const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    pcWriteBuffer[0] = 0x0;

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameterString == NULL) {
        return pdFALSE;
    }

    if (strncmp(pcParameterString, "help", xParameterStringLength) == 0) {
        prvGnssHelpSubCommand(pcWriteBuffer, xWriteBufferLen, pcCommandString);
    } else if (strncmp(pcParameterString, "get", xParameterStringLength) == 0) {
        prvGnssGetSubCommand(pcWriteBuffer, xWriteBufferLen, pcCommandString);
    } else if (strncmp(pcParameterString, "send", xParameterStringLength) ==
               0) {
        prvGnssSendSubCommand(pcWriteBuffer, xWriteBufferLen, pcCommandString);
    }
    return pdFALSE;
}
