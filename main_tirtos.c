/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,

 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== main_tirtos.c ========
 */
#include <stdint.h>

/* POSIX Header files */
#include <pthread.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>

/* Example/Board Header files */
#include "Board.h"
#include "uart_term.h"

extern void *mainThread(void *arg0);

/* Stack size in bytes */
#define THREADSTACKSIZE    4096
#define NUMMSGS         5


/*
 * Mailbox messages are stored in a queue that requires a header in front of
 * each message. Mailbox_MbxElem is defined such that the header and its size
 * are factored into the total data size requirement for a mailbox instance.
 * Because Mailbox_MbxElem contains Int data types, padding may be added to
 * this struct depending on the data members defined in MsgObj.
 */
typedef struct MailboxMsgObj {
    Mailbox_MbxElem  elem;      /* Mailbox header        */
    MsgObj           obj;       /* Application's mailbox */
} MailboxMsgObj;

/* This buffer is not directly accessed by the application */
MailboxMsgObj mailboxBuffer[NUMMSGS];
//static Mailbox_Handle mbxHandle;

Mailbox_Struct mbxStruct;
/*
 *  ======== main ========
 */
int main(void)
{
    pthread_t           thread;
    //pthread_t           cloud_tx_thread;
    //pthread_t           uart_rx_thread;
    pthread_attr_t      pAttrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;
    //Mailbox_Params      mbxParams;
    //Task_Params taskParams;
    //Task_Struct task0Struct, task1Struct;
    //Char task0Stack[THREADSTACKSIZE], task1Stack[THREADSTACKSIZE];

    /* Call board init functions */
    Board_initGeneral();

    /* Set priority and stack size attributes */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 2;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    retc = pthread_create(&thread, &pAttrs, mainThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    /* Construct a Mailbox instance
    Mailbox_Params_init(&mbxParams);
    mbxParams.buf = (Ptr)mailboxBuffer;
    mbxParams.bufSize = sizeof(mailboxBuffer);
    Mailbox_construct(&mbxStruct, sizeof(MsgObj), NUMMSGS, &mbxParams, NULL);
    mbxHandle = Mailbox_handle(&mbxStruct); */

    /* Construct read and writer tasks
    Task_Params_init(&taskParams);
    taskParams.stackSize = THREADSTACKSIZE;
    taskParams.stack = &task0Stack;
    taskParams.arg0=(UArg)mbxHandle;
    Task_construct(&task0Struct, (Task_FuncPtr)uartRxThread, &taskParams, NULL);
    taskParams.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr)cloudTxThread, &taskParams, NULL);


    retc = pthread_create(&uart_rx_thread, &pAttrs, uartRxThread, NULL);
    if(retc != 0)
    {
        UART_PRINT("could not create uart_rx_thread task\n\r");
        while (1);
    }

    retc = pthread_create(&cloud_tx_thread, &pAttrs, cloudTxThread, NULL);
    if(retc != 0)
    {
        UART_PRINT("could not create cloud_tx_thread task\n\r");
        while (1);
    }
*/
    BIOS_start();

    return (0);
}

/*
 *  ======== dummyOutput ========
 *  Dummy SysMin output function needed for benchmarks and size comparison
 *  of FreeRTOS and TI-RTOS solutions.
 */
void dummyOutput(void)
{
}
