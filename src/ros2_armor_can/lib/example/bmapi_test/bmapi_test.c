/**
 * @file        bmapi_test.c
 * @brief       BMAPI example: Transmit test CAN message and then wait until a new message is received.
 * @author      busmust
 * @version     1.0.0.1
 * @copyright   Copyright 2020 by Busmust Tech Co.,Ltd <br>
 *              All rights reserved. Property of Busmust Tech Co.,Ltd.<br>
 *              Restricted rights to use, duplicate or disclose of this code are granted through contract.
 */
#ifdef __GNUC__
#include <unistd.h>
#elif defined(_MSC_VER)
#include <Windows.h>
#define usleep(us) Sleep((us)/1000)
#endif
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "bmapi.h"

/**
 * @def   BMAPI_TASK_WRITE
 * @brief Define this macro to enable hardware-accelerated transmission.
 *        Even though it's not flexible, it's faster than software triggered transmissions and do not require CPU intervention.
 */
#define xBMAPI_TASK_WRITE
#define xBMAPI_ASYNC_WRITE
#define xBMAPI_MULTI_WRITE
#define xBMAPI_ISOTP_WRITE
#define xBMAPI_LOOPBACK_WRITE

#define xBMAPI_MULTI_READ
#define xBMAPI_ISOTP_READ

#ifdef BMAPI_LOOPBACK_WRITE
#define BMAPI_CAN_MODE BM_CAN_EXTERNAL_LOOPBACK_MODE
#else
#define BMAPI_CAN_MODE BM_CAN_NORMAL_MODE
#endif

#define xBMAPI_PARALLEL_READ_WRITE /* Not recommended to open, For partner test only */
#ifdef BMAPI_PARALLEL_READ_WRITE
#ifndef _MSC_VER
#error Write using 2 threads is only supported on Windows.
#endif
#endif

#define BMAPI_MAX_CHANNEL_COUNT 32
#define UDS_REQUEST_MSG_ID     0x7DF
#define UDS_RESPONSE_MSG_ID    0x64E
#define TEST_MSG_ID     0x123
#define TEST_MSG_DLC    0x8
#define TEST_MSG_COUNT  10000
#define TEST_MSG_ROUND  10000
#define TEST_MSG_CYCLE  10
#define TEST_MSG_TX_TIMEOUT 1000
#define TEST_MSG_RX_TIMEOUT 30000

static int transmit(BM_ChannelHandle channel, BM_NotificationHandle notification, uint32_t nroundmessages, uint32_t nrounds, uint32_t cycle, uint32_t channelid);
static int receive (BM_ChannelHandle channel, BM_NotificationHandle notification, uint32_t nmessages, uint32_t ninc);

typedef struct
{
    uint32_t channelid;
    BM_ChannelHandle channel;
    BM_NotificationHandle notification;
    HANDLE hEvent;
    uint32_t nroundmessages;
    uint32_t nrounds;
    uint32_t cycle;
    uint32_t ninc;
    BOOL bRequestExit;
} ThreadArgs;
#if defined(BMAPI_PARALLEL_READ_WRITE)
DWORD transmit_thread(LPVOID lpArg)
{
    ThreadArgs* args = (ThreadArgs*)lpArg;
    int result = transmit(args->channel, args->notification, args->nroundmessages, args->nrounds, args->cycle, args->channelid);
    SetEvent(args->hEvent);
    return result;
}
DWORD receive_thread(LPVOID lpArg)
{
    ThreadArgs* args = (ThreadArgs*)lpArg;
    int result = receive(args->channel, args->notification, args->nroundmessages, args->ninc);
    SetEvent(args->hEvent);
    return result;
}
DWORD fc_thread(LPVOID lpArg)
{
    ThreadArgs* args = (ThreadArgs*)lpArg;
    BM_ChannelHandle channel = args->channel;
    BM_NotificationHandle notification = args->notification;
    BM_CanMessageTypeDef msg = { 0 };

    printf("FC Thread started for channel 0x%p.\n", channel);
    while (BM_WaitForNotifications(&notification, 1, INFINITE) >= 0 && !args->bRequestExit)
    {
        printf("Notification for channel 0x%p.\n", channel);
        while (BM_ReadCanMessage(channel, &msg, NULL, NULL) == BM_ERROR_OK)
        {
            printf("RX: channel=0x%p, id=%x, dlc=%x, payload=[%02x %02x %02x %02x ...]\n", channel, msg.id.SID, msg.ctrl.rx.DLC, msg.payload[0], msg.payload[1], msg.payload[2], msg.payload[3]);
            if (BM_GET_STD_MSG_ID(msg.id) == UDS_RESPONSE_MSG_ID)
            {
                if (msg.payload[0] == 0x30)
                {
                    printf("FC for channel 0x%p.\n", channel);
                    SetEvent(args->hEvent);
                }
            }
        }
    }
    return 0;
}
#endif

/**
 * @brief BMAPI test program: Transmit or receive messages according to command line arguments:
 * @param[in]  argc    Number of command line arguments
 * @param[in]  argv[0] Path of this program
 * @param[in]  argv[1] Channel: Selected CAN port index
 * @param[in]  argv[2] Mode: only "tx" or "rx" is supported
 * @param[in]  argv[3] nmessages: Number of messages to receive/transmit within a round
 * @param[in]  argv[4] nrounds: Number of rounds to transmit
 * @param[in]  argv[4] ncycles: Round cycle, a delay value in milliseconds between consecutive rounds
 * @return     Program exit code
 */
int main(int argc, char* argv[])
{
    BM_ChannelHandle channel = NULL;
    BM_NotificationHandle notification = NULL;
    int nchannels = BMAPI_MAX_CHANNEL_COUNT;
    BM_ChannelInfoTypeDef channelinfos[BMAPI_MAX_CHANNEL_COUNT];
#if defined(BMAPI_PARALLEL_READ_WRITE)
    HANDLE threads[BMAPI_MAX_CHANNEL_COUNT];
#endif
    ThreadArgs threadargs[BMAPI_MAX_CHANNEL_COUNT] = { 0 };
    int channelid = 0;
    int firstchannelid = argc > 1 ? atoi(argv[1]) : 0;
    int lastchannelid = firstchannelid;
    const char* mode = argc > 2 ? argv[2] : "rx";
    int exitcode = 0;
    BM_BitrateTypeDef bitrate;
    BM_StatusTypeDef error = BM_ERROR_OK;
    int i;

    printf(
        "\n"
        "========== BMAPI TEST PROGRAM ==========\n"
        "========= BUSMUST TECH Co.,Ltd. ========\n"
    );

    /* Step1: Initialize BMAPI library before any other operation */
    printf("Initializing BMAPI library ...\n");
    error = BM_Init();
    if (error != BM_ERROR_OK)
    {
        exitcode = 1;
        goto __exit;
    }

    /* Step2: Enumerate connected device channels */
    printf("Enumerating channels ...\n");
    error = BM_Enumerate(channelinfos, &nchannels);
    if (error != BM_ERROR_OK)
    {
        exitcode = 2;
        goto __exit;
    }

    for (i = 0; i < nchannels; i++)
    {
        printf("[%d] %s\n", i, channelinfos[i].name);
    }

    if (argc > 1)
    {
        char* lastidstr = strchr(argv[1], '-');
        if (lastidstr)
        {
            lastchannelid = atoi(&lastidstr[1]);
        }
    }

    if (firstchannelid >= nchannels || lastchannelid >= nchannels)
    {
        printf("Input Error: There are %d available Busmust device, port index %d-%d is invalid.\n", nchannels, firstchannelid, lastchannelid);
        exitcode = 6;
    }
    else
    {
        for (channelid = firstchannelid; channelid <= lastchannelid; channelid++)
        {
            /* Step3: Open the selected channel and configure baudrate */
            printf("Opening channel %s in %s mode ...\n", channelinfos[channelid].name, mode);
            memset(&bitrate, 0, sizeof(bitrate));
            bitrate.nbitrate = 500;
            bitrate.dbitrate = 2000;
            bitrate.nsamplepos = 75;
            bitrate.dsamplepos = 75;
            /* BTR is not mandatory, hardware will calculate BTR using bitrate&samplepos automatically */
            error = BM_OpenEx(&channel, &channelinfos[channelid], BMAPI_CAN_MODE, BM_TRESISTOR_120, &bitrate, NULL, 0);
            if (error != BM_ERROR_OK)
            {
                printf("Failed to open %s, error=0x%08x.\n", channelinfos[channelid].name, error);
                exitcode = 3;
                goto __exit;
            }

            printf("Getting channel notification handle for channel %d(0x%p)...\n", channelid, channel);
            error = BM_GetNotification(channel, &notification);
            if (error != BM_ERROR_OK)
            {
                exitcode = 4;
                goto __exit;
            }

            threadargs[channelid].channel = channel;
            threadargs[channelid].notification = notification;
        }

        for (channelid = firstchannelid; channelid <= lastchannelid; channelid++)
        {
            /* Step5: TX|RX (according to command line arguments) */
            if (argc > 2 && strcmp(argv[2], "tx") == 0)
            {
                uint32_t nmessages = argc > 3 ? atoi(argv[3]) : TEST_MSG_COUNT;
                uint32_t nrounds = argc > 4 ? atoi(argv[4]) : TEST_MSG_ROUND;
                uint32_t ncyclems = argc > 5 ? atoi(argv[5]) : TEST_MSG_CYCLE;
#ifdef BMAPI_PARALLEL_READ_WRITE
                threadargs[channelid].channelid = channelid;
                threadargs[channelid].nroundmessages = nmessages;
                threadargs[channelid].nrounds = nrounds;
                threadargs[channelid].cycle = ncyclems;
                threadargs[channelid].hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
                threads[channelid] = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)transmit_thread, &threadargs[channelid], 0, NULL);
#else
                transmit(threadargs[channelid].channel, notification, nmessages, nrounds, ncyclems, channelid);
#endif
            }
            else
            {
                uint32_t nmessages = argc > 3 ? atoi(argv[3]) : TEST_MSG_COUNT;
                uint32_t ninc = argc > 4 ? atoi(argv[4]) : 1;
#ifdef BMAPI_PARALLEL_READ_WRITE
                threadargs[channelid].nroundmessages = nmessages;
                threadargs[channelid].ninc = ninc;
                threadargs[channelid].hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
                threads[channelid] = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receive_thread, &threadargs[channelid], 0, NULL);
#else
                receive(threadargs[channelid].channel, notification, nmessages, ninc);
#endif
            }
        }

#if defined(BMAPI_PARALLEL_READ_WRITE)
        for (channelid = firstchannelid; channelid <= lastchannelid; channelid++)
        {
            printf("Waiting for background thread for channel %d ...\n", channelid);
            if (WaitForSingleObject(threadargs[channelid].hEvent, INFINITE) != WAIT_OBJECT_0)
            {
                printf("Failed to wait for transmit thread.\n");
            }
            printf("Background thread exit for channel %d ...\n", channelid);
            CloseHandle(threads[channelid]);
        }
#endif
        for (channelid = firstchannelid; channelid <= lastchannelid; channelid++)
        {
            printf("Close channel %d ...\n", channelid);
            if (BM_Close(threadargs[channelid].channel) != BM_ERROR_OK)
            {
                printf("Failed to close channel %d.\n", channelid);
            }
        }
    }

__exit:
    if (error != BM_ERROR_OK)
    {
        char buffer[256] = { 0 };
        BM_GetErrorText(error, buffer, sizeof(buffer), 0);
        printf("Error 0X%08X: %s.\n", error, buffer);
    }
    /* Step6: Cleanup library resource */
    printf("Un-initialize library.\n");
    BM_UnInit();
    printf("Exit %s app.\n", mode);
    return exitcode;
}

static uint8_t isotpCallback(const BM_IsotpStatusTypeDef* status, uintptr_t userArg)
{
    if (status->version == 1)
    {
        printf(
            "\rISOTP: ts=%u, fc=%02x, blocksize=%d, stmin=0x%02x, progress=%d/%d.", 
            status->timestamp, status->flowcontrol, status->blocksize, status->stmin, status->ntransferredbytes, status->ntotalbytes
        );
        if (status->ntransferredbytes >= status->ntotalbytes && status->ntotalbytes > 0)
        {
            printf("\rTransfer complete.                                                                                        \n");
        }
    }
    return 0;
}

static int transmit(BM_ChannelHandle channel, BM_NotificationHandle notification, uint32_t nroundmessages, uint32_t nrounds, uint32_t cycle, uint32_t channelid)
{
    uint32_t n = 0;
    uint32_t nbadmsgs = 0;
    uint32_t nmessages = ((uint32_t)nroundmessages) * ((uint32_t)nrounds);
    BM_CanMessageTypeDef msg;
    BM_StatusTypeDef error = BM_ERROR_OK;

#ifdef BM_ACTIVATE_TEST
    BM_Deactivate(channel);
    error = BM_WriteCanMessage(channel, &messages, 0, TEST_MSG_TX_TIMEOUT, NULL);
    if (error != BM_ERROR_BUSOFF)
    {
        printf("Failed to deactivate CAN BUS.\n");
    }
    error = BM_ReadCanMessage(channel, &messages, NULL, NULL);
    if (error != BM_ERROR_BUSOFF)
    {
        printf("Failed to deactivate CAN BUS.\n");
    }
    BM_Activate(channel);
#endif

    /* Compose a CAN message object for TX */
    printf("Transmitting test message ...\n");
    memset(&msg, 0, sizeof(msg));
    msg.id.SID = TEST_MSG_ID;
    msg.ctrl.tx.DLC = TEST_MSG_DLC;
    msg.ctrl.tx.FDF = 1;
    msg.ctrl.tx.BRS = 0;

    time_t t1 = time(NULL);

#ifdef BMAPI_PARALLEL_READ_WRITE
    ThreadArgs fcThreadArgs;
    fcThreadArgs.channel = channel;
    fcThreadArgs.notification = notification;
    fcThreadArgs.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
    fcThreadArgs.bRequestExit = FALSE;
    HANDLE hFcThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)fc_thread, &fcThreadArgs, 0, NULL);

    uint32_t ndatasize = nroundmessages ? nroundmessages : 4080; /* Reuse command line variable 'nroundmessages' as ISOTP data size, default as 4080. */
    uint32_t ncfsize = 8 - 1;
    uint32_t ncf = (ndatasize - 6/*FF.payload.lenngth*/ + ncfsize - 1) / ncfsize;
    uint32_t imessage = 0;
    uint32_t idata = 0;
    BM_CanMessageTypeDef* messages = (BM_CanMessageTypeDef*)malloc(sizeof(BM_CanMessageTypeDef) * (uint32_t)ncf);
    BM_SET_STD_MSG_ID(msg.id, UDS_REQUEST_MSG_ID);
    msg.payload[0] = 0x10 | (((uint8_t)(ndatasize >> 8)) & 0x0F);
    msg.payload[1] = ((uint8_t)(ndatasize & 0xFFU));
    msg.payload[2] = 0x36; /* UDS SVC 36 */
    msg.payload[3] = 0x00;
    msg.payload[4] = (uint8_t)idata++;
    msg.payload[5] = (uint8_t)idata++;
    msg.payload[6] = (uint8_t)idata++;
    msg.payload[7] = (uint8_t)idata++;
    for (imessage = 0; imessage < (uint32_t)ncf; imessage++)
    {
        messages[imessage] = msg;
        messages[imessage].payload[0] = 0x20 | (imessage & 0xFU);
        messages[imessage].payload[1] = (uint8_t)idata++;
        messages[imessage].payload[2] = (uint8_t)idata++;
        messages[imessage].payload[3] = (uint8_t)idata++;
        messages[imessage].payload[4] = (uint8_t)idata++;
        messages[imessage].payload[5] = (uint8_t)idata++;
        messages[imessage].payload[6] = (uint8_t)idata++;
        messages[imessage].payload[7] = (uint8_t)idata++;
    }

    uint32_t iround;
    for (iround = 0; iround < (uint32_t)nrounds; iround++)
    {
        printf("Write ISOTP request [using BM_WriteCanMessage] [%d/%d] for channel 0x%p ...\n", iround + 1, nrounds, channel);
        msg.payload[3] = (iround + 1) & 0xFFU; /* UDS SVC 36 sequence id */
        error = BM_WriteCanMessage(channel, &msg, 0, TEST_MSG_TX_TIMEOUT, NULL);
        if (error != BM_ERROR_OK)
        {
            printf("Failed to transmit FF for channel 0x%p.\n", channel);
            nbadmsgs++;
        }
        else if (WaitForSingleObject(fcThreadArgs.hEvent, TEST_MSG_RX_TIMEOUT) != WAIT_OBJECT_0)
        {
            printf("Failed to receive FC for channel 0x%p.\n", channel);
        }
        else
        {
            uint32_t nreceived = ncf;
            error = BM_WriteMultipleCanMessage(channel, messages, &nreceived, NULL, TEST_MSG_TX_TIMEOUT, NULL);
            if (error != BM_ERROR_OK)
            {
                printf("Failed: error=0x%08x.\n", error);
            }
            n += nreceived;
        }
    }
    free(messages);

    fcThreadArgs.bRequestExit = TRUE;
    SetEvent(notification);
    Sleep(10);
    CloseHandle(fcThreadArgs.hEvent);
    CloseHandle(hFcThread);
#elif defined(BMAPI_ASYNC_WRITE)
    /**
     * @brief Transmit messages with pipelining:
     *        The 1st message will be transmitted to the bus immediately
     *        The 2nd message will be transmitted to the USB device's internal buffer immediately
     *        The 3rd message will be transmitted to the USB device's internal buffer after the 1st message's ACK is received and the app get notified
     */
    uint32_t previousTimestamp = 0;
    error = BM_WriteCanMessage(channel, &msg, 0, 0, NULL);
    if (BM_ERROR_OK != error)
    {
        printf("Failed to transmit message, error=%d.\n", error);
    }
    for (n = 1; n < nmessages; n++)
    {
        /* Transmit the CAN message to bus using opened device */
        *(uint32_t*)&msg.payload[0] = n;
        error = BM_WriteCanMessage(channel, &msg, 0, 0, NULL);
        if (BM_ERROR_OK != error)
        {
            printf("Failed to transmit message, error=%d.\n", error);
            break;
        }
        if (BM_WaitForNotifications(&notification, 1, TEST_MSG_TX_TIMEOUT) < 0)
        {
            printf("Transmit timeout.\n");
            break;
        }
        else
        {
            uint32_t timestamp = 0;
            if (BM_GetTimestamp(channel, &timestamp) == BM_ERROR_OK)
            {
                //printf("TX TS: %d (0x%08x)\n", timestamp, timestamp);
            }
            if (timestamp <= previousTimestamp)
            {
                printf("TX TS: %d (0x%08x) <= %d (0x%08x).\n", timestamp, timestamp, previousTimestamp, previousTimestamp);
            }
        }
#if defined(BMAPI_LOOPBACK_WRITE)
        /* Take loopback messages out, otherwise accumulated messages will make the RxQ overflow. */
        if (BM_WaitForNotifications(&notification, 1, TEST_MSG_TX_TIMEOUT) < 0)
        {
            printf("Loopback timeout.\n");
            break;
        }
        BM_DataTypeDef unusedAckData;
        BM_Read(channel, &unusedAckData);
#endif
    }
#elif defined(BMAPI_MULTI_WRITE)
    uint32_t i;
    uint32_t j;
    BM_CanMessageTypeDef* messages = (BM_CanMessageTypeDef*)malloc(sizeof(BM_CanMessageTypeDef) * (uint32_t)nroundmessages);
    for (i = 0; i < (uint32_t)nrounds; i++)
    {
        for (j = 0; j < (uint32_t)nroundmessages; j++)
        {
            messages[j] = msg;
            uint32_t payloadInt = (uint32_t)nroundmessages * i + j;
            *(uint32_t*)&messages[j].payload[0] = payloadInt;
        }
        printf("Write %d messages ...\n", j);
        error = BM_WriteMultipleCanMessage(channel, messages, &j, NULL, TEST_MSG_TX_TIMEOUT, NULL);
        if (error != BM_ERROR_OK)
        {
            printf("Failed: error=0x%08x.\n", error);
        }
#if TEST_MSG_TX_TIMEOUT == 0
        for (j = 0; j < (uint32_t)nroundmessages;)
        {
            if (BM_WaitForNotifications(&notification, 1, TEST_MSG_TX_TIMEOUT) < 0)
            {
                printf("Transmit timeout.\n");
                break;
            }
            while (j < nroundmessages)
            {
                BM_DataTypeDef unusedAckData;
                if (BM_Read(channel, &unusedAckData) != BM_ERROR_OK)
                {
                    break;
                }
                j++;
                n++;
            }
        }
#else
        n += j;
#endif
    }
    free(messages);
#elif defined(BMAPI_ISOTP_WRITE)
    uint32_t iround;
    printf("Write %u bytes in %d rounds ...\n", nmessages, nrounds);
    for (iround = 0; iround < nrounds; iround++)
    {
        uint32_t nbytes = nroundmessages;//1 * 1024 * 1024;
        uint8_t* data = (uint8_t*)malloc(nbytes);
        uint32_t i;
        data[0] = 0x36;
        data[1] = (iround + 1) & 0xFF;
        for (i = 2; i < nbytes; i++)
        {
            *(uint8_t*)&data[i] = ((i - 2) & 0xFFU);
        }
        BM_IsotpConfigTypeDef isotp;
        memset(&isotp, 0, sizeof(isotp));
        isotp.version = 1;
        isotp.mode = BM_ISOTP_NORMAL_TESTER; /* Acting as a UDS Tester to download data to ECU */
        isotp.paddingEnabled = 1;
        isotp.paddingValue = 0xCCU;
        isotp.testerTimeout.b = 20000;
        isotp.callbackFunc = isotpCallback;
        isotp.callbackUserarg = 0;
        BM_INIT_CAN_FD_DATA(isotp.testerDataTemplate, UDS_REQUEST_MSG_ID, TEST_MSG_DLC, 0, 1, 1, 0, 0, NULL);
        BM_INIT_CAN_FD_DATA(isotp.ecuDataTemplate, UDS_RESPONSE_MSG_ID, TEST_MSG_DLC, 0, 1, 1, 0, 0, NULL);

        printf("Write long ISOTP request [%d/%d] ...\n", iround + 1, nrounds);
        error = BM_WriteIsotp(channel, data, nbytes, TEST_MSG_TX_TIMEOUT, &isotp);
        if (error != BM_ERROR_OK)
        {
            printf("Failed to write long ISOTP data, error=%08x.\n", error);
            nbadmsgs++;
        }
        free(data);
        n += nbytes;
    }
#elif defined(BMAPI_TASK_WRITE)
    /**
     * @brief Transmit messages using hardware-accelerated TxTask:
     *        Configure the TX pattern (i.e. INCDATA) to the USB device as a task,
     *        it will execute the task and receive TXCMPLT as notifications.
     *        In this case we could reach the highest TX throughput.
     */
    BM_TxTaskTypeDef txtasks[16];
    int ntxtasks = sizeof(txtasks) / sizeof(txtasks[0]);
    for (int itxtask = 0; itxtask < ntxtasks; ++itxtask)
    {
        BM_TxTaskTypeDef txtask;
        static const uint16_t dlc2len[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
        memset(&txtask, 0, sizeof(txtask));
        txtask.type = BM_TXTASK_INCDATA;
        txtask.cycle = cycle;
        txtask.nrounds = nrounds;
        txtask.nmessages = nroundmessages;
        txtask.pattern.incdata.startbit = 0;
        txtask.pattern.incdata.nbits = 32;
        txtask.pattern.incdata.step = 1;
        txtask.pattern.incdata.min = 0;
        txtask.pattern.incdata.max = 0xFFFFFFFFU;
        txtask.pattern.incdata.format = 0x80; /* Intel little edian format */
        static uint32_t testids[] = { 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80 };
        txtask.id = testids[channelid] + itxtask;//TEST_MSG_ID;
        txtask.length = (uint8_t)dlc2len[TEST_MSG_DLC];
        txtask.flags = BM_MESSAGE_FLAGS_FDF | BM_MESSAGE_FLAGS_BRS;
        memset(txtask.payload, 0, sizeof(txtask.payload));
        txtasks[itxtask] = txtask;
    }
    error = BM_SetTxTasks(channel, txtasks, ntxtasks);
    if (BM_ERROR_OK != error)
    {
        printf("Failed to configure tx task, error=%d.\n", error);
    }

    /**
     * brief Wait for TXCMPLT event notifications until all TXCMPLT events are received.
     * So that we know the result of hardware-accelerated transmission.
     */
    while (n < nmessages)
    {
        if (BM_WaitForNotifications(&notification, 1, TEST_MSG_TX_TIMEOUT) < 0)
        {
            printf("Transmit timeout, offset=%d.\n", n);
            break;
        }

        BM_DataTypeDef ackData;
        while ((error = BM_Read(channel, &ackData)) == BM_ERROR_OK)
        {
            if ((ackData.header.type & (BM_ACK_DATA | BM_CAN_FD_DATA)) == (BM_ACK_DATA | BM_CAN_FD_DATA))
            {
                /* You might need more post-processing on the ACK data, i.e. To display the transmitted messages on GUI. */
                BM_CanMessageTypeDef* ackMsg = (BM_CanMessageTypeDef*)ackData.payload;
                if (ackMsg->id.SID == msg.id.SID)
                {
                    n++;
                }
                else
                {
                    nbadmsgs++;
                }
            }
        }
    }
#else
    /**
     * @brief Simple synchronous transmission:
     *        Each message is transmitted to the USB device and the Write function get blocked until the message's ACK is received
     *        You might want to set a timeout value (in milliseconds) to handle bus errors (i.e. BUSOFF)
     */
    uint32_t previousTimestamp = 0;
    for (n = 0; n < nmessages; n++)
    {
        /* Transmit the CAN message to bus using opened device */
        *(uint32_t*)&msg.payload[0] = n;
        error = BM_WriteCanMessage(channel, &msg, 0, TEST_MSG_TX_TIMEOUT, NULL);
        if (error == BM_ERROR_BUSTIMEOUT)
        {
            printf("Transmit timeout, offset=%d.\n", n);
        }
        else if (BM_ERROR_OK != error)
        {
            printf("Failed to transmit message, error=0X%08X.\n", error);
            break;
        }
        else
        {
            uint32_t timestamp = 0;
            if (BM_GetTimestamp(channel, &timestamp) == BM_ERROR_OK)
            {
                //printf("TX TS: %d (0x%08x)\n", timestamp, timestamp);
            }
            if (timestamp <= previousTimestamp)
            {
                printf("TX TS: %d (0x%08x) <= %d (0x%08x).\n", timestamp, timestamp, previousTimestamp, previousTimestamp);
            }
        }
    }
#endif

    time_t t2 = time(NULL);
    uint32_t td = (uint32_t)(t2 - t1);
    float fps = ((float)n) / ((float)td);

    printf(
        "\n"
        "================ TX STAT ================\n"
#ifdef BMAPI_ISOTP_WRITE
        "Transmitted bytes: %u\n"
#else
        "Transmitted messages: %u\n"
#endif
        "Bad messages: %u\n"
        "Total time: %u seconds\n"
#ifdef BMAPI_ISOTP_WRITE
        "Throughput: %.3f B/s\n"
#else
        "Frame rate: %.3f fps\n"
#endif
        "=========================================\n",
        n, nbadmsgs, td, fps
    );

    return n;
}

uint32_t verify(BM_CanMessageTypeDef* msg, uint32_t timestamp, uint32_t* expectedPayloadInt, uint32_t* nbadmsgs, uint32_t* nlostmsgs)
{
    uint32_t succeed = 0;
    uint32_t actualPayloadInt = ((uint32_t)msg->payload[0])
                              | ((uint32_t)msg->payload[1]) << 8
                              | ((uint32_t)msg->payload[2]) << 16
                              | ((uint32_t)msg->payload[3]) << 24;
    if (msg->id.SID != TEST_MSG_ID || msg->ctrl.rx.DLC != TEST_MSG_DLC/* || *(uint32_t*)&msg->payload[4] != 0*/)
    {
        printf("\nBad message received: id=%x, ts=%u, DLC=%d, payload=[%02x %02x %02x %02x %02x %02x %02x %02x].\n",
            msg->id.SID, timestamp, msg->ctrl.rx.DLC,
            msg->payload[0], msg->payload[1], msg->payload[2], msg->payload[3],
            msg->payload[4], msg->payload[5], msg->payload[6], msg->payload[7]
        );
        *nbadmsgs += 1;
    }
    else if (actualPayloadInt != *expectedPayloadInt)
    {
        uint32_t nlost = actualPayloadInt - *expectedPayloadInt;
        printf("\nLost message detected: ts=%u, expected=%08x, received=%08x, lost=%d.\n",
            timestamp, *expectedPayloadInt, actualPayloadInt, nlost
        );
        *nlostmsgs += nlost;
        *expectedPayloadInt = actualPayloadInt;
    }
    else
    {
#ifdef TEST_MSG_PRINT_ALL
        printf("Received: id=%x, ts=%u, DLC=%d, payload=[%02x %02x %02x %02x %02x %02x %02x %02x].\n",
            msg->id.SID, timestamp, msg->ctrl.rx.DLC,
            msg->payload[0], msg->payload[1], msg->payload[2], msg->payload[3],
            msg->payload[4], msg->payload[5], msg->payload[6], msg->payload[7]
        );
#else
        if (*expectedPayloadInt % 100 == 99)
        {
            printf(".");
            fflush(stdout);
        }
        if (*expectedPayloadInt % 10000 == 9999)
        {
            printf("\n");
        }
#endif
        succeed = 1;
    }

    return succeed;
}

int receive(BM_ChannelHandle channel, BM_NotificationHandle notification, uint32_t nmessages, uint32_t ninc)
{
    BM_StatusTypeDef error = BM_ERROR_OK;

    uint32_t n = 0;
    uint32_t napifailures = 0;
    uint32_t nlostmsgs = 0;
    uint32_t nbadmsgs = 0;

    /* SWait for RX CAN message using notification handle assigned to the channel */
    printf("Waiting for RX notification ...\n");

    time_t t1 = time(NULL);
#ifdef BMAPI_MULTI_READ
    uint32_t nrecv = nmessages;
    BM_CanMessageTypeDef* messages = (BM_CanMessageTypeDef*)malloc(sizeof(BM_CanMessageTypeDef) * nrecv);
    error = BM_ReadMultipleCanMessage(channel, messages, &nrecv, -1, NULL, NULL);
    if (error != BM_ERROR_OK)
    {
        printf("Failed to read %d messages, received=%d.\n", nmessages, nrecv);
    }
    free(messages);
    n = nrecv;
#elif defined(BMAPI_ISOTP_READ)
    uint32_t nbytes = 1 * 1024 * 1024;
    uint32_t i;
    uint32_t iround;
    uint8_t* data = (uint8_t*)malloc(nbytes);
    BM_IsotpConfigTypeDef isotp;
    memset(&isotp, 0, sizeof(isotp));
    isotp.version = 1;
    isotp.mode = BM_ISOTP_NORMAL_ECU; /* Simulating a ECU UDS Server here */
    isotp.paddingEnabled = 1;
    isotp.paddingValue = 0xCCU;
    isotp.testerTimeout.b = 20000;
    isotp.callbackFunc = isotpCallback;
    isotp.callbackUserarg = 0;
    BM_INIT_CAN_FD_DATA(isotp.testerDataTemplate, UDS_REQUEST_MSG_ID, TEST_MSG_DLC, 0, 1, 1, 0, 0, NULL);
    BM_INIT_CAN_FD_DATA(isotp.ecuDataTemplate, UDS_RESPONSE_MSG_ID, TEST_MSG_DLC, 0, 1, 1, 0, 0, NULL);
    for (iround = 0; iround < nmessages; iround++)
    {
        uint32_t nrxbytes = nbytes;
        memset(data, 0, nbytes);
        printf("Read ISOTP response [%d/%d].\n", iround + 1, nmessages);
        error = BM_ReadIsotp(channel, data, &nrxbytes, TEST_MSG_RX_TIMEOUT, &isotp);
        if (error != BM_ERROR_OK)
        {
            printf("Failed to read ISOTP data, error=%08x.\n", error);
        }
        else
        {
            printf("Verify ISOTP response of %d bytes ...\n", nrxbytes);
            if (data[0] != 0x36)
            {
                printf("DATA[0] = %02x != 0x36\n", data[0]);
            }
            if (data[1] != ((iround + 1) & 0xFFU))
            {
                printf("DATA[0] = %02x != %02x\n", data[1], ((iround + 1) & 0xFFU));
            }
            for (i = 2; i < nrxbytes; i++)
            {
                uint8_t expectedPayloadByte = (uint8_t)((i - 2) & 0xFFU);
                if (*(uint8_t*)&data[i] != expectedPayloadByte)
                {
                    printf("DATA[%d] = %02x != %02x\n", i, *(uint8_t*)&data[i], expectedPayloadByte);
                    nbadmsgs++;
                    break;
                }
            }
        }
        printf("Done.\n");
        n += nrxbytes;
    }
    free(data);
#else
    BM_CanMessageTypeDef msg;
    uint32_t port;
    uint32_t timestamp;
    uint32_t expectedPayloadInt = 0;
    while (n < nmessages)
    {
        /* Once a new notification is received, read the CAN message out of BMAPI internal buffer */
        /* Note if you read before getting a new notification event, you might get BM_ERROR_QRCVEMPTY. */
        
        /* Wait until a new notification event arrived */
        if (BM_WaitForNotifications(&notification, 1, TEST_MSG_RX_TIMEOUT) < 0)
        {
            printf("Receive timeout, offset=%d.\n", n);
        }

        /* Note that a notification event might indicate multiple new messages, so keep reading until you cannot get more. */
        while ((error = BM_ReadCanMessage(channel, &msg, &port, &timestamp)) == BM_ERROR_OK)
        {
            n += verify(&msg, timestamp, &expectedPayloadInt, &nbadmsgs, &nlostmsgs);
            expectedPayloadInt += ninc;
        }
        /* BM_ERROR_QRCVEMPTY should not be considered as an error, otherwise report the error */
        if (error != BM_ERROR_QRCVEMPTY)
        {
            napifailures++;
            printf("\nFailed to receive message, error=%d.\n", error);
        }
    }
#endif
    time_t t2 = time(NULL);
    uint32_t td = (uint32_t)(t2 - t1);
    float fps = ((float)n) / ((float)td);

    printf(
        "\n"
        "================ RX STAT ================\n"
#ifdef BMAPI_ISOTP_READ
        "Received bytes: %u\n"
#else
        "Received messages: %d\n"
#endif
        "Lost messages: %u\n"
        "Bad messages: %u\n"
        "API Failures: %u\n"
        "Total time: %u seconds\n"
#ifdef BMAPI_ISOTP_READ
        "Throughput: %.3f B/s\n"
#else
        "Frame rate: %.3f fps\n"
#endif
        "=========================================\n",
        n, nlostmsgs, nbadmsgs, napifailures, td, fps
    );

    return n;
}

/**
 * End of file
 */
