/**
 * @file        multichannel_rx_cpp.c
 * @brief       BMAPI example: Notification based, highly efficient, multiple channel receiving example.
 * @author      busmust
 * @version     1.0.0.1
 * @copyright   Copyright 2022 by Busmust Tech Co.,Ltd <br>
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
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "bmapi.h"

#define BMAPI_CAN_MODE BM_CAN_NORMAL_MODE

#define TEST_MSG_RX_TIMEOUT 10000
#define TEST_STARTUP_TIMEOUT 0 /* Startup and wait for Busmaster to transmit test messages */
#define TEST_CHANNEL_COUNT 24
#define TEST_PATTERN_VERIFY /* Enable this macro to verify rx messages */

static const uint8_t dlc2len[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

uint32_t msgcount[TEST_CHANNEL_COUNT] = { 0 };
uint32_t errcount[TEST_CHANNEL_COUNT] = { 0 };

time_t t0 = 0;
time_t t1 = 0;
time_t t2 = 0;

#ifdef TEST_PATTERN_VERIFY
static bool verify(BM_CanMessageTypeDef* msg)
{
    if (msg->ctrl.rx.DLC >= 8)
    {
        /* Assuming valid test message to be: DLC >= 8, payload[*]=id&0xFF */
        /* e.g. id=0x123, dlc=8, payload=[0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23] */
        uint8_t len = dlc2len[msg->ctrl.rx.DLC];
        uint8_t pattern = (msg->id.SID & 0xFFU);
        for (uint8_t i = 0; i < len; i++)
        {
            if (msg->payload[i] != pattern)
            {
                return 0;
            }
        }

        return 1;
    }

    return 0;
}

static void print_msg(int channel, uint32_t timestamp, const BM_CanMessageTypeDef* msg, int isTxMsg)
{
    int len = dlc2len[msg->ctrl.rx.DLC];
    printf(
        "CH%d %s: ts=%u, id=%x, len=%d, payload=[",
        channel + 1, isTxMsg ? "TX" : "RX", timestamp, msg->id.SID, len
    );
    for (int i = 0; i < len; i++)
    {
        printf("%02x", msg->payload[i]);
    }
    printf("]\n");
}
#endif

static void print_stat(int nvalidchanels)
{
    int elapsed = t2 > t0 ? (int)(t2 - t0) : 1;
#ifdef _MSC_VER
    printf("======== Elapsed %d seconds ========\n", elapsed);
    for (int i = 0; i < nvalidchanels; i++)
    {
        printf("[%d] MSG=%d, ERR=%d, FPS=%d\n", i, msgcount[i], errcount[i], msgcount[i] / elapsed);
    }
    printf("====================================\n");
#else
    printf(
        "\033[1A\033[K\033[1A\033[K\033[1A\033[K"
        "MSG : %09u  %09u  %09u  %09u  %09u  %09u  %09u  %09u %09u  %09u  %09u  %09u  %09u  %09u  %09u  %09u %09u  %09u  %09u  %09u  %09u  %09u  %09u  %09u\n"
        "ERR : %09u  %09u  %09u  %09u  %09u  %09u  %09u  %09u %09u  %09u  %09u  %09u  %09u  %09u  %09u  %09u %09u  %09u  %09u  %09u  %09u  %09u  %09u  %09u\n"
        "FPS : %09u  %09u  %09u  %09u  %09u  %09u  %09u  %09u %09u  %09u  %09u  %09u  %09u  %09u  %09u  %09u %09u  %09u  %09u  %09u  %09u  %09u  %09u  %09u\n",
        msgcount[0], msgcount[1], msgcount[2], msgcount[3], msgcount[4], msgcount[5], msgcount[6], msgcount[7],
        msgcount[8], msgcount[9], msgcount[10], msgcount[11], msgcount[12], msgcount[13], msgcount[14], msgcount[15],
        msgcount[16], msgcount[17], msgcount[18], msgcount[19], msgcount[20], msgcount[21], msgcount[22], msgcount[23],
        errcount[0], errcount[1], errcount[2], errcount[3], errcount[4], errcount[5], errcount[6], errcount[7],
        errcount[8], errcount[9], errcount[10], errcount[11], errcount[12], errcount[13], errcount[14], errcount[15],
        errcount[16], errcount[17], errcount[18], errcount[19], errcount[20], errcount[21], errcount[22], errcount[23],
        msgcount[0] / elapsed, msgcount[1] / elapsed, msgcount[2] / elapsed, msgcount[3] / elapsed, 
        msgcount[4] / elapsed, msgcount[5] / elapsed, msgcount[6] / elapsed, msgcount[7] / elapsed,
        msgcount[8] / elapsed, msgcount[9] / elapsed, msgcount[10] / elapsed, msgcount[11] / elapsed,
        msgcount[12] / elapsed, msgcount[13] / elapsed, msgcount[14] / elapsed, msgcount[15] / elapsed,
        msgcount[16] / elapsed, msgcount[17] / elapsed, msgcount[18] / elapsed, msgcount[19] / elapsed,
        msgcount[20] / elapsed, msgcount[21] / elapsed, msgcount[22] / elapsed, msgcount[23] / elapsed
    );
#endif
}

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
    BM_ChannelHandle channels[TEST_CHANNEL_COUNT] = { 0 };
    BM_NotificationHandle notifications[TEST_CHANNEL_COUNT] = { 0 };
    BM_ChannelInfoTypeDef channelinfos[TEST_CHANNEL_COUNT] = { 0 };
    int nchannels = sizeof(channelinfos) / sizeof(channelinfos[0]);
    const char* channelids = argc > 1 ? argv[1] : "0-7";
    bool debug = argc > 2 ? (strcmp(argv[2], "--debug") == 0) : false;
    int firstchannelid = atoi(channelids);
    int lastchannelid = firstchannelid;
    int notificationIndex = -1;
    int exitcode = 0;
    BM_BitrateTypeDef bitrate;
    BM_StatusTypeDef error = BM_ERROR_OK;
    int i;

    if (channelids)
    {
        const char* lastidstr = strchr(channelids, '-');
        if (lastidstr)
        {
            lastchannelid = atoi(&lastidstr[1]);
        }
    }

    printf(
        "\n"
        "========== BMAPI TEST PROGRAM ==========\n"
        "========= BUSMUST TECH Co.,Ltd. ========\n"
    );
    printf("========= BMAPI VER [%08x] =========\n", BM_GetVersion());

    /* Step1: Initialize BMAPI library before any other operation */
    printf("Initializing BMAPI library ...\n");
    error = BM_Init();
    if (error != BM_ERROR_OK)
    {
        exitcode = 1;
        goto __exit;
    }

#ifdef RESET_DEVICE_ON_STARTUP
    // The code below will recover everything even if the device is in unknown error state, as long as USB is still responsive.
    printf("Reset device.\n");
    BM_ResetDevice(BM_OpenCan(0)); // Assuming there is only 1 BM USB device connected.
    printf("Reset done.\n");
    usleep(TEST_STARTUP_TIMEOUT * 1000);
    printf("Reopen.\n");
#endif

    if (debug)
    {
        printf("Enter DEBUG mode.\n");
        BM_SetLogLevel(BM_LOG_DBG);
    }

    /* Step2: Enumerate connected device channels */
    printf("Enumerating channels ...\n");
    error = BM_Enumerate(channelinfos, &nchannels);
    if (error != BM_ERROR_OK || lastchannelid >= nchannels)
    {
        printf("Input Error: There are %d available Busmust device, port index %d is invalid.\n", nchannels, lastchannelid);
        exitcode = 2;
        goto __exit;
    }

    for (i = 0; i < nchannels; i++)
    {
        printf(
            "[%d] %s (firmware: %d.%d.%d.%d)\n", i, channelinfos[i].name, 
            channelinfos[i].version[0], channelinfos[i].version[1], channelinfos[i].version[2], channelinfos[i].version[3]
        );
    }
    for (int channelid = firstchannelid; channelid <= lastchannelid; channelid++)
    {
        /* Step3: Open the selected channel and configure baudrate */
        memset(&bitrate, 0, sizeof(bitrate));
        bitrate.nbitrate = 500;
        bitrate.dbitrate = 2000;
        bitrate.nsamplepos = 80;
        bitrate.dsamplepos = 80;
        /* BTR is not mandatory, hardware will calculate BTR using bitrate&samplepos automatically */
        error = BM_OpenEx(&channels[channelid], &channelinfos[channelid], BMAPI_CAN_MODE, BM_TRESISTOR_120, &bitrate, NULL, 0);
        if (error != BM_ERROR_OK)
        {
            exitcode = 3;
            goto __exit;
        }
        printf("Opened channel %s in RX mode (channel=%p) ...\n", channelinfos[channelid].name, channels[channelid]);

        error = BM_GetNotification(channels[channelid], &notifications[channelid]);
        if (error != BM_ERROR_OK)
        {
            exitcode = 4;
            goto __exit;
        }
    }
    printf("Waiting %d milliseconds for TX app to startup ...\n", TEST_STARTUP_TIMEOUT);
    usleep(TEST_STARTUP_TIMEOUT * 1000);

    /* Step4: Listen for notifications in parallel  */
    int nvalidchannels = lastchannelid - firstchannelid + 1;
    printf("Channel [%d-%d] RX stats :\n\n\n\n", firstchannelid, lastchannelid);
    while ((notificationIndex = BM_WaitForNotifications(&notifications[firstchannelid], nvalidchannels, TEST_MSG_RX_TIMEOUT)) >= 0)
    {
        uint64_t channelpending = (1ULL << nvalidchannels) - 1ULL;
        while (channelpending)
        {
            for (int notificationIndex = 0; notificationIndex < nvalidchannels; notificationIndex++)
            {
                int sourcechannelid = firstchannelid + notificationIndex;
                BM_StatusTypeDef error = BM_ERROR_OK;
                BM_DataTypeDef data;

                error = BM_Read(channels[sourcechannelid], &data);
                if (error == BM_ERROR_OK)
                {
                    msgcount[sourcechannelid]++;
#ifdef TEST_PATTERN_VERIFY
                    BM_CanMessageTypeDef* msg = (BM_CanMessageTypeDef*)data.payload;
                    bool isTxMsg = data.header.type & BM_ACK_DATA;
                    if (!verify(msg))
                    {
                        print_msg(sourcechannelid, data.timestamp, msg, isTxMsg);
                        errcount[sourcechannelid]++;
                    }
#endif
                }
                else if (error == BM_ERROR_QRCVEMPTY)
                {
                    /* BM_ERROR_QRCVEMPTY should not be considered as an error, otherwise report the error */
                    channelpending &= ~(1ULL << sourcechannelid);
                    continue;
                }
                else
                {
                    char errormsg[256] = { 0 };
                    BM_GetErrorText(error, errormsg, sizeof(errormsg), 0);
                    printf("\nFailed to receive message, error=%d(%s).\n", error, errormsg);
                    exitcode = 5;
                    goto __exit;
                }

                if ((t2 = time(NULL)) >= t1 + 3)
                {
                    /* Print RX stat each second */
                    //print_stat(nvalidchannels);
                    t1 = t2;
                }
                if (!t0)
                {
                    t0 = t2;
                }
            }
        }
    }
    t2 -= TEST_MSG_RX_TIMEOUT / 1000;
    print_stat(nvalidchannels);
    printf("\nTimeout, stopping app.\n");

    /* Step5: Close the opened channel and release connected hardware */
    for (int channelid = firstchannelid; channelid <= lastchannelid; channelid++)
    {
        printf("Closing channel %s ...\n", channelinfos[channelid].name);
        error = BM_Close(channels[channelid]);
        if (error != BM_ERROR_OK)
        {
            exitcode = 5;
            goto __exit;
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
    BM_UnInit();
    printf("Exit RX app.\n");
    return exitcode;
}

/**
 * End of file
 */
