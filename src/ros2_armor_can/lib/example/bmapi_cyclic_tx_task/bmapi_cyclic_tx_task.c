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

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
typedef int BOOL;

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

#ifdef xBMAPI_LOOPBACK_WRITE
#define BMAPI_CAN_MODE BM_CAN_EXTERNAL_LOOPBACK_MODE
#else
#define BMAPI_CAN_MODE BM_CAN_NORMAL_MODE
#endif

#define xBMAPI_PARALLEL_READ_WRITE /* Not recommended, For partner test only */
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
#define TEST_MSG_COUNT  30
#define TEST_MSG_ROUND  100
#define TEST_MSG_CYCLE  10
#define TEST_MSG_TX_TIMEOUT 1000
#define TEST_MSG_RX_TIMEOUT 30000
#define APP_N_TASK 16
#define APP_N_CHANNEL 32
#define MIN2(x,y) (((x) < (y)) ? (x) : (y) )

static int transmit(BM_ChannelHandle channel, BM_NotificationHandle notification, uint32_t nroundmessages, uint32_t nrounds, uint32_t cycle);
void app_processTxTask(BM_ChannelHandle handle);
int dataBytesToDlc(int n);
static BM_StatusTypeDef APP_SetTxTasks(BM_ChannelInfoTypeDef* info, BM_TxTaskTypeDef* txtasks, int ntxtasks);

static int m_nchannels;
int txTaskCounters[APP_N_CHANNEL][APP_N_TASK] = { 0 };
int txTaskRounds[APP_N_CHANNEL][APP_N_TASK] = { 0 };
int txTaskTimers[APP_N_CHANNEL][APP_N_TASK] = { 0 };
BM_TxTaskTypeDef m_tasks[APP_N_CHANNEL][APP_N_TASK];
BM_CanMessageTypeDef m_canMessage[TEST_MSG_COUNT];
BM_ChannelInfoTypeDef m_channelinfos[BMAPI_MAX_CHANNEL_COUNT];
//BM_ChannelHandle m_channel[APP_N_CHANNEL];

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
	int nchannels = BMAPI_MAX_CHANNEL_COUNT;
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
	error = BM_Enumerate(m_channelinfos, &nchannels);
	if (error != BM_ERROR_OK)
	{
		exitcode = 2;
		goto __exit;
	}

	for (i = 0; i < nchannels; i++)
	{
		printf("[%d] %s\n", i, m_channelinfos[i].name);
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
			printf("Opening channel %s in %s mode ...\n", m_channelinfos[channelid].name, mode);
			memset(&bitrate, 0, sizeof(bitrate));
			bitrate.nbitrate = 500;
			bitrate.dbitrate = 2000;
			bitrate.nsamplepos = 75;
			bitrate.dsamplepos = 75;
			/* BTR is not mandatory, hardware will calculate BTR using bitrate&samplepos automatically */
			error = BM_OpenEx(&channel, &m_channelinfos[channelid], BMAPI_CAN_MODE, BM_TRESISTOR_120, &bitrate, NULL, 0);
			if (error != BM_ERROR_OK)
			{
				printf("Failed to open %s, error=0x%08x.\n", m_channelinfos[channelid].name, error);
				exitcode = 3;
				goto __exit;
			}

			printf("Getting channel notification handle for channel %d(0x%p)...\n", channelid, channel);

			if (error != BM_ERROR_OK)
			{
				exitcode = 4;
				goto __exit;
			}
		}

		for (channelid = firstchannelid; channelid <= lastchannelid; channelid++)
		{
			/* Step5: TX|RX (according to command line arguments) */
			if (argc > 2 && strcmp(argv[2], "tx") == 0)
			{
				uint32_t nmessages = argc > 3 ? atoi(argv[3]) : TEST_MSG_COUNT;
				uint32_t nrounds = argc > 4 ? atoi(argv[4]) : TEST_MSG_ROUND;
				uint32_t ncyclems = argc > 5 ? atoi(argv[5]) : TEST_MSG_CYCLE;

				transmit(channel, NULL, nmessages, nrounds, ncyclems);
			}
		}

		for (channelid = firstchannelid; channelid <= lastchannelid; channelid++)
		{
			printf("Close channel %d ...\n", channelid);
			if (BM_Close(channel) != BM_ERROR_OK)
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

static int transmit(BM_ChannelHandle channel, BM_NotificationHandle notification, uint32_t nroundmessages, uint32_t nrounds, uint32_t cycle)
{
	uint32_t n = 0;
	uint32_t nbadmsgs = 0;
	uint32_t nmessages = ((uint32_t)nroundmessages) * ((uint32_t)nrounds);
	BM_CanMessageTypeDef msg;
	BM_StatusTypeDef error = BM_ERROR_OK;
	error = BM_GetNotification(channel, &notification);
	/* Compose a CAN message object for TX */
	printf("Transmitting test message ...\n");
	memset(&msg, 0, sizeof(msg));
	msg.id.SID = TEST_MSG_ID;
	msg.ctrl.tx.DLC = TEST_MSG_DLC;
	msg.ctrl.tx.FDF = 1;
	msg.ctrl.tx.BRS = 1;

	time_t t1 = time(NULL);

	/**
	 * @brief Transmit messages using hardware-accelerated TxTask:
	 *        Configure the TX pattern (i.e. INCDATA) to the USB device as a task,
	 *        it will execute the task and receive TXCMPLT as notifications.
	 *        In this case we could reach the highest TX throughput.
	 */
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
	txtask.id = TEST_MSG_ID;
	txtask.length = (uint8_t)dlc2len[TEST_MSG_DLC];
	txtask.flags = BM_MESSAGE_FLAGS_FDF | BM_MESSAGE_FLAGS_BRS;
	memset(txtask.payload, 0, sizeof(txtask.payload));

	//APP_SetTxTasks
	BM_TxTaskTypeDef m_txtask[APP_N_TASK];
	for (size_t i = 0; i < APP_N_TASK; i++)
	{
		memcpy(&m_txtask[i], &txtask, sizeof(txtask));
	}
	int ntask = 12;
	error = APP_SetTxTasks(&m_channelinfos[0], m_txtask, ntask);

	if (BM_ERROR_OK != error)
	{
		printf("Failed to configure tx task, error=%d.\n", error);
	}

	while (TRUE)
	{
		uint32_t cycleCount = 0;
		while (cycleCount < cycle)
		{
			cycleCount++;
			app_processTxTask(channel);
		}

		if (n < nmessages * ntask)
		{
			if (BM_WaitForNotifications(&notification, 1, TEST_MSG_TX_TIMEOUT) < 0)
			{
				printf("Transmit timeout, offset=%d.\n", n);
				break;
			}
		}
		BM_DataTypeDef ackData;
		static BM_StatusTypeDef lastStatus = BM_ERROR_OK;
		static int count = 0;
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
		if (lastStatus == error && lastStatus != BM_ERROR_OK)
		{
			count++;
		}
		else
		{
			count = 0;
		}
		if (count > 1000)
		{
			printf("Transmit complete, Total message=%d.\n", n);
			break;
		}
		lastStatus = error;
		usleep(1000); // 1ms
	}

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

static BM_StatusTypeDef APP_SetTxTasks(BM_ChannelInfoTypeDef* info, BM_TxTaskTypeDef* txtasks, int ntxtasks)
{
	for (int i = 0; i < ntxtasks; i++)
	{
		txTaskCounters[info->port][i] = 0;
		txTaskTimers[info->port][i] = 0;
		txTaskRounds[info->port][i] = 0;
		memcpy(&m_tasks[info->port][i], &txtasks[i], sizeof(txtasks[i]));
	}
	return BM_ERROR_OK;
}

void app_processTxTask(BM_ChannelHandle handle)
{
	uint32_t index;
	uint32_t count;
	uint16_t channel;
	for (channel = 0; channel < APP_N_CHANNEL; channel++)
	{
		for (index = 0U; index < APP_N_TASK; index++)
		{
			const BM_TxTaskTypeDef* ptask = &m_tasks[channel][index];
			BOOL transmitNow = FALSE;
			if (ptask->type == BM_TXTASK_INVALID)
			{
				transmitNow = FALSE;
			}
			else if (ptask->nrounds != 0xFFFFU && txTaskRounds[channel][index] >= ptask->nrounds)
			{
				/* Task already finished, re-configure all tasks to re-start this task. */
				transmitNow = FALSE;
			}
			else if (txTaskCounters[channel][index])
			{
				/* Current round ongoing, maybe there was no enough room in CANTXQ on previous tick, retry immediately in this tick */
				transmitNow = FALSE;
			}
			else if (txTaskTimers[channel][index])
			{
				/* Counting for next round */
				txTaskTimers[channel][index] -= 1U;
				transmitNow = !txTaskTimers[channel][index]; /* Check again because it might has been reloaded with counter */
			}
			else
			{
				/* Count timeout, time for next round! */
				transmitNow = TRUE;
			}
			if (transmitNow)
			{
				/* Transmit the message (to message Q) */
				for (count = txTaskCounters[channel][index]; count < ptask->nmessages; count++)
				{
					BM_CanMessageTypeDef canfd;
					*(uint32_t*)&canfd.id = 0;
					canfd.id.SID = ptask->id & 0x7FFU;
					canfd.id.EID = (ptask->id & 0x1FFFF800U) >> 11;
					*(uint32_t*)&canfd.ctrl.tx = 0;
					canfd.ctrl.tx.DLC = dataBytesToDlc(ptask->length);
					canfd.ctrl.tx.BRS = (ptask->flags & BM_MESSAGE_FLAGS_BRS) ? 1 : 0;
					canfd.ctrl.tx.ESI = (ptask->flags & BM_MESSAGE_FLAGS_ESI) ? 1 : 0;
					canfd.ctrl.tx.FDF = (ptask->flags & BM_MESSAGE_FLAGS_FDF) ? 1 : 0;
					canfd.ctrl.tx.IDE = (ptask->flags & BM_MESSAGE_FLAGS_IDE) ? 1 : 0;
					canfd.ctrl.tx.RTR = (ptask->flags & BM_MESSAGE_FLAGS_RTR) ? 1 : 0;
					canfd.ctrl.tx.SEQ = 0x80U | (count & 0x7FU); /* 8-bit */
					memcpy(canfd.payload, ptask->payload, MIN2(ptask->length, sizeof(canfd.payload)));
					m_canMessage[count] = canfd;
				}
				int nmessage = count;
				BM_StatusTypeDef aa = BM_WriteMultipleCanMessage(handle, m_canMessage, &nmessage, 0, 0, 0);
				txTaskCounters[channel][index] = count;
				if (txTaskCounters[channel][index] >= ptask->nmessages)
				{
					txTaskCounters[channel][index] = 0U;
					txTaskRounds[channel][index] += 1U;
					txTaskTimers[channel][index] = ptask->cycle;
				}
			}
		}
	}
}


int dataBytesToDlc(int n)
{
	int dlc = 0;
	if (n <= 8) {
		dlc = n;
	}
	else if (n <= 12) {
		dlc = 9;
	}
	else if (n <= 16) {
		dlc = 10;
	}
	else if (n <= 20) {
		dlc = 11;
	}
	else if (n <= 24) {
		dlc = 12;
	}
	else if (n <= 32) {
		dlc = 13;
	}
	else if (n <= 48) {
		dlc = 14;
	}
	else if (n <= 64) {
		dlc = 15;
	}
	return dlc;
}

/**
 * End of file
 */
