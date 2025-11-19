/**
 * @file        multichannel_logger.c
 * @brief       BMAPI example: Notification based, highly efficient, multiple channel receiving and logging example.
 * @author      busmust
 * @version     1.0.0.1
 * @copyright   Copyright 2024 by Busmust Tech Co.,Ltd <br>
 *              All rights reserved. Property of Busmust Tech Co.,Ltd.<br>
 *              Restricted rights to use, duplicate or disclose of this code are granted through contract.
 */
 //#define __GNUC__
#define _CRT_SECURE_NO_WARNINGS
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "bmapi.h"
#include <fcntl.h>
#ifdef __GNUC__
#include <unistd.h>
#include <sys/time.h>
#elif defined(_MSC_VER)
#include <Windows.h>
#define usleep(us) Sleep((us)/1000)
int gettimeofday(struct timeval* tp, void* tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = (long)clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif
#define BMAPI_CAN_MODE BM_CAN_NORMAL_MODE


#define TEST_MSG_RX_TIMEOUT 100
#define TEST_STARTUP_TIMEOUT 1 /* Startup and wait for Busmaster to transmit test messages */
#define TEST_CHANNEL_COUNT 24
#define PAYLOAD_COUNT 4
#define xTEST_PATTERN_VERIFY /* Enable this macro to verify rx messages */

uint32_t msgcount[TEST_CHANNEL_COUNT] = { 0 };
uint32_t errcount[TEST_CHANNEL_COUNT] = { 0 };

time_t t0 = 0;
time_t t1 = 0;
time_t t2 = 0;


typedef struct
{
	BM_BitrateTypeDef bitrate;
	int mode;
	int tres;
	const char* channels;
	const char* asc_name;
	int seg_sec;
} BM_Cmdline_t;


#ifdef TEST_PATTERN_VERIFY
static const uint8_t dlc2len[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

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

int canDlcToLength(int dlc)
{
	uint32_t DlcToDataBytes[16] =
	{
		0,  1,  2,  3,  4,  5,  6,  7,
		8, 12, 16, 20, 24, 32, 48, 64,
	};
	uint32_t length = dlc < 16 ? DlcToDataBytes[dlc] : 0;
	return length;
}

void hex2asc(unsigned char* s1, char* s2, int len)
{
	static const unsigned char hextable[] = { "0123456789ABCDEF" };
	int DataL, DataH;
	int i;
	for (i = 0; i < len; i++)
	{
		DataL = s2[i] & 0x0F;
		DataH = (s2[i] >> 4) & 0x0F;
		s1[3 * i] = hextable[DataH];
		s1[3 * i + 1] = hextable[DataL];
		s1[3 * i + 2] = ' ';
	}
	s1[len * 3] = '\0';
}

FILE* creat_ascfile(FILE* file, int* count,const char* name)
{
	char path_buffer[1024];
	if (file)
	{
		fclose(file);
	}
	(*count)++;
	sprintf(path_buffer, "%s_%d.asc", name, *count);
	file = fopen(path_buffer, "w+");
	return file;
}

static __inline char __can_dataBytesToDlc(uint8_t n)
{
	char dlc = '0';

	if (n <= 8) {
		dlc = '0' + n;
	}
	else if (n <= 12) {
		dlc = '9';
	}
	else if (n <= 16) {
		dlc = 'a';
	}
	else if (n <= 20) {
		dlc = 'b';
	}
	else if (n <= 24) {
		dlc = 'c';
	}
	else if (n <= 32) {
		dlc = 'd';
	}
	else if (n <= 48) {
		dlc = 'e';
	}
	else {
		dlc = 'f';
	}

	return dlc;
}

static struct timeval write_asc_header(FILE* file)
{
	const char* dayOfWeek[] = { "Sun" , "Mon" , "Tue", "Wed" , "Thu" , "Fri" , "Sat" , "Fre" };
	const char* monthOfYear[] = { "?", "Jan" , "Feb" , "Mar", "Apr" , "May" , "Jun" , "Jul" , "Aug", "Sep" , "Oct" , "Nov" , "Dec" };
	struct tm tm;
	time_t timep = 0;
	timep = time(NULL);
	tm = *(localtime(&(timep)));
	struct timeval timeval = { 0 };
	gettimeofday(&timeval, NULL);
	char szBuffer[512] = { 0 };
	sprintf(szBuffer, "date %s %s %d %d:%d:%d %s %d\n%s%s%s%s",
		dayOfWeek[tm.tm_wday],
		monthOfYear[tm.tm_mon + 1],
		tm.tm_mday,
		tm.tm_hour % 12,
		tm.tm_min,
		tm.tm_sec,
		tm.tm_hour > 12 ? "pm" : "am",
		tm.tm_year + 1900,
		"base hex  ",
		"timestamps absolute\n",
		"no internal events logged\n",
		"// version 8.1.0\n");
	fputs(szBuffer, file);
	return timeval;
}

static void write_asc_message(FILE* file, BM_DataTypeDef* data, struct timeval header_time)
{
	unsigned char ucData[64 * PAYLOAD_COUNT + 8] = { 0 };
	BM_CanMessageTypeDef* canMessage = (BM_CanMessageTypeDef*)data->payload;
	uint32_t mid = 0;
	bool ide = false;
	if (canMessage->ctrl.rx.IDE)
	{
		ide = true;
		mid = ((unsigned int)canMessage->id.EID << (11)) | ((unsigned int)canMessage->id.SID << 0U);
	}
	else
	{
		mid = canMessage->id.SID;
	}

	uint32_t flags = 0;
	if ((canMessage->ctrl.rx.FDF) || (canMessage->ctrl.rx.BRS))
	{
		flags |= 0x001000;
	}
	if (canMessage->ctrl.rx.BRS)
	{
		flags |= 0x002000;
	}
	if (data->header.type & (BM_ACK_DATA | BM_CAN_FD_DATA))
	{
		flags |= (0x1 << 6);
	}
	if (canMessage->ctrl.rx.RTR)
	{
		flags |= (0x1 << 4);
	}
	int length = canDlcToLength(canMessage->ctrl.rx.DLC);
	hex2asc(ucData, (char*)canMessage->payload, length);

	struct timeval current_time = { 0 };
	gettimeofday(&current_time, NULL);

	double timestamp = (current_time.tv_sec - header_time.tv_sec) + (current_time.tv_usec - header_time.tv_usec) / 1000000.0;
	unsigned char* pData = ucData;
	char szBuffer[512] = { 0 };
	sprintf(szBuffer, "%.6f CANFD %d %s %x%s %s  %d %d %c %d %s   %x %x %06x %x %x %x %x %x\n",
		(float)((double)timestamp),
		data->header.schn + 1,
		(data->header.type & (BM_ACK_DATA | BM_CAN_FD_DATA)) ? "Tx" : "Rx",
		mid,
		ide ? "x" : "",
		"", /* Message name */
		canMessage->ctrl.rx.BRS ? 1 : 0, /* BRS */
		0, /* ESI */
		__can_dataBytesToDlc(length),
		length,
		pData,
		0,
		0,
		flags,
		0, 0, 0, 0, 0);

	fputs(szBuffer, file);
}

static void parse_cmdLine_args(int argc, char* argv[], BM_Cmdline_t* cmdline)
{
	int iarg = 1;
	while (iarg < argc)
	{
		int value = 0;
		if (strcmp(argv[iarg], "--rbitrate") == 0)
		{
			iarg++;
			value = (int)strtoul(argv[iarg], NULL, 10);
			if (value)
			{
				cmdline->bitrate.nbitrate = value;
			}
		}
		else if (strcmp(argv[iarg], "--dbitrate") == 0)
		{
			iarg++;
			value = (int)strtoul(argv[iarg], NULL, 10);
			if (value)
			{
				cmdline->bitrate.dbitrate = value;
			}
		}
		else if (strcmp(argv[iarg], "--rsamplepos") == 0)
		{
			iarg++;
			value = (int)strtoul(argv[iarg], NULL, 10);
			if (value)
			{
				cmdline->bitrate.dsamplepos = value;
			}
		}
		else if (strcmp(argv[iarg], "--dsamplepos") == 0)
		{
			iarg++;
			value = (int)strtoul(argv[iarg], NULL, 10);
			if (value)
			{
				cmdline->bitrate.dsamplepos = value;
			}
		}
		else if (strcmp(argv[iarg], "--mode") == 0)
		{
			iarg++;
			if (strcmp(argv[iarg], "classic") == 0)
			{
				cmdline->mode = BM_CAN_CLASSIC_MODE;
			}
			else if (strcmp(argv[iarg], "normal") == 0)
			{
				cmdline->mode = BM_CAN_NORMAL_MODE;
			}
			else if (strcmp(argv[iarg], "listenonly") == 0)
			{
				cmdline->mode = BM_CAN_LISTEN_ONLY_MODE;
			}
			else
			{
				printf("Invalid mode value %s, set to default value BM_CAN_NORMAL_MODE.\n", argv[iarg]);
				cmdline->mode = BM_CAN_NORMAL_MODE;
			}
		}
		else if (strcmp(argv[iarg], "--tres") == 0)
		{
			iarg++;
			if (strcmp(argv[iarg], "off") == 0)
			{
				cmdline->tres = BM_TRESISTOR_DISABLED;
			}
			else if (strcmp(argv[iarg], "on") == 0 || strcmp(argv[iarg], "120") == 0)
			{
				cmdline->tres = BM_TRESISTOR_120;
			}
			else
			{
				printf("Invalid tseg value %s, set to default value 120.\n", argv[iarg]);
			}
		}
		else if (strcmp(argv[iarg], "--seg") == 0)
		{
			iarg++;
			cmdline->seg_sec = (int)strtoul(argv[iarg], NULL, 10);
		}
		else
		{
			break;
		}
		iarg++;
	}
	cmdline->channels = iarg < argc ? argv[iarg++] : NULL;

	cmdline->asc_name = iarg < argc ? argv[iarg++] : NULL;
}
FILE* add_ascfile(FILE * pf,time_t* oldTimep, BM_Cmdline_t* cmdline, int* ascCount, struct timeval* header_time)
{
	if (cmdline->seg_sec)
	{
		time_t timep = 0;
		timep = time(NULL);
		time_t diffTime;
		diffTime = (time_t)difftime(timep, *oldTimep);

		if ((int)diffTime >= cmdline->seg_sec)
		{
			pf = creat_ascfile(pf, ascCount, cmdline->asc_name);
			if (pf)
			{
				*header_time = write_asc_header(pf);
			}
			else
			{
				printf("Failed to create file %s.", cmdline->asc_name);
			}
			*oldTimep = timep;
		}
	}
	return pf;
}


/**
 * @brief BMAPI test program: Transmit or receive messages according to command line arguments:
 * @param[in]  argc    Number of command line arguments
 * @param[in]  argv[0] Path of this program
 * @param[in]  argv[1] multiChannelLogger
 * @param[in]  argv[2] Rbitrate: Followed by a number, no specified parameters default to 500
 * @param[in]  argv[3] Dbitrate: Followed by a number, no specified parameters default to 2000
 * @param[in]  argv[4] Rsamplepos: Followed by a number, no specified parameters default to 75
 * @param[in]  argv[5] Dsamplepos: Followed by a number, no specified parameters default to 80
 * @param[in]  argv[6] Mode: Followed by a number, no specified parameters default to narmal
 * @param[in]  argv[7] Tres: Followed by a number, no specified parameters default to 120
 * @param[in]  argv[8] Seg: Followed by a number, no specified parameters default to 120
 * @param[in]  argv[9] Channels: Number of rounds to transmit
 * @param[in]  argv[10] AscFileName: Round cycle, a delay value in milliseconds between consecutive rounds
 * @return     Program exit code
 */
int main(int argc, char* argv[])
{
	BM_StatusTypeDef error = BM_ERROR_OK;
	BM_ChannelHandle channels[TEST_CHANNEL_COUNT] = { 0 };
	BM_NotificationHandle notifications[TEST_CHANNEL_COUNT] = { 0 };
	BM_ChannelInfoTypeDef channelinfos[TEST_CHANNEL_COUNT] = { 0 };
	int nchannels = sizeof(channelinfos) / sizeof(channelinfos[0]);
	BM_Cmdline_t cmdline;
	if (argc < 3)
	{
		printf("Usage: multichannel_logger {any long options} <channel-range> <asc-file-name>\n");
		return 0;
	}
	memset(&cmdline, 0, sizeof(cmdline));
	cmdline.bitrate.nbitrate = 500;
	cmdline.bitrate.dbitrate = 2000;
	cmdline.bitrate.dsamplepos = 80;
	cmdline.bitrate.nsamplepos = 75;
	cmdline.mode = BM_CAN_NORMAL_MODE;
	cmdline.tres = BM_TRESISTOR_120;
	parse_cmdLine_args(argc, argv, &cmdline);

	int firstchannelid = atoi(&cmdline.channels[0]);
	int lastchannelid = firstchannelid;
	int notificationIndex = -1;
	int exitcode = 0;
	BM_BitrateTypeDef bitrate;
	bitrate = cmdline.bitrate;
	int i;
	FILE* pf;

	if (cmdline.channels)
	{
		const char* lastidstr = strchr(cmdline.channels, '-');
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

	//if (debug)
	//{
	//    printf("Enter DEBUG mode.\n");
	//    BM_SetLogLevel(BM_LOG_DBG);
	//}

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
		/* BTR is not mandatory, hardware will calculate BTR using bitrate&samplepos automatically */
		memset(&bitrate, 0, sizeof(bitrate));
		bitrate.nbitrate = cmdline.bitrate.nbitrate;
		bitrate.dbitrate = cmdline.bitrate.dbitrate;
		bitrate.nsamplepos = cmdline.bitrate.nsamplepos;
		bitrate.dsamplepos = cmdline.bitrate.dsamplepos;
		error = BM_OpenEx(&channels[channelid], &channelinfos[channelid], cmdline.mode, cmdline.tres, &bitrate, NULL, 0);
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

	struct timeval header_time = { 0 };



	printf("Waiting %d milliseconds for TX app to startup ...\n", TEST_STARTUP_TIMEOUT);
	usleep(TEST_STARTUP_TIMEOUT * 1000);

	/* Step4: Listen for notifications in parallel  */
	int nvalidchannels = lastchannelid - firstchannelid + 1;
	printf("Channel [%d-%d] RX stats :\n\n\n\n", firstchannelid, lastchannelid);

	int ascCount = 1;
	char path_buffer[1024];
	sprintf(path_buffer, "%s_%d.asc", cmdline.asc_name, ascCount);
	pf = fopen(path_buffer, "w+");

	if (pf)
	{
		header_time = write_asc_header(pf);
	}
	else
	{
		printf("Failed to create ASC file %s.\n", cmdline.asc_name);
		goto __exit;
	}
	time_t oldTimep = 0;
	oldTimep = time(NULL);
	while (true)
	{
		if ((notificationIndex = BM_WaitForNotifications(&notifications[firstchannelid], nvalidchannels, TEST_MSG_RX_TIMEOUT)) >= 0)
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

						pf = add_ascfile(pf, &oldTimep, &cmdline, &ascCount, &header_time);
						if (pf)
						{
							write_asc_message(pf, &data, header_time);
						}
#ifdef TEST_PATTERN_VERIFY
						if (!verify(msg))
						{
							bool isTxMsg = false;
							isTxMsg = (bool)data.header.type & BM_ACK_DATA;
							BM_CanMessageTypeDef* msg = NULL;
							msg = (BM_CanMessageTypeDef*)data.payload;
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
						print_stat(nvalidchannels);
						t1 = t2;
					}
					if (!t0)
					{
						t0 = t2;
					}
				}
			}
		}
		else
		{
			pf = add_ascfile(pf, &oldTimep, &cmdline, &ascCount, &header_time);
		}
	}
	t2 -= TEST_MSG_RX_TIMEOUT / 1000;
	print_stat(nvalidchannels);
	printf("\nTimeout, stopping app.\n");
	fclose(pf);

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
