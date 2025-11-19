/**
 * @file        bm.h
 * @brief       Busmust device communication API.
 * @author      busmust
 * @version     1.4.0.5
 * @copyright   Copyright 2020 by Busmust Tech Co.,Ltd <br>
 *              All rights reserved. Property of Busmust Tech Co.,Ltd.<br>
 *              Restricted rights to use, duplicate or disclose of this code are granted through contract.
 */
#ifndef __BMAPI_H__
#define __BMAPI_H__

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////// stdint.h //////////////////////////////////////////////////////////////////////////
#ifndef _STDINT

typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;

typedef signed char        int_least8_t;
typedef short              int_least16_t;
typedef int                int_least32_t;
typedef long long          int_least64_t;
typedef unsigned char      uint_least8_t;
typedef unsigned short     uint_least16_t;
typedef unsigned int       uint_least32_t;
typedef unsigned long long uint_least64_t;

typedef signed char        int_fast8_t;
typedef int                int_fast16_t;
typedef int                int_fast32_t;
typedef long long          int_fast64_t;
typedef unsigned char      uint_fast8_t;
typedef unsigned int       uint_fast16_t;
typedef unsigned int       uint_fast32_t;
typedef unsigned long long uint_fast64_t;

typedef long long          intmax_t;
typedef unsigned long long uintmax_t;

#ifndef _UINTPTR_T_DEFINED
    #define _UINTPTR_T_DEFINED
    #ifdef _WIN64
        typedef unsigned __int64  uintptr_t;
    #else
        typedef unsigned int uintptr_t;
    #endif
#endif

// These macros must exactly match those in the Windows SDK's intsafe.h.
#define INT8_MIN         (-127i8 - 1)
#define INT16_MIN        (-32767i16 - 1)
#define INT32_MIN        (-2147483647i32 - 1)
#define INT64_MIN        (-9223372036854775807i64 - 1)
#define INT8_MAX         127i8
#define INT16_MAX        32767i16
#define INT32_MAX        2147483647i32
#define INT64_MAX        9223372036854775807i64
#define UINT8_MAX        0xffui8
#define UINT16_MAX       0xffffui16
#define UINT32_MAX       0xffffffffui32
#define UINT64_MAX       0xffffffffffffffffui64

#define INT_LEAST8_MIN   INT8_MIN
#define INT_LEAST16_MIN  INT16_MIN
#define INT_LEAST32_MIN  INT32_MIN
#define INT_LEAST64_MIN  INT64_MIN
#define INT_LEAST8_MAX   INT8_MAX
#define INT_LEAST16_MAX  INT16_MAX
#define INT_LEAST32_MAX  INT32_MAX
#define INT_LEAST64_MAX  INT64_MAX
#define UINT_LEAST8_MAX  UINT8_MAX
#define UINT_LEAST16_MAX UINT16_MAX
#define UINT_LEAST32_MAX UINT32_MAX
#define UINT_LEAST64_MAX UINT64_MAX

#define INT_FAST8_MIN    INT8_MIN
#define INT_FAST16_MIN   INT32_MIN
#define INT_FAST32_MIN   INT32_MIN
#define INT_FAST64_MIN   INT64_MIN
#define INT_FAST8_MAX    INT8_MAX
#define INT_FAST16_MAX   INT32_MAX
#define INT_FAST32_MAX   INT32_MAX
#define INT_FAST64_MAX   INT64_MAX
#define UINT_FAST8_MAX   UINT8_MAX
#define UINT_FAST16_MAX  UINT32_MAX
#define UINT_FAST32_MAX  UINT32_MAX
#define UINT_FAST64_MAX  UINT64_MAX

#ifdef _WIN64
    #define INTPTR_MIN   INT64_MIN
    #define INTPTR_MAX   INT64_MAX
    #define UINTPTR_MAX  UINT64_MAX
#else
    #define INTPTR_MIN   INT32_MIN
    #define INTPTR_MAX   INT32_MAX
    #define UINTPTR_MAX  UINT32_MAX
#endif

#define INTMAX_MIN       INT64_MIN
#define INTMAX_MAX       INT64_MAX
#define UINTMAX_MAX      UINT64_MAX

#define PTRDIFF_MIN      INTPTR_MIN
#define PTRDIFF_MAX      INTPTR_MAX

#ifndef SIZE_MAX
    #define SIZE_MAX     UINTPTR_MAX
#endif

#define SIG_ATOMIC_MIN   INT32_MIN
#define SIG_ATOMIC_MAX   INT32_MAX

#define WCHAR_MIN        0x0000
#define WCHAR_MAX        0xffff

#define WINT_MIN         0x0000
#define WINT_MAX         0xffff

#define INT8_C(x)    (x)
#define INT16_C(x)   (x)
#define INT32_C(x)   (x)
#define INT64_C(x)   (x ## LL)

#define UINT8_C(x)   (x)
#define UINT16_C(x)  (x)
#define UINT32_C(x)  (x ## U)
#define UINT64_C(x)  (x ## ULL)

#define INTMAX_C(x)  INT64_C(x)
#define UINTMAX_C(x) UINT64_C(x)

#endif /* #ifndef _STDINT */

//////////////////////////////////////////////////////////////////////// bm_usb_def.h ////////////////////////////////////////////////////////////////////////

/**
 * @def   BM_DATA_HEADER_SIZE
 * @brief Size (in bytes) of BM Data header, which contains type, routing, length and timestamp.
 */
#define BM_DATA_HEADER_SIZE (8U)

/**
 * @def   BM_DATA_PAYLOAD_MAX_SIZE
 * @brief Size (in bytes) of BM Data payload, which contains a concrete message in CANFD|LIN|FLEXRAY|... type.
 */
#define BM_DATA_PAYLOAD_MAX_SIZE (72U)

/**
 * @def   BM_DATA_MAX_SIZE
 * @brief Size (in bytes) of BM Data, which contains a header and payload.
 */
#define BM_DATA_MAX_SIZE (BM_DATA_HEADER_SIZE + BM_DATA_PAYLOAD_MAX_SIZE)

/**
 * @enum  BM_CapabilityTypeDef
 * @brief Busmust Device capability flags, retrieved when enumerating devices using BM_Enumerate().
 */
typedef enum
{
    BM_NONE_CAP = 0x0000U,      /**< No capability */
    BM_LIN_CAP = 0x0001U,       /**< The device is capable of handling LIN messages */
    BM_CAN_CAP = 0x0002U,       /**< The device is capable of handling CAN messages */
    BM_CAN_FD_CAP = 0x0004U,    /**< The device is capable of handling CANFD (and CAN) messages */
    BM_FLEXRAY_CAP = 0x0008U,   /**< The device is capable of handling FLEXRAY messages */
    BM_MODBUS_CAP = 0x0010U,    /**< The device is capable of handling MODBUS messages */
    BM_ETHERNET_CAP = 0x0020U,  /**< The device is capable of handling ETHERNET messages */
    BM_ALL_CAP = 0xFFFFU        /**< Typically used for masking the CAP fields when programming */
} BM_CapabilityTypeDef;

/**
 * @enum  BM_DataTypeTypeDef
 * @brief Busmust data type flags, must be given in BM_DataTypeDef.
 */
typedef enum
{
    BM_UNKNOWN_DATA = 0,        /**< Unknown data type */
    BM_LIN_DATA,                /**< LIN message data type */
    BM_CAN_FD_DATA,             /**< CAN or CAN-FD message data type (check FDF flag further) */
    BM_FLEXRAY_DATA,            /**< Flexray message data type */
    BM_MODBUS_DATA,             /**< MODBUS message data type */
    BM_ETHERNET_DATA,           /**< Ethernet message data type */
    BM_ACK_DATA = 0x8U,         /**< ACK from bus, which indicates TXCMPLT event if this is BM_CAN_FD_DATA */
} BM_DataTypeTypeDef;

/**
 * @enum  BM_StatusTypeDef
 * @brief Busmust device & operation status, most APIs would return a status code to indicate the result of an operation.
 */
typedef enum
{
    BM_ERROR_OK = 0x00000,                      /**< SUCCESS: No error occurred */
    BM_ERROR_XMTFULL = 0x00001,                 /**< Low level Transmit buffer is full */
    BM_ERROR_OVERRUN = 0x00002,                 /**< Bus overrun (the device cannot keep up with the high bus throughput) */
    BM_ERROR_BUSLIGHT = 0x00004,                /**< CAN Bus communication is light, see ISO11898 for details */
    BM_ERROR_BUSHEAVY = 0x00008,                /**< CAN Bus communication is heavy, see ISO11898 for details */
    BM_ERROR_BUSWARNING = BM_ERROR_BUSHEAVY,    /**< CAN Bus communication is in warning state, see ISO11898 for details */
    BM_ERROR_BUSPASSIVE = 0x40000,              /**< CAN node is in passive state, see ISO11898 for details */
    BM_ERROR_BUSTIMEOUT = 0x80000,              /**< CAN node failed to transmit message within specified time, the node might be in PASSIVE or BUSOFF state */
    BM_ERROR_BUSOFF = 0x00010,                  /**< CAN bus is in BUSOFF state, see ISO11898 for details */
    BM_ERROR_ANYBUSERR = (BM_ERROR_BUSWARNING | BM_ERROR_BUSLIGHT | BM_ERROR_BUSHEAVY | BM_ERROR_BUSOFF | BM_ERROR_BUSPASSIVE), /**< CAN bus error occurred */
    BM_ERROR_QRCVEMPTY = 0x00020,               /**< Receive buffer is empty, this might NOT be an error if you use BMAPI in polling mode */
    BM_ERROR_QOVERRUN = 0x00040,                /**< BMAPI internal Q overrun */
    BM_ERROR_QXMTFULL = 0x00080,                /**< High level Transmit queue is full */
    BM_ERROR_REGTEST = 0x00100,                 /**< Reserved */
    BM_ERROR_NODRIVER = 0x00200,                /**< Reserved */
    BM_ERROR_HWINUSE = 0x00400,                 /**< Hardware is in use (opened by another application) */
    BM_ERROR_NETINUSE = 0x00800,                /**< Reserved */
    BM_ERROR_ILLHW = 0x01400,                   /**< Hardware error or invalid hardware handle */
    BM_ERROR_ILLNET = 0x01800,                  /**< Invalid bus */
    BM_ERROR_ILLCLIENT = 0x01C00,               /**< Invalid client */
    BM_ERROR_ILLHANDLE = (BM_ERROR_ILLHW | BM_ERROR_ILLNET | BM_ERROR_ILLCLIENT), /* Invalid handle*/
    BM_ERROR_RESOURCE = 0x02000,                /**< Out of resource */
    BM_ERROR_ILLPARAMTYPE = 0x04000,            /**< Invalid parameter type in API call */
    BM_ERROR_ILLPARAMVAL = 0x08000,             /**< Invalid parameter value in API call */
    BM_ERROR_UNKNOWN = 0x10000,                 /**< Unknown error */
    BM_ERROR_ILLDATA = 0x20000,                 /**< Invalid data received/transmitted */
    BM_ERROR_CAUTION = 0x2000000,               /**< Reserved */
    BM_ERROR_INITIALIZE = 0x4000000,            /**< The device/library is not initialized */
    BM_ERROR_ILLOPERATION = 0x8000000           /**< Invalid operation */
} BM_StatusTypeDef;

/**
 * @enum  BM_CanModeTypeDef
 * @brief CAN mode IDs, used by BM_SetCanMode() to change the operation mode of CAN device.
 */
typedef enum
{
    BM_CAN_OFF_MODE = 0x01,                     /**< The device is logically disconnected from CAN bus */
    BM_CAN_NORMAL_MODE = 0x00,                  /**< The device is running normally (with the capability to handle CAN and CANFD messages */
    BM_CAN_SLEEP_MODE = 0x01,                   /**< The device is logically disconnected from CAN bus */
    BM_CAN_INTERNAL_LOOPBACK_MODE = 0x02,       /**< The device is looping back messages internally without impacting the physical CAN bus */
    BM_CAN_LISTEN_ONLY_MODE = 0x03,             /**< The device is receiving messages without impacting the physical CAN bus (do not send ACKs to the bus) */
    BM_CAN_CONFIGURATION_MODE = 0x04,           /**< The device is under configuration and temporarily disconnected from CAN bus, For Internal usage only */
    BM_CAN_EXTERNAL_LOOPBACK_MODE = 0x05,       /**< The device is looping back messages externally, all transmitted messages are echoed by the device itself */
    BM_CAN_CLASSIC_MODE = 0x06,                 /**< The device is running normally (with the capability to handle only classical CAN2.0 messages */
    BM_CAN_RESTRICTED_MODE = 0x07,              /**< Reserved */
    BM_CAN_NORMAL_NON_ISO_MODE = 0x08,          /**< The device is running with the capability to handle CAN and NON-ISO(Bosch) CANFD messages */
} BM_CanModeTypeDef;

/**
 * @enum  BM_TerminalResistorTypeDef
 * @brief Terminal resistor values, used by BM_SetTerminalResistor() to change the terminal resistor of CAN device.
 */
typedef enum
{
    BM_TRESISTOR_AUTO = 0,              /**< Reserved, currently unsupported */
    BM_TRESISTOR_60 = 60,               /**< Currently unsupported */
    BM_TRESISTOR_120 = 120,             /**< 120Ohm */
    BM_TRESISTOR_DISABLED = 0xFFFFU,    /**< Disable terminal resistor */
} BM_TerminalResistorTypeDef;

/**
 * @enum  BM_LedTypeDef
 * @brief LED indicator status codes, used by BM_SetLed() to change the CAN LED indicator of CAN device.
 */
typedef enum
{
    BM_LED_OFF = 0,                     /**< CAN LED is OFF */
    BM_LED_ON = 1,                      /**< CAN LED is ON */
} BM_LedTypeDef;

/**
 * @enum  BM_MessageChannelTypeDef
 * @brief Message channel IDs in BM_DataTypeDef header, used for routing indication.
 * @note  You could also use integers directly, please note that valid channel IDs start from zero.
 */
typedef enum
{
    BM_MESSAGE_CHANNEL_0 = 0x0U,        /**< Channel 0 */
    BM_MESSAGE_CHANNEL_1 = 0x1U,        /**< Channel 1 */
    BM_MESSAGE_CHANNEL_2 = 0x2U,        /**< Channel 2 */
    BM_MESSAGE_CHANNEL_3 = 0x3U,        /**< Channel 3 */
    BM_MESSAGE_CHANNEL_4 = 0x4U,        /**< Channel 4 */
    BM_MESSAGE_CHANNEL_5 = 0x5U,        /**< Channel 5 */
    BM_MESSAGE_CHANNEL_6 = 0x6U,        /**< Channel 6 */
    BM_MESSAGE_CHANNEL_7 = 0x7U,        /**< Channel 7 */
    BM_MESSAGE_ANY_CHANNEL = 0xFU,      /**< Any channel, set this value in BM_DataTypeDef header if not used (e.g. TX.header.schn or RX.header.dchn) */
} BM_MessageChannelTypeDef;

/**
 * @enum  BM_MessageFlagsTypeDef
 * @brief CAN Message type flags, used in BM_CanMessageTypeDef.
 */
typedef enum
{
    BM_MESSAGE_FLAGS_NORMAL = 0,        /**< Normal CAN message */
    BM_MESSAGE_FLAGS_IDE = 0x01,        /**< Extended CAN message */
    BM_MESSAGE_FLAGS_RTR = 0x02,        /**< Remote CAN message */
    BM_MESSAGE_FLAGS_BRS = 0x04,        /**< CAN-FD bitrate switching is enabled */
    BM_MESSAGE_FLAGS_FDF = 0x08,        /**< CAN-FD message */
    BM_MESSAGE_FLAGS_ESI = 0x10,        /**< Reserved for gateways */
} BM_MessageFlagsTypeDef;

/**
 * @enum  BM_RxFilterTypeTypeDef
 * @brief CAN RX filter type IDs, used in BM_RxFilterTypeDef.
 */
typedef enum
{
    BM_RXFILTER_INVALID = 0,            /**< Invalid (unused) RX filter entry */
    BM_RXFILTER_BASIC,                  /**< Basic RX filter, traditional acceptance filter based on message ID mask */
    BM_RXFILTER_ADVANCED,               /**< Busmust advanced RX filter, check both message ID and message payload */
    BM_RXFILTER_E2EPASS,                /**< Busmust E2E RX filter, accept only messages that passed E2E checking */
    BM_RXFILTER_E2EFAIL,                /**< Busmust E2E RX filter, accept only messages that failed E2E checking (for debugging purpose) */
} BM_RxFilterTypeTypeDef;

/**
 * @enum  BM_TxTaskTypeTypeDef
 * @brief CAN TX task type IDs, used in BM_TxTaskTypeDef.
 */
typedef enum
{
    BM_TXTASK_INVALID = 0,              /**< Invalid (unused) TX task entry */
    BM_TXTASK_FIXED,                    /**< Basic TX task, send fixed ID and fixed payload */
    BM_TXTASK_INCDATA,                  /**< Self-increment Data TX task */
    BM_TXTASK_INCID,                    /**< Self-increment ID TX task */
    BM_TXTASK_RANDOMDATA,               /**< Random Data TX task */
    BM_TXTASK_RANDOMID,                 /**< Random ID TX task */
} BM_TxTaskTypeTypeDef;

/**
 * @enum  BM_StatTypeDef
 * @brief CAN runtime statistics item IDs, used in BM_GetStat().
 */
typedef enum
{
    BM_STAT_NONE = 0U,                  /**< Invalid statistics item */
    BM_STAT_TX_MESSAGE,                 /**< Number of TX messages */
    BM_STAT_RX_MESSAGE,                 /**< Number of RX messages */
    BM_STAT_TX_BYTE,                    /**< Number of TX bytes */
    BM_STAT_RX_BYTE,                    /**< Number of RX bytes */
    BM_STAT_TX_ERROR,                   /**< Number of TX errors */
    BM_STAT_RX_ERROR,                   /**< Number of RX errors */
} BM_StatTypeDef;

/**
 * @typedef BM_DataHeaderTypeDef
 * @brief   Busmust data header, each BM_DataTypeDef contains a header which indicates payload information.
 */
typedef struct
{
    uint16_t type : 4;                  /**< Data type, see BM_DataTypeTypeDef for details. */
    uint16_t flags : 4;                 /**< Reserved flags, keep 0 */
    uint16_t dchn : 4;                  /**< Destination channel ID, starting from zero, used by TX data to indicate the hardware about the target port. */
    uint16_t schn : 4;                  /**< Source channel ID, starting from zero, used by RX data to indicate the application about the source port. */
} BM_DataHeaderTypeDef;

/**
 * @def   BM_DATA_HEADER
 * @brief Helper macro to define a BM data header value
 */
#define BM_DATA_HEADER(type, flags, dchn, schn) (((type) & 0x0FU) | (((flags) << 4U) & 0xF0U) | (((dchn) << 8U) & 0x0F00U) | (((schn) << 12U) & 0xF000U))

/**
 * @typedef BM_DataTypeDef
 * @brief   Busmust data, abstract structure which holds concrete payload messages of various types (i.e. CAN messages).
 */
typedef struct
{
    uint16_t header;                            /**< data header, see BM_DataHeaderTypeDef for details. */
    uint16_t length;                            /**< length in bytes of the payload only (header excluded). */
    uint32_t timestamp;                         /**< 32-bit device local high precision timestamp in microseconds. */
    uint8_t payload[BM_DATA_PAYLOAD_MAX_SIZE];  /**< buffer holding concrete message payload (i.e. a CAN message in BM_CanMessageTypeDef format). */
} BM_DataTypeDef;

/**
 * @typedef BM_MessageIdTypeDef
 * @brief   Busmust CAN Message ID.
 * @note    You could also use a uint32_t, but please take care of memory alignments.
 */
typedef struct {
    uint32_t SID : 11;                          /**< Standard ID */
    uint32_t EID : 18;                          /**< Extended ID */
    uint32_t SID11 : 1;                         /**< Reserved */
    uint32_t unimplemented1 : 2;                /**< Reserved */
} BM_MessageIdTypeDef;

/**
 * @typedef BM_TxMessageCtrlTypeDef
 * @brief   Busmust TX CAN Message control fields.
 * @note    The first a few fields (until FDF) are bit compatible with BM_RxMessageCtrlTypeDef.
 */
typedef struct {
    uint32_t DLC : 4;                           /**< CAN message DLC(0-F), note this is not the message length */
    uint32_t IDE : 1;                           /**< This message is an extended CAN message */
    uint32_t RTR : 1;                           /**< This message is a remote CAN message */
    uint32_t BRS : 1;                           /**< This message requires CAN-FD bitrate switching */
    uint32_t FDF : 1;                           /**< This message is a CAN-FD CAN message */
    uint32_t ESI : 1;                           /**< Reserved for gateways */
    uint32_t SEQ : 23;                          /**< Reserved for hardware sync */
} BM_TxMessageCtrlTypeDef;

/**
 * @typedef BM_RxMessageCtrlTypeDef
 * @brief   Busmust RX CAN Message control fields.
 * @note    The first a few fields (until FDF) are bit compatible with BM_TxMessageCtrlTypeDef.
 */
typedef struct {
    uint32_t DLC : 4;                           /**< CAN message DLC(0-F), note this is not the message length */
    uint32_t IDE : 1;                           /**< This message is an extended CAN message */
    uint32_t RTR : 1;                           /**< This message is a remote CAN message */
    uint32_t BRS : 1;                           /**< This message requires CAN-FD bitrate switching */
    uint32_t FDF : 1;                           /**< This message is a CAN-FD CAN message */
    uint32_t ESI : 1;                           /**< Reserved for gateways */
    uint32_t unimplemented1 : 2;                /**< Reserved */
    uint32_t FilterHit : 5;                     /**< By wich RX filter the message is accepted */
    uint32_t unimplemented2 : 16;               /**< Reserved */
} BM_RxMessageCtrlTypeDef;

/**
 * @typedef BM_CanMessageTypeDef
 * @brief   Busmust CAN Message concrete type, usually used as payload of BM_DataTypeDef.
 * @note    The total length of this structure is 72B, it support both classic and FD CAN messages.
 */
typedef struct 
{
    uint32_t id;                                /**< CAN message ID, see BM_MessageIdTypeDef for details. */
    union
    {
        uint32_t tx;                            /**< TX CAN message control fields, invalid if this is NOT a TX can message. */
        uint32_t rx;                            /**< RX CAN message control fields, invalid if this is NOT a RX can message. */
    } ctrl;                                     /**< CAN message control fields, whether TX or RX is taken depends on the message direction. */
    uint8_t payload[64];                        /**< CAN message payload */
} BM_CanMessageTypeDef;

/**
 * @typedef BM_ChannelInfoTypeDef
 * @brief   Channel information, created when enumerating devices by BM_Enumerate() and used when opening device by BM_OpenEx().
 */
typedef struct
{
    char name[64];                              /**< Device full name, for display purpose */
    uint8_t  sn[16];                            /**< Device SN */
    uint8_t  uid[12];                           /**< Device UID */
    uint8_t  version[4];                        /**< Device Firmware Version */
    uint16_t vid;                               /**< Device VID */
    uint16_t pid;                               /**< Device PID */
    uint16_t port;                              /**< Port ID (0-7) of the device, note a multi-port device is enumerated as multiple dedicated BM_ChannelInfoTypeDef entries */
    uint16_t cap;                               /**< Device Capability flags, see BM_CapabilityTypeDef for details. */
    uint8_t  reserved[4];                       /**< Reserved */
} BM_ChannelInfoTypeDef;

/**
 * @typedef BM_CanStatusInfoTypedef
 * @brief   CAN channel status detailed information, retrieved by calling BM_GetCanStatus(), see ISO11898 for details.
 */
typedef struct
{
    uint8_t TXBO;                               /**< The CAN channel is in BUSOFF state */
    uint8_t reserved[1];                        /**< Reserved */
    uint8_t TXBP;                               /**< The CAN channel is in TX bus passive state */
    uint8_t RXBP;                               /**< The CAN channel is in RX bus passive state */
    uint8_t TXWARN;                             /**< The CAN channel is in TX warn state */
    uint8_t RXWARN;                             /**< The CAN channel is in RX warn state */
    uint8_t TEC;                                /**< TX Bus Error counter */
    uint8_t REC;                                /**< RX Bus Error counter */
} BM_CanStatusInfoTypedef;

/**
 * @typedef BM_BitrateTypeDef
 * @brief   CAN channel bitrate configuration, used by BM_SetBitrate().
 */
typedef struct
{
    uint16_t nbitrate;                          /**< Nominal bitrate in kbps, default as 500, note this is the only valid birate in CAN CLASSIC mode. */
    uint16_t dbitrate;                          /**< Data bitrate in kbps, default as 500, note this is ignored in CAN CLASSIC mode. */
    uint8_t nsamplepos;                         /**< Nominal sample position (percentage), 0-100, default as 75 */
    uint8_t dsamplepos;                         /**< Data sample position (percentage), 0-100, default as 75 */
    /* Setting any of the fields below would override the nbitrate configuration */
    uint8_t  clockfreq;                         /**< CAN controller clock in Mhz, default as 0 */
    uint8_t  reserved;                          /**< Reserved */
    uint8_t  nbtr0;                             /**< Nominal BTR0 register value, note this value is calculated using clockfreq, which might not be 16MHz */
    uint8_t  nbtr1;                             /**< Nominal BTR1 register value, note this value is calculated using clockfreq, which might not be 16MHz */
    uint8_t  dbtr0;                             /**< Data BTR0 register value, note this value is calculated using clockfreq, which might not be 16MHz */
    uint8_t  dbtr1;                             /**< Data BTR1 register value, note this value is calculated using clockfreq, which might not be 16MHz */
} BM_BitrateTypeDef;

/**
 * @typedef BM_RxFilterTypeDef
 * @brief   CAN channel RX filter item structure, used by BM_SetRxFilter().
 * @note    The filter support masking ID, flags and payload according to its type, 
 *          in order for a message to be accepted, all the fields are masked using AND logic:
 *          (flags & filter.flags_mask == filter.flags_value) AND (ID & filter.id_mask == filter.id_value) AND (payload & filter.payload_mask == filter.payload_value)
 */
typedef struct
{
    uint8_t  type;                              /**< Type ID of the RX filter, see BM_RxFilterTypeTypeDef for details. */
    uint8_t  unused;                            /**< Reserved */
    uint8_t  flags_mask;                        /**< CAN message control Flags masks, see BM_MessageFlagsTypeDef for details. */
    uint8_t  flags_value;                       /**< CAN message control Flags values, see BM_MessageFlagsTypeDef for details. */
    uint8_t  reserved[4];                       /**< Reserved */
    uint32_t id_mask;                           /**< CAN message ID masks, see BM_MessageIdTypeDef for details. */
    uint32_t id_value;                          /**< CAN message ID values, see BM_MessageIdTypeDef for details. */
    uint8_t payload_mask[8];                    /**< CAN message payload masks, for CAN-FD messages, only the first 8 bytes are checked. */
    uint8_t payload_value[8];                   /**< CAN message payload values, for CAN-FD messages, only the first 8 bytes are checked. */
} BM_RxFilterTypeDef;

/**
 * @typedef BM_TxTaskTypeDef
 * @brief   CAN channel TX task item structure, used by BM_SetTxTask().
 * @note    Once the CAN device is armed with TX tasks, it will try to parse the TX task and send CAN messages automatically.
 *          The difference with a software triggered CAN message in BusMaster is that 
 *          hardware triggered CAN messages are more precise in time and could reach a higher throughput.
 */
typedef struct
{
    uint8_t  type;                              /**< Type ID of the TX task, see BM_TxTaskTypeTypeDef for details. */
    uint8_t  unused;                            /**< Reserved */
    uint8_t  flags;                             /**< CAN message control Flags, see BM_MessageFlagsTypeDef for details. */
    uint8_t  length;                            /**< Length of payload in bytes (not DLC) */
    uint8_t  e2e;                               /**< Index of E2E (in E2E table), currently unsupported */
    uint8_t  reserved[1];                       /**< Reserved */

    uint16_t cycle;                             /**< ms delay between rounds */
    uint16_t nrounds;                           /**< num of cycles */
    uint16_t nmessages;                         /**< messages per round */

    uint32_t id;                                /**< CAN message arbitration id, see BM_MessageIdTypeDef for details. */

    union
    {
        /* The child fields are invalid if the TX task type is NOT BM_TXTASK_INCDATA */
        struct {
            uint16_t startbit;                  /**< Start bit of data increment, currently only 8-bit aligned value is accepted */
            uint8_t nbits;                      /**< Number of bits of data increment, currently only 32 is accepted */
            uint8_t format;                     /**< 0x80=Intel, 0x00=Motorola */
            uint32_t min;                       /**< Minimum value of the Increment range */
            uint32_t max;                       /**< Maximum value of the Increment range */
            uint32_t step;                      /**< Step of the Increment range */
        } incdata;

        /* The child fields are invalid if the TX task type is NOT BM_TXTASK_INCID */
        struct {
            uint32_t min;                       /**< Minimum value of the Increment range */
            uint32_t max;                       /**< Maximum value of the Increment range */
            uint32_t step;                      /**< Step of the Increment range */
        } incid;

        /* The child fields are invalid if the TX task type is NOT BM_TXTASK_RANDOMDATA */
        struct {
            uint16_t startbit;                  /**< Start bit of random data, currently only 8-bit aligned value is accepted */
            uint8_t nbits;                      /**< Number of bits of random data, currently only 32 is accepted */
            uint8_t format;                     /**< 0x80=Intel, 0x00=Motorola */
            uint32_t min;                       /**< Minimum value of the Increment range */
            uint32_t max;                       /**< Maximum value of the Increment range */
            uint32_t seed;                      /**< Random seed */
        } randomdata;

        /* The child fields are invalid if the TX task type is NOT BM_TXTASK_RANDOMID */
        struct {
            uint32_t min;                       /**< Minimum value of the random range */
            uint32_t max;                       /**< Maximum value of the random range */
            uint32_t seed;                      /**< Random seed */
        } randomid;

        uint8_t unused[48];                     /**< Reserved */
    } pattern;                                  /**< Changing pattern of a Volatile TX task */

    uint8_t  payload[64];                       /**< Default payload data, note this is also the template payload of the unchanged part in a volatile TX task */
} BM_TxTaskTypeDef;


/////////////////////////////////////////////////////////////////////////// BMAPI.h ///////////////////////////////////////////////////////////////////////////
/**
 * @def   BM_API_VERSION
 * @brief API version, format: major.minor.revision.build
 * @note  This macro might be checked by the application so that it could adapt to different versions of BMAPI.
 */
#define BM_API_VERSION         0x1002  /* 1.0.0.2 */

/**
 * @def   BMAPI
 * @brief All function declared with BMAPI modifier are exported by BMAP dynamic library (*.dll, *.so).
 */
#ifdef BMAPI_EXPORT
#ifdef _MSC_VER
#define BMAPI   __declspec(dllexport)
#else
#define BMAPI   __attribute__((visibility("default")))
#endif
#else
#ifdef _MSC_VER
#define BMAPI   __declspec(dllimport)
#else
#define BMAPI
#endif
#endif

/**
 * @typedef BM_ChannelHandle
 * @brief   Abstract handle to opened Busmust device channel, 
 *          most APIs would require a handle to operate on the target device channel.
 */
typedef void* BM_ChannelHandle;

/**
 * @typedef BM_NotificationHandle
 * @brief   Abstract handle to notification event of opened Busmust device channel, 
 *          call BM_WaitForNotifications to wait for new events (i.e. CAN RX message event).
 */
typedef void* BM_NotificationHandle;

/**
 * @typedef BM_AutosetCallbackHandle
 * @brief   Pointer to a Callback function when AUTOSET status is updates, indicating a bitrate option has passed or failed.
 * @param[in] bitrate      The bitrate option value which has passed or failed.
 * @param[in] tres         The terminal resistor option value which has passed or failed.
 * @param[in] nrxmessages  Number of received messages while listening to the bus using bitrate and tres.
 * @param[in] userarg      Arbitrary user argument passed by BM_Autoset().
 */
typedef void (*BM_AutosetCallbackHandle)(const BM_BitrateTypeDef* bitrate, BM_TerminalResistorTypeDef tres, int nrxmessages, uintptr_t userarg);

/**
 * @brief  Initialize BMAPI library, this function shall be called before any other API calls and shall only be called once.
 * @return Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_Init(void);

/**
 * @brief  Un-initialize BMAPI library, this function shall be called after any other API calls and shall only be called once.
 * @return Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_UnInit(void);

/**
 * @brief        Enumerate all connected Busmust device.
 * @param[out]   channelinfos  An array of BM_ChannelInfoTypeDef structure which holds info of all the enumerated Busmust devices.
 * @param[inout] nchannels     Number of device channels available, which is also the number of valid entries in channelinfos, 
 *                             this param must be initialized with the maximum length of the channelinfos array when calling this function.
 * @return       Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_Enumerate(BM_ChannelInfoTypeDef channelinfos[], int* nchannels);

/**
 * @brief Start AUTOSET sequence, for a BM-USB-CAN(FD) device, the AUTOSET sequence will detect the correct bitrate and terminal resistor.
 * @param[in]  channelinfo  Info of the device channel to run AUTOSET on, usually the info is filled by BM_Enumerate().
 * @param[out] bitrate      The detected bitrate.
 * @param[out] tres         The detected terminal resistor.
 * @param[in]  callback     A callback function which will be called on each step of AUTOSET.
 * @param[in]  userarg      Arbitrary user argument of the callback function, this argument will be passed to the callback as is.
 */
BMAPI BM_StatusTypeDef BM_Autoset(
    BM_ChannelInfoTypeDef* channelinfo,
    BM_BitrateTypeDef* bitrate,
    BM_TerminalResistorTypeDef* tres,
    BM_AutosetCallbackHandle callback,
    uintptr_t userarg
);

/**
 * @brief Open the specified CAN device port.
 * @param[in] port  Index of the port, starting from zero, note this is the index of all enumerated ports.
 * @return Handle to the opened CAN device channel, return NULL if failed to open the specified port.
 */
BMAPI BM_ChannelHandle BM_OpenCan(uint16_t port);

/**
 * @brief Open the specified device port using given configuration.
 * @param[out] handle      Handle to the opened device channel.
 * @param[in]  channelinfo Info of the device channel to open, usually the info is filled by BM_Enumerate().
 * @param[in]  mode        CAN operation mode option of the opened channel, see BM_CanModeTypeDef for details.
 * @param[in]  tres        Terminal resistor option of the opened channel, see BM_TerminalResistorTypeDef for details.
 * @param[in]  bitrate     Bitrate option of the opened channel, see BM_BitrateTypeDef for details.
 * @param[in]  rxfilter    CAN acceptance filters option of the opened channel, see BM_RxFilterTypeDef for details.
 * @param[in]  nrxfilters  Number of acceptance filters, usually there could be up to 2 filters.
 * @return Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_OpenEx(
        BM_ChannelHandle* handle,
        BM_ChannelInfoTypeDef* channelinfo,
        BM_CanModeTypeDef mode,
        BM_TerminalResistorTypeDef tres,
        const BM_BitrateTypeDef* bitrate,
        const BM_RxFilterTypeDef* rxfilter,
        int nrxfilters
        );

/**
 * @brief     Close an opened channel.
 * @param[in] handle  Handle to the channel to be closed.
 * @return    Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_Close(BM_ChannelHandle handle);

/**
 * @brief     Reset an opened channel.
 * @param[in] handle  Handle to the channel to be reset.
 * @return    Operation status, see BM_StatusTypeDef for details.
 * @note      The configuration options will not lost when the channel is reset, so BM_Reset() is basically identical to BM_Close() and then BM_OpenEx().
 */
BMAPI BM_StatusTypeDef BM_Reset(BM_ChannelHandle handle);

/**
 * @brief     Clear TX&RX message buffer of an opened channel.
 * @param[in] handle  Handle to the channel to be cleared.
 * @return    Operation status, see BM_StatusTypeDef for details.
 * @note      This function is available since BMAPI1.3, hardware status will not be changed when clearing buffer.
 */
BMAPI BM_StatusTypeDef BM_ClearBuffer(BM_ChannelHandle handle);

/**
 * @brief      Read any message/event out of the given channel.
 * @param[in]  handle  Handle to the channel to read from.
 * @param[out] data    A caller-allocated buffer to hold the message/event output, see BM_DataTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if no message is received.
 *             Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
 */
BMAPI BM_StatusTypeDef BM_Read(BM_ChannelHandle handle, BM_DataTypeDef* data);

/**
 * @brief      Read CAN message out of the given channel.
 * @param[in]  handle     Handle to the channel to read from.
 * @param[out] msg        A caller-allocated buffer to hold the CAN message output, see BM_CanMessageTypeDef for details.
 * @param[out] channel    The source channel ID from which the message is received, starting from zero, could be NULL if not required.
 * @param[out] timestamp  The device local high precision timestamp in microseconds, when the message is physically received on the CAN bus, could be NULL if not required.
 * @return     Operation status, see BM_StatusTypeDef for details. 
 * @note       Note this function is a simple wrapper of BM_Read(), see BM_Read() for details.
 */
BMAPI BM_StatusTypeDef BM_ReadCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef* msg, uint32_t* channel, uint32_t* timestamp);

/**
 * @brief      Write any message/event to the given channel.
 * @param[in]  handle  Handle to the channel to write to.
 * @param[in]  data      A caller-allocated buffer to hold the message/event input, see BM_DataTypeDef for details.
 * @param[in]  timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
 *                       Set any negative number (i.e. -1) to wait infinitely.
 *                       Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
 * @param[in]  timestamp The device local high precision timestamp in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       This function is allowed to be called from multiple threads since BMAPI1.3.
 */
BMAPI BM_StatusTypeDef BM_Write(BM_ChannelHandle handle, const BM_DataTypeDef* data, int timeout, uint32_t* timestamp);

/**
 * @brief      Write CAN message to the given channel.
 * @param[in]  handle     Handle to the channel to write to.
 * @param[in]  msg        A caller-allocated buffer to hold the CAN message output, see BM_CanMessageTypeDef for details.
 * @param[in]  _channel   The target channel ID to which the message is transmitted, starting from zero. This parameter is reserved for future, always 0 now.
 * @param[in]  timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
 *                       Set any negative number (i.e. -1) to wait infinitely.
 *                       Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
 * @param[in]  timestamp The device local high precision timestamp in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
 * @note       Note this function is a simple wrapper of BM_Write(), see BM_Write() for details.
 */
BMAPI BM_StatusTypeDef BM_WriteCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef* msg, uint32_t _channel, int timeout, uint32_t* timestamp);

/**
 * @brief Control the given channel, this is an advanced interface and is typically used internally by BMAPI.
 * @param[in]    handle   Handle to the channel to control.
 * @param[in]    command  Control command.
 * @param[in]    value    Control value.
 * @param[in]    index    Control index.
 * @param[inout] data     Control data, could be NULL.
 * @param[inout] nbytes   Length in bytes of the control data, could be zero.
 */
BMAPI BM_StatusTypeDef BM_Control(BM_ChannelHandle handle, uint8_t command, uint16_t value, uint16_t index, void* data, int nbytes);

/**
 * @brief      Get current CAN status of the given channel.
 * @param[in]  handle      Handle to the channel to operate on.
 * @param[out] statusinfo  Detailed information of current CAN status, see BM_CanStatusInfoTypedef for details.
 * @return     Current status code, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetStatus(BM_ChannelHandle handle, BM_CanStatusInfoTypedef* statusinfo);

/**
 * @brief      Get current local high precision device timestamp, in microseconds.
 * @param[in]  handle     Handle to the channel to operate on.
 * @param[out] timestamp  Timestamp value.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetTimestamp(BM_ChannelHandle handle, uint32_t* timestamp);

/**
 * @brief      Set CAN mode option of the given channel.
 * @param[in]  handle  Handle to the channel to operate on.
 * @param[in]  mode    Expected CAN mode, see BM_CanModeTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetCanMode(BM_ChannelHandle handle, BM_CanModeTypeDef mode);

/**
 * @brief      Set terminal resistor option of the given channel.
 * @param[in]  handle  Handle to the channel to operate on.
 * @param[in]  tres    Expected terminal resistor, see BM_TerminalResistorTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetTerminalRegister(BM_ChannelHandle handle, BM_TerminalResistorTypeDef  tres);

/**
 * @brief      Set bitrate option of the given channel.
 * @param[in]  handle  Handle to the channel to operate on.
 * @param[in]  bitrate Expected bitrate, see BM_BitrateTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetBitrate(BM_ChannelHandle handle, const BM_BitrateTypeDef* bitrate);

/**
 * @brief      Set TX tasks option of the given channel.
 * @param[in]  handle    Handle to the channel to operate on.
 * @param[in]  txtasks   An array of TX task information, see BM_TxTaskTypeDef for details.
 * @param[in]  ntxtasks  Number of valid TX tasks in the txtasks array.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetTxTasks(BM_ChannelHandle handle, BM_TxTaskTypeDef* txtasks, int ntxtasks);

/**
 * @brief      Set RX filters option of the given channel.
 * @param[in]  handle      Handle to the channel to operate on.
 * @param[in]  rxfilters   An array of RX filter information, see BM_RxFilterTypeDef for details.
 * @param[in]  nrxfilters  Number of valid RX filters in the txtasks rxfilters.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetRxFilters(BM_ChannelHandle handle, BM_RxFilterTypeDef* rxfilters, int nrxfilters);

/**
 * @brief Get the platform/OS independent notification handle for the given channel, so that the application could wait for notifications later.
 * @param[in]  handle        Handle to the channel that owns the notification handle.
 * @param[out] notification  The platform/OS independent notification handle.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       By using notification handles in a background thread, it is easy to implement an asynchronous message receiver as below:
 * @code
 *             channel = BM_OpenCan(...);
 *             BM_GetNotification(channel, notification);
 *             while (!exit) {
 *               BM_WaitForNotifications(&notification, 1, -1); // Wait infinitely for new message notification.
 *               BM_ReadCanMessage(...);
 *             }
 * @endcode
 */
BMAPI BM_StatusTypeDef BM_GetNotification(BM_ChannelHandle handle, BM_NotificationHandle* notification);

/**
 * @brief     A platform/OS independent implementation to wait for single/multiple notification handles.
 * @param[in] handles     An array of channel notification handles.
 * @param[in] nhandles    Number of valid notification handles.
 * @param[in] ntimeoutms  This function will block current thread for ntimeoutms milliseconds if no notification is received.
 *                        Note this function will return immediately once a new notification is received, the ntimeoutms param is ignored in this normal case.
 * @return    This function returns the index in handles array of the channel from which a new notification is posted.
 */
BMAPI int BM_WaitForNotifications(BM_NotificationHandle handles[], int nhandles, int ntimeoutms);

/**
 * @brief      Translate error code to string, this is a helper function to ease application programming.
 * @param[in]  errorcode  The errorcode to be translated.
 * @param[out] buffer     A caller-allocated string buffer to hold the translated string.
 * @param[in]  nbytes     Number in bytes of the string buffer.
 * @param[in]  language   Reserved, only English is supported currently.
 */
BMAPI void BM_GetErrorText(BM_StatusTypeDef errorcode, char* buffer, int nbytes, uint16_t language);

/**
 * @brief      Translate data (i.e. CAN message) to string, this is a helper function to ease application programming.
 * @param[in]  data       The message data to be translated.
 * @param[out] buffer     A caller-allocated string buffer to hold the translated string.
 * @param[in]  nbytes     Number in bytes of the string buffer.
 * @param[in]  language   Reserved, only English is supported currently.
 */
BMAPI void BM_GetDataText(BM_DataTypeDef* data, char* buffer, int nbytes, uint16_t language);

#ifdef __cplusplus
};
#endif

#endif /* #ifndef __BMAPI_H__ */
/**
 * End of file
 */
