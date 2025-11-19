using System;
using System.Text;
using System.Runtime.InteropServices;

/**
 * @enum  BM_CapabilityTypeDef
 * @brief Busmust Device capability flags, retrieved when enumerating devices using BM_Enumerate().
 */
public enum BM_CapabilityTypeDef
{
    BM_NONE_CAP = 0x0000,      /**< No capability */
    BM_LIN_CAP = 0x0001,       /**< The device is capable of handling LIN messages */
    BM_CAN_CAP = 0x0002,       /**< The device is capable of handling CAN messages */
    BM_CAN_FD_CAP = 0x0004,    /**< The device is capable of handling CANFD (and CAN) messages */
    BM_FLEXRAY_CAP = 0x0008,   /**< The device is capable of handling FLEXRAY messages */
    BM_MODBUS_CAP = 0x0010,    /**< The device is capable of handling MODBUS messages */
    BM_ETHERNET_CAP = 0x0020,  /**< The device is capable of handling ETHERNET messages */
    BM_ALL_CAP = 0xFFFF,       /**< Typically used for masking the CAP fields when programming */
};

/**
 * @enum  BM_DataTypeTypeDef
 * @brief Busmust data type flags, must be given in BM_DataTypeDef.
 */
public enum BM_DataTypeTypeDef
{
    BM_UNKNOWN_DATA = 0,        /**< Unknown data type */
    BM_LIN_DATA,                /**< LIN message data type */
    BM_CAN_FD_DATA,             /**< CAN or CAN-FD message data type (check FDF flag further) */
    BM_FLEXRAY_DATA,            /**< Flexray message data type */
    BM_MODBUS_DATA,             /**< MODBUS message data type */
    BM_ETHERNET_DATA,           /**< Ethernet message data type */
    BM_ACK_DATA = 0x8,          /**< ACK from bus, which indicates TXCMPLT event if this is BM_CAN_FD_DATA */
};

/**
 * @enum  BM_StatusTypeDef
 * @brief Busmust device & operation status, most APIs would return a status code to indicate the result of an operation.
 */
public enum BM_StatusTypeDef
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
};

/**
 * @enum  BM_CanModeTypeDef
 * @brief CAN mode IDs, used by BM_SetCanMode() to change the operation mode of CAN device.
 */
public enum BM_CanModeTypeDef
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
};

/**
 * @enum  BM_TerminalResistorTypeDef
 * @brief Terminal resistor values, used by BM_SetTerminalResistor() to change the terminal resistor of CAN device.
 */
public enum BM_TerminalResistorTypeDef
{
    BM_TRESISTOR_AUTO = 0,              /**< Reserved, currently unsupported */
    BM_TRESISTOR_60 = 60,               /**< Currently unsupported */
    BM_TRESISTOR_120 = 120,             /**< 120Ohm */
    BM_TRESISTOR_DISABLED = 0xFFFF,     /**< Disable terminal resistor */
};

/**
 * @enum  BM_LedTypeDef
 * @brief LED indicator status codes, used by BM_SetLed() to change the CAN LED indicator of CAN device.
 */
public enum BM_LedTypeDef
{
    BM_LED_OFF = 0,                     /**< CAN LED is OFF */
    BM_LED_ON = 1,                      /**< CAN LED is ON */
};

/**
 * @enum  BM_MessageChannelTypeDef
 * @brief Message channel IDs in BM_DataTypeDef header, used for routing indication.
 * @note  You could also use integers directly, please note that valid channel IDs start from zero.
 */
public enum BM_MessageChannelTypeDef
{
    BM_MESSAGE_CHANNEL_0 = 0x0,        /**< Channel 0 */
    BM_MESSAGE_CHANNEL_1 = 0x1,        /**< Channel 1 */
    BM_MESSAGE_CHANNEL_2 = 0x2,        /**< Channel 2 */
    BM_MESSAGE_CHANNEL_3 = 0x3,        /**< Channel 3 */
    BM_MESSAGE_CHANNEL_4 = 0x4,        /**< Channel 4 */
    BM_MESSAGE_CHANNEL_5 = 0x5,        /**< Channel 5 */
    BM_MESSAGE_CHANNEL_6 = 0x6,        /**< Channel 6 */
    BM_MESSAGE_CHANNEL_7 = 0x7,        /**< Channel 7 */
    BM_MESSAGE_ANY_CHANNEL = 0xF,      /**< Any channel, set this value in BM_DataTypeDef header if not used (e.g. TX.header.schn or RX.header.dchn) */
};

/**
 * @enum  BM_MessageFlagsTypeDef
 * @brief CAN Message type flags, used in BM_CanMessageTypeDef.
 */
public enum BM_MessageFlagsTypeDef
{
    BM_MESSAGE_FLAGS_NORMAL = 0,        /**< Normal CAN message */
    BM_MESSAGE_FLAGS_IDE = 0x01,        /**< Extended CAN message */
    BM_MESSAGE_FLAGS_RTR = 0x02,        /**< Remote CAN message */
    BM_MESSAGE_FLAGS_BRS = 0x04,        /**< CAN-FD bitrate switching is enabled */
    BM_MESSAGE_FLAGS_FDF = 0x08,        /**< CAN-FD message */
    BM_MESSAGE_FLAGS_ESI = 0x10,        /**< Reserved for gateways */
};

/**
 * @enum  BM_RxFilterTypeTypeDef
 * @brief CAN RX filter type IDs, used in BM_RxFilterTypeDef.
 */
public enum BM_RxFilterTypeTypeDef
{
    BM_RXFILTER_INVALID = 0,            /**< Invalid (unused) RX filter entry */
    BM_RXFILTER_BASIC,                  /**< Basic RX filter, traditional acceptance filter based on message ID mask */
    BM_RXFILTER_ADVANCED,               /**< Busmust advanced RX filter, check both message ID and message payload */
    BM_RXFILTER_E2EPASS,                /**< Busmust E2E RX filter, accept only messages that passed E2E checking */
    BM_RXFILTER_E2EFAIL,                /**< Busmust E2E RX filter, accept only messages that failed E2E checking (for debugging purpose) */
};

/**
 * @enum  BM_TxTaskTypeTypeDef
 * @brief CAN TX task type IDs, used in BM_TxTaskTypeDef.
 */
public enum BM_TxTaskTypeTypeDef
{
    BM_TXTASK_INVALID = 0,              /**< Invalid (unused) TX task entry */
    BM_TXTASK_FIXED,                    /**< Basic TX task, send fixed ID and fixed payload */
    BM_TXTASK_INCDATA,                  /**< Self-increment Data TX task */
    BM_TXTASK_INCID,                    /**< Self-increment ID TX task */
    BM_TXTASK_RANDOMDATA,               /**< Random Data TX task */
    BM_TXTASK_RANDOMID,                 /**< Random ID TX task */
};

/**
 * @enum  BM_StatTypeDef
 * @brief CAN runtime statistics item IDs, used in BM_GetStat().
 */
public enum BM_StatTypeDef
{
    BM_STAT_NONE = 0,                   /**< Invalid statistics item */
    BM_STAT_TX_MESSAGE,                 /**< Number of TX messages */
    BM_STAT_RX_MESSAGE,                 /**< Number of RX messages */
    BM_STAT_TX_BYTE,                    /**< Number of TX bytes */
    BM_STAT_RX_BYTE,                    /**< Number of RX bytes */
    BM_STAT_TX_ERROR,                   /**< Number of TX errors */
    BM_STAT_RX_ERROR,                   /**< Number of RX errors */
};

/**
 * @enum  BM_IsotpModeTypeDef
 * @brief ISOTP operation mode, used in BM_IsotpConfigTypeDef.
 */
public enum BM_IsotpModeTypeDef
{
    BM_ISOTP_NORMAL_TESTER = 0,         /**< Default mode: normal (non-extended-addressing) UDS client(tester) */
    BM_ISOTP_NORMAL_ECU,                /**< normal (non-extended-addressing) UDS server(ECU)                  */
    BM_ISOTP_EXTENDED_TESTER,           /**< Currently unsupported: extended-addressing UDS client(tester)     */
    BM_ISOTP_EXTENDED_ECU,              /**< Currently unsupported: extended-addressing UDS server(ECU)        */
};

/**
 * @enum  BM_LogLevelTypeDef
 * @brief Busmust library log level, see BM_SetLogLevel() for details.
 */
public enum BM_LogLevelTypeDef
{
    BM_LOG_NONE = 0,             		/**< Show nothing on debug console                                                                                 */
    BM_LOG_ERR,                  		/**< Show only ERR level messages on debug console, note this is the default level for release versions            */
    BM_LOG_WRN,                  		/**< Show ERR and WRN level messages on debug console                                                              */
    BM_LOG_INF,                  		/**< Show ERR|WRN|INF level messages on debug console                                                              */
    BM_LOG_DBG                   		/**< Show all messages on debug console, including debug messages, note this is NOT available for release versions */
};

/**
 * @typedef BM_CanMessageTypeDef
 * @brief   Busmust CAN Message concrete type, usually used as payload of BM_DataTypeDef.
 * @note    The total length of this structure is 72B, it support both classic and FD CAN messages.
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_CanMessageTypeDef
{
    uint id;                             /**< CAN message ID, see BM_MessageIdTypeDef for details. */
    uint ctrl;                           /**< CAN message control fields, whether TX or RX is taken depends on the message direction. */
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)]
    public byte[] payload;                      /**< buffer holding concrete message payload (i.e. a CAN message in BM_CanMessageTypeDef format). */
    public BM_CanMessageTypeDef(uint id = 0, uint DLC = 0, bool IDE = false, bool RTR = false, bool BRS = false, bool FDF = false, bool ESI = false)
    {
        if (IDE)
        {
            this.id = ((id >> 18) & 0x7FF) | ((id & 0x3FFFF) << 11);
        }
        else
        {
            this.id = id & 0x7FF;
        }
        this.ctrl = (DLC & 0x0F);
        if (IDE) { this.ctrl |= 1 << 4; }
        if (RTR) { this.ctrl |= 1 << 5; }
        if (BRS) { this.ctrl |= 1 << 6; }
        if (FDF) { this.ctrl |= 1 << 7; }
        if (ESI) { this.ctrl |= 1 << 8; };
        this.payload = new byte[64];
    }

    public uint GetID()
    {
        if (this.IsIDE())
        {
            return ((this.id & 0x7FF) << 18) | ((this.id >> 11) & 0x3FFFF);
        }
        else
        {
            return this.id & 0x7FF;
        }
    }

    public uint GetDLC()
    {
        return (ctrl & 0xF);
    }

    public bool IsIDE()
    {
        return (ctrl & (1 << 4)) != 0;
    }
    public bool IsRTR()
    {
        return (ctrl & (1 << 5)) != 0;
    }
    public bool IsBRS()
    {
        return (ctrl & (1 << 6)) != 0;
    }
    public bool IsFDF()
    {
        return (ctrl & (1 << 7)) != 0;
    }
    public bool IsESI()
    {
        return (ctrl & (1 << 8)) != 0;
    }

    public uint GetLength()
    {
        uint[] DlcToDataBytes = new uint[]
        {
            0,  1,  2,  3,  4,  5,  6,  7,
            8, 12, 16, 20, 24, 32, 48, 64,
        };
        uint length = DlcToDataBytes[GetDLC()];
        return length;
    }

    public uint lengthToDlc(uint n)
    {
        uint dlc = 0;
        if (n <= 8)
        {
            dlc = n;
        }
        else if (n <= 12)
        {
            dlc = 9;
        }
        else if (n <= 16)
        {
            dlc = 10;
        }
        else if (n <= 20)
        {
            dlc = 11;
        }
        else if (n <= 24)
        {
            dlc = 12;
        }
        else if (n <= 32)
        {
            dlc = 13;
        }
        else if (n <= 48)
        {
            dlc = 14;
        }
        else
        {
            dlc = 15;
        }
        return dlc;
    }

    public String GetTypeString()
    {
        String type = "";
        if (IsIDE()) { type += "IDE "; }
        if (IsRTR()) { type += "RTR "; }
        if (IsBRS()) { type += "BRS "; }
        if (IsFDF()) { type += "FDF "; }
        if (IsESI()) { type += "ESI "; }
        if (type.Length == 0)
        {
            type = "STD";
        }
        return type;
    }

    public String GetPyloadString()
    {
        uint length = GetLength();
        String payload = "";
        int i;
        for (i = 0; i < length; i++)
        {
            payload += Convert.ToString(this.payload[i], 16).ToUpper().PadLeft(2, '0') + " ";
        }
        return payload;
    }
};

/**
 * @typedef BM_DataHeaderTypeDef
 * @brief   Busmust data header, each BM_DataTypeDef contains a header which indicates payload information.
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_DataHeaderTypeDef
{
    //uint16_t type : 4;                  /**< Data type, see BM_DataTypeTypeDef for details. */
    //uint16_t flags : 4;                 /**< Reserved flags, keep 0 */
    //uint16_t dchn : 4;                  /**< Destination channel ID, starting from zero, used by TX data to indicate the hardware about the target port. */
    //uint16_t schn : 4;                  /**< Source channel ID, starting from zero, used by RX data to indicate the application about the source port. */
    ushort value;

    public BM_DataHeaderTypeDef(BM_DataTypeTypeDef type, ushort flags, ushort dchn, ushort schn)
    {
        this.value = (ushort)(((((ushort)type) & 0xF) << 0) | ((flags & 0xF) << 4) | ((dchn & 0xF) << 8) | ((schn & 0xF) << 12));
    }

    public BM_DataTypeTypeDef GetDataType()
    {
        return (BM_DataTypeTypeDef)(this.value & 0xF);
    }

    public ushort GetFlags()
    {
        return (ushort)((this.value >> 4) & 0xF);
    }

    public ushort GetDstChannel()
    {
        return (ushort)((this.value >> 8) & 0xF);
    }

    public ushort GetSrcChannel()
    {
        return (ushort)((this.value >> 12) & 0xF);
    }

    public bool IsAckData()
    {
        return ((ushort)(((BM_DataTypeTypeDef)(this.value & 0xF)) & BM_DataTypeTypeDef.BM_ACK_DATA)) != 0;
    }

    public void SetAckData()
    {
        this.value |= (ushort)BM_DataTypeTypeDef.BM_ACK_DATA;
    }
};

/**
 * @typedef BM_DataTypeDef
 * @brief   Busmust data, abstract structure which holds concrete payload messages of various types (i.e. CAN messages).
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_DataTypeDef
{
    public BM_DataHeaderTypeDef header;             /**< data header, see BM_DataHeaderTypeDef for details. */
    public ushort length;                           /**< length in bytes of the payload only (header excluded). */
    public uint timestamp;                          /**< 32-bit device local high precision timestamp in microseconds. */
    public BM_CanMessageTypeDef can;            /**< buffer holding concrete message payload (i.e. a CAN message in BM_CanMessageTypeDef format). */

    public BM_DataTypeDef(BM_DataHeaderTypeDef header, ushort length, uint timestamp, BM_CanMessageTypeDef canMsg)
    {
        this.header = header;
        this.length = length;
        this.timestamp = timestamp;
        this.can = canMsg;
    }
};

/**
 * @typedef BM_ChannelInfoTypeDef
 * @brief   Channel information, created when enumerating devices by BM_Enumerate() and used when opening device by BM_OpenEx().
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_ChannelInfoTypeDef
{
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)]
    public byte[] name;                              /**< Device full name, for display purpose */
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
    public byte[] sn;                                /**< Device SN */
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 12)]
    public byte[] uid;                               /**< Device UID */
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
    public byte[] version;                           /**< Device Firmware Version */
    public ushort vid;                               /**< Device VID */
    public ushort pid;                               /**< Device PID */
    public ushort port;                              /**< Port ID (0-7) of the device, note a multi-port device is enumerated as multiple dedicated BM_ChannelInfoTypeDef entries */
    public ushort cap;                               /**< Device Capability flags, see BM_CapabilityTypeDef for details. */
    public uint   reserved;                          /**< Reserved */

    private String ByteArrayToString(byte[] d)
    {
        String s = System.Text.Encoding.Default.GetString(d);
        int index = s.IndexOf('\0');
        if (index >= 0)
        {
            s = s.Remove(index);
        }
        return s;
    }

    public String GetName()
    {
        return ByteArrayToString(name);
    }

    public String GetSn()
    {
        return ByteArrayToString(sn);
    }

    public String GetUid()
    {
        return ByteArrayToString(uid);
    }
};

/**
 * @typedef BM_CanStatusInfoTypedef
 * @brief   CAN channel status detailed information, retrieved by calling BM_GetCanStatus(), see ISO11898 for details.
 */
[StructLayout(LayoutKind.Sequential)]
struct BM_CanStatusInfoTypedef
{
    public byte TXBO;                               /**< The CAN channel is in BUSOFF state */
    public byte reserved;                           /**< Reserved */
    public byte TXBP;                               /**< The CAN channel is in TX bus passive state */
    public byte RXBP;                               /**< The CAN channel is in RX bus passive state */
    public byte TXWARN;                             /**< The CAN channel is in TX warn state */
    public byte RXWARN;                             /**< The CAN channel is in RX warn state */
    public byte TEC;                                /**< TX Bus Error counter */
    public byte REC;                                /**< RX Bus Error counter */
};

/**
 * @typedef BM_BitrateTypeDef
 * @brief   CAN channel bitrate configuration, used by BM_SetBitrate().
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_BitrateTypeDef
{
    public ushort nbitrate;                          /**< Nominal bitrate in kbps, default as 500, note this is the only valid birate in CAN CLASSIC mode. */
    public ushort dbitrate;                          /**< Data bitrate in kbps, default as 500, note this is ignored in CAN CLASSIC mode. */
    public byte nsamplepos;                          /**< Nominal sample position (percentage), 0-100, default as 75 */
    public byte dsamplepos;                          /**< Data sample position (percentage), 0-100, default as 75 */
    public byte clockfreq;                           /**< CAN controller clock in Mhz, default as 0 */
    public byte reserved;                            /**< Reserved */
    public byte nbtr0;                               /**< Nominal BTR0 register value, note this value is calculated using clockfreq, which might not be 16MHz */
    public byte nbtr1;                               /**< Nominal BTR1 register value, note this value is calculated using clockfreq, which might not be 16MHz */
    public byte dbtr0;                               /**< Data BTR0 register value, note this value is calculated using clockfreq, which might not be 16MHz */
    public byte dbtr1;                               /**< Data BTR1 register value, note this value is calculated using clockfreq, which might not be 16MHz */
};

/**
 * @typedef BM_RxFilterTypeDef
 * @brief   CAN channel RX filter item structure, used by BM_SetRxFilter().
 * @note    The filter support masking ID, flags and payload according to its type, 
 *          in order for a message to be accepted, all the fields are masked using AND logic:
 *          (flags & filter.flags_mask == filter.flags_value) AND (ID & filter.id_mask == filter.id_value) AND (payload & filter.payload_mask == filter.payload_value)
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_RxFilterTypeDef
{
    public byte type;                              /**< Type ID of the RX filter, see BM_RxFilterTypeTypeDef for details. */
    public byte unused;                            /**< Reserved */
    public byte flags_mask;                        /**< CAN message control Flags masks, see BM_MessageFlagsTypeDef for details. */
    public byte flags_value;                       /**< CAN message control Flags values, see BM_MessageFlagsTypeDef for details. */
    public uint reserved;                          /**< Reserved */
    uint id_mask;                           /**< CAN message ID masks, see BM_MessageIdTypeDef for details. */
    uint id_value;                          /**< CAN message ID values, see BM_MessageIdTypeDef for details. */
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
    public byte[] payload_mask;                    /**< CAN message payload masks, for CAN-FD messages, only the first 8 bytes are checked. */
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
    public byte[] payload_value;                   /**< CAN message payload values, for CAN-FD messages, only the first 8 bytes are checked. */

    public void SetIdFilter(uint mask, uint value, bool ide)
    {
        if ((mask > 0x7FF || value > 0x7FF) || ide)
        {
            this.id_mask = ((mask >> 18) & 0x7FF) | ((mask & 0x3FFFF) << 11);
            this.id_value = ((value >> 18) & 0x7FF) | ((value & 0x3FFFF) << 11);
        }
        else
        {
            this.id_mask = mask & 0x7FF;
            this.id_value = value & 0x7FF;
        }
    }
};

/**
 * @typedef BM_TxTaskTypeDef
 * @brief   CAN channel TX task item structure, used by BM_SetTxTask().
 * @note    Once the CAN device is armed with TX tasks, it will try to parse the TX task and send CAN messages automatically.
 *          The difference with a software triggered CAN message in BusMaster is that 
 *          hardware triggered CAN messages are more precise in time and could reach a higher throughput.
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_TxTaskTypeDef
{
    public byte type;                                /**< Type ID of the TX task, see BM_TxTaskTypeTypeDef for details. */
    public byte unused;                              /**< Reserved */
    public byte flags;                               /**< CAN message control Flags, see BM_MessageFlagsTypeDef for details. */
    public byte length;                              /**< Length of payload in bytes (not DLC) */
    public byte e2e;                                 /**< Index of E2E (in E2E table), currently unsupported */
    public byte reserved;                            /**< Reserved */

    public ushort cycle;                             /**< ms delay between rounds */
    public ushort nrounds;                           /**< num of cycles */
    public ushort nmessages;                         /**< messages per round */

    public uint id;                                  /**< CAN message arbitration id, see BM_MessageIdTypeDef for details. */

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 48)]
    byte[] pattern;                                  /**< Reserved */

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
    public byte[] payload;                           /**< Default payload data, note this is also the template payload of the unchanged part in a volatile TX task */
};

/**
 * @typedef BM_IsotpStatusTypeDef
 * @brief   ISOTP status report, used by ISOTP operation callback function.
 */
public struct BM_IsotpStatusTypeDef
{
    public byte version;                           /**< Currently always 0x01 */
    public byte flowcontrol;                       /**< Current flow control status, 0=continue, 1=wait, 2=overflow, ff=timeout, */
    public byte stmin;                             /**< Current flow control status, i.e. 30 00 00 */
    public byte blocksize;                         /**< Current flow control status, i.e. 30 00 00 */
    public uint ntransferredbytes;                 /**< Number of transferred bytes by now. */
    public uint ntotalbytes;                       /**< Number of total bytes indicated by ISOTP FF or SF. */
    public uint timestamp;                         /**< Current timestamp reported by device. */
    public uint reserved1;                         /**< Reserved for future */
    public uint reserved2;                         /**< Reserved for future */
    public uint reserved3;                         /**< Reserved for future */
    public uint reserved4;                         /**< Reserved for future */
};

/**
 * @typedef BM_IsotpTimeoutConfigTypeDef
 * @brief   ISOTP Protocol (See ISO15765-2 for details) timeout configuration, used by BM_ConfigIsotp().
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_IsotpTimeoutConfigTypeDef
{
    public ushort a;                             /**< A timeout in milliseconds: =N_As if writing as tester or reading as ECU, otherwise =N_Ar */
    public ushort b;                             /**< B timeout in milliseconds: =N_Bs if writing as tester or reading as ECU, otherwise =N_Br */
    public ushort c;                             /**< C timeout in milliseconds: =N_Cs if writing as tester or reading as ECU, otherwise =N_Cr */
}

/**
 * @typedef BM_IsotpFlowcontrolConfigTypeDef
 * @brief   ISOTP Protocol (See ISO15765-2 for details) flowcontrol configuration, used by BM_ConfigIsotp().
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_IsotpFlowcontrolConfigTypeDef
{
    public byte stmin;                           /**< STmin raw value (0x00-0x7F or 0xF1-0xF9) if Busmust device is acting as UDS server       */
    public byte blocksize;                       /**< Blocksize if can card is acting as UDS server, 0 means no further FC is needed           */
    public byte fcFrameLength;                   /**< Flow control frame length in bytes                                                       */
    public byte reserved;
}

/**
 * @typedef BM_IsotpConfigTypeDef
 * @brief   ISOTP Protocol (See ISO15765-2 for details) configuration, used by BM_ConfigIsotp().
 */
[StructLayout(LayoutKind.Sequential)]
public struct BM_IsotpConfigTypeDef
{
    public byte version;                                /**< Currently must be set to 0x01                                                            */
    public byte mode;                                   /**< Currently only 0 is supported: normal (non-extended-addressing) UDS client(tester)       */
    public BM_IsotpTimeoutConfigTypeDef testerTimeout;  /**< Tester ISOTP timeout configuration                                                       */
    public BM_IsotpTimeoutConfigTypeDef ecuTimeout;     /**< ECU ISOTP timeout configuration                                                          */
    public BM_IsotpFlowcontrolConfigTypeDef flowcontrol;/**< ISOTP Flowcontrol configuration                                                          */
    public byte extendedAddress;                        /**< UDS Address in Extended Addressing mode                                                  */
    public byte paddingEnabled;                         /**< Enable padding for unused payload bytes/                                                 */
    public byte paddingValue;                           /**< Padding byte value (i.e. 0xCC) for unused payload bytes                                  */
    public byte longPduEnabled;                         /**< Enable long PDU (>4095), note if CAN message DLC>8, long PDU is enabled by default       */
    public ushort padding;
    public IntPtr callbackFunc;                         /**< TODO: Callback function when any progress is made, used typically by GUI to show progress bar  */
    public IntPtr callbackUserarg;                      /**< TODO: Callback userarg when any progress is made, used typically by GUI to show progress bar   */
    public BM_DataTypeDef testerDataTemplate;           /**< All tester messages will be formatted/checked using this template, configure CAN message ID and IDE/FDF flags here  */
    public BM_DataTypeDef ecuDataTemplate;              /**< All ECU messages will be formatted/checked using this template, configure CAN message ID and IDE/FDF flags here */
};

class BMAPI
{
    /**
     * @brief  Initialize BMAPI library, this function shall be called before any other API calls and shall only be called once.
     * @return Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_Init();

    /**
     * @brief  Un-initialize BMAPI library, this function shall be called after any other API calls and shall only be called once.
     * @return Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_UnInit();

    /**
     * @brief        Enumerate all connected Busmust device.
     * @param[out]   channelinfos  An array of BM_ChannelInfoTypeDef structure which holds info of all the enumerated Busmust devices.
     * @param[inout] nchannels     Number of device channels available, which is also the number of valid entries in channelinfos, 
     *                             this param must be initialized with the maximum length of the channelinfos array when calling this function.
     * @return       Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_Enumerate(IntPtr channelinfos, ref int nchannels);

    /**
     * @brief Open the specified CAN device port.
     * @param[in] port  Index of the port, starting from zero, note this is the index of all enumerated ports.
     * @return Handle to the opened CAN device channel, return NULL if failed to open the specified port.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr BM_OpenCan(int index);

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
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_OpenEx(
        ref IntPtr handle,
        ref BM_ChannelInfoTypeDef channelinfo,
        BM_CanModeTypeDef mode,
        BM_TerminalResistorTypeDef tres,
        ref BM_BitrateTypeDef bitrate,
        IntPtr rxfilter,
        int nrxfilters
    );

    /**
     * @brief     Close an opened channel.
     * @param[in] handle  Handle to the channel to be closed.
     * @return    Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_Close(IntPtr handle);

    /**
     * @brief     Reset an opened channel.
     * @param[in] handle  Handle to the channel to be reset.
     * @return    Operation status, see BM_StatusTypeDef for details.
     * @note      The configuration options will not lost when the channel is reset, so BM_Reset() is basically identical to BM_Close() and then BM_OpenEx().
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_Reset(IntPtr handle);

    /**
	 * @brief     Reset the device hardware which an opened channel belongs to, in case the device hardware is in unknown error state and is unrecoverable using BM_OpenEx.
	 * @param[in] handle  Handle to a opened channel (which belongs to the physical device to reset).
	 * @return    Operation status, see BM_StatusTypeDef for details.
	 * @note      !!!CAUTION!!! Note this API will break all active USB/Ethernet connection, just like you have manually unplugged it and then plugged it back.
	 *            All opened channel handles that belongs to the physical device under reset will be invalid after reset.
	 *            You MUST re-open and re-configure all channels using BM_OpenEx or other configuration APIs after reset.
	 */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_ResetDevice(IntPtr handle);

    /**
     * @brief     Activate an opened channel, and thus goes on bus for the selected port and channels. 
                  At this point, the user can transmit and receive messages on the bus.
     * @param[in] handle  Handle to the channel to be activated.
     * @return    Operation status, see BM_StatusTypeDef for details.
     * @note      Channel is default to be activated after BM_OpenEx() is called.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_Activate(IntPtr handle);

    /**
     * @brief     Deactivate an opened channel, and thus the selected channels goes off the bus and stay in BUSOFF state until re-activation.
     * @param[in] handle  Handle to the channel to be deactivated.
     * @return    Operation status, see BM_StatusTypeDef for details.
     * @note      Any call to BM_Write() or BM_Read() will return BM_ERROR_BUSOFF immediately if the channel is deactivated.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_Deactivate(IntPtr handle);

    /**
     * @brief     Clear TX&RX message buffer of an opened channel.
     * @param[in] handle  Handle to the channel to be cleared.
     * @return    Operation status, see BM_StatusTypeDef for details.
     * @note      This function is available since BMAPI1.3, hardware status will not be changed when clearing buffer.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_ClearBuffer(IntPtr handle);

    /**
     * @brief      Read any message/event out of the given channel.
     * @param[in]  handle  Handle to the channel to read from.
     * @param[out] data    A caller-allocated buffer to hold the message/event output, see BM_DataTypeDef for details.
     * @return     Operation status, see BM_StatusTypeDef for details.
     * @note       This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if no message is received.
     *             Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_Read(IntPtr handle, ref BM_DataTypeDef data);

    /**
     * @brief        Read multiple messages/events out of the given channel.
     * @param[in]    handle  Handle to the channel to read from.
     * @param[out]   data       A caller-allocated buffer to hold the messages/events array output, see BM_DataTypeDef for details.
     * @param[inout] nmessages  Number of read messages, user shall initialize this param with the size (in messages) of the data buffer.
     * @param[in]    timeout    Timeout (in milliseconds) before the message is received successfully from the bus.
     *                          Set any negative number (i.e. -1) to wait infinitely.
     *                          Set 0 if you would like to receive asynchronously: read from BMAPI internal buffer and return immediately, use BM_WaitForNotifications() before reading.
     * @return       Operation status, see BM_StatusTypeDef for details.
     * @note         This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if not all messages are received.
     *               Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_ReadMultiple(IntPtr handle, IntPtr data, ref uint nmessages, int timeout);

    /**
     * @brief        Read data block using ISOTP protocol.
     *               This API enables rapid transmission using ISOTP without app intervention, a simple example would be reading VIN using UDS:
     *               uint8_t request[] = { 0x22, 0xF1, 0x80 };
     *               uint8_t response[4096];
     *               uint32_t nbytes = sizeof(response);
     *               BM_WriteIsotp(channel, request, sizeof(request), config);
     *               BM_ReadIsotp(channel, response, nbytes, config);
     *               assert(response[0] == 0x62 && response[1] == 0xF1 && response[2] == 0x80);
     * @param[in]    handle    Handle to the channel to read from.
     * @param[in]    data      A caller-allocated buffer to hold the data block output.
     * @param[inout] nbytes    Length of the received data block, in bytes. Caller must initialize this arg with the size of the caller-allocated buffer.
     * @param[in]    timeout   Timeout (in milliseconds) before the message is received successfully from the bus.
     *                         Set any negative number (i.e. -1) to wait infinitely.
     *                         Set 0 if you would like to receive asynchronously: read from BMAPI internal buffer and return immediately, use BM_WaitForNotifications() before reading.
     * @param[in]    config    ISOTP configuration used by current transfer.
     * @return     Operation status, see BM_StatusTypeDef for details.
     * @note       This function is allowed to be called from multiple threads since BMAPI1.5.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_ReadIsotp(IntPtr handle, IntPtr data, ref uint nbytes, int timeout, ref BM_IsotpConfigTypeDef config);

    public BM_StatusTypeDef BM_ReadIsotpBytes(IntPtr handle, ref byte[] payload, ref uint nbytes, int timeout, ref BM_IsotpConfigTypeDef config)
    {
        IntPtr infomem = Marshal.UnsafeAddrOfPinnedArrayElement(payload, payload.Length);
        BM_StatusTypeDef status = BM_ReadIsotp(handle, infomem, ref nbytes, timeout, ref config);
        Marshal.Copy(infomem, payload, 0, (int)nbytes);
        return status;
    }

    /**
     * @brief      Read CAN message out of the given channel.
     * @param[in]  handle     Handle to the channel to read from.
     * @param[out] msg        A caller-allocated buffer to hold the CAN message output, see BM_CanMessageTypeDef for details.
     * @param[out] channel    The source channel ID from which the message is received, starting from zero, could be NULL if not required.
     * @param[out] timestamp  The device local high precision timestamp in microseconds, when the message is physically received on the CAN bus, could be NULL if not required.
     * @return     Operation status, see BM_StatusTypeDef for details. 
     * @note       Note this function is a simple wrapper of BM_Read(), see BM_Read() for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_ReadCanMessage(IntPtr handle, ref BM_CanMessageTypeDef msg, ref uint channel, ref uint timestamp);

    /**
     * @brief        Read multiple CAN messages out of the given channel.
     * @param[in]    handle  Handle to the channel to read from.
     * @param[out]   data       A caller-allocated buffer to hold the CAN message array output, see BM_CanMessageTypeDef for details.
     * @param[inout] nmessages  Number of read messages, user shall initialize this param with the size (in messages) of the data buffer.
     * @param[in]    timeout    Timeout (in milliseconds) before the message is received successfully from the bus.
     *                          Set any negative number (i.e. -1) to wait infinitely.
     *                          Set 0 if you would like to receive asynchronously: read from BMAPI internal buffer and return immediately, use BM_WaitForNotifications() before reading.
     * @param[out]   channel    The source channel ID from which the message is received, starting from zero, could be NULL if not required.
     * @param[out]   timestamp  The device local high precision timestamp array in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
     * @return       Operation status, see BM_StatusTypeDef for details.
     * @note         This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if not all messages are received.
     *               Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_ReadMultipleCanMessage(IntPtr handle, IntPtr msg, ref uint nmessages, int timeout, IntPtr channel, IntPtr timestamp);

    /**
     * @brief      Write any message/event to the given channel.
     * @param[in]  handle  Handle to the channel to write to.
     * @param[in]  data      A caller-allocated buffer to hold the message/event input, see BM_DataTypeDef for details.
     * @param[in]  timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
     *                       Set any negative number (i.e. -1) to wait infinitely.
     *                       Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
     * @param[in]  timestamp The device local high precision timestamp in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
     * @return     Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_Write(IntPtr handle, ref BM_DataTypeDef data, int timeout, ref uint timestamp);

    /**
     * @brief        Write multiple messages/events to the given channel.
     * @param[in]    handle    Handle to the channel to write to.
     * @param[in]    data      A caller-allocated buffer to hold the messages/events array input, see BM_DataTypeDef for details.
     * @param[inout] nmessages Number of written messages, user shall initialize this param with the size (in messages) of the data buffer.
     * @param[in]    timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
     *                         Set any negative number (i.e. -1) to wait infinitely.
     *                         Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
     * @param[out]   timestamp The device local high precision timestamp array in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
     * @return     Operation status, see BM_StatusTypeDef for details.
     * @note       This function is allowed to be called from multiple threads since BMAPI1.3.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_WriteMultiple(IntPtr handle, IntPtr data, ref uint nmessages, int timeout, IntPtr timestamp);

    /**
     * @brief        Write data block using ISOTP protocol.
     *               This API enables rapid transmission using ISOTP without app intervention, a simple example would be writing VIN using UDS:
     *               uint8_t request[] = { 0x2E, 0xF1, 0x80, ... ... };
     *               BM_WriteIsotp(channel, request, sizeof(request), config);
     * @param[in]    handle    Handle to the channel to write to.
     * @param[in]    data      A caller-allocated buffer to hold the data block input.
     * @param[in]    nbytes    Length of the data block, in bytes.
     * @param[in]    timeout   Timeout (in milliseconds) before any message segment is transmitted successfully to the bus.
     *                         Note this is only for bus level timeout waiting for CAN ACK, for setting ISOTP protocol timeouts, see BM_IsotpConfigTypeDef for details.
     * @param[in]    config    ISOTP configuration used by current transfer.
     * @return     Operation status, see BM_StatusTypeDef for details.
     * @note       This function is allowed to be called from multiple threads since BMAPI1.5.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_WriteIsotp(IntPtr handle, IntPtr data, uint nbytes, int timeout, ref BM_IsotpConfigTypeDef config);

    public BM_StatusTypeDef BM_WriteIsotpBytes(IntPtr handle, byte[] payload, int timeout, ref BM_IsotpConfigTypeDef config)
    {
        IntPtr infomem = Marshal.UnsafeAddrOfPinnedArrayElement(payload, payload.Length);
        return BM_WriteIsotp(handle, infomem, (uint)payload.Length, timeout, ref config);
    }

    /**
     * @brief      Write CAN message to the given channel.
     * @param[in]  handle     Handle to the channel to write to.
     * @param[in]  msg        A caller-allocated buffer to hold the CAN message output, see BM_CanMessageTypeDef for details.
     * @param[in]  channel    The target channel ID to which the message is transmitted, starting from zero.
     * @param[in]  timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
     *                       Set any negative number (i.e. -1) to wait infinitely.
     *                       Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
     * @param[in]  timestamp The device local high precision timestamp in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
     * @note       Note this function is a simple wrapper of BM_Write(), see BM_Write() for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_WriteCanMessage(IntPtr handle, ref BM_CanMessageTypeDef msg, uint channel, int timeout, ref uint timestamp);

    /**
     * @brief        Write multiple CAN messages to the given channel.
     * @param[in]    handle    Handle to the channel to write to.
     * @param[in]    msg       A caller-allocated buffer to hold the CAN message array input, see BM_CanMessageTypeDef for details.
     * @param[inout] nmessages Number of written messages, user shall initialize this param with the size (in messages) of the data buffer.
     * @param[in]    _channel  The target channel ID to which the message is transmitted, starting from zero. This parameter is reserved for future, always 0 now, or simply pass NULL into the API.
     * @param[in]    timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
     *                         Set any negative number (i.e. -1) to wait infinitely.
     *                         Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
     * @param[out]   timestamp The device local high precision timestamp array in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
     * @return     Operation status, see BM_StatusTypeDef for details.
     * @note       This function is allowed to be called from multiple threads since BMAPI1.3.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_WriteMultipleCanMessage(IntPtr handle, IntPtr msg, ref uint nmessages, IntPtr _channel, int timeout, IntPtr timestamp);

    /**
     * @brief Control the given channel, this is an advanced interface and is typically used internally by BMAPI.
     * @param[in]    handle   Handle to the channel to control.
     * @param[in]    command  Control command.
     * @param[in]    value    Control value.
     * @param[in]    index    Control index.
     * @param[inout] data     Control data, could be NULL.
     * @param[inout] nbytes   Length in bytes of the control data, could be zero.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_Control(IntPtr handle, byte command, ushort value, ushort index, IntPtr data, int nbytes);

    /**
     * @brief      Get current CAN status of the given channel.
     * @param[in]  handle      Handle to the channel to operate on.
     * @param[out] statusinfo  Detailed information of current CAN status, see BM_CanStatusInfoTypedef for details.
     * @return     Current status code, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_GetStatus(IntPtr handle, ref BM_CanStatusInfoTypedef statusinfo);

    /**
     * @brief      Get current local high precision device timestamp, in microseconds.
     * @param[in]  handle     Handle to the channel to operate on.
     * @param[out] timestamp  Timestamp value.
     * @return     Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_GetTimestamp(IntPtr handle, ref uint timestamp);

    /**
     * @brief      Set CAN mode option of the given channel.
     * @param[in]  handle  Handle to the channel to operate on.
     * @param[in]  mode    Expected CAN mode, see BM_CanModeTypeDef for details.
     * @return     Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_SetCanMode(IntPtr handle, BM_CanModeTypeDef mode);

    /**
     * @brief      Set terminal resistor option of the given channel.
     * @param[in]  handle  Handle to the channel to operate on.
     * @param[in]  tres    Expected terminal resistor, see BM_TerminalResistorTypeDef for details.
     * @return     Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_SetTerminalRegister(IntPtr handle, BM_TerminalResistorTypeDef tres);

    /**
     * @brief      Set bitrate option of the given channel.
     * @param[in]  handle  Handle to the channel to operate on.
     * @param[in]  bitrate Expected bitrate, see BM_BitrateTypeDef for details.
     * @return     Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_SetBitrate(IntPtr handle, ref BM_BitrateTypeDef bitrate);

    /**
     * @brief      Set TX tasks option of the given channel.
     * @param[in]  handle    Handle to the channel to operate on.
     * @param[in]  txtasks   An array of TX task information, see BM_TxTaskTypeDef for details.
     * @param[in]  ntxtasks  Number of valid TX tasks in the txtasks array.
     * @return     Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_SetTxTasks(IntPtr handle, IntPtr txtasks, int ntxtasks);

    /**
     * @brief      Set RX filters option of the given channel.
     * @param[in]  handle      Handle to the channel to operate on.
     * @param[in]  rxfilters   An array of RX filter information, see BM_RxFilterTypeDef for details.
     * @param[in]  nrxfilters  Number of valid RX filters in the rxfilters array.
     * @return     Operation status, see BM_StatusTypeDef for details.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_SetRxFilters(IntPtr handle, IntPtr rxfilters, int nrxfilters);

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
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern BM_StatusTypeDef BM_GetNotification(IntPtr handle, ref IntPtr notification);

    /**
     * @brief     A platform/OS independent implementation to wait for single/multiple notification handles.
     * @param[in] handles     An array of channel notification handles.
     * @param[in] nhandles    Number of valid notification handles.
     * @param[in] ntimeoutms  This function will block current thread for ntimeoutms milliseconds if no notification is received.
     *                        Note this function will return immediately once a new notification is received, the ntimeoutms param is ignored in this normal case.
     * @return    This function returns the index in handles array of the channel from which a new notification is posted.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern int BM_WaitForNotifications(IntPtr[] handles, int nhandles, int ntimeoutms);

    /**
     * @brief      Translate error code to string, this is a helper function to ease application programming.
     * @param[in]  errorcode  The errorcode to be translated.
     * @param[out] buffer     A caller-allocated string buffer to hold the translated string.
     * @param[in]  nbytes     Number in bytes of the string buffer.
     * @param[in]  language   Reserved, only English is supported currently.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern void BM_GetErrorText(BM_StatusTypeDef errorcode, char[] buffer, int nbytes, ushort language);

    /**
     * @brief      Translate data (i.e. CAN message) to string, this is a helper function to ease application programming.
     * @param[in]  data       The message data to be translated.
     * @param[out] buffer     A caller-allocated string buffer to hold the translated string.
     * @param[in]  nbytes     Number in bytes of the string buffer.
     * @param[in]  language   Reserved, only English is supported currently.
     */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
    public static extern void BM_GetDataText(ref BM_DataTypeDef data, char[] buffer, int nbytes, ushort language);
	
	/**
	 * @brief      Get library log level.
	 * @return     Current log level, all messages equal to or less than this level are currently printed on debug console.
	 */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
	public static extern BM_LogLevelTypeDef BM_GetLogLevel();

	/**
	 * @brief      Set library log level.
	 * @param[in]  level       Target log level, all messages equal to or less than this level will be printed on debug console.
	 */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
	public static extern void BM_SetLogLevel(BM_LogLevelTypeDef level);

	/**
	 * @brief      Get library (*.dll|*.so) BMAPI version.
	 * @return     32-bit version code:
	 *             bit31-28 = major
	 *             bit27-24 = minor
	 *             bit23-16 = revision
	 *             bit15-00 = build
	 * @note       This API is used to get the library version, 
	 *             in case that *.h is mismatch with *.dll|*.so, use the macro BM_API_VERSION defined in bmapi.h to check consistency.
	 */
    [DllImport("BMAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
	public static extern uint BM_GetVersion();
}