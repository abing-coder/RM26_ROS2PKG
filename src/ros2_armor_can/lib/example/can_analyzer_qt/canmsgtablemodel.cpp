#include "canmsgtablemodel.h"

CanMsgTableModel::CanMsgTableModel(QObject *parent)
    : QAbstractTableModel(parent)
{
}

QVariant CanMsgTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        switch (section)
        {
        case 0: return QString("Time (s)"); break;
        case 1: return QString("TX/RX"); break;
        case 2: return QString("Channel"); break;
        case 3: return QString("ID (HEX)"); break;
        case 4: return QString("Type"); break;
        case 5: return QString("Length (B)"); break;
        case 6: return QString("Payload"); break;
        default: break;
        }
    }

    return QVariant();
}

int CanMsgTableModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;

    return msgpool.size();
}

int CanMsgTableModel::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;

    return 7;
}

QString CanMsgTableModel::msgId(const BM_DataTypeDef& msg) const
{
    const BM_CanMessageTypeDef* canMsg = (const BM_CanMessageTypeDef*)msg.payload;
    uint32_t canMsgId = canMsg->id.SID;
    if (canMsg->ctrl.rx.IDE)
    {
        canMsgId = (canMsg->id.SID << 18) | canMsg->id.EID;
    }
    return (QString("%1").arg(canMsgId, 8, 16)).toUpper();
}

uint32_t CanMsgTableModel::msgLength(const BM_DataTypeDef& msg) const
{
    const BM_CanMessageTypeDef* canMsg = (const BM_CanMessageTypeDef*)msg.payload;
    static const uint32_t DlcToDataBytes[16] =
    {
        0,  1,  2,  3,  4,  5,  6,  7,
        8, 12, 16, 20, 24, 32, 48, 64,
    };
    uint32_t length = DlcToDataBytes[canMsg->ctrl.rx.DLC];
    return length;
}

QString CanMsgTableModel::msgType(const BM_DataTypeDef& msg) const
{
    const BM_CanMessageTypeDef* canMsg = (const BM_CanMessageTypeDef*)msg.payload;
    QString type;
    if (canMsg->ctrl.rx.IDE)
    {
        type += "IDE ";
    }
    if (canMsg->ctrl.rx.FDF)
    {
        type += "FDF ";
    }
    if (canMsg->ctrl.rx.BRS)
    {
        type += "BRS ";
    }
    if (canMsg->ctrl.rx.RTR)
    {
        type += "RTR";
    }
    if (type.isEmpty())
    {
        type = "STD";
    }
    return type;
}

QString CanMsgTableModel::msgPayload(const BM_DataTypeDef& msg) const
{
    const BM_CanMessageTypeDef* canMsg = (const BM_CanMessageTypeDef*)msg.payload;
    uint32_t length = msgLength(msg);
    QString payload;
    uint32_t i;
    for (i = 0; i < length; i++)
    {
        payload += QString("%1 ").arg(canMsg->payload[i], 2, 16, QLatin1Char('0'));
    }
    return payload;
}


QVariant CanMsgTableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    const MsgDataTypeDef& myMsg = msgpool.at(index.row());

    if (role == Qt::DisplayRole)
    {
        switch (index.column())
        {
        case 0: return QString("%1").arg(myMsg.msg.timestamp * 1E-6f, 0, 'g', 6, QLatin1Char('0')); break;
        case 1: return QString((myMsg.msg.header.type & BM_ACK_DATA) ? "TX" : "RX"); break;
        case 2: return QString("%1").arg(myMsg.port); break;
        case 3: return msgId(myMsg.msg); break;
        case 4: return msgType(myMsg.msg); break;
        case 5: return msgLength(myMsg.msg); break;
        case 6: return msgPayload(myMsg.msg); break;
        default: break;
        }
    }
    return QVariant();
}

int CanMsgTableModel::insertMessage(const BM_DataTypeDef& msg, int port)
{
    MsgDataTypeDef msgData;
    bool allChanged = false;
    msgData.port = port;
    msgData.msg = msg;
    if (msgpool.size() > 1000000)
    {
        beginRemoveRows(QModelIndex(), 0, 0);
        msgpool.pop_front();
        endRemoveRows();
        allChanged = true;
    }
    int newItemRow = msgpool.size();
    beginInsertRows(QModelIndex(), newItemRow, newItemRow);
    msgpool.push_back(msgData);
    endInsertRows();

    if (allChanged)
    {
        emit dataChanged(QModelIndex(), QModelIndex());
    }
    else
    {
        emit dataChanged(index(newItemRow, 0), index(newItemRow, columnCount() - 1));
    }

    return newItemRow;
}

void CanMsgTableModel::clearAllMessages()
{
    int n = msgpool.size();
    if (n > 0)
    {
        beginRemoveRows(QModelIndex(), 0, n - 1);
        msgpool.clear();
        endRemoveRows();
        emit dataChanged(QModelIndex(), QModelIndex());
    }
}

