#ifndef CANMSGTABLEMODEL_H
#define CANMSGTABLEMODEL_H

#include <QAbstractTableModel>
#include "bmapi.h"

class CanMsgTableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    explicit CanMsgTableModel(QObject *parent = nullptr);

    // Header:
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    // Basic functionality:
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

public:
    int  insertMessage(const BM_DataTypeDef& msg, int port);
    void clearAllMessages();

protected:
    QString msgId(const BM_DataTypeDef& msg) const;
    QString msgType(const BM_DataTypeDef& msg) const;
    uint32_t msgLength(const BM_DataTypeDef& msg) const;
    QString msgPayload(const BM_DataTypeDef& msg) const;

private:
    typedef struct MsgData
    {
        BM_DataTypeDef msg;
        uint32_t port;
    } MsgDataTypeDef;
    QList<MsgDataTypeDef> msgpool;

};

#endif // CANMSGTABLEMODEL_H
