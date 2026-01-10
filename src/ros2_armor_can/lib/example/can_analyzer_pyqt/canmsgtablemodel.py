#!/usr/bin/env python
#coding=utf-8
import time
import math

from PyQt5.QtCore import QAbstractTableModel, QVariant, Qt, QModelIndex

class CanMsgTableModel(QAbstractTableModel):
    def __init__(self):
        super().__init__()
        self.headers = [
                "Time (s)",
                "TX/RX",
                "Channel",
                "ID (HEX)", 
                "Type",
                "Length (B)", 
                "Payload",
            ]
        self.msgpool = []

    def headerData(self, section, orientation, role):
        if (orientation == Qt.Horizontal and role == Qt.DisplayRole):
            if section < len(self.headers):
                return self.headers[section]
        return QVariant()

    def rowCount(self, parent):
        if (parent.isValid()):
            return 0
        return len(self.msgpool)

    def columnCount(self, parent):
        if (parent.isValid()):
            return 0
        return len(self.headers)

    def data(self, index, role):
        if (not index.isValid()):
            return QVariant()

        msg = self.msgpool[index.row()]
        if (role == Qt.DisplayRole):
            if (index.column() == 0):
                return time.strftime('%H:%M:%S', time.localtime(msg.timestamp)) + ('.%.6f' % (msg.timestamp - math.ceil(msg.timestamp)))[2:]
            elif (index.column() == 1):
                return 'TX' if msg.channel is None else 'RX'
            elif (index.column() == 2):
                return msg._dict['port']
            elif (index.column() == 3):
                return '%0X' % msg.arbitration_id
            elif (index.column() == 4):
                typestr = ''
                if (msg.is_extended_id):
                    typestr += 'IDE '
                if (msg.is_fd):
                    typestr += 'FDF '
                if (msg.bitrate_switch):
                    typestr += 'BRS '
                if (msg.is_remote_frame):
                    typestr += 'RTR '
                if (msg.error_state_indicator):
                    typestr += 'ESI '
                if (msg.is_error_frame):
                    typestr += 'ERR '
                if (len(typestr) == 0):
                    typestr = 'STD'
                return typestr
            elif (index.column() == 5):
                return '%d' % msg.dlc
            elif (index.column() == 6):
                return msg.data.hex().upper()

        return QVariant()


    def insertMessage(self, msg, port):
        msg.port = port
        allChanged = False   
        if (len(self.msgpool) > 1000000):
            self.beginRemoveRows(QModelIndex(), 0, 0)
            self.msgpool = self.msgpool[1:]
            self.endRemoveRows()
            allChanged = True
        
        newItemRow = len(self.msgpool)
        self.beginInsertRows(QModelIndex(), newItemRow, newItemRow)
        self.msgpool.append(msg)
        self.endInsertRows()

        if (allChanged):
            self.dataChanged.emit(QModelIndex(), QModelIndex())
        else:
            self.dataChanged.emit(self.index(newItemRow, 0), self.index(newItemRow, len(self.headers) - 1))
    
        return newItemRow

    def clearAllMessages(self):
        n = len(self.msgpool)
        if (n > 0):
            self.beginRemoveRows(QModelIndex(), 0, n - 1)
            self.msgpool.clear()
            self.endRemoveRows()
            self.dataChanged.emit(QModelIndex(), QModelIndex())
    


