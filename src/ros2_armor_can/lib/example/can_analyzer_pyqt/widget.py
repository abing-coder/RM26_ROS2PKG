# coding:utf-8

import time
from PyQt5.QtWidgets import QWidget, QMessageBox, QHeaderView
from PyQt5 import Qt
from PyQt5 import QtCore
from PyQt5.QtCore import qDebug
import ui_widget
from canmsgtablemodel import CanMsgTableModel
from mycombobox import MyComboBox
from can.interfaces.bmcan import BmCanBus
import can

tr = QtCore.QCoreApplication.translate
class Widget(QWidget, ui_widget.Ui_Widget):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.openButton.setCheckable(False)
        self.openButton.clicked.disconnect()
        self.openButton.clicked.connect(self.on_openButton_clicked)
        self.txButton.setCheckable(False)
        self.txButton.clicked.disconnect()
        self.txButton.clicked.connect(self.on_txButton_clicked)
        self.modeCombo.addItem("Normal")
        self.modeCombo.addItem("Classic")
        self.modeCombo.addItem("Loopback")
        self.modeCombo.addItem("ListenOnly")
        
        self.msgTableModel = CanMsgTableModel()
        self.msgTableView.setModel(self.msgTableModel)
        self.msgTableView.horizontalHeader().setSectionResizeMode(self.msgTableView.horizontalHeader().count() - 1, QHeaderView.Stretch)

        self.isOpened = False
        self.isTransmitting = False
        self.pendingTxMsgCount = 0

        self.rxTimer = QtCore.QTimer()
        self.txTimer = QtCore.QTimer()
        self.rxTimer.timeout.connect(self.readPendingMessages)
        self.txTimer.timeout.connect(self.writePendingMessages)

        self.channelhandles = []

        self.enumerate()

    def enumerate(self):
        self.channels = BmCanBus.enumerate()
        self.channelCombo.clear()
        for channel in self.channels:
            self.channelCombo.add_item(channel['name'])

    def readPendingMessages(self):
        newMsgInserted = False
        for index in range(len(self.channelhandles)):           
            if (self.channelhandles[index] and self.msgTableModel):
                while True:
                    try:
                        msg = self.channelhandles[index].recv(timeout=0)
                    except:
                        msg = None
                        pass
                    if msg is not None: # Return None indicates timeout condition
                        self.msgTableModel.insertMessage(msg, index + 1)
                        newMsgInserted = True
                    else:
                        break
            if (newMsgInserted):
                self.msgTableView.scrollToBottom()

    def writePendingMessages(self):           
        i = self.txChannelCombo.currentIndex()
        if i >= 0 and self.msgTableModel:
            if (self.pendingTxMsgCount >= 0):
                try:
                    self.channelhandles[i].send(self.pendingTxMsg, timeout=0.5)
                except Exception as exc:
                    qDebug(str(exc))
                else:
                    timestamp = time.time()
                    #BM_StatusTypeDef status = BM_Write(channel, &pendingTxMsg, 100, &timestamp)
                    self.pendingTxMsgCount -= 1
                    if (self.pendingTxMsgCount <= 0 and self.isTransmitting):
                        self.on_txButton_clicked()

                    ackMsg = self.pendingTxMsg
                    ackMsg.timestamp = timestamp
                    ackMsg.is_ack = True
                    self.msgTableModel.insertMessage(ackMsg, i + 1)
                    self.msgTableView.scrollToBottom()
        pass

    def on_openButton_clicked(self):
        success = True
        self.channelhandles.clear()
        self.txChannelCombo.clear()  
        if not self.isOpened:
            if not self.channels:
                QMessageBox.critical(None, "Error", "Please plugin your Busmust device and press 'Enumerate'.")
                success = False
            for index in range(len(self.channels)):
                if self.channelCombo.isChecked(index):#被勾选的子项
                    channel = self.channels[index]
                    self.txChannelCombo.addItem(str(len(self.channelhandles) + 1))    
                    mode = self.modeCombo.currentText().lower()
                    tres = self.tresCheck.isChecked()
                    nbitrate = int(self.nominalBitrateEdit.text())
                    dbitrate = int(self.dataBitrateEdit.text())
                    samplepos = int(self.samplePosSpin.value())
                    filters = []
                    if self.rxAdvancedFilterCheck.isChecked():
                        QMessageBox.critical(None, 'Error', 'Advanced filter is currently not supported by demo.', QMessageBox.Ok)
                    else:
                        rxid = int(self.rxIdEdit.text(), 16)
                        rxmask = int(self.rxMaskEdit.text(), 16)
                        rxstd = self.rxStandardCheck.isChecked()
                        rxext = self.rxExtendedCheck.isChecked()
                        if not rxstd and not rxext:
                            rxstd = True
                            rxext = True
                            self.rxStandardCheck.setChecked()
                            self.rxExtendedCheck.setChecked()
                        if rxstd:
                            filters.append({"can_id": rxid, "can_mask": rxmask, "extended": False})
                        if rxext:
                            filters.append({"can_id": rxid, "can_mask": rxmask, "extended": True})

                        try:
                            print('Open channel %s: nbitrate=%d, dbitrate=%d, tres=%s' % (channel['name'], nbitrate, dbitrate, str(tres)))
                            handle = BmCanBus(bustype='bmcan',
                                channel=channel['name'], bitrate=nbitrate * 1000, data_bitrate=dbitrate * 1000, tres=tres, 
                                fd='classic' not in mode, 
                                receive_own_messages='loopback' in mode,
                                listen_only='listen' in mode,
                                can_filters=filters
                            )
                            self.channelhandles.append(handle)
                        except Exception as exc:
                            QMessageBox.critical(None, 'Error', "Failed to open channel %s:\r\n%s" % (channel['name'], str(exc)), QMessageBox.Ok)
                            success = False
        else:
            if self.isTransmitting:
                self.on_txButton_clicked()
            for channelhandle in self.channelhandles:
                if channelhandle is not None:
                    channelhandle.shutdown()
            self.channelhandles.clear()

        if not self.isOpened and success:
            #msgTableModel.clearAllMessages()
            self.rxTimer.start(50)
            self.openButton.setText(tr("Widget", "Close"))
            self.rxIdEdit.setEnabled(False)
            self.rxMaskEdit.setEnabled(False)
            self.rxExtendedCheck.setEnabled(False)
            self.rxStandardCheck.setEnabled(False)
            #self.rxAdvancedFilterCheck.setEnabled(False)
            self.rxDataMaskEdit.setEnabled(False)
            self.modeCombo.setEnabled(False)
            self.tresCheck.setEnabled(False)
            self.nominalBitrateEdit.setEnabled(False)
            self.dataBitrateEdit.setEnabled(False)
            self.samplePosSpin.setEnabled(False)
            self.enumerateButton.setEnabled(False)
            self.txButton.setEnabled(True)
            self.isOpened = True
            self.channelCombo.setEnabled(False)
        else:
            self.rxTimer.stop()
            self.openButton.setText(tr("Widget", "Open"))
            self.rxIdEdit.setEnabled(True)
            self.rxMaskEdit.setEnabled(True)
            self.rxExtendedCheck.setEnabled(True)
            self.rxStandardCheck.setEnabled(True)
            #self.rxAdvancedFilterCheck.setEnabled(True)
            self.rxDataMaskEdit.setEnabled(self.rxAdvancedFilterCheck.isChecked())
            self.modeCombo.setEnabled(True)
            self.tresCheck.setEnabled(True)
            self.nominalBitrateEdit.setEnabled(True)
            self.dataBitrateEdit.setEnabled(True)
            self.samplePosSpin.setEnabled(True)
            self.enumerateButton.setEnabled(True)
            self.txButton.setEnabled(False)
            self.isOpened = False
            self.channelCombo.setEnabled(True)

    def on_enumerateButton_clicked(self):
        self.enumerate()

    def on_txButton_clicked(self):
        success = False
        isOk = False
        DlcToDataBytes = [0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64,]
        i = self.txChannelCombo.currentIndex()
        if i >= 0:
            if (not self.isTransmitting and self.channelhandles[i]):
                length = int(self.txLengthEdit.text(), 10)
                dlc = 0
                dlcToLength = 0
                if (length <= 8):
                    dlc = length
                elif (length <= 12):
                    dlc = 9
                elif (length <= 16):
                    dlc = 10
                elif (length <= 20):
                    dlc = 11
                elif (length <= 24):
                    dlc = 12
                elif (length <= 32):
                    dlc = 13
                elif (length <= 48):
                    dlc = 14
                else:
                    dlc = 15
                dlcToLength = DlcToDataBytes[dlc]
                if not (dlcToLength == length):
                    QMessageBox.information(None, 'warning', 'According to ISO standard, CANFD message payload length could only be 0-8,12,16,20,24,32,48 or 64.',QMessageBox.Yes)
                data1 = bytes.fromhex(self.txDataEdit.text())
                payload = str(self.txDataEdit.text().replace(' ', ''))
                data2Len = int(dlcToLength - len(payload) / 2)
                if data2Len > 0:
                    data2 = bytes(data2Len)
                    data = data1 + data2 
                else:
                    data = data1
                self.pendingTxMsg = can.Message(
                    arbitration_id=int(self.txIdEdit.text(), 16),
                    dlc=dlcToLength,
                    data=data[:dlcToLength],
                    is_extended_id=self.txExtendedCheck.isChecked(),
                    is_fd=self.txFdCheck.isChecked(),
                    bitrate_switch=self.txFdCheck.isChecked(),
                )
                self.isTransmitting = True
                self.pendingTxMsgCount = int(self.txCountEdit.text())
                self.txTimer.start(int(self.txCycleEdit.text()))
                self.txButton.setText(tr("Widget", "Stop"))
                self.writePendingMessages()
            else:
                self.txTimer.stop()
                self.pendingTxMsgCount = 0
                self.txButton.setText(tr("Widget", "Transmit"))
                self.isTransmitting = False 
                
