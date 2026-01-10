#include "widget.h"
#include "ui_widget.h"

#include <QMessageBox>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , msgTableModel(NULL)
    , pendingTxMsgCount(0)
    , openedChannelCount(0)
{
    memset(channels, NULL, sizeof(channels));
    ui->setupUi(this);
    ui->modeCombo->clear();
    ui->modeCombo->addItem(tr("Normal"), (int)BM_CAN_NORMAL_MODE);
    ui->modeCombo->addItem(tr("CAN Only"), (int)BM_CAN_CLASSIC_MODE);
    ui->modeCombo->addItem(tr("Loopback"), (int)BM_CAN_EXTERNAL_LOOPBACK_MODE);
    ui->modeCombo->addItem(tr("Listen only"), (int)BM_CAN_LISTEN_ONLY_MODE);

    msgTableModel = new CanMsgTableModel;
    ui->msgTableView->setModel(msgTableModel);
    ui->msgTableView->horizontalHeader()->setSectionResizeMode(ui->msgTableView->horizontalHeader()->count() - 1, QHeaderView::Stretch);

    rxTimer = new QTimer(this);
    connect(rxTimer, &QTimer::timeout, this, &Widget::readPendingMessages);

    txTimer = new QTimer(this);
    connect(txTimer, &QTimer::timeout, this, &Widget::writePendingMessages);

    BM_Init();
    enumerate();
}

Widget::~Widget()
{
    BM_UnInit();
    delete ui;
}

int Widget::lengthToDlc(int n)
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
    else
    {
        dlc = 15;
    }
    return dlc;
}

void Widget::enumerate()
{
    int n = sizeof(channelinfos) / sizeof(channelinfos[0]);
    BM_Enumerate(channelinfos, &n);
    ui->channelCombo->clear();
    for (int i = 0; i < n; i++)
    {
        ui->channelCombo->appendItem(channelinfos[i].name, true);
    }
}

void Widget::on_openButton_clicked(bool checked)
{
    bool success = true;
    openedChannelCount = 0;
    ui->sendcomboBox->clear();
    if (checked)
    {
        for (int i = 0; i < ui->channelCombo->count(); i++)
        {
            if (ui->channelCombo->isChecked(i) == true)
            {
                BM_ChannelInfoTypeDef* channelinfo = &channelinfos[i]; // Iterate all enumerated ports here
                ui->sendcomboBox->addItem(QString::number(openedChannelCount + 1)); // Show opened port index here

                BM_CanModeTypeDef mode = (BM_CanModeTypeDef)ui->modeCombo->currentData().value<int>();
                BM_TerminalResistorTypeDef tres = ui->tresCheck->isChecked() ? BM_TRESISTOR_120 : BM_TRESISTOR_DISABLED;
                BM_BitrateTypeDef bitrate;
                memset(&bitrate, 0, sizeof(bitrate));
                bitrate.nbitrate = ui->nominalBitrateEdit->text().toUInt();
                bitrate.dbitrate = ui->dataBitrateEdit->text().toUInt();
                bitrate.nsamplepos = ui->samplePosSpin->value();
                bitrate.dsamplepos = bitrate.nsamplepos;
                BM_RxFilterTypeDef rxfilter;
                memset(&rxfilter, 0, sizeof(rxfilter));
                rxfilter.type = ui->rxAdvancedFilterCheck->isChecked() ? BM_RXFILTER_ADVANCED : BM_RXFILTER_BASIC;
                uint rxId = ui->rxIdEdit->text().toUInt(NULL, 16);
                uint rxMask = ui->rxMaskEdit->text().toUInt(NULL, 16);
                bool rxStd = ui->rxStandardCheck->isChecked();
                bool rxExt = ui->rxExtendedCheck->isChecked();
                if (rxId > 0x7FF || rxMask > 0x7FF)
                {
                    rxfilter.id_mask = ((rxMask >> 18) & 0x7FF) | ((rxMask & 0x3FFFF) << 11);
                    rxfilter.id_value = ((rxId >> 18) & 0x7FF) | ((rxId & 0x3FFFF) << 11);
                }
                else
                {
                    rxfilter.id_mask = rxMask;
                    rxfilter.id_value = rxId;
                }
                if (rxStd && !rxExt)
                {
                    rxfilter.flags_mask = BM_MESSAGE_FLAGS_IDE;
                    rxfilter.flags_value = 0;
                }
                else if (!rxStd && rxExt)
                {
                    rxfilter.flags_mask = BM_MESSAGE_FLAGS_IDE;
                    rxfilter.flags_value = BM_MESSAGE_FLAGS_IDE;
                }
                else
                {
                    /* Default as all enabled */
                    ui->rxStandardCheck->setChecked(true);
                    ui->rxExtendedCheck->setChecked(true);
                }
                if (rxfilter.type == BM_RXFILTER_ADVANCED)
                {
                    memset(rxfilter.payload_mask, 0, sizeof(rxfilter.payload_mask));
                    memset(rxfilter.payload_value, 0, sizeof(rxfilter.payload_value));
                    int j = 0;
                    QString payload = ui->rxDataMaskEdit->text().replace(QLatin1Char('x'), QLatin1Char('0'), Qt::CaseInsensitive);
                    while (j*2 + 2 <= payload.length())
                    {
                        rxfilter.payload_mask[j] = payload.mid(j*2, 2).toUInt(NULL, 16);
                        rxfilter.payload_value[j] = rxfilter.payload_mask[j];
                        j++;
                    }
                }
                BM_StatusTypeDef status = BM_OpenEx(&channels[openedChannelCount], channelinfo, mode, tres, &bitrate, &rxfilter, 1);
                if (status != BM_ERROR_OK)
                {
                    char buffer[1024];
                    BM_GetErrorText(status, buffer, sizeof(buffer), 0);
                    QString prompt = QString(tr("Failed to open channel %1, error=%2(%3)."))
                            .arg(channelinfo->name)
                            .arg((int)status, 8, 16)
                            .arg(buffer);
                    QMessageBox::critical(this, tr("Error"), prompt);
                    success = false;
                    BM_Close(channels[i]);
                }
                openedChannelCount++;
            }
        }
    }

    if (checked && success)
    {
        msgTableModel->clearAllMessages();
        rxTimer->start(50);
        ui->openButton->setText(tr("Close"));
        ui->rxIdEdit->setEnabled(false);
        ui->rxMaskEdit->setEnabled(false);
        ui->rxExtendedCheck->setEnabled(false);
        ui->rxStandardCheck->setEnabled(false);
        //ui->rxAdvancedFilterCheck->setEnabled(false);
        ui->rxDataMaskEdit->setEnabled(false);
        ui->modeCombo->setEnabled(false);
        ui->tresCheck->setEnabled(false);
        ui->nominalBitrateEdit->setEnabled(false);
        ui->dataBitrateEdit->setEnabled(false);
        ui->samplePosSpin->setEnabled(false);
        ui->enumerateButton->setEnabled(false);
        ui->txButton->setEnabled(true);
        ui->channelCombo->setEnabled(false);
    }
    else
    {
        rxTimer->stop();
        ui->openButton->setChecked(false);
        ui->openButton->setText(tr("Open"));
        ui->rxIdEdit->setEnabled(true);
        ui->rxMaskEdit->setEnabled(true);
        ui->rxExtendedCheck->setEnabled(true);
        ui->rxStandardCheck->setEnabled(true);
        //ui->rxAdvancedFilterCheck->setEnabled(true);
        ui->rxDataMaskEdit->setEnabled(ui->rxAdvancedFilterCheck->isChecked());
        ui->modeCombo->setEnabled(true);
        ui->tresCheck->setEnabled(true);
        ui->nominalBitrateEdit->setEnabled(true);
        ui->dataBitrateEdit->setEnabled(true);
        ui->samplePosSpin->setEnabled(true);
        ui->enumerateButton->setEnabled(true);
        ui->txButton->setEnabled(false);
        ui->channelCombo->setEnabled(true);
    }

    if (success == false)
    {
        QMessageBox::critical(this, tr("Error"), tr("Please plugin your Busmust device and press 'Enumerate'."));
    }
}

void Widget::on_enumerateButton_clicked()
{
    enumerate();
}

void Widget::on_txButton_clicked(bool checked)
{
    bool success = false;
    int port = ui->sendcomboBox->currentIndex();
    if (port >= 0 && checked && channels[port] && port < int(sizeof(channels) / sizeof(channels[0])))
    {
        BM_CanMessageTypeDef* canMsg = (BM_CanMessageTypeDef*)pendingTxMsg.payload;
        uint id = ui->txIdEdit->text().toUInt(NULL, 16);
        memset(&pendingTxMsg, 0, sizeof(pendingTxMsg));
        canMsg->ctrl.tx.BRS = ui->txFdCheck->isChecked();
        canMsg->ctrl.tx.FDF = ui->txFdCheck->isChecked();
        canMsg->ctrl.tx.IDE = ui->txExtendedCheck->isChecked();
        if (canMsg->ctrl.tx.IDE)
        {
            canMsg->id.SID = (id >> 18) & 0x7FF;
            canMsg->id.EID = (id & 0x3FFFF);
        }
        else
        {
            canMsg->id.SID = id & 0x7FF;
        }
        uint length = ui->txLengthEdit->text().toUInt();
        int dlc = lengthToDlc(length);
        uint dlcToDataBytes[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
        int dlcToLength = dlcToDataBytes[dlc];
        if (dlcToLength != length)
        {
            QMessageBox::warning(this, tr("Warning"), tr("According to ISO standard, CANFD message payload length could only be 0-8,12,16,20,24,32,48 or 64."));
        }
        canMsg->ctrl.tx.DLC = dlc;
        QString payload = ui->txDataEdit->text().remove(QLatin1Char(' '));
        int j = 0;
        while (j*2 + 2 <= payload.length())
        {
            canMsg->payload[j] = payload.mid(j*2, 2).toUInt(NULL, 16);
            j++;
        }
        pendingTxMsg.header.type = BM_CAN_FD_DATA;
        pendingTxMsg.length = sizeof(BM_CanMessageTypeDef);
        success = true;
    }


    if (checked && success)
    {
        pendingTxMsgCount = ui->txCountEdit->text().toUInt();
        txTimer->start(ui->txCycleEdit->text().toUInt());
        ui->txButton->setText(tr("Stop"));
        writePendingMessages();
    }
    else
    {
        txTimer->stop();
        pendingTxMsgCount = 0;
        ui->txButton->setChecked(false);
        ui->txButton->setText(tr("Transmit"));
    }
}

void Widget::readPendingMessages(void)
{
    BM_DataTypeDef msg;
    bool newMsgInserted = false;
    for (int i = 0; i < openedChannelCount; i++)
    {
        if (channels[i] && msgTableModel)
        {
            while (BM_Read(channels[i], &msg) == BM_ERROR_OK)
            {
                msgTableModel->insertMessage(msg, i + 1);
                newMsgInserted = true;
            }
        }
        if (newMsgInserted)
        {
            ui->msgTableView->scrollToBottom();
        }
    }
}

void Widget::writePendingMessages(void)
{
    int port = ui->sendcomboBox->currentIndex();
    if (channels[port] && msgTableModel)
    {
        if (pendingTxMsgCount > 0)
        {
            uint32_t timestamp = 0;
            BM_StatusTypeDef status = BM_Write(channels[port], &pendingTxMsg, 100, &timestamp);
            if (status == BM_ERROR_OK)
            {
                if (--pendingTxMsgCount <= 0)
                {
                    on_txButton_clicked(false);
                }
                BM_DataTypeDef ackMsg = pendingTxMsg;
                ackMsg.timestamp = timestamp;
                ackMsg.header.type |= BM_ACK_DATA;
                msgTableModel->insertMessage(ackMsg, port + 1);
                ui->msgTableView->scrollToBottom();
            }
        }
    }
}

void Widget::on_rxAdvancedFilterCheck_clicked(bool checked)
{
    ui->rxDataMaskEdit->setEnabled(checked);
}



