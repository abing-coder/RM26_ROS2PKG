Imports System.Runtime.InteropServices
Partial Public Class MainForm
    Private rxMsgBuf As BM_CanMessageTypeDef
    Private txMsgBuf As BM_CanMessageTypeDef
    Private channelinfos(31) As BM_ChannelInfoTypeDef
    Private channelHandles(31) As IntPtr
    Private openedChannelCount As Integer
    Private nchannels As Integer 'Number Of enumerated channels
    Private channelSelected(31) As Boolean
    Private pendingTxMsgCount As Integer

    Public Sub MainForm()
        Array.Clear(channelHandles, 0, channelHandles.Length)
        Array.Clear(channelSelected, 0, channelSelected.Length)
        openedChannelCount = 0
        modeCombo.SelectedIndex = 0
        txTypeCombo.SelectedIndex = 0
        rxMsgBuf = New BM_CanMessageTypeDef()
        txMsgBuf = New BM_CanMessageTypeDef()
        BMAPI.BM_Init()
        enumerate()
    End Sub

    Private Function enumerate()
        Dim infosize As Integer = Marshal.SizeOf(GetType(BM_ChannelInfoTypeDef))
        Dim ninfos As Integer = 32
        Dim infomem As IntPtr
        infomem = Marshal.AllocHGlobal(ninfos * infosize)
        BMAPI.BM_Enumerate(infomem, ninfos)
        channelComboBox.Items.Clear()
        nchannels = ninfos
        For i As Integer = 0 To ninfos - 1
            Dim ptr As IntPtr = (infomem.ToInt64() + i * infosize)
            channelinfos(i) = Marshal.PtrToStructure(ptr, GetType(BM_ChannelInfoTypeDef))
            Dim name As String = channelinfos(i).GetName()
            channelComboBox.AddItems(name)
        Next
        Marshal.FreeHGlobal(infomem)
        Return vbNull
    End Function

    Private Sub openButton_Click(sender As Object, e As EventArgs) Handles openButton.Click
        Dim success As Boolean = True
        Dim isOpen As Boolean = False
        Dim infomem As IntPtr
        Dim tres As BM_TerminalResistorTypeDef
        If openButton.Text = "Open" Then
            isOpen = True
        End If
        Dim ninfos As Integer = 32
        infomem = Marshal.AllocHGlobal(Marshal.SizeOf(GetType(BM_ChannelInfoTypeDef)))
        For i As Integer = 0 To nchannels - 1
            If (isOpen And channelSelected(i)) Then
                Dim mode As BM_CanModeTypeDef = modeCombo.SelectedIndex
                If tresCheck.Checked Then
                    tres = BM_TerminalResistorTypeDef.BM_TRESISTOR_120
                Else
                    tres = BM_TerminalResistorTypeDef.BM_TRESISTOR_DISABLED
                End If

                Dim bitrate As BM_BitrateTypeDef = New BM_BitrateTypeDef()
                bitrate.nbitrate = Convert.ToUInt16(nominalBitrateEdit.Text)
                bitrate.dbitrate = Convert.ToUInt16(dataBitrateEdit.Text)
                bitrate.nsamplepos = Convert.ToByte(samplePosSpin.Value)
                bitrate.dsamplepos = Convert.ToByte(samplePosSpin.Value)
                Dim rxfilterMem As IntPtr = Marshal.AllocHGlobal(Marshal.SizeOf(GetType(BM_RxFilterTypeDef)))
                Dim rxfilter As BM_RxFilterTypeDef = Marshal.PtrToStructure(rxfilterMem, GetType(BM_RxFilterTypeDef))
                If rxAdvancedFilterCheck.Checked Then
                    rxfilter.type = BM_RxFilterTypeTypeDef.BM_RXFILTER_ADVANCED
                Else
                    rxfilter.type = BM_RxFilterTypeTypeDef.BM_RXFILTER_BASIC
                End If
                rxfilter.SetIdFilter(Convert.ToUInt32(rxMaskEdit.Text, 16), Convert.ToUInt32(rxIdEdit.Text, 16), False)
                If (rxStandardCheck.Checked And Not (rxExtendedCheck.Checked)) Then
                    rxfilter.flags_mask = BM_MessageFlagsTypeDef.BM_MESSAGE_FLAGS_IDE
                    rxfilter.flags_value = 0
                ElseIf (Not (rxStandardCheck.Checked) And rxExtendedCheck.Checked) Then
                    rxfilter.flags_mask = BM_MessageFlagsTypeDef.BM_MESSAGE_FLAGS_IDE
                    rxfilter.flags_value = BM_MessageFlagsTypeDef.BM_MESSAGE_FLAGS_IDE
                Else
                    '/* Default as all enabled */
                    rxStandardCheck.Checked = True
                    rxExtendedCheck.Checked = True
                    rxfilter.flags_mask = 0
                    rxfilter.flags_value = 0
                End If

                If (rxAdvancedFilterCheck.Checked) Then
                    Dim j As Integer
                    Dim payload As String = rxDataMaskEdit.Text.Replace(" ", "").ToLower().Replace("x", "0")
                    While (j * 2 + 2 <= payload.Length)
                        rxfilter.payload_mask(j) = Convert.ToByte(payload.Substring(j * 2, 2), 16)
                        rxfilter.payload_value(j) = rxfilter.payload_mask(j)
                        j = j + 1
                    End While
                    System.Windows.Forms.MessageBox.Show("Advanced RX filters are currently unsupported.", "Warning")
                    rxAdvancedFilterCheck.Enabled = False
                End If
                Marshal.StructureToPtr(rxfilter, rxfilterMem, True)

                Dim openResult As BM_StatusTypeDef = BMAPI.BM_OpenEx(
                        channelHandles(openedChannelCount),
                        channelinfos(i),
                        mode,
                        tres,
                        bitrate,
                        rxfilterMem,
                        1
                    )

                If (openResult = BM_StatusTypeDef.BM_ERROR_OK) Then
                    sendComboBox.Items.Add(openedChannelCount + 1)
                    openedChannelCount = openedChannelCount + 1
                Else
                    System.Windows.Forms.MessageBox.Show("Failed to open channel.", "Error")
                    success = False
                End If
            Else
                If (Not (txButton.Text = "Transmit")) Then
                    txButton_Click()
                End If

                If (rxTimer.Enabled) Then
                    rxTimer.Stop()
                End If
            End If


            If (isOpen And success And openedChannelCount > 0) Then
                sendComboBox.SelectedIndex = 0
                msgListView.Items.Clear()
                rxTimer = New System.Windows.Forms.Timer()
                rxTimer.Interval = 50
                rxTimer.Start()
                openButton.Text = "Close"
                rxIdEdit.Enabled = False
                rxMaskEdit.Enabled = False
                rxExtendedCheck.Enabled = False
                rxStandardCheck.Enabled = False
                'rxAdvancedFilterCheck.Enabled = false
                rxDataMaskEdit.Enabled = False
                modeCombo.Enabled = False
                tresCheck.Enabled = False
                nominalBitrateEdit.Enabled = False
                dataBitrateEdit.Enabled = False
                samplePosSpin.Enabled = False
                enumerateButton.Enabled = False
                txButton.Enabled = True
                channelComboBox.Enabled = False

            Else
                For j As Integer = 0 To openedChannelCount - 1
                    BMAPI.BM_Close(channelHandles(j))
                Next
                openedChannelCount = 0
                sendComboBox.Items.Clear()
                sendComboBox.Text = ""
                openButton.Text = "Open"
                rxIdEdit.Enabled = True
                rxMaskEdit.Enabled = True
                rxExtendedCheck.Enabled = True
                rxStandardCheck.Enabled = True
                'rxAdvancedFilterCheck.Enabled = true
                rxDataMaskEdit.Enabled = rxAdvancedFilterCheck.Checked
                modeCombo.Enabled = True
                tresCheck.Enabled = True
                nominalBitrateEdit.Enabled = True
                dataBitrateEdit.Enabled = True
                samplePosSpin.Enabled = True
                enumerateButton.Enabled = True
                txButton.Enabled = False
                channelComboBox.Enabled = True
            End If
        Next

    End Sub


    Private Function rxAdvancedFilterCheck_CheckedChanged()
        rxDataMaskEdit.Enabled = rxAdvancedFilterCheck.Checked
        Return vbNull
    End Function
    Private Sub enumerateButton_Click(sender As Object, e As EventArgs) Handles enumerateButton.Click
        enumerate()
    End Sub

    Private Function txButton_Click() Handles txButton.Click
        Dim success As Boolean = False
        Dim isTransmit As Boolean = False
        If txButton.Text = "Transmit" Then
            isTransmit = True
        End If
        Dim channelIndex As Integer
        channelIndex = sendComboBox.SelectedIndex
        If (channelHandles(channelIndex) = IntPtr.Zero) Then
            Return False
        End If
        If (isTransmit And channelIndex >= 0) Then
            Dim id As Integer = Convert.ToUInt32(txIdEdit.Text, 16)
            Dim length As Integer = Convert.ToUInt32(txLengthEdit.Text)
            Dim dlc As Integer = txMsgBuf.lengthToDLc(length)
            Dim DlcToDataBytes() As Integer = New Integer(15) {0, 1, 2, 3,
                                                           4, 5, 6, 7,
                                                           8, 12, 16, 20,
                                                           24, 32, 48, 64}
            Dim dlcToLength As Integer = DlcToDataBytes(dlc)
            If Not (length = dlcToLength) Then
                System.Windows.Forms.MessageBox.Show("According to ISO standard, CANFD message payload length could only be 0-8,12,16,20,24,32,48 or 64.", "Warning")
            End If

            txMsgBuf = New BM_CanMessageTypeDef()
            txMsgBuf.BM_CanMessageTypeDef(id, dlc, txExtendedCheck.Checked, False, txFdCheck.Checked, txFdCheck.Checked, False)
            Dim payload As String = txDataEdit.Text.Replace(" ", "")
            Dim j As Integer = 0
            While (j * 2 + 2 <= payload.Length)
                txMsgBuf.payload(j) = Convert.ToByte(payload.Substring(j * 2, 2), 16)
                j = j + 1
            End While
            success = True
        End If

        If (isTransmit And success) Then
            pendingTxMsgCount = Convert.ToUInt32(txCountEdit.Text)
            Dim cycle As Integer = Convert.ToInt32(txCycleEdit.Text)
            txTimer = New System.Windows.Forms.Timer()
            txTimer.Interval = cycle
            txTimer.Start()
            txButton.Text = "Stop"
            writePendingMessages()
        Else
            If (txTimer.Enabled) Then
                txTimer.Stop()
                pendingTxMsgCount = 0
                txButton.Text = "Transmit"
            End If
        End If
        Return vbNull
    End Function


    Private Function readPendingMessages()
        For i As Integer = 0 To openedChannelCount - 1
            If (channelHandles(i)) Then
                Dim timestamp As Integer = 0
                Dim port As Integer = 0
                'msgListView.BeginUpdate();
                While (BMAPI.BM_ReadCanMessage(channelHandles(i), rxMsgBuf, port, timestamp) = BM_StatusTypeDef.BM_ERROR_OK)
                    DisplayMessage(i + 1, rxMsgBuf, timestamp, port, False)
                    'msgListView.EndUpdate();
                End While
            End If

        Next
        Return vbNull
    End Function

    Private Function writePendingMessages()
        Dim channelIndex As Integer = sendComboBox.SelectedIndex
        If channelIndex = -1 Then
            Return False
        End If
        If (channelHandles(channelIndex) = IntPtr.Zero) Then
            Return False
        End If
        If (channelIndex >= 0) Then
            If (pendingTxMsgCount > 0) Then
                Dim timestamp As Integer = 0
                Dim port As Integer = 0
                Dim status As BM_StatusTypeDef = BMAPI.BM_WriteCanMessage(channelHandles(channelIndex), txMsgBuf, port, 100, timestamp)
                If (status = BM_StatusTypeDef.BM_ERROR_OK) Then
                    pendingTxMsgCount = pendingTxMsgCount - 1
                    If (pendingTxMsgCount <= 0) Then
                        txButton_Click()
                    End If
                    DisplayMessage(channelIndex + 1, txMsgBuf, timestamp, port, True)
                End If
            End If
        End If

        Return vbNull
    End Function
    Private Function DisplayMessage(i As Integer, canMsg As BM_CanMessageTypeDef, timestamp As Integer, port As Integer, tx As Boolean)
        Dim item As System.Windows.Forms.ListViewItem = New System.Windows.Forms.ListViewItem()
        item.Text = Convert.ToString(timestamp * 0.000001F)

        If tx Then
            item.SubItems.Add("TX")
        Else
            item.SubItems.Add("RX")
        End If

        item.SubItems.Add(Convert.ToString(i))
        item.SubItems.Add(Convert.ToString(canMsg.GetID(), 16).ToUpper())
        item.SubItems.Add(canMsg.GetTypeString())
        item.SubItems.Add(Convert.ToString(canMsg.GetLength()))
        item.SubItems.Add(canMsg.GetPayloadString())
        msgListView.Items.Add(item)
        msgListView.Items(msgListView.Items.Count - 1).EnsureVisible()
        Return vbNull
    End Function

    'Protected Function WndProc(m As System.Windows.Forms.Message)
    '    Const WM_DEVICECHANGE As Integer = &H219
    '    Try
    '        If (m.Msg = WM_DEVICECHANGE) Then
    '            If (openButton.Text = "Open") Then
    '                enumerate()
    '            End If

    '        End If


    '    Catch
    '        Dim ex As Exception = New Exception
    '        System.Windows.Forms.MessageBox.Show(ex.Message)
    '    End Try
    '    WndProc(m)
    '    Return vbNull
    'End Function


    ' See for details https//www.shuzhiduo.com/A/1O5EZB2Gd7/
    Private Sub channelComboBox_ItemClick(sender As Object, e As System.Windows.Forms.ItemCheckEventArgs) Handles channelComboBox.ItemClick
        channelSelected(e.Index) = Not channelSelected(e.Index)
    End Sub


    Private Sub txTimer_Tick(ByVal sender As Object, ByVal e As System.EventArgs) Handles txTimer.Tick
        writePendingMessages()
    End Sub



    Private Sub rxTimer_Tick(sender As Object, e As EventArgs) Handles rxTimer.Tick
        readPendingMessages()
    End Sub
End Class
