using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Runtime.InteropServices;

namespace can_analyzer_csharp
{
    public partial class MainForm : Form
    {
        private BM_CanMessageTypeDef rxMsgBuf;
        private BM_CanMessageTypeDef txMsgBuf;
        private BM_ChannelInfoTypeDef[] channelinfos = new BM_ChannelInfoTypeDef[32];
        private IntPtr[] channelHandles = new IntPtr[32];
        private uint openedChannelCount;
        private System.Windows.Forms.Timer rxTimer;
        private System.Windows.Forms.Timer txTimer;
        int nchannels = 0;////Number of enumerated channels
        private bool[] channelSelected = new bool[32];
        private uint pendingTxMsgCount;
        public MainForm()
        {
            InitializeComponent();
            Array.Clear(channelHandles, 0, channelHandles.Length);
            Array.Clear(channelSelected, 0, channelSelected.Length);
            this.openedChannelCount = 0;
            this.modeCombo.SelectedIndex = 0;
            this.txTypeCombo.SelectedIndex = 0;
            this.rxMsgBuf = new BM_CanMessageTypeDef();
            this.txMsgBuf = new BM_CanMessageTypeDef();
            BMAPI.BM_Init();
            enumerate();
        }

        private void enumerate()
        {
            int ninfos = 32;
            IntPtr infomem = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(BM_ChannelInfoTypeDef)) * ninfos);
            BMAPI.BM_Enumerate(infomem, ref ninfos);
            this.channelComboBox.Items.Clear();
            nchannels = ninfos;
            for (int i = 0; i < ninfos; i++)
            {
                IntPtr ptr = (IntPtr)(infomem.ToInt64() + i * Marshal.SizeOf(typeof(BM_ChannelInfoTypeDef)));
                channelinfos[i] = (BM_ChannelInfoTypeDef)Marshal.PtrToStructure(ptr, typeof(BM_ChannelInfoTypeDef));
                this.channelComboBox.AddItems(channelinfos[i].GetName());
            }
            Marshal.FreeHGlobal(infomem);
        }


        private void openButton_Click(object sender, EventArgs e)
        {
            bool success = true;
            bool isOpen = this.openButton.Text == "Open" ? true : false;
            int ninfos = 32;
            IntPtr infomem = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(BM_ChannelInfoTypeDef)) * ninfos);
            for (int i = 0; i < nchannels; i++)
            {
                if (isOpen && channelSelected[i] == true)
                {
                    BM_CanModeTypeDef mode = (BM_CanModeTypeDef)modeCombo.SelectedIndex;
                    BM_TerminalResistorTypeDef tres = tresCheck.Checked ? BM_TerminalResistorTypeDef.BM_TRESISTOR_120 : BM_TerminalResistorTypeDef.BM_TRESISTOR_DISABLED;
                    BM_BitrateTypeDef bitrate = new BM_BitrateTypeDef();
                    bitrate.nbitrate = Convert.ToUInt16(nominalBitrateEdit.Text);
                    bitrate.dbitrate = Convert.ToUInt16(dataBitrateEdit.Text);
                    bitrate.nsamplepos = Convert.ToByte(samplePosSpin.Value);
                    bitrate.dsamplepos = Convert.ToByte(samplePosSpin.Value);
                    IntPtr rxfilterMem = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(BM_RxFilterTypeDef)));
                    BM_RxFilterTypeDef rxfilter = (BM_RxFilterTypeDef)Marshal.PtrToStructure(rxfilterMem, typeof(BM_RxFilterTypeDef));
                    rxfilter.type = rxAdvancedFilterCheck.Checked ? (byte)BM_RxFilterTypeTypeDef.BM_RXFILTER_ADVANCED : (byte)BM_RxFilterTypeTypeDef.BM_RXFILTER_BASIC;
                    rxfilter.SetIdFilter(Convert.ToUInt32(rxMaskEdit.Text, 16), Convert.ToUInt32(rxIdEdit.Text, 16), false);
                    if (rxStandardCheck.Checked && !rxExtendedCheck.Checked)
                    {
                        rxfilter.flags_mask = (byte)BM_MessageFlagsTypeDef.BM_MESSAGE_FLAGS_IDE;
                        rxfilter.flags_value = 0;
                    }
                    else if (!rxStandardCheck.Checked && rxExtendedCheck.Checked)
                    {
                        rxfilter.flags_mask = (byte)BM_MessageFlagsTypeDef.BM_MESSAGE_FLAGS_IDE;
                        rxfilter.flags_value = (byte)BM_MessageFlagsTypeDef.BM_MESSAGE_FLAGS_IDE;
                    }
                    else
                    {
                        /* Default as all enabled */
                        rxStandardCheck.Checked = true;
                        rxExtendedCheck.Checked = true;
                        rxfilter.flags_mask = 0;
                        rxfilter.flags_value = 0;
                    }
                    if (rxAdvancedFilterCheck.Checked)
                    {
                        int j = 0;
                        String payload = rxDataMaskEdit.Text.Replace(" ", "").ToLower().Replace('x', '0');
                        while (j * 2 + 2 <= payload.Length)
                        {
                            rxfilter.payload_mask[j] = Convert.ToByte(payload.Substring(j * 2, 2), 16);
                            rxfilter.payload_value[j] = rxfilter.payload_mask[j];
                            j++;
                        }
                        MessageBox.Show("Advanced RX filters are currently unsupported.", "Warning");
                        rxAdvancedFilterCheck.Enabled = false;
                    }
                    Marshal.StructureToPtr(rxfilter, rxfilterMem, true);

                    BM_StatusTypeDef openResult = BMAPI.BM_OpenEx(
                        ref channelHandles[this.openedChannelCount],
                        ref channelinfos[i],
                        mode,
                        tres,
                        ref bitrate,
                        rxfilterMem,
                        1
                    );

                    if (this.channelHandles[this.openedChannelCount] == IntPtr.Zero)
                    {
                        MessageBox.Show("Failed to open channel.", "Error");
                        success = false;
                    }
                    else
                    {
                        this.sendComboBox.Items.Add(this.openedChannelCount + 1);
                    }

                    this.openedChannelCount++;
                }
                else
                {
                    if (txButton.Text != "Transmit")
                    {
                        txButton_Click(null, null);
                    }
                    if (rxTimer != null)
                    {
                        rxTimer.Stop();
                    }
                }
            }

            if (isOpen && success && this.openedChannelCount > 0)
            {
                sendComboBox.SelectedIndex = 0;
                msgListView.Items.Clear();
                rxTimer = new System.Windows.Forms.Timer();
                rxTimer.Interval = 50;
                rxTimer.Tick += new EventHandler(this.readPendingMessages);
                rxTimer.Start();
                openButton.Text = "Close";
                rxIdEdit.Enabled = false;
                rxMaskEdit.Enabled = false;
                rxExtendedCheck.Enabled = false;
                rxStandardCheck.Enabled = false;
                //rxAdvancedFilterCheck.Enabled = false;
                rxDataMaskEdit.Enabled = false;
                modeCombo.Enabled = false;
                tresCheck.Enabled = false;
                nominalBitrateEdit.Enabled = false;
                dataBitrateEdit.Enabled = false;
                samplePosSpin.Enabled = false;
                enumerateButton.Enabled = false;
                txButton.Enabled = true;
                channelComboBox.Enabled = false;
            }

            else
            {
                for (int i = 0; i < this.openedChannelCount; i++)
                {
                    BMAPI.BM_Close(this.channelHandles[i]);
                }
                this.openedChannelCount = 0;
                sendComboBox.Items.Clear();
                sendComboBox.Text = "";
                openButton.Text = "Open";
                rxIdEdit.Enabled = true;
                rxMaskEdit.Enabled = true;
                rxExtendedCheck.Enabled = true;
                rxStandardCheck.Enabled = true;
                //rxAdvancedFilterCheck.Enabled = true;
                rxDataMaskEdit.Enabled = rxAdvancedFilterCheck.Checked;
                modeCombo.Enabled = true;
                tresCheck.Enabled = true;
                nominalBitrateEdit.Enabled = true;
                dataBitrateEdit.Enabled = true;
                samplePosSpin.Enabled = true;
                enumerateButton.Enabled = true;
                txButton.Enabled = false;
                channelComboBox.Enabled = true;
            }
        }

        private void rxAdvancedFilterCheck_CheckedChanged(object sender, EventArgs e)
        {
            this.rxDataMaskEdit.Enabled = this.rxAdvancedFilterCheck.Checked;
        }

        private void enumerateButton_Click(object sender, EventArgs e)
        {
            enumerate();
        }

        private void txButton_Click(object sender, EventArgs e)
        {
            bool success = false;
            bool isTransmit = this.txButton.Text == "Transmit" ? true : false;
            int channelIndex = this.sendComboBox.SelectedIndex;
            if (isTransmit && channelIndex >= 0 && this.channelHandles[channelIndex] != IntPtr.Zero)
            {
                uint id = Convert.ToUInt32(txIdEdit.Text, 16);
                uint length = Convert.ToUInt32(txLengthEdit.Text);
                uint dlc = txMsgBuf.lengthToDlc(length);
                uint[] DlcToDataBytes = new uint[]
                {
                    0,  1,  2,  3,  4,  5,  6,  7,
                    8, 12, 16, 20, 24, 32, 48, 64,
                };
                uint dlcToLength = DlcToDataBytes[dlc];
                if (dlcToLength != length)
                {
                    MessageBox.Show("According to ISO standard, CANFD message payload length could only be 0-8,12,16,20,24,32,48 or 64.", "Warning");
                }
                txMsgBuf = new BM_CanMessageTypeDef(id, dlc, txExtendedCheck.Checked, false, txFdCheck.Checked, txFdCheck.Checked, false);
                String payload = txDataEdit.Text.Replace(" ", "");
                int j = 0;
                while (j * 2 + 2 <= payload.Length)
                {
                    this.txMsgBuf.payload[j] = Convert.ToByte(payload.Substring(j * 2, 2), 16);
                    j++;
                }
                success = true;
            }


            if (isTransmit && success)
            {
                this.pendingTxMsgCount = Convert.ToUInt32(txCountEdit.Text);
                int cycle = Convert.ToInt32(txCycleEdit.Text);
                txTimer = new System.Windows.Forms.Timer();
                txTimer.Interval = cycle;
                txTimer.Tick += new EventHandler(this.writePendingMessages);
                txTimer.Start();
                txButton.Text = "Stop";
                writePendingMessages(null, null);
            }
            else
            {
                if (txTimer != null)
                {
                    txTimer.Stop();
                }
                pendingTxMsgCount = 0;
                txButton.Text = "Transmit";
            }
        }


        private void readPendingMessages(object source, EventArgs e)
        {
            for (int i = 0; i < this.openedChannelCount; i++)
            {
                if (this.channelHandles[i] != IntPtr.Zero)
                {
                    uint timestamp = 0;
                    uint port = 0;
                    //msgListView.BeginUpdate();
                    while (BMAPI.BM_ReadCanMessage(this.channelHandles[i], ref this.rxMsgBuf, ref port, ref timestamp) == BM_StatusTypeDef.BM_ERROR_OK)
                    {
                        DisplayMessage(i + 1, this.rxMsgBuf, timestamp, port, tx: false);
                    }
                    //msgListView.EndUpdate();
                }
            }
        }

        private void writePendingMessages(object source, EventArgs e)
        {
            int channelIndex = this.sendComboBox.SelectedIndex;
            if (channelIndex >= 0 && this.channelHandles[channelIndex] != IntPtr.Zero)
            {
                if (this.pendingTxMsgCount > 0)
                {
                    uint timestamp = 0;
                    uint port = 0;
                    BM_StatusTypeDef status = BMAPI.BM_WriteCanMessage(this.channelHandles[channelIndex], ref this.txMsgBuf, port, 100, ref timestamp);
                    if (status == BM_StatusTypeDef.BM_ERROR_OK)
                    {
                        if (--pendingTxMsgCount <= 0)
                        {
                            txButton_Click(null, null);
                        }
                        DisplayMessage(channelIndex + 1, this.txMsgBuf, timestamp, port, tx: true);
                    }
                }
            }
        }

        private void DisplayMessage(int i, BM_CanMessageTypeDef canMsg, uint timestamp, uint port = 0, bool tx = false)
        {
            ListViewItem item = new ListViewItem();
            item.Text = Convert.ToString(timestamp * 1E-6f);
            item.SubItems.Add(tx ? "TX" : "RX");
            item.SubItems.Add(Convert.ToString(i));
            item.SubItems.Add(Convert.ToString(canMsg.GetID(), 16).ToUpper());
            item.SubItems.Add(canMsg.GetTypeString());
            item.SubItems.Add(Convert.ToString(canMsg.GetLength()));
            item.SubItems.Add(canMsg.GetPyloadString());
            msgListView.Items.Add(item);
            msgListView.Items[msgListView.Items.Count - 1].EnsureVisible();
        }

        protected override void WndProc(ref Message m)
        {
            const int WM_DEVICECHANGE = 0x219;
            try
            {
                if (m.Msg == WM_DEVICECHANGE)
                {
                    if (openButton.Text == "Open")
                    {
                        enumerate();
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
            base.WndProc(ref m);
        }

        // See for details: https://www.shuzhiduo.com/A/1O5EZB2Gd7/
        private void channelComboBox_ItemClick(object sender, ItemCheckEventArgs e)
        {
            channelSelected[e.Index] = !channelSelected[e.Index];
        }
    }
}

