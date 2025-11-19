namespace can_analyzer_csharp
{
    public class DoubleBufferListView : System.Windows.Forms.ListView
    {
        public DoubleBufferListView()
        {
            SetStyle(System.Windows.Forms.ControlStyles.DoubleBuffer |
              System.Windows.Forms.ControlStyles.OptimizedDoubleBuffer |
              System.Windows.Forms.ControlStyles.AllPaintingInWmPaint, true);
            UpdateStyles();
        }
    }
    partial class MainForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.label1 = new System.Windows.Forms.Label();
            this.channelComboBox = new ComboBoxCustomer.ComCheckBoxList();
            this.tresCheck = new System.Windows.Forms.CheckBox();
            this.label2 = new System.Windows.Forms.Label();
            this.modeCombo = new System.Windows.Forms.ComboBox();
            this.label3 = new System.Windows.Forms.Label();
            this.nominalBitrateEdit = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.dataBitrateEdit = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.samplePosSpin = new System.Windows.Forms.NumericUpDown();
            this.openButton = new System.Windows.Forms.Button();
            this.label6 = new System.Windows.Forms.Label();
            this.rxIdEdit = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.rxMaskEdit = new System.Windows.Forms.TextBox();
            this.rxStandardCheck = new System.Windows.Forms.CheckBox();
            this.rxExtendedCheck = new System.Windows.Forms.CheckBox();
            this.rxAdvancedFilterCheck = new System.Windows.Forms.CheckBox();
            this.label8 = new System.Windows.Forms.Label();
            this.rxDataMaskEdit = new System.Windows.Forms.TextBox();
            this.enumerateButton = new System.Windows.Forms.Button();
            this.label9 = new System.Windows.Forms.Label();
            this.txIdEdit = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.txTypeCombo = new System.Windows.Forms.ComboBox();
            this.txFdCheck = new System.Windows.Forms.CheckBox();
            this.txExtendedCheck = new System.Windows.Forms.CheckBox();
            this.label16 = new System.Windows.Forms.Label();
            this.sendComboBox = new System.Windows.Forms.ComboBox();
            this.label11 = new System.Windows.Forms.Label();
            this.txCycleEdit = new System.Windows.Forms.TextBox();
            this.label12 = new System.Windows.Forms.Label();
            this.txCountEdit = new System.Windows.Forms.TextBox();
            this.label13 = new System.Windows.Forms.Label();
            this.txLengthEdit = new System.Windows.Forms.TextBox();
            this.label14 = new System.Windows.Forms.Label();
            this.txDataEdit = new System.Windows.Forms.TextBox();
            this.txButton = new System.Windows.Forms.Button();
            this.msgListView = new can_analyzer_csharp.DoubleBufferListView();
            this.columnHeader1 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader2 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader3 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader4 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader5 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader6 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader7 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.bMCanModeTypeDefBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.flowLayoutPanel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.samplePosSpin)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.bMCanModeTypeDefBindingSource)).BeginInit();
            this.SuspendLayout();
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.flowLayoutPanel1.Controls.Add(this.label1);
            this.flowLayoutPanel1.Controls.Add(this.channelComboBox);
            this.flowLayoutPanel1.Controls.Add(this.tresCheck);
            this.flowLayoutPanel1.Controls.Add(this.label2);
            this.flowLayoutPanel1.Controls.Add(this.modeCombo);
            this.flowLayoutPanel1.Controls.Add(this.label3);
            this.flowLayoutPanel1.Controls.Add(this.nominalBitrateEdit);
            this.flowLayoutPanel1.Controls.Add(this.label4);
            this.flowLayoutPanel1.Controls.Add(this.dataBitrateEdit);
            this.flowLayoutPanel1.Controls.Add(this.label5);
            this.flowLayoutPanel1.Controls.Add(this.samplePosSpin);
            this.flowLayoutPanel1.Controls.Add(this.openButton);
            this.flowLayoutPanel1.Controls.Add(this.label6);
            this.flowLayoutPanel1.Controls.Add(this.rxIdEdit);
            this.flowLayoutPanel1.Controls.Add(this.label7);
            this.flowLayoutPanel1.Controls.Add(this.rxMaskEdit);
            this.flowLayoutPanel1.Controls.Add(this.rxStandardCheck);
            this.flowLayoutPanel1.Controls.Add(this.rxExtendedCheck);
            this.flowLayoutPanel1.Controls.Add(this.rxAdvancedFilterCheck);
            this.flowLayoutPanel1.Controls.Add(this.label8);
            this.flowLayoutPanel1.Controls.Add(this.rxDataMaskEdit);
            this.flowLayoutPanel1.Controls.Add(this.enumerateButton);
            this.flowLayoutPanel1.Controls.Add(this.label9);
            this.flowLayoutPanel1.Controls.Add(this.txIdEdit);
            this.flowLayoutPanel1.Controls.Add(this.label10);
            this.flowLayoutPanel1.Controls.Add(this.txTypeCombo);
            this.flowLayoutPanel1.Controls.Add(this.txFdCheck);
            this.flowLayoutPanel1.Controls.Add(this.txExtendedCheck);
            this.flowLayoutPanel1.Controls.Add(this.label16);
            this.flowLayoutPanel1.Controls.Add(this.sendComboBox);
            this.flowLayoutPanel1.Controls.Add(this.label11);
            this.flowLayoutPanel1.Controls.Add(this.txCycleEdit);
            this.flowLayoutPanel1.Controls.Add(this.label12);
            this.flowLayoutPanel1.Controls.Add(this.txCountEdit);
            this.flowLayoutPanel1.Controls.Add(this.label13);
            this.flowLayoutPanel1.Controls.Add(this.txLengthEdit);
            this.flowLayoutPanel1.Controls.Add(this.label14);
            this.flowLayoutPanel1.Controls.Add(this.txDataEdit);
            this.flowLayoutPanel1.Controls.Add(this.txButton);
            this.flowLayoutPanel1.Location = new System.Drawing.Point(13, 447);
            this.flowLayoutPanel1.MinimumSize = new System.Drawing.Size(917, 89);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(958, 89);
            this.flowLayoutPanel1.TabIndex = 1;
            // 
            // label1
            // 
            this.label1.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(3, 8);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(47, 12);
            this.label1.TabIndex = 0;
            this.label1.Text = "Channel";
            // 
            // channelComboBox
            // 
            this.channelComboBox.DataSource = null;
            this.channelComboBox.Location = new System.Drawing.Point(56, 2);
            this.channelComboBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.channelComboBox.Name = "channelComboBox";
            this.channelComboBox.Size = new System.Drawing.Size(192, 20);
            this.channelComboBox.TabIndex = 2;
            this.channelComboBox.ItemClick += new ComboBoxCustomer.ComCheckBoxList.CheckBoxListItemClick(this.channelComboBox_ItemClick);
            // 
            // tresCheck
            // 
            this.tresCheck.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.tresCheck.AutoSize = true;
            this.tresCheck.Checked = true;
            this.tresCheck.CheckState = System.Windows.Forms.CheckState.Checked;
            this.tresCheck.Location = new System.Drawing.Point(254, 6);
            this.tresCheck.Name = "tresCheck";
            this.tresCheck.Size = new System.Drawing.Size(54, 16);
            this.tresCheck.TabIndex = 5;
            this.tresCheck.Text = "T-Res";
            this.tresCheck.UseVisualStyleBackColor = true;
            // 
            // label2
            // 
            this.label2.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(314, 8);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(53, 12);
            this.label2.TabIndex = 2;
            this.label2.Text = "CAN Mode";
            // 
            // modeCombo
            // 
            this.modeCombo.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.modeCombo.FormattingEnabled = true;
            this.modeCombo.Items.AddRange(new object[] {
            "Normal",
            "Reserved",
            "Internal Loopback",
            "Listen Only",
            "Reserved",
            "External Loopback",
            "CAN Only",
            "Reserved"});
            this.modeCombo.Location = new System.Drawing.Point(373, 3);
            this.modeCombo.Name = "modeCombo";
            this.modeCombo.Size = new System.Drawing.Size(116, 20);
            this.modeCombo.TabIndex = 3;
            // 
            // label3
            // 
            this.label3.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(495, 8);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(77, 12);
            this.label3.TabIndex = 6;
            this.label3.Text = "Nominal kbps";
            // 
            // nominalBitrateEdit
            // 
            this.nominalBitrateEdit.Location = new System.Drawing.Point(578, 3);
            this.nominalBitrateEdit.Name = "nominalBitrateEdit";
            this.nominalBitrateEdit.Size = new System.Drawing.Size(24, 21);
            this.nominalBitrateEdit.TabIndex = 7;
            this.nominalBitrateEdit.Text = "500";
            // 
            // label4
            // 
            this.label4.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(608, 8);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(59, 12);
            this.label4.TabIndex = 8;
            this.label4.Text = "Data kbps";
            // 
            // dataBitrateEdit
            // 
            this.dataBitrateEdit.Location = new System.Drawing.Point(673, 3);
            this.dataBitrateEdit.Name = "dataBitrateEdit";
            this.dataBitrateEdit.Size = new System.Drawing.Size(34, 21);
            this.dataBitrateEdit.TabIndex = 10;
            this.dataBitrateEdit.Text = "2000";
            // 
            // label5
            // 
            this.label5.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(713, 8);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(77, 12);
            this.label5.TabIndex = 11;
            this.label5.Text = "Sample pos %";
            // 
            // samplePosSpin
            // 
            this.samplePosSpin.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.samplePosSpin.Increment = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.samplePosSpin.Location = new System.Drawing.Point(796, 4);
            this.samplePosSpin.Minimum = new decimal(new int[] {
            60,
            0,
            0,
            0});
            this.samplePosSpin.Name = "samplePosSpin";
            this.samplePosSpin.Size = new System.Drawing.Size(37, 21);
            this.samplePosSpin.TabIndex = 12;
            this.samplePosSpin.Value = new decimal(new int[] {
            75,
            0,
            0,
            0});
            // 
            // openButton
            // 
            this.openButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.flowLayoutPanel1.SetFlowBreak(this.openButton, true);
            this.openButton.Location = new System.Drawing.Point(839, 3);
            this.openButton.Name = "openButton";
            this.openButton.Size = new System.Drawing.Size(101, 23);
            this.openButton.TabIndex = 13;
            this.openButton.Text = "Open";
            this.openButton.UseVisualStyleBackColor = true;
            this.openButton.Click += new System.EventHandler(this.openButton_Click);
            // 
            // label6
            // 
            this.label6.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(3, 37);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(35, 12);
            this.label6.TabIndex = 14;
            this.label6.Text = "RX ID";
            // 
            // rxIdEdit
            // 
            this.rxIdEdit.Location = new System.Drawing.Point(44, 32);
            this.rxIdEdit.Name = "rxIdEdit";
            this.rxIdEdit.Size = new System.Drawing.Size(70, 21);
            this.rxIdEdit.TabIndex = 15;
            this.rxIdEdit.Text = "00000000";
            // 
            // label7
            // 
            this.label7.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(120, 37);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(29, 12);
            this.label7.TabIndex = 16;
            this.label7.Text = "Mask";
            // 
            // rxMaskEdit
            // 
            this.rxMaskEdit.Location = new System.Drawing.Point(155, 32);
            this.rxMaskEdit.Name = "rxMaskEdit";
            this.rxMaskEdit.Size = new System.Drawing.Size(84, 21);
            this.rxMaskEdit.TabIndex = 19;
            this.rxMaskEdit.Text = "00000000";
            // 
            // rxStandardCheck
            // 
            this.rxStandardCheck.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.rxStandardCheck.AutoSize = true;
            this.rxStandardCheck.Checked = true;
            this.rxStandardCheck.CheckState = System.Windows.Forms.CheckState.Checked;
            this.rxStandardCheck.Location = new System.Drawing.Point(245, 35);
            this.rxStandardCheck.Name = "rxStandardCheck";
            this.rxStandardCheck.Size = new System.Drawing.Size(84, 16);
            this.rxStandardCheck.TabIndex = 18;
            this.rxStandardCheck.Text = "Accept Std";
            this.rxStandardCheck.UseVisualStyleBackColor = true;
            // 
            // rxExtendedCheck
            // 
            this.rxExtendedCheck.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.rxExtendedCheck.AutoSize = true;
            this.rxExtendedCheck.Checked = true;
            this.rxExtendedCheck.CheckState = System.Windows.Forms.CheckState.Checked;
            this.rxExtendedCheck.Location = new System.Drawing.Point(335, 35);
            this.rxExtendedCheck.Name = "rxExtendedCheck";
            this.rxExtendedCheck.Size = new System.Drawing.Size(84, 16);
            this.rxExtendedCheck.TabIndex = 20;
            this.rxExtendedCheck.Text = "Accept Ext";
            this.rxExtendedCheck.UseVisualStyleBackColor = true;
            // 
            // rxAdvancedFilterCheck
            // 
            this.rxAdvancedFilterCheck.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.rxAdvancedFilterCheck.AutoSize = true;
            this.rxAdvancedFilterCheck.Enabled = false;
            this.rxAdvancedFilterCheck.Location = new System.Drawing.Point(425, 35);
            this.rxAdvancedFilterCheck.Name = "rxAdvancedFilterCheck";
            this.rxAdvancedFilterCheck.Size = new System.Drawing.Size(132, 16);
            this.rxAdvancedFilterCheck.TabIndex = 21;
            this.rxAdvancedFilterCheck.Text = "Advanced RX Filter";
            this.rxAdvancedFilterCheck.UseVisualStyleBackColor = true;
            this.rxAdvancedFilterCheck.CheckedChanged += new System.EventHandler(this.rxAdvancedFilterCheck_CheckedChanged);
            // 
            // label8
            // 
            this.label8.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(563, 37);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(59, 12);
            this.label8.TabIndex = 22;
            this.label8.Text = "Data Mask";
            // 
            // rxDataMaskEdit
            // 
            this.rxDataMaskEdit.Enabled = false;
            this.rxDataMaskEdit.Location = new System.Drawing.Point(628, 32);
            this.rxDataMaskEdit.Name = "rxDataMaskEdit";
            this.rxDataMaskEdit.Size = new System.Drawing.Size(205, 21);
            this.rxDataMaskEdit.TabIndex = 23;
            this.rxDataMaskEdit.Text = "xx xx xx xx xx xx xx xx";
            // 
            // enumerateButton
            // 
            this.enumerateButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.flowLayoutPanel1.SetFlowBreak(this.enumerateButton, true);
            this.enumerateButton.Location = new System.Drawing.Point(839, 32);
            this.enumerateButton.Name = "enumerateButton";
            this.enumerateButton.Size = new System.Drawing.Size(101, 23);
            this.enumerateButton.TabIndex = 24;
            this.enumerateButton.Text = "Enumerate";
            this.enumerateButton.UseVisualStyleBackColor = true;
            this.enumerateButton.Click += new System.EventHandler(this.enumerateButton_Click);
            // 
            // label9
            // 
            this.label9.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(3, 66);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(35, 12);
            this.label9.TabIndex = 25;
            this.label9.Text = "TX ID";
            // 
            // txIdEdit
            // 
            this.txIdEdit.Location = new System.Drawing.Point(44, 61);
            this.txIdEdit.Name = "txIdEdit";
            this.txIdEdit.Size = new System.Drawing.Size(70, 21);
            this.txIdEdit.TabIndex = 26;
            this.txIdEdit.Text = "7FF";
            // 
            // label10
            // 
            this.label10.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(120, 66);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(29, 12);
            this.label10.TabIndex = 27;
            this.label10.Text = "Type";
            // 
            // txTypeCombo
            // 
            this.txTypeCombo.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.txTypeCombo.FormattingEnabled = true;
            this.txTypeCombo.Items.AddRange(new object[] {
            "SW"});
            this.txTypeCombo.Location = new System.Drawing.Point(155, 61);
            this.txTypeCombo.Name = "txTypeCombo";
            this.txTypeCombo.Size = new System.Drawing.Size(61, 20);
            this.txTypeCombo.TabIndex = 28;
            // 
            // txFdCheck
            // 
            this.txFdCheck.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.txFdCheck.AutoSize = true;
            this.txFdCheck.Location = new System.Drawing.Point(222, 64);
            this.txFdCheck.Name = "txFdCheck";
            this.txFdCheck.Size = new System.Drawing.Size(36, 16);
            this.txFdCheck.TabIndex = 29;
            this.txFdCheck.Text = "FD";
            this.txFdCheck.UseVisualStyleBackColor = true;
            // 
            // txExtendedCheck
            // 
            this.txExtendedCheck.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.txExtendedCheck.AutoSize = true;
            this.txExtendedCheck.Location = new System.Drawing.Point(264, 64);
            this.txExtendedCheck.Name = "txExtendedCheck";
            this.txExtendedCheck.Size = new System.Drawing.Size(42, 16);
            this.txExtendedCheck.TabIndex = 30;
            this.txExtendedCheck.Text = "Ext";
            this.txExtendedCheck.UseVisualStyleBackColor = true;
            // 
            // label16
            // 
            this.label16.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label16.AutoSize = true;
            this.label16.Location = new System.Drawing.Point(312, 66);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(17, 12);
            this.label16.TabIndex = 5;
            this.label16.Text = "CH";
            // 
            // sendComboBox
            // 
            this.sendComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.sendComboBox.FormattingEnabled = true;
            this.sendComboBox.Location = new System.Drawing.Point(335, 61);
            this.sendComboBox.Name = "sendComboBox";
            this.sendComboBox.Size = new System.Drawing.Size(30, 20);
            this.sendComboBox.TabIndex = 40;
            // 
            // label11
            // 
            this.label11.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(371, 66);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(59, 12);
            this.label11.TabIndex = 31;
            this.label11.Text = "Cycle(ms)";
            // 
            // txCycleEdit
            // 
            this.txCycleEdit.Location = new System.Drawing.Point(436, 61);
            this.txCycleEdit.Name = "txCycleEdit";
            this.txCycleEdit.Size = new System.Drawing.Size(36, 21);
            this.txCycleEdit.TabIndex = 32;
            this.txCycleEdit.Text = "1000";
            // 
            // label12
            // 
            this.label12.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(478, 66);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(35, 12);
            this.label12.TabIndex = 33;
            this.label12.Text = "Count";
            // 
            // txCountEdit
            // 
            this.txCountEdit.Location = new System.Drawing.Point(519, 61);
            this.txCountEdit.Name = "txCountEdit";
            this.txCountEdit.Size = new System.Drawing.Size(16, 21);
            this.txCountEdit.TabIndex = 34;
            this.txCountEdit.Text = "1";
            // 
            // label13
            // 
            this.label13.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(541, 66);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(41, 12);
            this.label13.TabIndex = 35;
            this.label13.Text = "Length";
            // 
            // txLengthEdit
            // 
            this.txLengthEdit.Location = new System.Drawing.Point(588, 61);
            this.txLengthEdit.Name = "txLengthEdit";
            this.txLengthEdit.Size = new System.Drawing.Size(18, 21);
            this.txLengthEdit.TabIndex = 36;
            this.txLengthEdit.Text = "8";
            // 
            // label14
            // 
            this.label14.Anchor = System.Windows.Forms.AnchorStyles.Left;
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(612, 66);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(29, 12);
            this.label14.TabIndex = 37;
            this.label14.Text = "Data";
            // 
            // txDataEdit
            // 
            this.txDataEdit.Location = new System.Drawing.Point(647, 61);
            this.txDataEdit.Name = "txDataEdit";
            this.txDataEdit.Size = new System.Drawing.Size(186, 21);
            this.txDataEdit.TabIndex = 38;
            this.txDataEdit.Text = "01 23 45 67 89 AB CD EF";
            // 
            // txButton
            // 
            this.txButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.txButton.Enabled = false;
            this.flowLayoutPanel1.SetFlowBreak(this.txButton, true);
            this.txButton.Location = new System.Drawing.Point(839, 61);
            this.txButton.MaximumSize = new System.Drawing.Size(101, 30);
            this.txButton.Name = "txButton";
            this.txButton.Size = new System.Drawing.Size(101, 23);
            this.txButton.TabIndex = 39;
            this.txButton.Text = "Transmit";
            this.txButton.UseVisualStyleBackColor = true;
            this.txButton.Click += new System.EventHandler(this.txButton_Click);
            // 
            // msgListView
            // 
            this.msgListView.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.msgListView.Columns.AddRange(new System.Windows.Forms.ColumnHeader[] {
            this.columnHeader1,
            this.columnHeader2,
            this.columnHeader3,
            this.columnHeader4,
            this.columnHeader5,
            this.columnHeader6,
            this.columnHeader7});
            this.msgListView.HideSelection = false;
            this.msgListView.Location = new System.Drawing.Point(12, 12);
            this.msgListView.Name = "msgListView";
            this.msgListView.Size = new System.Drawing.Size(960, 415);
            this.msgListView.TabIndex = 0;
            this.msgListView.UseCompatibleStateImageBehavior = false;
            this.msgListView.View = System.Windows.Forms.View.Details;
            // 
            // columnHeader1
            // 
            this.columnHeader1.Text = "Time (s)";
            this.columnHeader1.Width = 80;
            // 
            // columnHeader2
            // 
            this.columnHeader2.Text = "TX/RX";
            // 
            // columnHeader3
            // 
            this.columnHeader3.Text = "Channel";
            this.columnHeader3.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader3.Width = 80;
            // 
            // columnHeader4
            // 
            this.columnHeader4.Text = "ID (HEX)";
            this.columnHeader4.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader4.Width = 80;
            // 
            // columnHeader5
            // 
            this.columnHeader5.Text = "Type";
            this.columnHeader5.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader5.Width = 80;
            // 
            // columnHeader6
            // 
            this.columnHeader6.Text = "Length (B)";
            this.columnHeader6.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader6.Width = 80;
            // 
            // columnHeader7
            // 
            this.columnHeader7.Text = "Payload";
            this.columnHeader7.Width = 490;
            // 
            // bMCanModeTypeDefBindingSource
            // 
            this.bMCanModeTypeDefBindingSource.DataSource = typeof(BM_CanModeTypeDef);
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(992, 552);
            this.Controls.Add(this.flowLayoutPanel1);
            this.Controls.Add(this.msgListView);
            this.DoubleBuffered = true;
            this.MaximizeBox = false;
            this.MaximumSize = new System.Drawing.Size(1008, 591);
            this.MinimumSize = new System.Drawing.Size(1008, 591);
            this.Name = "MainForm";
            this.Text = "BUSMUST CAN Analyzer";
            this.flowLayoutPanel1.ResumeLayout(false);
            this.flowLayoutPanel1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.samplePosSpin)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.bMCanModeTypeDefBindingSource)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private can_analyzer_csharp.DoubleBufferListView msgListView;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.ComboBox modeCombo;
        private System.Windows.Forms.CheckBox tresCheck;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox nominalBitrateEdit;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox dataBitrateEdit;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.NumericUpDown samplePosSpin;
        private System.Windows.Forms.Button openButton;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox rxIdEdit;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox rxMaskEdit;
        private System.Windows.Forms.BindingSource bMCanModeTypeDefBindingSource;
        private System.Windows.Forms.CheckBox rxExtendedCheck;
        private System.Windows.Forms.CheckBox rxAdvancedFilterCheck;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox rxDataMaskEdit;
        private System.Windows.Forms.Button enumerateButton;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox txIdEdit;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.ComboBox txTypeCombo;
        private System.Windows.Forms.CheckBox txFdCheck;
        private System.Windows.Forms.CheckBox txExtendedCheck;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox txCycleEdit;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox txCountEdit;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox txLengthEdit;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.TextBox txDataEdit;
        private System.Windows.Forms.Button txButton;
        private System.Windows.Forms.CheckBox rxStandardCheck;
        private System.Windows.Forms.ColumnHeader columnHeader1;
        private System.Windows.Forms.ColumnHeader columnHeader2;
        private System.Windows.Forms.ColumnHeader columnHeader3;
        private System.Windows.Forms.ColumnHeader columnHeader4;
        private System.Windows.Forms.ColumnHeader columnHeader5;
        private System.Windows.Forms.ColumnHeader columnHeader6;
        private System.Windows.Forms.ColumnHeader columnHeader7;
        private ComboBoxCustomer.ComCheckBoxList channelComboBox;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.ComboBox sendComboBox;
    }
}

