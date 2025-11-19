using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ControlEx
{
    #region 简单的多选下拉框
    public class MultiComboBox : UserControl
    {
        #region 属性
        public ComboBox ComboBox { get; set; }
        public CheckedListBox CheckedListBox { get; set; }
        public ComboBox.ObjectCollection Items
        {
            get
            {
                return ComboBox?.Items;
            }
        }
        #endregion

        /// <summary>
        /// 多选下拉框
        /// </summary>
        public MultiComboBox()
        {
            // 面板
            this.VerticalScroll.Enabled = true;
            this.AutoSize = true;
            int SelectedIndex;
            // 多选列表
            CheckedListBox = new CheckedListBox();
            CheckedListBox.CheckOnClick = true;
            CheckedListBox.BorderStyle = BorderStyle.Fixed3D;
            CheckedListBox.Visible = false;
            CheckedListBox.Margin = new Padding(0);
            CheckedListBox.MouseUp += (ss, se) =>
            {
                // 更新ComboBox显示文本
                var lst = new List<string>();
                foreach (var v in CheckedListBox.CheckedItems)
                {
                    lst.Add(v.ToString());
                }
                ComboBox.Text = string.Join(",", lst);
                ComboBox.Tag = lst;
            };
            CheckedListBox.MouseLeave += (ss, se) =>
            {
                // 隐藏下拉多选框
                CheckedListBox.Hide();
            };

            // 下拉框
            ComboBox = new ComboBox();
            ComboBox.Width = 150;
            ComboBox.DrawMode = DrawMode.OwnerDrawFixed;
            ComboBox.IntegralHeight = false;
            ComboBox.DroppedDown = false;
            ComboBox.DropDownHeight = 1;
            ComboBox.Margin = new Padding(0);
            ComboBox.Location = new Point(0, 0);
            ComboBox.DropDownStyle = ComboBoxStyle.DropDown;
            ComboBox.AutoCompleteSource = AutoCompleteSource.ListItems;
            ComboBox.MouseDown += (ss, se) =>
            {
                ComboBox.DroppedDown = false;
            };
            ComboBox.MouseLeave += (ss, se) =>
            {
                // 不在下拉区则隐藏下拉
                var curMousePos = this.PointToClient(Control.MousePosition);
                var downArea = CheckedListBox.Location;
                if (curMousePos.X < downArea.X || curMousePos.X > (downArea.X + CheckedListBox.Width)
                || curMousePos.Y < downArea.Y || curMousePos.Y > (downArea.Y + CheckedListBox.Height))
                {
                    CheckedListBox.Hide();
                }
            };
            ComboBox.DropDown += (ss, se) =>
            {
                // 显示下拉多选框
                CheckedListBox.Items.Clear();

                // 添加并设置选中项
                var lastChecked = ComboBox.Tag as List<string>;
                ComboBox.BeginUpdate();
                foreach (var v in this.Items)
                {
                    var ck = false;
                    if (lastChecked != null && lastChecked.IndexOf(v.ToString()) >= 0)
                    {
                        ck = true;
                    }
                    CheckedListBox.Items.Add(v, ck);
                }
                // 显示下拉框
                CheckedListBox.Width = ComboBox.Width;
                CheckedListBox.ItemHeight = ComboBox.ItemHeight;
                CheckedListBox.Size = new Size(ComboBox.DropDownWidth, this.Items.Count * 18);
                CheckedListBox.Location = new Point(ComboBox.Left, ComboBox.Height);
                this.Controls.Add(CheckedListBox);
                CheckedListBox.Visible = true;
                ComboBox.EndUpdate();
            };

            // 添加控件
            this.Controls.Add(ComboBox);
        }

        private void InitializeComponent()
        {
            this.SuspendLayout();
            // 
            // MultiComboBox
            // 
            this.Name = "MultiComboBox";
            this.Load += new System.EventHandler(this.MultiComboBox_Load);
            this.ResumeLayout(false);

        }

        private void MultiComboBox_Load(object sender, EventArgs e)
        {

        }
    }
    #endregion
}
