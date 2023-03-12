using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using System.Threading;

namespace PI_Motor_Speed_Control
{
    public partial class Form1 : Form
    {

        private byte[] rxbuffer = new byte[1000];
        private byte[] txbuffer = new byte[9];
        private bool isProcessing = false;
        private bool isNewData2Parse = false;
        private bool isWait4Ack = false;
        private bool isSetting1 = false;
        private bool isSetting2 = false;
        private byte settingIndex = 0;

        public Form1()
        {
            InitializeComponent();

            ConStatus.ForeColor = Color.Red;
            ConStatus.Value = 100;

            string[] Ports = SerialPort.GetPortNames();
            COM_List.Items.Clear();
            COM_List.Items.Add("<none>");
            COM_List.Items.AddRange(Ports);

            this.COM_List.DropDownStyle = ComboBoxStyle.DropDownList;

            checkBox2.Checked = false;

            cleanUI();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (!SerialPort.IsOpen)
            {
                try
                {
                    SerialPort.PortName = COM_List.Text;
                    SerialPort.BaudRate = 115200;
                    SerialPort.DataBits = 8;
                    SerialPort.StopBits = (StopBits)Enum.Parse(typeof(StopBits), "One");
                    SerialPort.Parity = (Parity)Enum.Parse(typeof(Parity), "None");

                    SerialPort.Open();
                }
                catch (Exception err)
                {
                    MessageBox.Show(err.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }

                if (SerialPort.IsOpen)
                {
                    timer1.Start();
                    ConStatus.ForeColor = Color.Lime;
                    Con_button.Text = "DISCONNECT";

                    Array.Clear(txbuffer, 0, txbuffer.Length);

                    txbuffer[0] = 0xAB;
                    txbuffer[1] = 0b01001011;
                    txbuffer[2] = 0x00;

                    UInt16 check = CheckSumCrc16(txbuffer, 6);
                    txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                    txbuffer[7] = (byte)(check & 0x00FF);
                    txbuffer[8] = 0xBA;

                    while (SerialPort.BytesToRead > 0)
                    {
                        byte[] t = new byte[1];
                        SerialPort.Read(t, 0, 1);
                    }

                    isWait4Ack = true;
                    timer2.Start();

                }
                else
                {
                    timer1.Stop();
                    ConStatus.ForeColor = Color.Red;
                    Con_button.Text = "CONNECT";
                }
            }
            else
            {
                Array.Clear(txbuffer, 0, txbuffer.Length);

                txbuffer[0] = 0xAB;
                txbuffer[1] = 0b01001100;
                UInt16 check = CheckSumCrc16(txbuffer, 6);
                txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                txbuffer[7] = (byte)(check & 0x00FF);
                txbuffer[8] = 0xBA;

                if (SerialPort.IsOpen)
                    SerialPort.Write(txbuffer, 0, 9);

                timer1.Stop();
                SerialPort.Close();
                ConStatus.ForeColor = Color.Red;
                Con_button.Text = "CONNECT";

                cleanUI();
            }
        }

        private void toolStripComboBox1_Click(object sender, EventArgs e)
        {
            string[] Ports = SerialPort.GetPortNames();
            COM_List.Items.Clear();
            COM_List.Items.Add("<none>");
            COM_List.Items.AddRange(Ports);
        }

        private void Serial_Port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            
        }

        private void comboBox1_DrawItem(object sender, DrawItemEventArgs e)
        {
            // By using Sender, one method could handle multiple ComboBoxes
            ComboBox cbx = sender as ComboBox;
            if (cbx != null)
            {
                // Always draw the background
                e.DrawBackground();

                // Drawing one of the items?
                if (e.Index >= 0)
                {
                    // Set the string alignment.  Choices are Center, Near and Far
                    StringFormat sf = new StringFormat();
                    sf.LineAlignment = StringAlignment.Center;
                    sf.Alignment = StringAlignment.Center;

                    // Set the Brush to ComboBox ForeColor to maintain any ComboBox color settings
                    // Assumes Brush is solid
                    Brush brush = new SolidBrush(cbx.ForeColor);

                    // If drawing highlighted selection, change brush
                    if ((e.State & DrawItemState.Selected) == DrawItemState.Selected)
                        brush = SystemBrushes.HighlightText;

                    // Draw the string
                    e.Graphics.DrawString(cbx.Items[e.Index].ToString(), cbx.Font, brush, e.Bounds, sf);
                }
            }
        }

        private void getSerialData()
        {
            byte ID = (byte)((rxbuffer[1] & (0x01 << Constants.ID_offset)) >> Constants.ID_offset);
            byte RW = (byte)(rxbuffer[1] & (0x01 << Constants.RW_offset));
            byte codec = (byte)(rxbuffer[1] & (byte)0x0F);

            switch (codec)
            {
                case Constants.Cur_codec:
                    if (ID == 0)
                    {
                        current1.Text = BitConverter.ToInt32(rxbuffer, 2).ToString();
                        current1.SelectAll();
                        current1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        current2.Text = BitConverter.ToInt32(rxbuffer, 2).ToString();
                        current2.SelectAll();
                        current2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.Spd_codec:
                    if (ID == 0)
                    {
                        speed1.Text = BitConverter.ToInt32(rxbuffer, 2).ToString();
                        speed1.SelectAll();
                        speed1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        speed2.Text = BitConverter.ToInt32(rxbuffer, 2).ToString();
                        speed2.SelectAll();
                        speed2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.Dut_codec:
                    if (ID == 0)
                    {
                        duty1.Text = (BitConverter.ToSingle(rxbuffer, 2) * 100).ToString("0.0");
                        duty1.SelectAll();
                        duty1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        duty2.Text = BitConverter.ToSingle(rxbuffer, 2).ToString("0.0");
                        duty2.SelectAll();
                        duty2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.Vcc_codec:
                    if (ID == 0)
                    {
                        vcc1.Text = BitConverter.ToUInt32(rxbuffer, 2).ToString();
                        vcc1.SelectAll();
                        vcc1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        vcc2.Text = BitConverter.ToUInt32(rxbuffer, 2).ToString();
                        vcc2.SelectAll();
                        vcc2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.Ena_codec:
                    if (ID == 0)
                    {
                        if (rxbuffer[2] == 0)
                            ena1.ForeColor = Color.Red;
                        else
                            ena1.ForeColor = Color.Lime;
                    }
                    else
                    {
                        if (rxbuffer[2] == 0)
                            ena2.ForeColor = Color.Red;
                        else
                            ena2.ForeColor = Color.Lime;
                    }
                    break;
                case Constants.Ack_codec:
                    if (isWait4Ack == true)
                    {
                        isWait4Ack = false;
                        if (isSetting1 == true)
                        {
                            settingIndex++;
                            progressBar1.Value = (int)(settingIndex * 100 / 8);
                            if (settingIndex == 8)
                            {
                                timer2.Stop();
                                isSetting1 = false;
                                settingIndex = 0;
                                setting1.Enabled = true;
                            }
                        }
                        else if (isSetting2 == true)
                        {
                            settingIndex++;
                            progressBar2.Value = (int)(settingIndex * 100 / 8);
                            if (settingIndex == 8)
                            {
                                timer2.Stop();
                                isSetting2 = false;
                                settingIndex = 0;
                                setting2.Enabled = true;
                            }
                        }
                            
                    }
                    break;
                case Constants.Rty_codec:
                    isWait4Ack = true;
                    SerialPort.Write(txbuffer, 0, 9);
                    break;
                case Constants.KPR_codec:
                    if (ID == 0)
                    {
                        kpo1.Text = BitConverter.ToSingle(rxbuffer, 2).ToString("0.00");
                        kpo1.SelectAll();
                        kpo1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        kpo2.Text = BitConverter.ToSingle(rxbuffer, 2).ToString("0.00");
                        kpo2.SelectAll();
                        kpo2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.KIR_codec:
                    if (ID == 0)
                    {
                        kio1.Text = BitConverter.ToSingle(rxbuffer, 2).ToString("0.00");
                        kio1.SelectAll();
                        kio1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        kio2.Text = BitConverter.ToSingle(rxbuffer, 2).ToString("0.00");
                        kio2.SelectAll();
                        kio2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.KPC_codec:
                    if (ID == 0)
                    {
                        kpi1.Text = BitConverter.ToSingle(rxbuffer, 2).ToString("0.00");
                        kpi1.SelectAll();
                        kpi1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        kpi2.Text = BitConverter.ToSingle(rxbuffer, 2).ToString("0.00");
                        kpi2.SelectAll();
                        kpi2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.KIC_codec:
                    if (ID == 0)
                    {
                        kii1.Text = BitConverter.ToSingle(rxbuffer, 2).ToString("0.00");
                        kii1.SelectAll();
                        kii1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        kii2.Text = BitConverter.ToSingle(rxbuffer, 2).ToString("0.00");
                        kii2.SelectAll();
                        kii2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.REF_codec:
                    if (ID == 0)
                    {
                        ref1.Text = BitConverter.ToInt32(rxbuffer, 2).ToString();
                        ref1.SelectAll();
                        ref1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        ref2.Text = BitConverter.ToInt32(rxbuffer, 2).ToString();
                        ref2.SelectAll();
                        ref2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.TS_codec:
                    if (ID == 0)
                    {
                        ts1.Text = BitConverter.ToUInt32(rxbuffer, 2).ToString();
                        ts1.SelectAll();
                        ts1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        ts2.Text = BitConverter.ToUInt32(rxbuffer, 2).ToString();
                        ts2.SelectAll();
                        ts2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.mVpA_codec:
                    if (ID == 0)
                    {
                        convmVpA1.Text = BitConverter.ToUInt32(rxbuffer, 2).ToString();
                        convmVpA1.SelectAll();
                        convmVpA1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        convmVpA2.Text = BitConverter.ToUInt32(rxbuffer, 2).ToString();
                        convmVpA2.SelectAll();
                        convmVpA2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                case Constants.tpR_codec:
                    if (ID == 0)
                    {
                        convTpR1.Text = BitConverter.ToUInt32(rxbuffer, 2).ToString();
                        convTpR1.SelectAll();
                        convTpR1.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    else
                    {
                        convTpR2.Text = BitConverter.ToUInt32(rxbuffer, 2).ToString();
                        convTpR2.SelectAll();
                        convTpR2.SelectionAlignment = HorizontalAlignment.Center;
                    }
                    break;
                default:
                    break;
            }

        }

        private UInt16 CheckSumCrc16(byte[] ucBufferTemp, int ucLength)
        {
            UInt32 CRCFull = 0xFFFF;
            char CRCLSB;
            for (byte i = 0; i < ucLength; i++)
            {
                CRCFull = (UInt16)(CRCFull ^ ucBufferTemp[i]);
                for (int j = 0; j < 8; j++)
                {
                    CRCLSB = (char)(CRCFull & 0x0001);
                    CRCFull = (UInt16)((CRCFull >> 1) & 0x7FFFF);

                    if (CRCLSB == 1)
                    {
                        CRCFull = (UInt16)(CRCFull ^ 0xA001);
                    }
                }
            }
            return (UInt16)CRCFull;
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (SerialPort.IsOpen == true)
            {
                //do nothing, this is OK
            }
            else
            {
                timer1.Stop();
                MessageBox.Show("COM DISCONNECTED", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                timer1.Stop();
                SerialPort.Close();
                ConStatus.ForeColor = Color.Red;
                Con_button.Text = "CONNECT";
                cleanUI();
                return;
            }

            if ((isProcessing == false) && (isNewData2Parse == false) && (SerialPort.BytesToRead > 0))
            {
                SerialPort.Read(rxbuffer, 0, 1);
                if (rxbuffer[0] == 0xAB)
                {
                    isProcessing = true;
                    Array.Clear(rxbuffer, 0, rxbuffer.Length);
                    rxbuffer[0] = 0xAB;
                }
                else
                {   
                    while (true)
                    {
                        if (LogTextBox.Text.Length >= 10000)
                            LogTextBox.Clear();
                        LogTextBox.AppendText(((char)rxbuffer[0]).ToString());
                        if (SerialPort.BytesToRead > 0)
                        {
                            SerialPort.Read(rxbuffer, 0, 1);
                        }
                        else if (SerialPort.BytesToRead == 0)
                        {
                            break;
                        }
                        if (rxbuffer[0] == '\n')
                        {
                            LogTextBox.AppendText(((char)rxbuffer[0]).ToString());
                            break;
                        }
                    }
                    
                    //LogTextBox.Text += ((char)rxbuffer[0]).ToString();
                    rxbuffer[0] = 0x00;
                }
            }
            else if ((isProcessing == true) && (isNewData2Parse == false))
            {
                if (SerialPort.BytesToRead >= 8)
                {
                    SerialPort.Read(rxbuffer, 1, 8);
                    UInt16 check = CheckSumCrc16(rxbuffer, 6);
                    if ((rxbuffer[0] != 0xAB) || (rxbuffer[8] != 0xBA))
                    {
                        //error no end frame - no need request resent
                        isProcessing = false;
                    }
                    else if ((rxbuffer[6] != (byte)((check & 0xFF00) >> 8)) || (rxbuffer[7] != (byte)(check & 0x00FF)))
                    {
                        //error in CRC - need request resent (for later)
                        //rxbuffer[1] |= (0x01 >> 5);
                        //SerialPort.Write(rxbuffer, 0, 9);
                        isProcessing = false;
                    }
                    else
                    {
                        //no error - frame ok - data ok - begin to parse
                        isNewData2Parse = true;
                    }
                }
            }
            
            if (isNewData2Parse == true)
            {
                getSerialData();
                isNewData2Parse = false;
                isProcessing = false;
            }
        }

        private void exit_Click(object sender, EventArgs e)
        {
            //timer1.Stop();
            this.Close();
        }

        private void LogTextBox_TextChanged(object sender, EventArgs e)
        {
            // set the current caret position to the end
            //LogTextBox.SelectionStart = LogTextBox.Text.Length;
            // scroll it automatically
            //LogTextBox.ScrollToCaret();
        }

        private void checkBox2_CheckedChanged(object sender, EventArgs e)
        {
            Array.Clear(txbuffer, 0, txbuffer.Length);

            txbuffer[0] = 0xAB;
            txbuffer[1] = 0b01001101;

            if (checkBox2.Checked == true)
                txbuffer[2] = 0x01;
            else
                txbuffer[2] = 0x00;

            UInt16 check = CheckSumCrc16(txbuffer, 6);
            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
            txbuffer[7] = (byte)(check & 0x00FF);
            txbuffer[8] = 0xBA;

            if (SerialPort.IsOpen)
            {
                isWait4Ack = true;
                SerialPort.Write(txbuffer, 0, 9);
            }
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            if (ena1.ForeColor == Color.Lime || ena2.ForeColor == Color.Lime)
            {
                MessageBox.Show("Turn off both motors to update setting", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (!SerialPort.IsOpen)
            {
                MessageBox.Show("Connect to device to update setting", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            isSetting1 = true;
            isSetting2 = false;
            progressBar1.Value = 0;
            settingIndex = 0;
            timer2.Start();
            setting1.Enabled = false;

        }

        private void setting2_Click(object sender, EventArgs e)
        {
            if (ena1.ForeColor == Color.Lime || ena2.ForeColor == Color.Lime)
            {
                MessageBox.Show("Turn off both motors to update setting", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (!SerialPort.IsOpen)
            {
                MessageBox.Show("Connect to device to update setting", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            isSetting2 = true;
            isSetting1 = false;
            progressBar2.Value = 0;
            settingIndex = 0;
            timer2.Start();
            setting2.Enabled = false;
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            timer1.Stop();
            Array.Clear(txbuffer, 0, txbuffer.Length);

            txbuffer[0] = 0xAB;
            txbuffer[1] = 0b01001100;
            UInt16 check = CheckSumCrc16(txbuffer, 6);
            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
            txbuffer[7] = (byte)(check & 0x00FF);
            txbuffer[8] = 0xBA;

            if (SerialPort.IsOpen)
                SerialPort.Write(txbuffer, 0, 9);

            
            if (SerialPort.IsOpen) SerialPort.Close();
        }

        private void timer2_Tick(object sender, EventArgs e)
        {
            if ((isSetting1 == false) && (isSetting2 == false))
            {
                if (SerialPort.IsOpen && (isWait4Ack == true))
                {
                    SerialPort.Write(txbuffer, 0, 9);
                }
                else if (isWait4Ack == false)
                {
                    timer2.Stop();
                }
            }
            else if ((isSetting1 == true) && (isSetting2 == false))
            {
                if (settingIndex == 0)
                {
                    if (isWait4Ack == false)
                    {
                        float KpOut = 0;
                        try
                        {
                            KpOut = (float)Convert.ToSingle(kpo1.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Kpo)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (KpOut < 0)
                        {
                            MessageBox.Show("Invalid value of K (K > 0)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(KpOut);
                            
                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b01000000;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 1)
                {
                    if (isWait4Ack == false)
                    {
                        float KiOut = 0;
                        try
                        {
                            KiOut = (float)Convert.ToSingle(kio1.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Kio)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (KiOut < 0)
                        {
                            MessageBox.Show("Invalid value of K (K > 0)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(KiOut);
                            
                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b01000001;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 2)
                {
                    if (isWait4Ack == false)
                    {
                        float KpIn = 0;
                        try
                        {
                            KpIn = (float)Convert.ToSingle(kpi1.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Kpi)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (KpIn < 0)
                        {
                            MessageBox.Show("Invalid value of K (K > 0)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(KpIn);

                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b01000010;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 3)
                {
                    if (isWait4Ack == false)
                    {
                        float KiIn = 0;
                        try
                        {
                            KiIn = (float)Convert.ToSingle(kii1.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Kii)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (KiIn < 0)
                        {
                            MessageBox.Show("Invalid value of K (K > 0)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(KiIn);
                            
                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b01000011;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 4)
                {
                    if (isWait4Ack == false)
                    {
                        Int32 RefRPM = 0;
                        try
                        {
                            RefRPM = (Int32)Convert.ToInt32(ref1.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(REF)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        byte[] temp = new byte[4];
                        temp = BitConverter.GetBytes(RefRPM);
                        
                        Array.Clear(txbuffer, 0, txbuffer.Length);

                        txbuffer[0] = 0xAB;
                        txbuffer[1] = 0b01000100;
                        txbuffer[2] = temp[0];
                        txbuffer[3] = temp[1];
                        txbuffer[4] = temp[2];
                        txbuffer[5] = temp[3];
                        UInt16 check = CheckSumCrc16(txbuffer, 6);
                        txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                        txbuffer[7] = (byte)(check & 0x00FF);
                        txbuffer[8] = 0xBA;

                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 5)
                {
                    if (isWait4Ack == false)
                    {
                        UInt32 Ts = 0;
                        try
                        {
                            Ts = (UInt32)Convert.ToUInt32(ts1.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Ts)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (Ts < 50)
                        {
                            MessageBox.Show("Invalid value of sampling time (Ts >= 100)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(Ts);
                            
                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b01000101;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 6)
                {
                    if (isWait4Ack == false)
                    {
                        byte convmVpA = 0;
                        try
                        {
                            convmVpA = (byte)Convert.ToByte(convmVpA1.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(mVpA)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        byte[] temp = new byte[1];
                        temp = BitConverter.GetBytes(convmVpA);
                        
                        Array.Clear(txbuffer, 0, txbuffer.Length);

                        txbuffer[0] = 0xAB;
                        txbuffer[1] = 0b01000110;
                        txbuffer[2] = temp[0];
                        txbuffer[3] = 0x00;
                        txbuffer[4] = 0x00;
                        txbuffer[5] = 0x00;
                        UInt16 check = CheckSumCrc16(txbuffer, 6);
                        txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                        txbuffer[7] = (byte)(check & 0x00FF);
                        txbuffer[8] = 0xBA;

                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 7)
                {
                    if (isWait4Ack == false)
                    {
                        UInt16 convTpR = 0;
                        try
                        {
                            convTpR = (UInt16)Convert.ToUInt16(convTpR1.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(TpR)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        byte[] temp = new byte[2];
                        temp = BitConverter.GetBytes(convTpR);
                        
                        Array.Clear(txbuffer, 0, txbuffer.Length);

                        txbuffer[0] = 0xAB;
                        txbuffer[1] = 0b01000111;
                        txbuffer[2] = temp[0];
                        txbuffer[3] = temp[1];
                        txbuffer[4] = 0x00;
                        txbuffer[5] = 0x00;
                        UInt16 check = CheckSumCrc16(txbuffer, 6);
                        txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                        txbuffer[7] = (byte)(check & 0x00FF);
                        txbuffer[8] = 0xBA;

                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
            }
            else if ((isSetting1 == false) && (isSetting2 == true))
            {
                if (settingIndex == 0)
                {
                    if (isWait4Ack == false)
                    {
                        float KpOut = 0;
                        try
                        {
                            KpOut = (float)Convert.ToSingle(kpo2.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Kpo)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (KpOut < 0)
                        {
                            MessageBox.Show("Invalid value of K (K > 0)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(KpOut);
                            
                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b11000000;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 1)
                {
                    if (isWait4Ack == false)
                    {
                        float KiOut = 0;
                        try
                        {
                            KiOut = (float)Convert.ToSingle(kio2.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Kio)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (KiOut < 0)
                        {
                            MessageBox.Show("Invalid value of K (K > 0)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(KiOut);
                            
                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b11000001;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 2)
                {
                    if (isWait4Ack == false)
                    {
                        float KpIn = 0;
                        try
                        {
                            KpIn = (float)Convert.ToSingle(kpi2.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Kpi)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (KpIn < 0)
                        {
                            MessageBox.Show("Invalid value of K (K > 0)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(KpIn);
                            
                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b11000010;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 3)
                {
                    if (isWait4Ack == false)
                    {
                        float KiIn = 0;
                        try
                        {
                            KiIn = (float)Convert.ToSingle(kii2.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Kii)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (KiIn < 0)
                        {
                            MessageBox.Show("Invalid value of K (K > 0)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(KiIn);
                            
                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b11000011;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 4)
                {
                    if (isWait4Ack == false)
                    {
                        Int32 RefRPM = 0;
                        try
                        {
                            RefRPM = (Int32)Convert.ToInt32(ref2.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Ref)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        byte[] temp = new byte[4];
                        temp = BitConverter.GetBytes(RefRPM);
                        
                        Array.Clear(txbuffer, 0, txbuffer.Length);

                        txbuffer[0] = 0xAB;
                        txbuffer[1] = 0b11000100;
                        txbuffer[2] = temp[0];
                        txbuffer[3] = temp[1];
                        txbuffer[4] = temp[2];
                        txbuffer[5] = temp[3];
                        UInt16 check = CheckSumCrc16(txbuffer, 6);
                        txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                        txbuffer[7] = (byte)(check & 0x00FF);
                        txbuffer[8] = 0xBA;

                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 5)
                {
                    if (isWait4Ack == false)
                    {
                        UInt32 Ts = 0;
                        try
                        {
                            Ts = (UInt32)Convert.ToUInt32(ts2.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(Ts)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        if (Ts < 50)
                        {
                            MessageBox.Show("Invalid value of sampling time (Ts >= 100)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                            timer2.Stop();
                            isSetting1 = false;
                        }
                        else
                        {
                            byte[] temp = new byte[4];
                            temp = BitConverter.GetBytes(Ts);
                            
                            Array.Clear(txbuffer, 0, txbuffer.Length);

                            txbuffer[0] = 0xAB;
                            txbuffer[1] = 0b11000101;
                            txbuffer[2] = temp[0];
                            txbuffer[3] = temp[1];
                            txbuffer[4] = temp[2];
                            txbuffer[5] = temp[3];
                            UInt16 check = CheckSumCrc16(txbuffer, 6);
                            txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                            txbuffer[7] = (byte)(check & 0x00FF);
                            txbuffer[8] = 0xBA;

                            isWait4Ack = true;
                            SerialPort.Write(txbuffer, 0, 9);
                        }
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 6)
                {
                    if (isWait4Ack == false)
                    {
                        byte convmVpA = 0;
                        try
                        {
                            convmVpA = (byte)Convert.ToByte(convmVpA2.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(mVpA)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        byte[] temp = new byte[4];
                        temp = BitConverter.GetBytes(convmVpA);
                        
                        Array.Clear(txbuffer, 0, txbuffer.Length);

                        txbuffer[0] = 0xAB;
                        txbuffer[1] = 0b11000110;
                        txbuffer[2] = temp[0];
                        txbuffer[3] = 0;
                        txbuffer[4] = 0;
                        txbuffer[5] = 0;
                        UInt16 check = CheckSumCrc16(txbuffer, 6);
                        txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                        txbuffer[7] = (byte)(check & 0x00FF);
                        txbuffer[8] = 0xBA;

                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
                else if (settingIndex == 7)
                {
                    if (isWait4Ack == false)
                    {
                        UInt16 convTpR = 0;
                        try
                        {
                            convTpR = (UInt16)Convert.ToUInt16(convTpR2.Text);
                        }
                        catch (Exception err)
                        {
                            MessageBox.Show(err.Message + "(TpR)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }

                        byte[] temp = new byte[4];
                        temp = BitConverter.GetBytes(convTpR);
                        
                        Array.Clear(txbuffer, 0, txbuffer.Length);

                        txbuffer[0] = 0xAB;
                        txbuffer[1] = 0b11000111;
                        txbuffer[2] = temp[0];
                        txbuffer[3] = temp[1];
                        txbuffer[4] = 0;
                        txbuffer[5] = 0;
                        UInt16 check = CheckSumCrc16(txbuffer, 6);
                        txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                        txbuffer[7] = (byte)(check & 0x00FF);
                        txbuffer[8] = 0xBA;

                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                    else
                    {
                        isWait4Ack = true;
                        SerialPort.Write(txbuffer, 0, 9);
                    }
                }
            }
        }

        private void cleanUI()
        {
            kpo1.Text = "0.0";
            kio1.Text = "0.0";
            kpi1.Text = "0.0";
            kii1.Text = "0.0";
            ref1.Text = "0";
            ts1.Text = "0";
            convmVpA1.Text = "0";
            convTpR1.Text = "0";
            progressBar1.Value = 0;
            progressBar1.ForeColor = Color.Lime;


            kpo2.Text = "0.0";
            kio2.Text = "0.0";
            kpi2.Text = "0.0";
            kii2.Text = "0.0";
            ref2.Text = "0";
            ts2.Text = "0";
            convmVpA2.Text = "0";
            convTpR2.Text = "0";
            progressBar2.Value = 0;
            progressBar2.ForeColor = Color.Lime;

            current1.Text = "0";
            speed1.Text = "0";
            duty1.Text = "0.0";
            vcc1.Text = "0";
            ena1.ForeColor = Color.Red;

            current2.Text = "0";
            speed2.Text = "0";
            duty2.Text = "0.0";
            vcc2.Text = "0";
            ena2.ForeColor = Color.Red;

            current1.SelectAll();
            current1.SelectionAlignment = HorizontalAlignment.Center;
            speed1.SelectAll();
            speed1.SelectionAlignment = HorizontalAlignment.Center;
            duty1.SelectAll();
            duty1.SelectionAlignment = HorizontalAlignment.Center;
            vcc1.SelectAll();
            vcc1.SelectionAlignment = HorizontalAlignment.Center;
            ena1.ForeColor = Color.Red;
            ena1.Value = 100;

            current2.SelectAll();
            current2.SelectionAlignment = HorizontalAlignment.Center;
            speed2.SelectAll();
            speed2.SelectionAlignment = HorizontalAlignment.Center;
            duty2.SelectAll();
            duty2.SelectionAlignment = HorizontalAlignment.Center;
            vcc2.SelectAll();
            vcc2.SelectionAlignment = HorizontalAlignment.Center;
            ena2.ForeColor = Color.Red;
            ena2.Value = 100;

            kpo1.SelectAll();
            kpo1.SelectionAlignment = HorizontalAlignment.Center;
            kpi1.SelectAll();
            kpi1.SelectionAlignment = HorizontalAlignment.Center;
            kio1.SelectAll();
            kio1.SelectionAlignment = HorizontalAlignment.Center;
            kii1.SelectAll();
            kii1.SelectionAlignment = HorizontalAlignment.Center;

            ref1.SelectAll();
            ref1.SelectionAlignment = HorizontalAlignment.Center;
            ts1.SelectAll();
            ts1.SelectionAlignment = HorizontalAlignment.Center;
            convTpR1.SelectAll();
            convTpR1.SelectionAlignment = HorizontalAlignment.Center;
            convmVpA1.SelectAll();
            convmVpA1.SelectionAlignment = HorizontalAlignment.Center;

            kpo2.SelectAll();
            kpo2.SelectionAlignment = HorizontalAlignment.Center;
            kpi2.SelectAll();
            kpi2.SelectionAlignment = HorizontalAlignment.Center;
            kio2.SelectAll();
            kio2.SelectionAlignment = HorizontalAlignment.Center;
            kii2.SelectAll();
            kii2.SelectionAlignment = HorizontalAlignment.Center;

            ref2.SelectAll();
            ref2.SelectionAlignment = HorizontalAlignment.Center;
            ts2.SelectAll();
            ts2.SelectionAlignment = HorizontalAlignment.Center;
            convTpR2.SelectAll();
            convTpR2.SelectionAlignment = HorizontalAlignment.Center;
            convmVpA2.SelectAll();
            convmVpA2.SelectionAlignment = HorizontalAlignment.Center;

            checkBox2.Checked = false;
            LogTextBox.Clear();
        }

        private void ena1_Click(object sender, EventArgs e)
        {  
            if (ena1.ForeColor == Color.Red)
            {
                txbuffer[0] = 0xAB;
                txbuffer[1] = 0b01001000;
                txbuffer[2] = 0x01;
                UInt16 check = CheckSumCrc16(txbuffer, 6);
                txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                txbuffer[7] = (byte)(check & 0x00FF);
                txbuffer[8] = 0xBA;
                isWait4Ack = true;
                timer2.Start();
            }
            else
            {
                txbuffer[0] = 0xAB;
                txbuffer[1] = 0b01001000;
                txbuffer[2] = 0x00;
                UInt16 check = CheckSumCrc16(txbuffer, 6);
                txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                txbuffer[7] = (byte)(check & 0x00FF);
                txbuffer[8] = 0xBA;
                isWait4Ack = true;
                timer2.Start();
            }
        }

        private void ena2_Click(object sender, EventArgs e)
        {
            if (ena2.ForeColor == Color.Red)
            {
                txbuffer[0] = 0xAB;
                txbuffer[1] = 0b11001000;
                txbuffer[2] = 0x01;
                UInt16 check = CheckSumCrc16(txbuffer, 6);
                txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                txbuffer[7] = (byte)(check & 0x00FF);
                txbuffer[8] = 0xBA;
                isWait4Ack = true;
                timer2.Start();
            }
            else
            {
                txbuffer[0] = 0xAB;
                txbuffer[1] = 0b11001000;
                txbuffer[2] = 0x00;
                UInt16 check = CheckSumCrc16(txbuffer, 6);
                txbuffer[6] = (byte)((check & 0xFF00) >> 8);
                txbuffer[7] = (byte)(check & 0x00FF);
                txbuffer[8] = 0xBA;
                isWait4Ack = true;
                timer2.Start();
            }
        }
    }
}







