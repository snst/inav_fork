using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace NavControlWin
{

    public partial class Form1 : Form
    {
        System.IO.Ports.SerialPort m_serialPort = null;
        Random rnd = new Random();
        double timeStamp = 0;
        MSP m_msp = new MSP();
        Logger m_log = new Logger();
        int baroOffset = 0;

        BackgroundWorker m_serialReceiver = new BackgroundWorker();
        

        public Form1()
        {
            InitializeComponent();
            setStatus("ready");
            enumerateSerialPorts();
            updateSerialPortStatus();
            chartPE.ChartAreas[0].AxisX.ScaleView.Size = 400;
            //         chartPE.ChartAreas[0].AxisX.ScrollBar.Enabled = false;

            initLines();

            UsbNotification.RegisterUsbDeviceNotification(this.Handle);

            if (checkAutoConnect.Checked)
            {
                connectSelectedSerialPort();
                updateSerialPortStatus();
            }

         }

        protected override void WndProc(ref Message m)
        {
            base.WndProc(ref m);
            if (m.Msg == UsbNotification.WmDevicechange)
            {
                switch ((int)m.WParam)
                {
                    case UsbNotification.DbtDeviceremovecomplete:
                        Usb_DeviceRemoved(); // this is where you do your magic
                        break;
                    case UsbNotification.DbtDevicearrival:
                        Usb_DeviceAdded(); // this is where you do your magic
                        break;
                }
            }
        }

        public void Usb_DeviceAdded()
        {
            enumerateSerialPorts();
            //if(checkAutoConnect.Checked)
            //{
            //    connectSelectedSerialPort();
            //    updateSerialPortStatus();
            //}
        }

        public void Usb_DeviceRemoved()
        {
            enumerateSerialPorts();
            closeSerialPort();
            updateSerialPortStatus();
        }

        private void scaleChart(System.Windows.Forms.DataVisualization.Charting.Chart chart)
        {
            if (chart.ChartAreas[0].AxisX.Maximum > chart.ChartAreas[0].AxisX.ScaleView.Size)
            {
                chart.ChartAreas[0].AxisX.ScaleView.Scroll(chart.ChartAreas[0].AxisX.Maximum);
            }
        }

        private void addFloatVectorToChart(System.Windows.Forms.DataVisualization.Charting.Chart chart, float[] data, double ts)
        {
            scaleChart(chart);

            chart.Series[0].Points.AddXY(ts, data[0]);
            chart.Series[1].Points.AddXY(ts, data[1]);
            chart.Series[2].Points.AddXY(ts, data[2]);
        }

        double fA = 0.0f;

        double clamp(double oVal, double nVal, double d)
        {
            if (nVal > oVal + d) return nVal - d;
            else if (nVal < oVal - d) return nVal + d;
            else return oVal;
        }


        DateTime lastTime = DateTime.Now;
        DateTime startTime = DateTime.Now;

        double ts = 0;

        private void timer1_Tick(object sender, EventArgs e)
        {
           // sendCmd(MSP.msp.MSP_SONAR_ALTITUDE);
            sendCmd(MSP.msp.MSP_STSC_SONAR);
            sendCmd(MSP.msp.MSP_STSC_BARO);
//            sendCmd(MSP.msp.MSP_STSC_POS_EST);

            scaleChart(chartPE);

            ts = ((long) ((DateTime.Now - startTime).TotalMilliseconds)) / 20;




            DateTime now = DateTime.Now;
            double dt = (now - lastTime).Milliseconds / 1000.0f;
            lastTime = now;

            float scaleVel = 1;

            
            this.chartPE.Series[0].Points.AddXY(ts, m_msp.accZ / 5);
            this.chartPE.Series[1].Points.AddXY(ts, m_msp.pePosZ);
            this.chartPE.Series[2].Points.AddXY(ts, m_msp.peVelZ / scaleVel);
            this.chartPE.Series[3].Points.AddXY(ts, m_msp.sonarAlt);
            this.chartPE.Series[4].Points.AddXY(ts, m_msp.sonarAltMean);
            this.chartPE.Series[5].Points.AddXY(ts, m_msp.sonarVel / scaleVel);
            this.chartPE.Series[6].Points.AddXY(ts, m_msp.baroAlt);
            this.chartPE.Series[7].Points.AddXY(ts, m_msp.baroAltMean);
            this.chartPE.Series[8].Points.AddXY(ts, m_msp.baroVel / scaleVel);
            // 9 Timestamp
            this.chartPE.Series[10].Points.AddXY(ts, m_msp.baroResidual);
            this.chartPE.Series[11].Points.AddXY(ts, m_msp.sonarResidual);
            this.chartPE.Series[12].Points.AddXY(ts, m_msp.baroSonarResidual);
            this.chartPE.Series[13].Points.AddXY(ts, m_msp.baroOffset);

            //            this.chartPE.Series[11].Points.AddXY(ts, m_msp.sonarBaroResidual);

            //            this.chartPE.Series[12].Points.AddXY(ts, Math.Max(0, m_msp.baroTemp-2500));
            //            this.chartPE.Series[13].Points.AddXY(ts, Math.Max(0,m_msp.baroRawTemp - 8700900));
            //            this.chartPE.Series[14].Points.AddXY(ts, m_msp.sonarAltMean);

            //this.txtStatus.Text = String.Format("{0}", m_msp.readFlag);
            string fl = "";
            if ((m_msp.readFlag & 1) > 0) fl += "G";
            if ((m_msp.readFlag & 2) > 0) fl += "B";
            if ((m_msp.readFlag & 4) > 0) fl += "S";
            if ((m_msp.readFlag & 8) > 0) fl += "Z";
            if ((m_msp.readFlag & 16) > 0) fl += "b";
            if ((m_msp.readFlag & 32) > 0) fl += "s";

            this.txtStatus.Text = fl;

            //addFloatVectorToChart(chartIMU_NEU_acc, m_msp.pe_imuA_NEU, ts);
            //addFloatVectorToChart(chartIMU_NEU_vel, m_msp.pe_imuV_NEU, ts);
            //addFloatVectorToChart(chartIMU_NEU_pos, m_msp.pe_imuP_NEU, ts);

            //            if (cbLog.Checked)
            //              m_log.add(m_msp.pe_imuA_NEU[2], baroAlt, m_msp.st_a, (int)m_msp.st_v);

        }

        private void enumerateSerialPorts()
        {
            comboSerialPorts.Items.Clear();
            foreach (String portName in System.IO.Ports.SerialPort.GetPortNames())
            {
                comboSerialPorts.Items.Add(portName);
            }

            if (comboSerialPorts.Items.Count>0)
            {
                comboSerialPorts.SelectedIndex = 0;
            }
        }

        private bool isSerialPortOpen()
        {
            return m_serialPort != null && m_serialPort.IsOpen;
        }

        private void updateSerialPortStatus()
        {
            bool isOpen = isSerialPortOpen();
            if (isOpen)
            {
                btnConnectSerialPort.Text = "disconnect";
                setStatus("connected");
            }
            else
            {
                btnConnectSerialPort.Text =  "connect";
                setStatus("disconnected");
            }

            timer1.Enabled = isOpen;
        }

        private void closeSerialPort()
        {
            stopSerialReceiver();

            if (m_serialPort != null)
            {
                m_serialPort.Close();
                m_serialPort = null;
            }
        }

        private void connectSelectedSerialPort()
        {
            closeSerialPort();

            if (comboSerialPorts.SelectedIndex != -1)
            {
                String port = comboSerialPorts.SelectedItem.ToString();
                serialPortOpen(port);
            }

            updateSerialPortStatus();

            bool isOpen = isSerialPortOpen();
            if (isOpen)
            {
                startSerialReceiver();
            }
        }


        private void serialPortClose()
        {
            if (m_serialPort != null)
            {
                if (m_serialPort.IsOpen)
                {
                    m_serialPort.Close();
                    setStatus("Closed serial port");
                }
            }
        }

        private bool serialPortOpen(String port)
        {
            m_serialPort = new System.IO.Ports.SerialPort(port);
            m_serialPort.BaudRate = 115200;
            m_serialPort.Open();
            return m_serialPort.IsOpen;
        }

        private void setStatus(String txt)
        {
            toolStripStatusLabel1.Text = txt;
        }

        private void btnConnectSerialPort_Click(object sender, EventArgs e)
        {
            if(isSerialPortOpen())
            {
                closeSerialPort();
            }
            else
            {
//                m_msp.hasData = false;
                connectSelectedSerialPort();
            }

            updateSerialPortStatus();
        }

        private void btnRefreshSerialPorts_Click(object sender, EventArgs e)
        {
            enumerateSerialPorts();
        }

        private void send(byte[] b)
        {
            try
            {
                if (m_serialPort != null && m_serialPort.IsOpen)
                {
                    m_serialPort.Write(b, 0, b.Length);
                }
            }
            catch (Exception)
            {
            }
        }

        private void sendCmd(MSP.msp cmd)
        {
            send(m_msp.cmdRequest(cmd));
        }

        private void stopSerialReceiver()
        {
            try
            {
                m_serialReceiver.CancelAsync();
            }
            catch (Exception e) { }
        }

        private void startSerialReceiver()
        {
            m_serialReceiver.DoWork += new DoWorkEventHandler(
            delegate (object o, DoWorkEventArgs args)
            {
                byte[] buffer = new byte[400];
                int size = 0;

                int toRead = 0;

                try
                {
                    while (m_serialPort.IsOpen)
                    {
                        toRead = 6;
                        size = 0;
                        while (toRead > 0)
                        {
                            int n = m_serialPort.Read(buffer, size, toRead);
                            if (n > 0)
                            {
                                size += n;
                                toRead -= n;
                                if (size == 6) // header
                                {
                                    toRead += buffer[3];
                                }
                            }
                        }

                        m_msp.process(buffer, size);
                    }
                }
                catch (Exception e) { }
                

                //BackgroundWorker b = o as BackgroundWorker;

                //// do some simple processing for 10 seconds
                //for (int i = 1; i <= 10; i++)
                //{
                //    // report the progress in percent
                //    b.ReportProgress(i * 10);
                //    Thread.Sleep(1000);
                //}

            });

                try
                {
                    m_serialReceiver.CancelAsync();
                    Thread.Sleep(500);
                }
                catch (Exception)
                {
                }
            m_serialReceiver.RunWorkerAsync();

      //      send(m_msp.cmdApiVersion());
        }


        private void btnClear_Click(object sender, EventArgs e)
        {
            m_log.clear();
        }

        private void button4_Click(object sender, EventArgs e)
        {
            m_log.dump();
        }

        private void cbArmed_CheckedChanged(object sender, EventArgs e)
        {
            if (cbArmed.Checked)
                m_msp.stscFlags |= 1;
            else
                m_msp.stscFlags &= ~((short)1);

            updateFlags();
        }

        public void updateFlags()
        {
            send(m_msp.cmdWriteFlag());
        }

        private void cbLine_CheckedChanged(object sender, EventArgs e)
        {
            CheckBox cb = (CheckBox)sender;
            int index = Int32.Parse((string)cb.Tag);
            this.chartPE.Series[index].Enabled = cb.Checked;
        }
        private void initLines()
        {
            cbLine_CheckedChanged(cb0, null);
            cbLine_CheckedChanged(cb1, null);
            cbLine_CheckedChanged(cb2, null);
            cbLine_CheckedChanged(cb3, null);
            cbLine_CheckedChanged(cb4, null);
            cbLine_CheckedChanged(cb5, null);
            cbLine_CheckedChanged(cb6, null);
            cbLine_CheckedChanged(cb7, null);
            cbLine_CheckedChanged(cb8, null);
            cbLine_CheckedChanged(cb9, null);
            cbLine_CheckedChanged(cb10, null);
            cbLine_CheckedChanged(cb11, null);
            cbLine_CheckedChanged(cb12, null);
            cbLine_CheckedChanged(cb13, null);
            cbLine_CheckedChanged(cb14, null);


            this.chartPE.Series[0].Name = cb0.Text;
            this.chartPE.Series[1].Name = cb1.Text;
            this.chartPE.Series[2].Name = cb2.Text;
            this.chartPE.Series[3].Name = cb3.Text;
            this.chartPE.Series[4].Name = cb4.Text;
            this.chartPE.Series[5].Name = cb5.Text;
            this.chartPE.Series[6].Name = cb6.Text;
            this.chartPE.Series[7].Name = cb7.Text;
            this.chartPE.Series[8].Name = cb8.Text;
            this.chartPE.Series[9].Name = cb9.Text;
            this.chartPE.Series[10].Name = cb10.Text;
            this.chartPE.Series[11].Name = cb11.Text;
            this.chartPE.Series[12].Name = cb12.Text;
            this.chartPE.Series[13].Name = cb13.Text;
            this.chartPE.Series[14].Name = cb14.Text;

        }

        private void updateSettings()
        {
            cbArmed_CheckedChanged(null, null);
            cbUseSonar_CheckedChanged(null, null);
            cbUseBaro_CheckedChanged(null, null);
            cbUseAccelBias_CheckedChanged(null, null);
            nuSettings_ValueChanged(null, null);
        }

        private void cbUseSonar_CheckedChanged(object sender, EventArgs e)
        {
            if (cbUseSonar.Checked)
                m_msp.stscFlags |= 2;
            else
                m_msp.stscFlags &= ~((short)2);

            updateFlags();
        }

        private void nuSettings_ValueChanged(object sender, EventArgs e)
        {
            send(m_msp.cmdSetSonar(nuMode.Value, nuBaroVelScale.Value, nuBaroVelScaleMin.Value, nuBaro_wzp.Value, nuFakeSonatAlt.Value, nuSonar.Value));
        }

        private void cbUseBaro_CheckedChanged(object sender, EventArgs e)
        {
            if (cbUseBaro.Checked)
                m_msp.stscFlags |= 4;
            else
                m_msp.stscFlags &= ~((short)4);

            updateFlags();
        }


        private void cbUseAccelBias_CheckedChanged(object sender, EventArgs e)
        {
            if (cbUseAccelBias.Checked)
                m_msp.stscFlags |= 8;
            else
                m_msp.stscFlags &= ~((short)8);

            updateFlags();

        }

        private void btnUpdate_Click(object sender, EventArgs e)
        {
            updateSettings();
        }


        private void btnTimestamp_Click(object sender, EventArgs e)
        {
            this.chartPE.Series[9].Points.AddXY(ts, m_msp.pePosZ);

        }

    }






}
