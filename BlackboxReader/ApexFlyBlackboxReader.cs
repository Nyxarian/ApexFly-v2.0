using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows.Forms;

namespace ApexFlyBlackboxReader
{
    internal sealed class MainForm : Form
    {
        private readonly ComboBox portBox = new ComboBox();
        private readonly Button refreshButton = new Button();
        private readonly Button connectButton = new Button();
        private readonly ComboBox modeBox = new ComboBox();
        private readonly ComboBox dumpCountBox = new ComboBox();
        private readonly Button readButton = new Button();
        private readonly Button stopButton = new Button();
        private readonly Button openFolderButton = new Button();
        private readonly Button openTextButton = new Button();
        private readonly Button loadFolderButton = new Button();
        private readonly Button waveButton = new Button();
        private readonly Label connectionLabel = new Label();
        private readonly Label statusLabel = new Label();
        private readonly ProgressBar progressBar = new ProgressBar();
        private readonly Label countLabel = new Label();
        private readonly Label latestLabel = new Label();
        private readonly Label pathLabel = new Label();
        private readonly ListView sessionView = new ListView();
        private readonly TextBox logBox = new TextBox();
        private readonly System.Windows.Forms.Timer receiveTimer = new System.Windows.Forms.Timer();

        private readonly Regex dumpStartRegex = new Regex(@"\[BB\] dump start records=(\d+)", RegexOptions.Compiled);
        private readonly Regex dumpProgressRegex = new Regex(@"\[BB\] dump progress scanned=(\d+)\s+total=(\d+)", RegexOptions.Compiled);
        private readonly Regex dumpDoneRegex = new Regex(@"\[BB\] dump done scanned=(\d+)\s+samples=(\d+)\s+sessions=(\d+)", RegexOptions.Compiled);
        private readonly Regex dumpallStartRegex = new Regex(@"\[BB\] dumpall start records=(\d+)", RegexOptions.Compiled);
        private readonly Regex dumpallProgressRegex = new Regex(@"\[BB\] dumpall progress scanned=(\d+)\s+total=(\d+)", RegexOptions.Compiled);
        private readonly Regex dumpallDoneRegex = new Regex(@"\[BB\] dumpall done scanned=(\d+)", RegexOptions.Compiled);
        private readonly Regex sessionBeginRegex = new Regex(@"^# session=(\d+)\s+t0_utc=(\S+)(?:\s+unix=(\d+))?", RegexOptions.Compiled);
        private readonly Regex sessionEndRegex = new Regex(@"^# end session=(\d+)\s+samples=(\d+)\s+t_ms=(\d+)(?:\s+t1_utc=(\S+))?(?:\s+unix=(\d+))?(?:\s+utc=(\d+))?", RegexOptions.Compiled);
        private static readonly Regex pidPairRegex = new Regex(@"(\d+)=(-?\d+(?:\.\d+)?)", RegexOptions.Compiled);
        private readonly Regex hexLineRegex = new Regex(@"^([0-9A-Fa-f]{8})(?:\s+[0-9A-Fa-f]{2}){1,16}$", RegexOptions.Compiled);

        private static readonly string[] PidNames =
        {
            "roll_outer_kp", "roll_outer_ki", "roll_outer_kd",
            "roll_inner_kp", "roll_inner_ki", "roll_inner_kd",
            "pitch_outer_kp", "pitch_outer_ki", "pitch_outer_kd",
            "pitch_inner_kp", "pitch_inner_ki", "pitch_inner_kd",
            "yaw_rate_kp", "yaw_rate_ki", "yaw_rate_kd",
            "roll_max_rate_dps", "roll_max_corr_us",
            "pitch_max_rate_dps", "pitch_max_corr_us",
            "yaw_max_rate_dps", "yaw_max_corr_us",
            "roll_trim_deg", "pitch_trim_deg",
            "yaw_angle_kp", "yaw_angle_ki", "yaw_angle_kd",
            "vertical_speed_kp", "vertical_speed_ki", "vertical_speed_kd", "max_throttle_us"
        };

        private SerialPort serialPort;
        private readonly StringBuilder receiveBuffer = new StringBuilder();
        private bool sawDumpDone;
        private StreamWriter rawWriter;
        private StreamWriter allDataWriter;
        private StreamWriter dumpallTextWriter;
        private FileStream dumpallBinWriter;
        private bool captureActive;
        private bool dumpStarted;
        private bool dumpallMode;
        private int commandAttempts;
        private DateTime lastCommandTime;
        private DateTime lastDataTime;
        private long expectedRecords;
        private long scannedRecords;
        private long sampleRecords;
        private long sessionRecords;
        private long hexBytes;
        private int unexpectedHexLines;
        private long requestedDumpCount;
        private string currentExportDirectory;
        private SessionState currentSession;
        private readonly List<SessionState> sessions = new List<SessionState>();
        private readonly Dictionary<int, StreamWriter> sessionCsvWriters = new Dictionary<int, StreamWriter>();
        private readonly Dictionary<int, StreamWriter> sessionVofaWriters = new Dictionary<int, StreamWriter>();

        private sealed class SessionState
        {
            public int SessionId;
            public string T0Utc = "NA";
            public string T1Utc = "NA";
            public readonly string[] Pid = new string[30];
            public long Samples;
            public long LastTms;
            public bool Complete;
        }

        public MainForm()
        {
            Text = "ApexFly v2.0 黑匣子读取";
            ClientSize = new Size(980, 720);
            MinimumSize = new Size(900, 640);
            StartPosition = FormStartPosition.CenterScreen;
            Font = new Font("Microsoft YaHei UI", 9F, FontStyle.Regular, GraphicsUnit.Point);
            BackColor = Color.FromArgb(30, 33, 38);
            ForeColor = Color.White;
            BuildInterface();
            RefreshPorts();
            receiveTimer.Interval = 40;
            receiveTimer.Tick += ReceiveTimerTick;
            FormClosing += MainFormClosing;
        }

        private void BuildInterface()
        {
            Controls.Add(MakeLabel("串口", 18, 20, 40, 28));
            portBox.SetBounds(58, 18, 100, 30);
            portBox.DropDownStyle = ComboBoxStyle.DropDownList;
            Controls.Add(portBox);

            StyleButton(refreshButton, "刷新", 166, 17, 64, 32);
            refreshButton.Click += delegate { RefreshPorts(); };
            Controls.Add(refreshButton);

            StyleButton(connectButton, "连接", 234, 17, 72, 32);
            connectButton.Click += ConnectButtonClick;
            Controls.Add(connectButton);

            Controls.Add(MakeLabel("模式", 316, 20, 36, 28));
            modeBox.SetBounds(352, 18, 108, 30);
            modeBox.DropDownStyle = ComboBoxStyle.DropDownList;
            modeBox.Items.AddRange(new object[] { "场次 CSV", "全部原始" });
            modeBox.SelectedIndex = 0;
            Controls.Add(modeBox);

            Controls.Add(MakeLabel("最近", 468, 20, 36, 28));
            dumpCountBox.SetBounds(504, 18, 82, 30);
            dumpCountBox.DropDownStyle = ComboBoxStyle.DropDownList;
            dumpCountBox.Items.AddRange(new object[] { "20000", "50000", "100000", "全部" });
            dumpCountBox.SelectedIndex = 0;
            Controls.Add(dumpCountBox);

            StyleButton(readButton, "读取", 594, 17, 72, 32);
            readButton.Click += ReadButtonClick;
            Controls.Add(readButton);

            StyleButton(stopButton, "停止", 670, 17, 64, 32);
            stopButton.Enabled = false;
            stopButton.Click += StopButtonClick;
            Controls.Add(stopButton);

            StyleButton(openFolderButton, "打开导出目录", 740, 17, 118, 32);
            openFolderButton.Click += OpenFolderButtonClick;
            Controls.Add(openFolderButton);

            connectionLabel.SetBounds(862, 20, 100, 26);
            connectionLabel.TextAlign = ContentAlignment.MiddleRight;
            connectionLabel.Text = "未连接";
            connectionLabel.ForeColor = Color.IndianRed;
            Controls.Add(connectionLabel);

            GroupBox statusGroup = MakeGroup("读取状态", 18, 58, 944, 150);
            Controls.Add(statusGroup);

            statusLabel.SetBounds(16, 24, 910, 22);
            statusLabel.Text = "等待连接飞控。先上锁，关掉 VOFA，再点连接。";
            statusGroup.Controls.Add(statusLabel);

            progressBar.SetBounds(16, 50, 910, 22);
            progressBar.Maximum = 1000;
            progressBar.Style = ProgressBarStyle.Continuous;
            progressBar.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            statusGroup.Controls.Add(progressBar);

            countLabel.SetBounds(16, 78, 910, 22);
            countLabel.Text = "场次: 0    样本: 0    扫描: 0/0    原始字节: 0";
            statusGroup.Controls.Add(countLabel);

            latestLabel.SetBounds(16, 100, 910, 22);
            latestLabel.Text = "最新数据: --";
            statusGroup.Controls.Add(latestLabel);

            pathLabel.SetBounds(16, 122, 910, 22);
            pathLabel.AutoEllipsis = true;
            pathLabel.Text = "导出目录: --";
            statusGroup.Controls.Add(pathLabel);

            StyleButton(openTextButton, "打开已有文本", 18, 216, 118, 28);
            openTextButton.Click += OpenTextButtonClick;
            Controls.Add(openTextButton);
            StyleButton(loadFolderButton, "加载导出文件夹", 144, 216, 126, 28);
            loadFolderButton.Click += LoadFolderButtonClick;
            Controls.Add(loadFolderButton);
            StyleButton(waveButton, "看波形", 278, 216, 88, 28);
            waveButton.Click += WaveButtonClick;
            Controls.Add(waveButton);

            GroupBox sessionGroup = MakeGroup("已解析场次", 18, 250, 944, 170);
            Controls.Add(sessionGroup);
            sessionView.SetBounds(12, 22, 920, 136);
            sessionView.View = View.Details;
            sessionView.FullRowSelect = true;
            sessionView.GridLines = true;
            sessionView.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            sessionView.BackColor = Color.FromArgb(18, 20, 23);
            sessionView.ForeColor = Color.Gainsboro;
            sessionView.Columns.Add("场次", 60);
            sessionView.Columns.Add("北京时间", 160);
            sessionView.Columns.Add("样本", 70);
            sessionView.Columns.Add("时长 ms", 80);
            sessionView.Columns.Add("完整", 50);
            sessionView.Columns.Add("Roll外环Kp / Pitch外环Kp / 内环Kp", 470);
            sessionView.DoubleClick += WaveButtonClick;
            sessionGroup.Controls.Add(sessionView);

            GroupBox logGroup = MakeGroup("运行日志", 18, 430, 944, 268);
            Controls.Add(logGroup);
            logBox.SetBounds(12, 22, 920, 232);
            logBox.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            logBox.Multiline = true;
            logBox.ReadOnly = true;
            logBox.ScrollBars = ScrollBars.Vertical;
            logBox.BackColor = Color.FromArgb(18, 20, 23);
            logBox.ForeColor = Color.Gainsboro;
            logBox.Font = new Font("Consolas", 9F);
            logGroup.Controls.Add(logBox);
        }

        private static void StyleButton(Button button, string text, int x, int y, int width, int height)
        {
            button.Text = text;
            button.SetBounds(x, y, width, height);
            button.FlatStyle = FlatStyle.System;
        }

        private Label MakeLabel(string text, int x, int y, int width, int height)
        {
            Label label = new Label();
            label.Text = text;
            label.SetBounds(x, y, width, height);
            label.TextAlign = ContentAlignment.MiddleLeft;
            return label;
        }

        private GroupBox MakeGroup(string text, int x, int y, int width, int height)
        {
            GroupBox group = new GroupBox();
            group.Text = text;
            group.ForeColor = Color.White;
            group.SetBounds(x, y, width, height);
            group.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            if (text == "运行日志") group.Anchor |= AnchorStyles.Bottom;
            return group;
        }

        private void RefreshPorts()
        {
            string selected = portBox.SelectedItem as string;
            string[] ports = SerialPort.GetPortNames();
            Array.Sort(ports, StringComparer.OrdinalIgnoreCase);
            portBox.Items.Clear();
            portBox.Items.AddRange(ports);
            if (!string.IsNullOrEmpty(selected) && portBox.Items.Contains(selected)) portBox.SelectedItem = selected;
            else if (portBox.Items.Count > 0) portBox.SelectedIndex = 0;
            AddLog("已刷新串口列表: " + (ports.Length == 0 ? "未发现串口" : string.Join(", ", ports)));
        }

        private void ConnectButtonClick(object sender, EventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                DisconnectSerial();
                return;
            }
            if (portBox.SelectedItem == null)
            {
                MessageBox.Show("请先选择串口", Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            try
            {
                serialPort = new SerialPort(portBox.SelectedItem.ToString(), 115200, Parity.None, 8, StopBits.One);
                serialPort.Handshake = Handshake.None;
                serialPort.DtrEnable = false;
                serialPort.RtsEnable = false;
                serialPort.ReadBufferSize = 1024 * 1024;
                serialPort.WriteBufferSize = 4096;
                serialPort.Encoding = Encoding.ASCII;
                serialPort.NewLine = "\r\n";
                serialPort.Open();
                try
                {
                    serialPort.DtrEnable = false;
                    serialPort.RtsEnable = false;
                }
                catch
                {
                }
                receiveBuffer.Length = 0;
                receiveTimer.Start();
                connectButton.Text = "断开";
                connectionLabel.Text = "已连接 " + serialPort.PortName;
                connectionLabel.ForeColor = Color.LightGreen;
                statusLabel.Text = "已连接，飞机上锁后可以读取黑匣子";
                AddLog("串口连接成功: " + serialPort.PortName + " 115200 8N1，DTR/RTS 已关掉");
            }
            catch (Exception ex)
            {
                MessageBox.Show("串口连接失败:\r\n" + ex.Message + "\r\n关掉 VOFA 和其他串口助手后再试。", Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
                DisconnectSerial();
            }
        }

        private void DisconnectSerial()
        {
            if (captureActive) FinishCapture(false, "串口已断开");
            receiveTimer.Stop();
            try { if (serialPort != null && serialPort.IsOpen) serialPort.Close(); } catch { }
            if (serialPort != null) serialPort.Dispose();
            serialPort = null;
            connectButton.Text = "连接";
            connectionLabel.Text = "未连接";
            connectionLabel.ForeColor = Color.IndianRed;
            statusLabel.Text = "等待连接飞控";
            AddLog("串口已断开");
        }

        private void ReadButtonClick(object sender, EventArgs e)
        {
            long selectedCount = 0;
            bool readAll;
            if (serialPort == null || !serialPort.IsOpen)
            {
                MessageBox.Show("请先连接飞控调试串口", Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            readAll = dumpCountBox.SelectedItem != null && dumpCountBox.SelectedItem.ToString() == "全部";
            if (!readAll && (!long.TryParse(dumpCountBox.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out selectedCount) || selectedCount <= 0))
            {
                MessageBox.Show("请选择合法的读取条数", Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            dumpallMode = modeBox.SelectedIndex == 1;
            string modeText = dumpallMode ? "全部原始日志（含校验失败的老数据）" : "新格式场次 CSV";
            string countText = readAll ? "全部记录" : ("最近 " + selectedCount.ToString(CultureInfo.InvariantCulture) + " 条");
            DialogResult result = MessageBox.Show("读取黑匣子时必须保持飞机上锁、电机停止。\r\n\r\n模式: " + modeText + "\r\n范围: " + countText + "\r\n\r\n确定开始读取吗？", Text, MessageBoxButtons.YesNo, MessageBoxIcon.Warning);
            if (result != DialogResult.Yes) return;
            StartCapture(readAll ? 0 : selectedCount, dumpallMode);
        }

        private void StartCapture(long dumpCount, bool rawMode)
        {
            CloseWriters();
            sessions.Clear();
            currentSession = null;
            sessionView.Items.Clear();
            string sessionName = "blackbox_" + DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            currentExportDirectory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "exports", sessionName);
            Directory.CreateDirectory(currentExportDirectory);
            UTF8Encoding utf8 = new UTF8Encoding(false);
            rawWriter = new StreamWriter(Path.Combine(currentExportDirectory, "serial_raw.log"), false, utf8, 65536);
            if (rawMode)
            {
                dumpallTextWriter = new StreamWriter(Path.Combine(currentExportDirectory, "dumpall_raw.txt"), false, utf8, 65536);
                dumpallBinWriter = new FileStream(Path.Combine(currentExportDirectory, "dumpall.bin"), FileMode.Create, FileAccess.Write, FileShare.Read);
            }
            else
            {
                allDataWriter = new StreamWriter(Path.Combine(currentExportDirectory, "blackbox_data.csv"), false, utf8, 65536);
                allDataWriter.WriteLine("session,t_ms,roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate,roll_corr,pitch_corr,yaw_corr,thr,m1,m2,m3,m4,alt_cm,vz_cms,mag_hdg,vz_corr_us,vz_target_cms,alt_state,armed,conn");
            }
            captureActive = true;
            dumpStarted = false;
            sawDumpDone = false;
            dumpallMode = rawMode;
            commandAttempts = 0;
            lastDataTime = DateTime.Now;
            requestedDumpCount = dumpCount;
            expectedRecords = 0;
            scannedRecords = 0;
            sampleRecords = 0;
            sessionRecords = 0;
            hexBytes = 0;
            unexpectedHexLines = 0;
            progressBar.Value = 0;
            readButton.Enabled = false;
            stopButton.Enabled = true;
            pathLabel.Text = "导出目录: " + currentExportDirectory;
            statusLabel.Text = rawMode ? "正在请求飞控导出全部原始日志..." : "正在请求飞控导出场次 CSV...";
            latestLabel.Text = "最新数据: --";
            UpdateProgress();
            AddLog(rawMode ? "开始读取全部原始日志" : "开始读取场次 CSV");
            SendDumpCommand();
        }

        private string BuildCommand()
        {
            string verb = dumpallMode ? "dumpall" : "dump";
            if (requestedDumpCount == 0) return verb + " all\r\n";
            return verb + " " + requestedDumpCount.ToString(CultureInfo.InvariantCulture) + "\r\n";
        }

        private void SendSlow(string command)
        {
            if (serialPort == null || !serialPort.IsOpen) return;
            for (int index = 0; index < command.Length; index++)
            {
                serialPort.Write(command[index].ToString());
                Thread.Sleep(8);
            }
        }

        private void SendDumpCommand()
        {
            if (!captureActive || dumpStarted || serialPort == null || !serialPort.IsOpen) return;
            try
            {
                commandAttempts++;
                string command = BuildCommand();
                SendSlow(command);
                lastCommandTime = DateTime.Now;
                AddLog(command.Trim() + " -> 飞控 (" + commandAttempts.ToString(CultureInfo.InvariantCulture) + ")");
            }
            catch (Exception ex)
            {
                FinishCapture(false, "发送读取命令失败: " + ex.Message);
            }
        }

        private void StopButtonClick(object sender, EventArgs e)
        {
            try { SendSlow("stop\r\n"); } catch { }
            AddLog("stop -> 飞控");
            FinishCapture(false, "用户停止了读取。若飞机还在刷十六进制，给飞机断电再上电。");
        }

        private void ReceiveTimerTick(object sender, EventArgs e)
        {
            try
            {
                if (serialPort != null && serialPort.IsOpen && serialPort.BytesToRead > 0)
                {
                    receiveBuffer.Append(serialPort.ReadExisting());
                    DrainLines(true);
                    if (receiveBuffer.Length > 262144) receiveBuffer.Remove(0, receiveBuffer.Length - 65536);
                }
                if (captureActive && !dumpStarted && commandAttempts < 8 && (DateTime.Now - lastCommandTime).TotalMilliseconds > 4000.0) SendDumpCommand();
                if (captureActive && !dumpStarted && commandAttempts >= 8 && (DateTime.Now - lastCommandTime).TotalMilliseconds > 20000.0)
                {
                    FinishCapture(false, "飞控没有响应读取命令。先确认已上锁；若刚跑过 dumpall，等它打完 dumpall done 再点读取。");
                }
                /* 必须等到 dump done。心跳 rec= 不能当结束，否则最新场次还没打出来就被掐掉。 */
                if (captureActive && dumpStarted && sawDumpDone && (DateTime.Now - lastDataTime).TotalMilliseconds > 1500.0)
                {
                    FinishCapture(true, dumpallMode ? "全部原始日志读取完成" : "场次 CSV 读取完成");
                }
                if (captureActive && dumpStarted && !sawDumpDone && (DateTime.Now - lastDataTime).TotalMilliseconds > 120000.0)
                {
                    FinishCapture(false, "超过两分钟没有新数据。若还在刷进度，再等；否则点停止后看已收到的场次。");
                }
            }
            catch (Exception ex)
            {
                AddLog("串口读取异常: " + ex.Message);
                DisconnectSerial();
            }
        }

        private void DrainLines(bool live)
        {
            while (true)
            {
                string text = receiveBuffer.ToString();
                int newlineIndex = text.IndexOf('\n');
                if (newlineIndex < 0) break;
                string line = text.Substring(0, newlineIndex).TrimEnd('\r');
                receiveBuffer.Remove(0, newlineIndex + 1);
                ProcessLine(line, live);
            }
        }

        private void ProcessLine(string line, bool live)
        {
            if (string.IsNullOrEmpty(line)) return;
            if (live && captureActive && rawWriter != null) rawWriter.WriteLine(line);
            bool heartbeat = line.StartsWith("[ATT]", StringComparison.Ordinal) || line.StartsWith("[RADIO]", StringComparison.Ordinal) || line.StartsWith("[PID]", StringComparison.Ordinal) || line.StartsWith("[BB] rec=", StringComparison.Ordinal);
            if (!heartbeat) lastDataTime = DateTime.Now;

            if (line.Contains("[BB] disarm first"))
            {
                AddLog(line);
                if (captureActive) FinishCapture(false, "飞机还在解锁，请先上锁再读取");
                return;
            }

            Match begin = dumpallMode ? dumpallStartRegex.Match(line) : dumpStartRegex.Match(line);
            if (!begin.Success && line.IndexOf("[BB] dumpall start", StringComparison.Ordinal) >= 0) begin = dumpallStartRegex.Match(line);
            if (begin.Success)
            {
                dumpStarted = true;
                expectedRecords = ParseLong(begin.Groups[1].Value);
                if (requestedDumpCount > 0 && expectedRecords > requestedDumpCount) expectedRecords = requestedDumpCount;
                statusLabel.Text = dumpallMode ? "正在读取全部原始日志..." : "正在读取场次 CSV...";
                AddLog(line);
                UpdateProgress();
                return;
            }
            if (line.IndexOf("[BB] dumpall start", StringComparison.Ordinal) >= 0 || (!dumpallMode && line.IndexOf("[BB] dump start", StringComparison.Ordinal) >= 0))
            {
                dumpStarted = true;
                statusLabel.Text = dumpallMode ? "正在读取全部原始日志..." : "正在读取场次 CSV...";
                AddLog(line);
                return;
            }

            Match progress = dumpallMode ? dumpallProgressRegex.Match(line) : dumpProgressRegex.Match(line);
            if (progress.Success)
            {
                dumpStarted = true;
                scannedRecords = ParseLong(progress.Groups[1].Value);
                if (expectedRecords == 0) expectedRecords = ParseLong(progress.Groups[2].Value);
                UpdateProgress();
                return;
            }

            Match end = dumpallMode ? dumpallDoneRegex.Match(line) : dumpDoneRegex.Match(line);
            if (!end.Success && line.IndexOf("[BB] dump done", StringComparison.Ordinal) >= 0) end = dumpDoneRegex.Match(line);
            if (!end.Success && line.IndexOf("[BB] dumpall done", StringComparison.Ordinal) >= 0) end = dumpallDoneRegex.Match(line);
            if (end.Success || line.IndexOf("[BB] dump done", StringComparison.Ordinal) >= 0 || line.IndexOf("[BB] dumpall done", StringComparison.Ordinal) >= 0 || line.IndexOf("[BB] dump abort", StringComparison.Ordinal) >= 0)
            {
                dumpStarted = true;
                sawDumpDone = true;
                if (end.Success) scannedRecords = ParseLong(end.Groups[1].Value);
                if (!dumpallMode && end.Success && end.Groups.Count > 3)
                {
                    sampleRecords = ParseLong(end.Groups[2].Value);
                    sessionRecords = ParseLong(end.Groups[3].Value);
                }
                AddLog(line);
                return;
            }

            if (line.StartsWith("[ATT]", StringComparison.Ordinal) || line.StartsWith("[RADIO]", StringComparison.Ordinal) || line.StartsWith("[PID]", StringComparison.Ordinal)) return;
            if (line.StartsWith("[BB]"))
            {
                dumpStarted = true;
                AddLog(line);
                return;
            }
            if (line.StartsWith("# session=", StringComparison.Ordinal) || line.StartsWith("# pid", StringComparison.Ordinal) || hexLineRegex.IsMatch(line.Trim()))
            {
                dumpStarted = true;
            }

            if (dumpallMode)
            {
                HandleHexLine(line);
                return;
            }
            if (hexLineRegex.IsMatch(line.Trim()))
            {
                unexpectedHexLines++;
                dumpStarted = true;
                if (unexpectedHexLines == 1)
                {
                    AddLog("飞机还在打 dumpall 十六进制，场次 CSV 进不去。点停止，或给飞机断电再上电。关电脑软件停不了飞机。");
                    statusLabel.Text = "飞机还在 dumpall，关软件没用，请断电或发 stop";
                    try { SendSlow("stop\r\n"); } catch { }
                }
                if (unexpectedHexLines >= 40)
                {
                    FinishCapture(false, "场次 CSV 读不到：飞机仍在 dumpall。请给飞机断电再上电，然后重新连接。");
                }
                return;
            }

            HandleCsvLine(line);
        }

        private void HandleHexLine(string line)
        {
            if (!hexLineRegex.IsMatch(line.Trim())) return;
            dumpStarted = true;
            if (dumpallTextWriter != null) dumpallTextWriter.WriteLine(line);
            string[] parts = line.Trim().Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 2) return;
            for (int index = 1; index < parts.Length; index++)
            {
                byte value;
                if (!byte.TryParse(parts[index], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out value)) continue;
                if (dumpallBinWriter != null) dumpallBinWriter.WriteByte(value);
                hexBytes++;
            }
            scannedRecords = hexBytes / 64;
            if ((hexBytes % 4096) == 0) UpdateProgress();
        }

        private void HandleCsvLine(string line)
        {
            Match sessionBegin = sessionBeginRegex.Match(line);
            if (sessionBegin.Success)
            {
                dumpStarted = true;
                FlushCurrentSession(false);
                currentSession = new SessionState();
                currentSession.SessionId = (int)ParseLong(sessionBegin.Groups[1].Value);
                currentSession.T0Utc = NormalizeTimeText(sessionBegin.Groups[2].Value);
                sessions.Add(currentSession);
                sessionRecords = sessions.Count;
                EnsureSessionWriters(currentSession);
                RefreshSessionView();
                return;
            }

            if (line.StartsWith("# pid", StringComparison.Ordinal))
            {
                dumpStarted = true;
                if (currentSession == null) return;
                foreach (Match pair in pidPairRegex.Matches(line))
                {
                    int id = (int)ParseLong(pair.Groups[1].Value);
                    if (id >= 0 && id < currentSession.Pid.Length) currentSession.Pid[id] = pair.Groups[2].Value;
                }
                StreamWriter pidHeaderWriter;
                if (sessionCsvWriters.TryGetValue(currentSession.SessionId, out pidHeaderWriter)) pidHeaderWriter.WriteLine(line);
                RefreshSessionView();
                return;
            }

            if (line.StartsWith("# altitude", StringComparison.Ordinal))
            {
                if (currentSession != null)
                {
                    foreach (Match pair in pidPairRegex.Matches(line.Replace("vz_kp", "26").Replace("vz_ki", "27").Replace("vz_kd", "28").Replace("hover_pwm", "29").Replace("max_thr", "29")))
                    {
                        int id = (int)ParseLong(pair.Groups[1].Value);
                        if (id >= 26 && id < currentSession.Pid.Length) currentSession.Pid[id] = pair.Groups[2].Value;
                    }
                    StreamWriter altitudeHeaderWriter;
                    if (sessionCsvWriters.TryGetValue(currentSession.SessionId, out altitudeHeaderWriter)) altitudeHeaderWriter.WriteLine(line);
                    RefreshSessionView();
                }
                return;
            }

            if (line.StartsWith("t_ms,", StringComparison.Ordinal)) return;

            Match sessionEnd = sessionEndRegex.Match(line);
            if (sessionEnd.Success)
            {
                if (currentSession != null)
                {
                    currentSession.Complete = true;
                    currentSession.LastTms = ParseLong(sessionEnd.Groups[3].Value);
                    if (sessionEnd.Groups[4].Success) currentSession.T1Utc = NormalizeTimeText(sessionEnd.Groups[4].Value);
                }
                FlushCurrentSession(true);
                currentSession = null;
                RefreshSessionView();
                return;
            }

            if (line.Length == 0 || !char.IsDigit(line[0])) return;
            string[] values = line.Split(',');
            if (values.Length < 17) return;
            /* 旧 17/19 列样本补齐成统一 23 列，避免新表头与旧数据错列。 */
            if (values.Length == 17)
            {
                string armed = values[15];
                string connected = values[16];
                line = string.Join(",", values, 0, 15) + ",0,0,0,0,0,0," + armed + "," + connected;
                values = line.Split(',');
            }
            else if (values.Length == 19)
            {
                string armed = values[17];
                string connected = values[18];
                line = string.Join(",", values, 0, 17) + ",0,0,0,0," + armed + "," + connected;
                values = line.Split(',');
            }
            if (currentSession == null)
            {
                currentSession = new SessionState();
                currentSession.SessionId = 0;
                sessions.Add(currentSession);
                EnsureSessionWriters(currentSession);
            }
            currentSession.Samples++;
            currentSession.LastTms = ParseLong(values[0]);
            sampleRecords++;
            string row = currentSession.SessionId.ToString(CultureInfo.InvariantCulture) + "," + line;
            if (allDataWriter != null) allDataWriter.WriteLine(row);
            StreamWriter csvWriter;
            if (sessionCsvWriters.TryGetValue(currentSession.SessionId, out csvWriter)) csvWriter.WriteLine(line);
            StreamWriter vofaWriter;
            if (sessionVofaWriters.TryGetValue(currentSession.SessionId, out vofaWriter)) vofaWriter.WriteLine(line);
            if ((sampleRecords % 50) == 0)
            {
                string altitudeText = values.Length >= 19 ? values[15] : "-";
                latestLabel.Text = string.Format(CultureInfo.InvariantCulture, "最新数据: 场次 {0}    Roll {1} deg    Pitch {2} deg    Yaw {3} deg    Thr {4} us    Alt {5} cm", currentSession.SessionId, values[1], values[2], values[3], values[10], altitudeText);
                UpdateProgress();
            }
        }

        private void EnsureSessionWriters(SessionState session)
        {
            if (sessionCsvWriters.ContainsKey(session.SessionId)) return;
            UTF8Encoding utf8 = new UTF8Encoding(false);
            string stem = Path.Combine(currentExportDirectory, "session_" + session.SessionId.ToString("000", CultureInfo.InvariantCulture));
            StreamWriter csv = new StreamWriter(stem + ".csv", false, utf8, 32768);
            csv.WriteLine("# session=" + session.SessionId.ToString(CultureInfo.InvariantCulture) + " t0_utc=" + session.T0Utc);
            csv.WriteLine("t_ms,roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate,roll_corr,pitch_corr,yaw_corr,thr,m1,m2,m3,m4,alt_cm,vz_cms,mag_hdg,vz_corr_us,vz_target_cms,alt_state,armed,conn");
            sessionCsvWriters[session.SessionId] = csv;
            StreamWriter vofa = new StreamWriter(stem + ".vofa.csv", false, utf8, 32768);
            sessionVofaWriters[session.SessionId] = vofa;
        }

        private void FlushCurrentSession(bool complete)
        {
            if (currentSession == null) return;
            currentSession.Complete = complete || currentSession.Complete;
            WritePidFile(currentSession);
        }

        private void WritePidFile(SessionState session)
        {
            if (string.IsNullOrEmpty(currentExportDirectory)) return;
            string path = Path.Combine(currentExportDirectory, "session_" + session.SessionId.ToString("000", CultureInfo.InvariantCulture) + ".pid.csv");
            using (StreamWriter writer = new StreamWriter(path, false, new UTF8Encoding(false)))
            {
                writer.WriteLine("id,name,value");
                for (int index = 0; index < PidNames.Length; index++)
                {
                    writer.WriteLine(index.ToString(CultureInfo.InvariantCulture) + "," + PidNames[index] + "," + (session.Pid[index] ?? ""));
                }
            }
            StreamWriter csvWriter;
            if (sessionCsvWriters.TryGetValue(session.SessionId, out csvWriter))
            {
                /* 场次 CSV 头里补 PID，方便以后对照。 */
            }
        }

        private void RefreshSessionView()
        {
            sessionView.BeginUpdate();
            sessionView.Items.Clear();
            foreach (SessionState session in sessions)
            {
                string pid = (session.Pid[0] ?? "-") + " / " + (session.Pid[6] ?? "-") + " / " + (session.Pid[3] ?? "-") + "," + (session.Pid[9] ?? "-");
                ListViewItem item = new ListViewItem(session.SessionId.ToString(CultureInfo.InvariantCulture));
                item.SubItems.Add(session.T0Utc);
                item.SubItems.Add(session.Samples.ToString("N0", CultureInfo.InvariantCulture));
                item.SubItems.Add(session.LastTms.ToString("N0", CultureInfo.InvariantCulture));
                item.SubItems.Add(session.Complete ? "是" : "否");
                item.SubItems.Add(pid);
                item.Tag = session.SessionId;
                sessionView.Items.Add(item);
            }
            sessionView.EndUpdate();
        }

        private void UpdateProgress()
        {
            long displayed = scannedRecords > 0 ? scannedRecords : (dumpallMode ? (hexBytes / 64) : sampleRecords);
            if (expectedRecords > 0)
            {
                long scaled = displayed * progressBar.Maximum / expectedRecords;
                if (scaled < 0) scaled = 0;
                if (scaled > progressBar.Maximum) scaled = progressBar.Maximum;
                progressBar.Value = (int)scaled;
            }
            countLabel.Text = "场次: " + sessions.Count.ToString("N0", CultureInfo.InvariantCulture) + "    样本: " + sampleRecords.ToString("N0", CultureInfo.InvariantCulture) + "    扫描: " + displayed.ToString("N0", CultureInfo.InvariantCulture) + "/" + expectedRecords.ToString("N0", CultureInfo.InvariantCulture) + "    原始字节: " + hexBytes.ToString("N0", CultureInfo.InvariantCulture);
        }

        private void FinishCapture(bool completed, string message)
        {
            if (!captureActive) return;
            if (receiveBuffer.Length > 0)
            {
                if (rawWriter != null) rawWriter.Write(receiveBuffer.ToString());
                DrainLines(false);
            }
            captureActive = false;
            if (currentSession != null) FlushCurrentSession(currentSession.Complete);
            WriteIndex();
            CloseWriters();
            readButton.Enabled = true;
            stopButton.Enabled = false;
            if (completed) progressBar.Value = progressBar.Maximum;
            statusLabel.Text = message;
            RefreshSessionView();
            UpdateProgress();
            AddLog(message);
            if (!string.IsNullOrEmpty(currentExportDirectory)) AddLog("文件已保存: " + currentExportDirectory);
        }

        private void WriteIndex()
        {
            if (string.IsNullOrEmpty(currentExportDirectory)) return;
            string path = Path.Combine(currentExportDirectory, "INDEX.txt");
            using (StreamWriter writer = new StreamWriter(path, false, new UTF8Encoding(false)))
            {
                writer.WriteLine("ApexFly v2.0 黑匣子导出");
                writer.WriteLine("模式: " + (dumpallMode ? "dumpall 全部原始" : "dump 场次 CSV"));
                writer.WriteLine("时间: " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture));
                writer.WriteLine("扫描: " + scannedRecords.ToString(CultureInfo.InvariantCulture));
                writer.WriteLine("样本: " + sampleRecords.ToString(CultureInfo.InvariantCulture));
                writer.WriteLine("场次: " + sessions.Count.ToString(CultureInfo.InvariantCulture));
                writer.WriteLine("原始字节: " + hexBytes.ToString(CultureInfo.InvariantCulture));
                writer.WriteLine();
                foreach (SessionState session in sessions)
                {
                    writer.WriteLine("session=" + session.SessionId.ToString(CultureInfo.InvariantCulture) + " t0_bjt=" + session.T0Utc + " t1_bjt=" + session.T1Utc + " samples=" + session.Samples.ToString(CultureInfo.InvariantCulture) + " t_ms=" + session.LastTms.ToString(CultureInfo.InvariantCulture) + " complete=" + (session.Complete ? "1" : "0") + " roll_kp=" + (session.Pid[0] ?? "") + " pitch_kp=" + (session.Pid[6] ?? "") + " roll_inner_kp=" + (session.Pid[3] ?? "") + " pitch_inner_kp=" + (session.Pid[9] ?? ""));
                }
            }
        }

        private void CloseWriters()
        {
            try { if (rawWriter != null) { rawWriter.Flush(); rawWriter.Dispose(); } } catch { }
            try { if (allDataWriter != null) { allDataWriter.Flush(); allDataWriter.Dispose(); } } catch { }
            try { if (dumpallTextWriter != null) { dumpallTextWriter.Flush(); dumpallTextWriter.Dispose(); } } catch { }
            try { if (dumpallBinWriter != null) { dumpallBinWriter.Flush(); dumpallBinWriter.Dispose(); } } catch { }
            foreach (StreamWriter writer in sessionCsvWriters.Values)
            {
                try { writer.Flush(); writer.Dispose(); } catch { }
            }
            foreach (StreamWriter writer in sessionVofaWriters.Values)
            {
                try { writer.Flush(); writer.Dispose(); } catch { }
            }
            sessionCsvWriters.Clear();
            sessionVofaWriters.Clear();
            rawWriter = null;
            allDataWriter = null;
            dumpallTextWriter = null;
            dumpallBinWriter = null;
        }

        private void WaveButtonClick(object sender, EventArgs e)
        {
            int sessionId = -1;
            if (sessionView.SelectedItems.Count > 0 && sessionView.SelectedItems[0].Tag is int) sessionId = (int)sessionView.SelectedItems[0].Tag;
            else if (sessions.Count > 0) sessionId = sessions[sessions.Count - 1].SessionId;
            if (sessionId < 0)
            {
                MessageBox.Show("先读取、加载文件夹，或在表格里选一场。", Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            string folder = currentExportDirectory;
            if (string.IsNullOrEmpty(folder) || !Directory.Exists(folder))
            {
                MessageBox.Show("没有导出目录。", Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            string path = Path.Combine(folder, "session_" + sessionId.ToString("000", CultureInfo.InvariantCulture) + ".csv");
            if (!File.Exists(path))
            {
                MessageBox.Show("找不到 " + path, Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            try
            {
                using (WaveformForm form = new WaveformForm(path, "场次 " + sessionId.ToString(CultureInfo.InvariantCulture)))
                {
                    form.ShowDialog(this);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void LoadFolderButtonClick(object sender, EventArgs e)
        {
            using (FolderBrowserDialog dialog = new FolderBrowserDialog())
            {
                dialog.Description = "选择一次导出目录，例如 blackbox_20260823_132942";
                string seed = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "exports");
                if (Directory.Exists(seed)) dialog.SelectedPath = seed;
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                LoadExportFolder(dialog.SelectedPath);
            }
        }

        private void LoadExportFolder(string folder)
        {
            string[] files = Directory.GetFiles(folder, "session_*.csv");
            Array.Sort(files, StringComparer.OrdinalIgnoreCase);
            sessions.Clear();
            sessionView.Items.Clear();
            currentExportDirectory = folder;
            pathLabel.Text = "导出目录: " + folder;
            foreach (string path in files)
            {
                string name = Path.GetFileName(path);
                if (name.IndexOf(".vofa", StringComparison.OrdinalIgnoreCase) >= 0 || name.IndexOf(".pid", StringComparison.OrdinalIgnoreCase) >= 0) continue;
                int sessionId;
                string idText = Path.GetFileNameWithoutExtension(name).Replace("session_", "");
                if (!int.TryParse(idText, NumberStyles.Integer, CultureInfo.InvariantCulture, out sessionId)) continue;
                SessionState session = ReadSessionSummary(path, sessionId);
                sessions.Add(session);
            }
            sampleRecords = 0;
            foreach (SessionState session in sessions) sampleRecords += session.Samples;
            RefreshSessionView();
            UpdateProgress();
            AddLog("已加载 " + sessions.Count.ToString(CultureInfo.InvariantCulture) + " 场，目录 " + folder);
            statusLabel.Text = "已加载导出文件夹，选一场后点「看波形」或双击表格。";
        }

        private static SessionState ReadSessionSummary(string csvPath, int sessionId)
        {
            SessionState session = new SessionState();
            session.SessionId = sessionId;
            long samples = 0;
            long lastT = 0;
            using (StreamReader reader = new StreamReader(csvPath))
            {
                string line;
                while ((line = reader.ReadLine()) != null)
                {
                    if (line.StartsWith("# pid", StringComparison.Ordinal))
                    {
                        foreach (Match pair in pidPairRegex.Matches(line))
                        {
                            int id = 0;
                            int.TryParse(pair.Groups[1].Value, NumberStyles.Integer, CultureInfo.InvariantCulture, out id);
                            if (id >= 0 && id < session.Pid.Length) session.Pid[id] = pair.Groups[2].Value;
                        }
                    }
                    else if (line.StartsWith("# altitude", StringComparison.Ordinal))
                    {
                        string altitudeLine = line.Replace("vz_kp", "26").Replace("vz_ki", "27").Replace("vz_kd", "28").Replace("hover_pwm", "29").Replace("max_thr", "29");
                        foreach (Match pair in pidPairRegex.Matches(altitudeLine))
                        {
                            int id = 0;
                            int.TryParse(pair.Groups[1].Value, NumberStyles.Integer, CultureInfo.InvariantCulture, out id);
                            if (id >= 26 && id < session.Pid.Length) session.Pid[id] = pair.Groups[2].Value;
                        }
                    }
                    if (line.Length == 0 || line[0] == '#' || line.StartsWith("t_ms", StringComparison.Ordinal)) continue;
                    string[] parts = line.Split(',');
                    if (parts.Length < 15) continue;
                    long t;
                    if (long.TryParse(parts[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out t)) lastT = t;
                    samples++;
                }
            }
            string pidPath = Path.Combine(Path.GetDirectoryName(csvPath) ?? "", "session_" + sessionId.ToString("000", CultureInfo.InvariantCulture) + ".pid.csv");
            if (File.Exists(pidPath))
            {
                foreach (string pidLine in File.ReadAllLines(pidPath))
                {
                    string[] parts = pidLine.Split(',');
                    int id;
                    if (parts.Length >= 3 && int.TryParse(parts[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out id) && id >= 0 && id < session.Pid.Length)
                    {
                        session.Pid[id] = parts[2];
                    }
                }
            }
            session.Samples = samples;
            session.LastTms = lastT;
            session.Complete = samples > 0;
            return session;
        }

        private void OpenFolderButtonClick(object sender, EventArgs e)
        {
            string folder = !string.IsNullOrEmpty(currentExportDirectory) && Directory.Exists(currentExportDirectory) ? currentExportDirectory : Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "exports");
            Directory.CreateDirectory(folder);
            try { Process.Start(new ProcessStartInfo { FileName = "explorer.exe", Arguments = folder, UseShellExecute = true }); }
            catch (Exception ex) { MessageBox.Show(ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Error); }
        }

        private void OpenTextButtonClick(object sender, EventArgs e)
        {
            if (captureActive)
            {
                MessageBox.Show("正在读取，请先停止。", Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            using (OpenFileDialog dialog = new OpenFileDialog())
            {
                dialog.Filter = "文本|*.txt;*.log;*.csv|全部|*.*";
                dialog.Title = "打开已有 dump 文本";
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                try
                {
                    dumpallMode = Path.GetFileName(dialog.FileName).IndexOf("dumpall", StringComparison.OrdinalIgnoreCase) >= 0;
                    StartOfflineParse(dialog.FileName);
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        private void StartOfflineParse(string inputPath)
        {
            CloseWriters();
            sessions.Clear();
            currentSession = null;
            sessionView.Items.Clear();
            string sessionName = "offline_" + DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            currentExportDirectory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "exports", sessionName);
            Directory.CreateDirectory(currentExportDirectory);
            UTF8Encoding utf8 = new UTF8Encoding(false);
            File.Copy(inputPath, Path.Combine(currentExportDirectory, "serial_raw.log"), true);
            if (dumpallMode)
            {
                dumpallTextWriter = new StreamWriter(Path.Combine(currentExportDirectory, "dumpall_raw.txt"), false, utf8, 65536);
                dumpallBinWriter = new FileStream(Path.Combine(currentExportDirectory, "dumpall.bin"), FileMode.Create, FileAccess.Write, FileShare.Read);
            }
            else
            {
                allDataWriter = new StreamWriter(Path.Combine(currentExportDirectory, "blackbox_data.csv"), false, utf8, 65536);
                allDataWriter.WriteLine("session,t_ms,roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate,roll_corr,pitch_corr,yaw_corr,thr,m1,m2,m3,m4,alt_cm,vz_cms,mag_hdg,vz_corr_us,vz_target_cms,alt_state,armed,conn");
            }
            captureActive = true;
            dumpStarted = true;
            expectedRecords = 0;
            scannedRecords = 0;
            sampleRecords = 0;
            sessionRecords = 0;
            hexBytes = 0;
            unexpectedHexLines = 0;
            sawDumpDone = false;
            string text = File.ReadAllText(inputPath, Encoding.UTF8);
            receiveBuffer.Length = 0;
            receiveBuffer.Append(text.Replace("\r\n", "\n").Replace('\r', '\n'));
            if (!text.EndsWith("\n", StringComparison.Ordinal)) receiveBuffer.Append('\n');
            DrainLines(false);
            FinishCapture(true, "已解析 " + inputPath);
        }

        private void AddLog(string message)
        {
            logBox.AppendText("[" + DateTime.Now.ToString("HH:mm:ss", CultureInfo.InvariantCulture) + "] " + message + Environment.NewLine);
            logBox.SelectionStart = logBox.TextLength;
            logBox.ScrollToCaret();
        }

        private static long ParseLong(string value)
        {
            long result;
            return long.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out result) ? result : 0;
        }

        private static string NormalizeTimeText(string value)
        {
            if (string.IsNullOrEmpty(value) || value == "000000" || value == "0") return "NA";
            return value;
        }

        private void MainFormClosing(object sender, FormClosingEventArgs e)
        {
            receiveTimer.Stop();
            if (captureActive) FinishCapture(false, "程序已关闭");
            try { if (serialPort != null && serialPort.IsOpen) serialPort.Close(); } catch { }
            if (serialPort != null) serialPort.Dispose();
        }
    }

    internal static class Program
    {
        [STAThread]
        private static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.SetUnhandledExceptionMode(UnhandledExceptionMode.CatchException);
            Application.ThreadException += delegate(object sender, System.Threading.ThreadExceptionEventArgs args)
            {
                MessageBox.Show(args.Exception.ToString(), "ApexFly v2.0 黑匣子读取", MessageBoxButtons.OK, MessageBoxIcon.Error);
            };
            AppDomain.CurrentDomain.UnhandledException += delegate(object sender, UnhandledExceptionEventArgs args)
            {
                MessageBox.Show(Convert.ToString(args.ExceptionObject), "ApexFly v2.0 黑匣子读取", MessageBoxButtons.OK, MessageBoxIcon.Error);
            };
            try
            {
                Application.Run(new MainForm());
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString(), "ApexFly v2.0 黑匣子读取", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
