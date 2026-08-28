using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.IO;
using System.Windows.Forms;

namespace ApexFlyBlackboxReader
{
    internal sealed class WaveformForm : Form
    {
        private readonly string csvPath;
        private readonly string titleText;
        private readonly List<float> tMs = new List<float>();
        private readonly Dictionary<string, List<float>> series = new Dictionary<string, List<float>>();
        private readonly Dictionary<string, CheckBox> boxes = new Dictionary<string, CheckBox>();
        private readonly Dictionary<string, Color> colors = new Dictionary<string, Color>();
        private readonly Dictionary<string, string> labels = new Dictionary<string, string>();
        private readonly string[] pidValues = new string[30];

        private readonly PlotCanvas commandCanvas = new PlotCanvas();
        private readonly PlotCanvas attitudeCanvas = new PlotCanvas();
        private readonly SplitContainer split = new SplitContainer();
        private readonly HScrollBar timeScroll = new HScrollBar();
        private readonly Label infoLabel = new Label();
        private readonly Label pidLabel = new Label();
        private readonly Label titleLabel = new Label();

        private float viewStart;
        private float viewEnd;
        private float dataStart;
        private float dataEnd;
        private float commandYZoom = 1f;
        private float attitudeYZoom = 1f;
        private bool dragging;
        private int dragX;
        private float dragStart;
        private float dragEnd;
        private bool syncingScroll;
        private bool cursorVisible;
        private float cursorTime;
        private float[] bucketMin = new float[8];
        private float[] bucketMax = new float[8];
        private bool[] bucketHas = new bool[8];
        private readonly List<float> labelYs = new List<float>();

        private static readonly string[] ChannelOrder =
        {
            "roll", "pitch", "yaw",
            "roll_rate", "pitch_rate", "yaw_rate",
            "roll_corr", "pitch_corr", "yaw_corr",
            "thr", "m1", "m2", "m3", "m4",
            "alt_cm", "vz_cms", "mag_hdg", "vz_corr_us", "hover_us", "alt_state"
        };

        private static readonly string[] CommandChannels =
        {
            "thr", "m1", "m2", "m3", "m4", "roll_corr", "pitch_corr", "yaw_corr", "vz_corr_us", "hover_us", "alt_state"
        };

        private static readonly string[] AttitudeChannels =
        {
            "roll", "pitch", "yaw", "roll_rate", "pitch_rate", "yaw_rate", "alt_cm", "vz_cms", "mag_hdg"
        };

        public WaveformForm(string csvPath, string titleText)
        {
            this.csvPath = csvPath;
            this.titleText = titleText;
            Text = "波形 - " + titleText;
            ClientSize = new Size(1180, 760);
            MinimumSize = new Size(900, 560);
            StartPosition = FormStartPosition.CenterParent;
            Font = new Font("Microsoft YaHei UI", 9F);
            BackColor = Color.FromArgb(30, 33, 38);
            ForeColor = Color.White;
            DoubleBuffered = true;
            KeyPreview = true;
            LoadCsv();
            LoadPid();
            BuildUi();
            ResetView();
            Shown += delegate { if (split.Height > 80) split.SplitterDistance = Math.Max(120, (int)(split.Height * 0.38)); Redraw(); };
            KeyDown += FormKeyDown;
        }

        private static bool IsCommandChannel(string name)
        {
            return name == "thr" || name == "m1" || name == "m2" || name == "m3" || name == "m4"
                || name == "roll_corr" || name == "pitch_corr" || name == "yaw_corr"
                || name == "vz_corr_us" || name == "hover_us" || name == "alt_state";
        }

        private void LoadCsv()
        {
            foreach (string name in ChannelOrder) series[name] = new List<float>();
            using (StreamReader reader = new StreamReader(csvPath))
            {
                string line;
                while ((line = reader.ReadLine()) != null)
                {
                    line = line.Trim();
                    if (line.Length == 0 || line[0] == '#' || line.StartsWith("t_ms", StringComparison.Ordinal) || line.StartsWith("session,", StringComparison.Ordinal)) continue;
                    string[] parts = line.Split(',');
                    int offset = 0;
                    /* 合并表比单场 CSV 多一个 session 列；17/19/23 列是单场，18/20/24 列是合并表。 */
                    if ((parts.Length == 18) || (parts.Length == 20) || (parts.Length == 24)) offset = 1;
                    if (parts.Length < offset + 15) continue;
                    float t;
                    if (!float.TryParse(parts[offset], NumberStyles.Float, CultureInfo.InvariantCulture, out t)) continue;
                    tMs.Add(t);
                    AddValue("roll", parts, offset + 1);
                    AddValue("pitch", parts, offset + 2);
                    AddValue("yaw", parts, offset + 3);
                    AddValue("roll_rate", parts, offset + 4);
                    AddValue("pitch_rate", parts, offset + 5);
                    AddValue("yaw_rate", parts, offset + 6);
                    AddValue("roll_corr", parts, offset + 7);
                    AddValue("pitch_corr", parts, offset + 8);
                    AddValue("yaw_corr", parts, offset + 9);
                    AddValue("thr", parts, offset + 10);
                    AddValue("m1", parts, offset + 11);
                    AddValue("m2", parts, offset + 12);
                    AddValue("m3", parts, offset + 13);
                    AddValue("m4", parts, offset + 14);
                    if (parts.Length >= offset + 19)
                    {
                        AddValue("alt_cm", parts, offset + 15);
                        AddValue("vz_cms", parts, offset + 16);
                    }
                    else
                    {
                        series["alt_cm"].Add(0f);
                        series["vz_cms"].Add(0f);
                    }
                    if (parts.Length >= offset + 23)
                    {
                        AddValue("mag_hdg", parts, offset + 17);
                        AddValue("vz_corr_us", parts, offset + 18);
                        AddValue("hover_us", parts, offset + 19);
                        AddValue("alt_state", parts, offset + 20);
                    }
                    else
                    {
                        series["mag_hdg"].Add(0f);
                        series["vz_corr_us"].Add(0f);
                        series["hover_us"].Add(0f);
                        series["alt_state"].Add(0f);
                    }
                }
            }
            if (tMs.Count == 0) throw new InvalidOperationException("这个 CSV 里没有样本。");
            dataStart = tMs[0];
            dataEnd = tMs[tMs.Count - 1];
            if (dataEnd <= dataStart) dataEnd = dataStart + 1f;
        }

        private void AddValue(string name, string[] parts, int index)
        {
            float value;
            if (!float.TryParse(parts[index], NumberStyles.Float, CultureInfo.InvariantCulture, out value)) value = 0f;
            series[name].Add(value);
        }

        private void LoadPid()
        {
            string pidPath = csvPath;
            if (pidPath.EndsWith(".csv", StringComparison.OrdinalIgnoreCase)) pidPath = pidPath.Substring(0, pidPath.Length - 4) + ".pid.csv";
            else pidPath += ".pid.csv";
            if (!File.Exists(pidPath)) return;
            foreach (string line in File.ReadAllLines(pidPath))
            {
                string[] parts = line.Split(',');
                int id;
                if (parts.Length >= 3 && int.TryParse(parts[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out id) && id >= 0 && id < pidValues.Length)
                {
                    pidValues[id] = parts[2];
                }
            }
        }

        private string PidText()
        {
            string rollOut = PidOrDash(0);
            string rollIn = PidOrDash(3);
            string pitchOut = PidOrDash(6);
            string pitchIn = PidOrDash(9);
            string yawRate = PidOrDash(12);
            string yawAngle = PidOrDash(23);
            string vzPid = PidOrDash(26) + "/" + PidOrDash(27) + "/" + PidOrDash(28);
            return "PID  Roll 外" + rollOut + "/内" + rollIn + "    Pitch 外" + pitchOut + "/内" + pitchIn + "    Yaw角" + yawAngle + " 角速度" + yawRate + "    垂速" + vzPid + " 悬停" + PidOrDash(29);
        }

        private string PidOrDash(int id)
        {
            return string.IsNullOrEmpty(pidValues[id]) ? "-" : pidValues[id];
        }

        private void BuildUi()
        {
            colors["roll"] = Color.FromArgb(255, 140, 60);
            colors["pitch"] = Color.FromArgb(80, 200, 255);
            colors["yaw"] = Color.FromArgb(240, 220, 80);
            colors["roll_rate"] = Color.FromArgb(255, 90, 90);
            colors["pitch_rate"] = Color.FromArgb(90, 255, 160);
            colors["yaw_rate"] = Color.FromArgb(200, 160, 255);
            colors["roll_corr"] = Color.FromArgb(255, 110, 180);
            colors["pitch_corr"] = Color.FromArgb(110, 255, 220);
            colors["yaw_corr"] = Color.FromArgb(180, 180, 255);
            colors["thr"] = Color.Gainsboro;
            colors["m1"] = Color.FromArgb(255, 80, 80);
            colors["m2"] = Color.FromArgb(80, 255, 80);
            colors["m3"] = Color.FromArgb(80, 160, 255);
            colors["m4"] = Color.FromArgb(255, 220, 80);
            colors["alt_cm"] = Color.FromArgb(120, 220, 255);
            colors["vz_cms"] = Color.FromArgb(255, 180, 90);
            colors["mag_hdg"] = Color.FromArgb(120, 255, 150);
            colors["vz_corr_us"] = Color.FromArgb(255, 120, 180);
            colors["hover_us"] = Color.FromArgb(210, 210, 210);
            colors["alt_state"] = Color.FromArgb(180, 140, 255);

            labels["roll"] = "横滚角";
            labels["pitch"] = "俯仰角";
            labels["yaw"] = "偏航角";
            labels["roll_rate"] = "横滚角速度";
            labels["pitch_rate"] = "俯仰角速度";
            labels["yaw_rate"] = "偏航角速度";
            labels["roll_corr"] = "横滚修正";
            labels["pitch_corr"] = "俯仰修正";
            labels["yaw_corr"] = "偏航修正";
            labels["thr"] = "油门";
            labels["m1"] = "M1";
            labels["m2"] = "M2";
            labels["m3"] = "M3";
            labels["m4"] = "M4";
            labels["alt_cm"] = "高度cm";
            labels["vz_cms"] = "实际垂速";
            labels["mag_hdg"] = "磁航向";
            labels["vz_corr_us"] = "垂速修正";
            labels["hover_us"] = "悬停PWM";
            labels["alt_state"] = "高度状态";

            Panel top = new Panel();
            top.Dock = DockStyle.Top;
            top.Height = 132;
            top.BackColor = Color.FromArgb(24, 26, 30);
            Controls.Add(top);

            titleLabel.SetBounds(12, 6, 760, 20);
            titleLabel.Text = titleText + "    " + tMs.Count.ToString("N0", CultureInfo.InvariantCulture) + " 点    " + (dataEnd - dataStart).ToString("0", CultureInfo.InvariantCulture) + " ms";
            top.Controls.Add(titleLabel);

            pidLabel.SetBounds(12, 26, 900, 20);
            pidLabel.ForeColor = Color.FromArgb(210, 200, 140);
            pidLabel.Text = PidText();
            top.Controls.Add(pidLabel);

            string[] defaults = { "roll", "pitch", "roll_rate", "pitch_rate", "thr", "alt_cm" };
            HashSet<string> on = new HashSet<string>(defaults);
            int x = 12;
            int y = 50;
            for (int i = 0; i < ChannelOrder.Length; i++)
            {
                CheckBox box = new CheckBox();
                box.Text = labels[ChannelOrder[i]];
                box.ForeColor = colors[ChannelOrder[i]];
                box.Checked = on.Contains(ChannelOrder[i]);
                box.SetBounds(x, y, 92, 22);
                box.CheckedChanged += delegate { Redraw(); };
                top.Controls.Add(box);
                boxes[ChannelOrder[i]] = box;
                x += 94;
                if (x > 980)
                {
                    x = 12;
                    y += 24;
                }
            }

            int bx = 980;
            top.Controls.Add(MakeTopButton("全场", ref bx, delegate { ResetView(); Redraw(); }));
            top.Controls.Add(MakeTopButton("时间+", ref bx, delegate { ZoomTime(0.8f, (viewStart + viewEnd) * 0.5f); }));
            top.Controls.Add(MakeTopButton("时间-", ref bx, delegate { ZoomTime(1.25f, (viewStart + viewEnd) * 0.5f); }));

            infoLabel.Dock = DockStyle.Bottom;
            infoLabel.Height = 24;
            infoLabel.TextAlign = ContentAlignment.MiddleLeft;
            infoLabel.Padding = new Padding(8, 0, 0, 0);
            infoLabel.Text = "上图：油门/电机/修正    下图：姿态    时间轴共用。滚轮或时间±缩放，左键拖动，底栏滚动。";
            Controls.Add(infoLabel);

            timeScroll.Dock = DockStyle.Bottom;
            timeScroll.Height = 18;
            timeScroll.Scroll += TimeScrollChanged;
            Controls.Add(timeScroll);

            split.Dock = DockStyle.Fill;
            split.Orientation = Orientation.Horizontal;
            split.BackColor = Color.FromArgb(50, 54, 60);
            split.SplitterWidth = 8;
            Controls.Add(split);
            split.BringToFront();

            split.Panel1.Controls.Add(BuildPane(commandCanvas, "油门 / 电机 / 修正 (µs)", true));
            split.Panel2.Controls.Add(BuildPane(attitudeCanvas, "姿态 / 高度 / 升速", false));
        }

        private Button MakeTopButton(string text, ref int right, EventHandler click)
        {
            Button button = new Button();
            button.Text = text;
            button.SetBounds(right, 8, 72, 28);
            button.FlatStyle = FlatStyle.Flat;
            button.BackColor = Color.FromArgb(50, 56, 64);
            button.ForeColor = Color.White;
            button.Click += click;
            right -= 78;
            return button;
        }

        private Control BuildPane(PlotCanvas canvas, string caption, bool commandPane)
        {
            Panel pane = new Panel();
            pane.Dock = DockStyle.Fill;
            pane.BackColor = Color.FromArgb(14, 16, 18);

            Panel header = new Panel();
            header.Dock = DockStyle.Top;
            header.Height = 26;
            header.BackColor = Color.FromArgb(22, 24, 28);
            pane.Controls.Add(header);

            Label cap = new Label();
            cap.Text = caption;
            cap.SetBounds(8, 4, 420, 18);
            cap.ForeColor = Color.Silver;
            cap.Anchor = AnchorStyles.Top | AnchorStyles.Left;
            header.Controls.Add(cap);

            FlowLayoutPanel yButtons = new FlowLayoutPanel();
            yButtons.FlowDirection = FlowDirection.RightToLeft;
            yButtons.WrapContents = false;
            yButtons.Dock = DockStyle.Right;
            yButtons.Width = 160;
            yButtons.Padding = new Padding(0, 1, 6, 0);
            header.Controls.Add(yButtons);

            Button yReset = SmallHeaderButton("Y复位");
            Button yMinus = SmallHeaderButton("Y-");
            Button yPlus = SmallHeaderButton("Y+");
            yPlus.Click += delegate { ChangeYZoom(commandPane, 1.25f); };
            yMinus.Click += delegate { ChangeYZoom(commandPane, 0.8f); };
            yReset.Click += delegate
            {
                if (commandPane) commandYZoom = 1f;
                else attitudeYZoom = 1f;
                Redraw();
            };
            yButtons.Controls.Add(yReset);
            yButtons.Controls.Add(yMinus);
            yButtons.Controls.Add(yPlus);

            canvas.Dock = DockStyle.Fill;
            canvas.Paint += delegate(object sender, PaintEventArgs e) { PaintPane(e.Graphics, canvas, commandPane); };
            canvas.MouseWheel += PlotWheel;
            canvas.MouseDown += PlotDown;
            canvas.MouseMove += PlotMove;
            canvas.MouseUp += delegate { dragging = false; };
            canvas.MouseLeave += delegate { cursorVisible = false; Redraw(); };
            canvas.Resize += delegate { Redraw(); };
            pane.Controls.Add(canvas);
            canvas.BringToFront();
            return pane;
        }

        private Button SmallHeaderButton(string text)
        {
            Button button = new Button();
            button.Text = text;
            button.Width = text.Length > 2 ? 52 : 40;
            button.Height = 22;
            button.Margin = new Padding(3, 0, 0, 0);
            button.FlatStyle = FlatStyle.Flat;
            button.BackColor = Color.FromArgb(46, 50, 58);
            button.ForeColor = Color.White;
            return button;
        }

        private void ChangeYZoom(bool commandPane, float factor)
        {
            if (commandPane)
            {
                commandYZoom *= factor;
                if (commandYZoom < 0.25f) commandYZoom = 0.25f;
                if (commandYZoom > 20f) commandYZoom = 20f;
            }
            else
            {
                attitudeYZoom *= factor;
                if (attitudeYZoom < 0.25f) attitudeYZoom = 0.25f;
                if (attitudeYZoom > 20f) attitudeYZoom = 20f;
            }
            Redraw();
        }

        private void ResetView()
        {
            viewStart = dataStart;
            viewEnd = dataEnd;
            commandYZoom = 1f;
            attitudeYZoom = 1f;
        }

        private void Redraw()
        {
            UpdateTimeScroll();
            commandCanvas.Invalidate();
            attitudeCanvas.Invalidate();
        }

        private void FormKeyDown(object sender, KeyEventArgs e)
        {
            float span = viewEnd - viewStart;
            if (e.KeyCode == Keys.Left)
            {
                viewStart -= span * 0.1f;
                viewEnd -= span * 0.1f;
                ClampView();
                Redraw();
            }
            else if (e.KeyCode == Keys.Right)
            {
                viewStart += span * 0.1f;
                viewEnd += span * 0.1f;
                ClampView();
                Redraw();
            }
            else if (e.KeyCode == Keys.Oemplus || e.KeyCode == Keys.Add)
            {
                ZoomTime(0.8f, (viewStart + viewEnd) * 0.5f);
            }
            else if (e.KeyCode == Keys.OemMinus || e.KeyCode == Keys.Subtract)
            {
                ZoomTime(1.25f, (viewStart + viewEnd) * 0.5f);
            }
        }

        private void TimeScrollChanged(object sender, ScrollEventArgs e)
        {
            if (syncingScroll) return;
            float span = viewEnd - viewStart;
            viewStart = dataStart + e.NewValue;
            viewEnd = viewStart + span;
            ClampView();
            commandCanvas.Invalidate();
            attitudeCanvas.Invalidate();
        }

        private void UpdateTimeScroll()
        {
            int spanMs = Math.Max(1, (int)Math.Round(viewEnd - viewStart));
            int totalMs = Math.Max(1, (int)Math.Round(dataEnd - dataStart));
            int maxStart = Math.Max(0, totalMs - spanMs);
            syncingScroll = true;
            timeScroll.Minimum = 0;
            timeScroll.SmallChange = Math.Max(1, spanMs / 20);
            timeScroll.LargeChange = Math.Max(1, spanMs);
            timeScroll.Maximum = maxStart + timeScroll.LargeChange - 1;
            int value = (int)Math.Round(viewStart - dataStart);
            if (value < 0) value = 0;
            if (value > maxStart) value = maxStart;
            if (timeScroll.Maximum < timeScroll.LargeChange) timeScroll.Maximum = timeScroll.LargeChange;
            timeScroll.Value = Math.Min(value, Math.Max(0, timeScroll.Maximum - timeScroll.LargeChange + 1));
            timeScroll.Enabled = maxStart > 0;
            syncingScroll = false;
        }

        private void PlotDown(object sender, MouseEventArgs e)
        {
            PlotCanvas canvas = (PlotCanvas)sender;
            canvas.Focus();
            if (e.Button != MouseButtons.Left) return;
            dragging = true;
            dragX = e.X;
            dragStart = viewStart;
            dragEnd = viewEnd;
        }

        private void PlotMove(object sender, MouseEventArgs e)
        {
            PlotCanvas canvas = (PlotCanvas)sender;
            Rectangle plot = PlotRect(canvas);
            if (plot.Width <= 0) return;
            cursorTime = XToTime(e.X, plot);
            cursorVisible = e.X >= plot.Left && e.X <= plot.Right;
            infoLabel.Text = "t=" + cursorTime.ToString("0", CultureInfo.InvariantCulture) + " ms    上图油门/电机    下图姿态    时间窗口 "
                + viewStart.ToString("0", CultureInfo.InvariantCulture) + " ~ " + viewEnd.ToString("0", CultureInfo.InvariantCulture) + " ms";
            if (dragging)
            {
                float span = dragEnd - dragStart;
                float dt = -(e.X - dragX) / (float)plot.Width * span;
                viewStart = dragStart + dt;
                viewEnd = dragEnd + dt;
                ClampView();
                Redraw();
                return;
            }
            commandCanvas.Invalidate();
            attitudeCanvas.Invalidate();
        }

        private void PlotWheel(object sender, MouseEventArgs e)
        {
            PlotCanvas canvas = (PlotCanvas)sender;
            Rectangle plot = PlotRect(canvas);
            if (plot.Width <= 0) return;
            float t = XToTime(e.X, plot);
            ZoomTime(e.Delta > 0 ? 0.8f : 1.25f, t);
        }

        private void ZoomTime(float factor, float centerT)
        {
            float span = viewEnd - viewStart;
            if (span < 1f) span = 1f;
            float newSpan = span * factor;
            float minSpan = Math.Max(20f, (dataEnd - dataStart) / 400f);
            if (newSpan < minSpan) newSpan = minSpan;
            if (newSpan > (dataEnd - dataStart)) newSpan = dataEnd - dataStart;
            float ratio = (centerT - viewStart) / span;
            if (ratio < 0f) ratio = 0f;
            if (ratio > 1f) ratio = 1f;
            viewStart = centerT - newSpan * ratio;
            viewEnd = viewStart + newSpan;
            ClampView();
            Redraw();
        }

        private void ClampView()
        {
            float span = viewEnd - viewStart;
            if (span < 1f) span = 1f;
            if (viewStart < dataStart)
            {
                viewStart = dataStart;
                viewEnd = viewStart + span;
            }
            if (viewEnd > dataEnd)
            {
                viewEnd = dataEnd;
                viewStart = viewEnd - span;
                if (viewStart < dataStart) viewStart = dataStart;
            }
        }

        private static Rectangle PlotRect(Control canvas)
        {
            return new Rectangle(56, 8, Math.Max(10, canvas.ClientSize.Width - 12), Math.Max(10, canvas.ClientSize.Height - 28));
        }

        private float XToTime(int x, Rectangle plot)
        {
            float u = (x - plot.Left) / (float)plot.Width;
            if (u < 0f) u = 0f;
            if (u > 1f) u = 1f;
            return viewStart + u * (viewEnd - viewStart);
        }

        private void PaintPane(Graphics g, PlotCanvas canvas, bool commandPane)
        {
            g.SmoothingMode = SmoothingMode.None;
            g.Clear(Color.FromArgb(14, 16, 18));
            Rectangle plot = PlotRect(canvas);
            using (Brush back = new SolidBrush(Color.FromArgb(18, 20, 23)))
            {
                g.FillRectangle(back, plot);
            }
            g.DrawRectangle(Pens.DimGray, plot);

            string[] names = commandPane ? CommandChannels : AttitudeChannels;
            float yZoom = commandPane ? commandYZoom : attitudeYZoom;
            float yMin;
            float yMax;
            bool any = ComputeRange(names, out yMin, out yMax);
            if (!any)
            {
                using (Brush brush = new SolidBrush(Color.Gray))
                    g.DrawString(commandPane ? "勾选油门 / 电机 / 修正" : "勾选姿态角 / 角速度", Font, brush, plot.Left + 12, plot.Top + 12);
                DrawTimeGrid(g, plot, false);
                DrawCursor(g, plot);
                return;
            }
            ApplyYZoom(ref yMin, ref yMax, yZoom, commandPane);

            DrawTimeGrid(g, plot, true);
            DrawYGrid(g, plot, yMin, yMax);

            labelYs.Clear();
            foreach (string name in names)
            {
                if (!boxes[name].Checked) continue;
                DrawSeries(g, series[name], colors[name], plot, yMin, yMax);
                DrawLineLabel(g, name, series[name], plot, yMin, yMax);
            }
            DrawCursor(g, plot);
        }

        private bool ComputeRange(string[] names, out float yMin, out float yMax)
        {
            yMin = 0f;
            yMax = 1f;
            bool any = false;
            int first = IndexAtOrAfter(viewStart);
            int last = IndexAtOrAfter(viewEnd);
            if (last >= tMs.Count) last = tMs.Count - 1;
            if (first < 0) first = 0;
            foreach (string name in names)
            {
                if (!boxes[name].Checked) continue;
                List<float> values = series[name];
                for (int i = first; i <= last; i++)
                {
                    float v = values[i];
                    if (!any)
                    {
                        yMin = v;
                        yMax = v;
                        any = true;
                    }
                    else
                    {
                        if (v < yMin) yMin = v;
                        if (v > yMax) yMax = v;
                    }
                }
            }
            return any;
        }

        private static void ApplyYZoom(ref float yMin, ref float yMax, float yZoom, bool commandPane)
        {
            if (Math.Abs(yMax - yMin) < 0.001f)
            {
                yMin -= 1f;
                yMax += 1f;
            }
            float pad = (yMax - yMin) * 0.08f;
            yMin -= pad;
            yMax += pad;
            float center;
            if (!commandPane && yMin <= 0f && yMax >= 0f) center = 0f;
            else center = (yMin + yMax) * 0.5f;
            float half = (yMax - yMin) * 0.5f / yZoom;
            yMin = center - half;
            yMax = center + half;
        }

        private void DrawTimeGrid(Graphics g, Rectangle plot, bool withLabels)
        {
            using (Pen grid = new Pen(Color.FromArgb(40, 50, 55)))
            using (Brush text = new SolidBrush(Color.Silver))
            {
                for (int i = 0; i <= 6; i++)
                {
                    float t = viewStart + (viewEnd - viewStart) * i / 6f;
                    int x = plot.Left + (int)(plot.Width * i / 6f);
                    g.DrawLine(grid, x, plot.Top, x, plot.Bottom);
                    if (withLabels) g.DrawString(t.ToString("0", CultureInfo.InvariantCulture) + "ms", Font, text, x - 18, plot.Bottom + 4);
                }
            }
        }

        private void DrawYGrid(Graphics g, Rectangle plot, float yMin, float yMax)
        {
            using (Pen grid = new Pen(Color.FromArgb(40, 50, 55)))
            using (Brush text = new SolidBrush(Color.Silver))
            {
                for (int i = 0; i <= 4; i++)
                {
                    float yVal = yMax - (yMax - yMin) * i / 4f;
                    int y = plot.Top + (int)(plot.Height * i / 4f);
                    g.DrawLine(grid, plot.Left, y, plot.Right, y);
                    string textVal = Math.Abs(yVal) >= 100f ? yVal.ToString("0", CultureInfo.InvariantCulture) : yVal.ToString("0.00", CultureInfo.InvariantCulture);
                    g.DrawString(textVal, Font, text, 4, y - 8);
                }
            }
        }

        private void DrawCursor(Graphics g, Rectangle plot)
        {
            if (!cursorVisible) return;
            int x = plot.Left + (int)((cursorTime - viewStart) / (viewEnd - viewStart) * plot.Width);
            if (x < plot.Left || x > plot.Right) return;
            using (Pen pen = new Pen(Color.FromArgb(180, 255, 255, 255)))
            {
                pen.DashStyle = DashStyle.Dot;
                g.DrawLine(pen, x, plot.Top, x, plot.Bottom);
            }
        }

        private void EnsureBuckets(int width)
        {
            if (bucketMin.Length >= width) return;
            int size = Math.Max(width, bucketMin.Length * 2);
            bucketMin = new float[size];
            bucketMax = new float[size];
            bucketHas = new bool[size];
        }

        private void DrawSeries(Graphics g, List<float> values, Color color, Rectangle plot, float yMin, float yMax)
        {
            int w = plot.Width;
            if (w <= 0) return;
            float span = viewEnd - viewStart;
            if (span <= 0f) return;
            EnsureBuckets(w);
            Array.Clear(bucketHas, 0, w);

            int first = IndexAtOrAfter(viewStart);
            if (first > 0) first--;
            int last = IndexAtOrAfter(viewEnd);
            if (last < tMs.Count - 1) last++;
            if (last >= tMs.Count) last = tMs.Count - 1;
            if (first < 0) first = 0;

            for (int i = first; i <= last; i++)
            {
                int x = (int)((tMs[i] - viewStart) / span * w);
                if (x < 0 || x >= w) continue;
                float v = values[i];
                if (!bucketHas[x])
                {
                    bucketMin[x] = v;
                    bucketMax[x] = v;
                    bucketHas[x] = true;
                }
                else
                {
                    if (v < bucketMin[x]) bucketMin[x] = v;
                    if (v > bucketMax[x]) bucketMax[x] = v;
                }
            }

            using (Pen pen = new Pen(color, 1.4f))
            {
                int prevX = -1;
                int prevY0 = 0;
                int prevY1 = 0;
                for (int x = 0; x < w; x++)
                {
                    if (!bucketHas[x]) continue;
                    int y0 = ValueToY(bucketMax[x], plot, yMin, yMax);
                    int y1 = ValueToY(bucketMin[x], plot, yMin, yMax);
                    int px = plot.Left + x;
                    g.DrawLine(pen, px, y0, px, y1);
                    if (prevX >= 0)
                    {
                        g.DrawLine(pen, prevX, prevY0, px, y0);
                        if (prevY0 != prevY1 || y0 != y1) g.DrawLine(pen, prevX, prevY1, px, y1);
                    }
                    prevX = px;
                    prevY0 = y0;
                    prevY1 = y1;
                }
            }
        }

        private void DrawLineLabel(Graphics g, string name, List<float> values, Rectangle plot, float yMin, float yMax)
        {
            int last = IndexAtOrAfter(viewEnd);
            if (last >= tMs.Count) last = tMs.Count - 1;
            if (last < 0) return;
            while (last > 0 && tMs[last] > viewEnd) last--;
            if (last < 0 || last >= values.Count) return;
            float yVal = values[last];
            float ly = ValueToY(yVal, plot, yMin, yMax) - 8;
            if (ly < plot.Top) ly = plot.Top;
            if (ly > plot.Bottom - 16) ly = plot.Bottom - 16;
            foreach (float used in labelYs)
            {
                if (Math.Abs(used - ly) < 14f) ly = used + 14f;
            }
            if (ly > plot.Bottom - 16) ly = plot.Bottom - 16;
            labelYs.Add(ly);
            string text = labels[name];
            SizeF size = g.MeasureString(text, Font);
            float lx = plot.Right - size.Width - 6f;
            using (Brush back = new SolidBrush(Color.FromArgb(160, 10, 12, 14)))
            using (Brush brush = new SolidBrush(colors[name]))
            {
                g.FillRectangle(back, lx - 2f, ly, size.Width + 4f, size.Height);
                g.DrawString(text, Font, brush, lx, ly);
            }
        }

        private static int ValueToY(float value, Rectangle plot, float yMin, float yMax)
        {
            float u = (value - yMin) / (yMax - yMin);
            int y = plot.Bottom - (int)(u * plot.Height);
            if (y < plot.Top) y = plot.Top;
            if (y > plot.Bottom) y = plot.Bottom;
            return y;
        }

        private int IndexAtOrAfter(float t)
        {
            int lo = 0;
            int hi = tMs.Count - 1;
            while (lo < hi)
            {
                int mid = (lo + hi) / 2;
                if (tMs[mid] < t) lo = mid + 1;
                else hi = mid;
            }
            return lo;
        }

        private sealed class PlotCanvas : Panel
        {
            public PlotCanvas()
            {
                DoubleBuffered = true;
                ResizeRedraw = true;
                SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
                UpdateStyles();
                BackColor = Color.FromArgb(14, 16, 18);
                TabStop = true;
            }

            protected override void OnMouseEnter(EventArgs e)
            {
                base.OnMouseEnter(e);
                Focus();
            }
        }
    }
}
