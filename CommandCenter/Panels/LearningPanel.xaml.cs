using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using CommandCenter.Core;
using CommandCenter.Learning;

namespace CommandCenter.Panels;

public partial class LearningPanel : UserControl
{
    RobotClient _robot = null!;
    AutoPilot _pilot = null!;
    readonly ColorBlobTracker _tracker = new();
    DateTime _lastFeed = DateTime.MinValue;

    // the robot's map, as last fetched from it
    readonly List<(double x, double y, int hits)> _cells = new();
    const int MaxDrawnCells = 6000;
    double _poseX, _poseY, _poseTheta;
    int _busy = -1;
    string _motion = "?";
    bool _fetching;
    string _fetchError = "waiting for the robot…";
    DateTime _lastFetch = DateTime.MinValue;
    DateTime _lastFrame = DateTime.MinValue;

    // recording state
    bool _recording;
    double _recL, _recR;
    DateTime _recStepStart;
    readonly List<PathPlan.PathStep> _recorded = new();

    DispatcherTimer? _ui;

    public LearningPanel() => InitializeComponent();

    public void Init(RobotClient robot)
    {
        _robot = robot;
        _pilot = new AutoPilot(
            (l, r) => Task.Run(() => _robot.Drive(l, r)),
            () => _robot.State.LidarScan.Select(t => (t.AngleRad, t.DistMm)).ToArray(),
            () => _tracker.LatestBoxes);

        // feed the color-blob tracker from the live video (throttled inside)
        _robot.VideoFrame += f =>
        {
            if ((DateTime.UtcNow - _lastFeed).TotalMilliseconds > 100)
            {
                _lastFeed = DateTime.UtcNow;
                Task.Run(() => _tracker.Feed(f));
            }
        };

        _ui = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(160) };
        _ui.Tick += (_, _) =>
        {
            if ((DateTime.UtcNow - _lastFetch).TotalMilliseconds > 400) FetchMap();
            DrawMemory();
            DecisionText.Text = _decision;
            RobotStateText.Text = _selfDrive
                ? "robot self-drive: engaged"
                : "robot self-drive: stopped";
            AvoidBtn.Content = _avoidance ? "🛡 Avoidance: on" : "🛡 Avoidance: off";
            GoBtn.Content = _selfDrive ? "🧠 Engaged…" : "🧠 Engage";
            FollowBtn.Content = _pilot.CurrentMode == AutoPilot.Mode.FollowObject ? "🎯 Engaged…" : "🎯 Engage";
            TrackerInfo.Text = _tracker.LatestBoxes.Length > 0
                ? $"target locked ({_tracker.LastBlobPixels} px blob)"
                : $"searching hue {HueSlider.Value:0}°…";
        };

        HueSlider.ValueChanged += (_, _) =>
        {
            var c = HsvToRgb(HueSlider.Value, 1, 1);
            ((SolidColorBrush)HuePreview.Fill).Color = Color.FromRgb(c.r, c.g, c.b);
            _tracker.TargetHue = HueSlider.Value;
        };
        _ui.Start();

        LoadPathList();
        FetchMap();
    }

    // ───────── the robot's own state and map ─────────
    bool _selfDrive, _avoidance;
    string _decision = "—";

    async void FetchMap()
    {
        if (_fetching) return;
        _fetching = true;
        try
        {
            var sur = await _robot.GetSurroundingsAsync();
            var st = await _robot.GetSelfDriveStatusAsync();
            var lidar = await _robot.GetLidarStatusAsync();

            _busy = sur.TryGetProperty("busy_cells", out var bc) ? bc.GetInt32() : -1;
            if (sur.TryGetProperty("map", out var map))
            {
                _motion = map.TryGetProperty("motion", out var m) ? (m.GetString() ?? "?") : "?";
                if (map.TryGetProperty("pose", out var p) && p.GetArrayLength() == 3)
                {
                    _poseX = p[0].GetDouble();
                    _poseY = p[1].GetDouble();
                    _poseTheta = p[2].GetDouble();
                }
            }

            // Places, drawn in the map's own frame: the robot is at its pose, so a
            // wall keeps one cell however the robot turns.  One-hit cells are left
            // out of the drawing the way the map leaves them out of its memory.
            var fresh = new List<(double x, double y, int hits)>();
            if (sur.TryGetProperty("cells", out var arr))
                foreach (var c in arr.EnumerateArray())
                {
                    int hits = c.GetProperty("hits").GetInt32();
                    if (hits >= 2)
                        fresh.Add((c.GetProperty("x").GetDouble(),
                                   c.GetProperty("y").GetDouble(), hits));
                }
            fresh.Sort((a, b) => b.hits.CompareTo(a.hits));
            if (fresh.Count > MaxDrawnCells) fresh.RemoveRange(MaxDrawnCells, fresh.Count - MaxDrawnCells);

            _selfDrive = st.TryGetProperty("active", out var act) && act.GetBoolean();
            _decision = st.TryGetProperty("decision", out var dec)
                ? (dec.ValueKind == System.Text.Json.JsonValueKind.String
                    ? (dec.GetString() ?? "—")
                    : dec.ToString())
                : "—";
            _avoidance = lidar.TryGetProperty("avoidance_active", out var av) && av.GetBoolean();

            _cells.Clear();
            _cells.AddRange(fresh);
            _fetchError = "";
            _lastFetch = DateTime.UtcNow;
        }
        catch (Exception ex)
        {
            // Keep the last view, but say so: a stale panel that looks live is
            // exactly what "the detection is live" got wrong on the camera side.
            _fetchError = "robot map unavailable (" + ex.Message + ")";
        }
        finally { _fetching = false; }
    }

    // ───────── recording ─────────
    void OnRecord(object sender, RoutedEventArgs e)
    {
        if (!_recording)
        {
            _recorded.Clear();
            _recL = _recR = 0; _recStepStart = DateTime.UtcNow;
            _recording = true;
            RecBtn.Content = "⏹ Stop Recording";
            RecInfo.Text = "recording… drive the robot";
            _robot.CommandSent += OnCmdForRecording;
            StepList.ItemsSource = null;
        }
        else
        {
            _recording = false;
            _robot.CommandSent -= OnCmdForRecording;
            FlushStep(force: true);
            RecBtn.Content = "⏺ Start Recording";
            RecInfo.Text = $"{_recorded.Count} steps captured — refine & save";
            RenderSteps();
        }
    }

    void OnCmdForRecording(double l, double r)
    {
        if (!_recording) return;
        if (Math.Abs(l - _recL) > 0.02 || Math.Abs(r - _recR) > 0.02)
        {
            FlushStep();
            _recL = l; _recR = r;
            _recStepStart = DateTime.UtcNow;
        }
    }

    void FlushStep(bool force = false)
    {
        double secs = (DateTime.UtcNow - _recStepStart).TotalSeconds;
        if (force || secs > 0.05) _recorded.Add(new PathPlan.PathStep(_recL, _recR, Math.Max(0.05, secs)));
    }

    void RenderSteps()
    {
        StepList.ItemsSource = _recorded
            .Select((s, i) => $"{i,3}: L {s.L:+0.00;-0.00}  R {s.R:+0.00;-0.00}  {s.Seconds,5:0.00}s")
            .ToList();
    }

    void OnSave(object sender, RoutedEventArgs e)
    {
        if (_recorded.Count == 0) { MessageBox.Show("Nothing recorded yet."); return; }
        FlushStep(force: true);
        var raw = new PathPlan { Name = $"teach_{DateTime.Now:HHmmss}", Steps = _recorded.ToList() };
        var refined = raw.Refined();
        raw.Save(); refined.Save();
        LoadPathList();
        RecInfo.Text = $"saved: {raw.Name} ({raw.Steps.Count} steps) + refined ({refined.Steps.Count} steps)";
        _recorded.Clear();
        RenderSteps();
    }

    void LoadPathList()
    {
        var items = PathPlan.Saved().Select(p => System.IO.Path.GetFileName(p.name)).ToList();
        PathBox.ItemsSource = items;
        if (items.Count > 0) PathBox.SelectedIndex = 0;
    }

    // ───────── modes ─────────
    async void OnReplay(object sender, RoutedEventArgs e)
    {
        if (PathBox.SelectedItem is not string name) { MessageBox.Show("Pick a saved path first."); return; }
        var file = PathPlan.Saved().First(p => p.name == name + ".json").file;
        var plan = PathPlan.Load(file);
        if (plan == null) return;
        _pilot.Start(AutoPilot.Mode.Replay, plan);
        DecisionText.Text = "replay started";
        await Task.CompletedTask;
    }

    /// <summary>Engage the robot's own self-drive, with avoidance under it.</summary>
    async void OnReactive(object sender, RoutedEventArgs e)
    {
        try
        {
            await _robot.SetAvoidanceAsync(true);
            await _robot.SetSelfDriveAsync(true);
            _avoidance = _selfDrive = true;
            DecisionText.Text = "engaging the robot's self-drive…";
            FetchMap();
        }
        catch (Exception ex) { DecisionText.Text = "could not engage: " + ex.Message; }
    }

    async void OnToggleAvoidance(object sender, RoutedEventArgs e)
    {
        try
        {
            bool on = !_avoidance;
            await _robot.SetAvoidanceAsync(on);
            _avoidance = on;
            DecisionText.Text = on ? "avoidance on — the robot is learning while you drive"
                                   : "avoidance off";
        }
        catch (Exception ex) { DecisionText.Text = "could not switch avoidance: " + ex.Message; }
    }

    /// <summary>Stop means stop: avoidance drives the robot too, on its own.</summary>
    async void OnStopAuto(object sender, RoutedEventArgs e)
    {
        try
        {
            await _robot.SetSelfDriveAsync(false);
            // Measured while testing this panel: stopping self-drive alone left
            // avoidance on, the avoider kept cruising (23.5 m of wheel travel
            // afterwards) and the panel still said "parked".
            await _robot.SetAvoidanceAsync(false);
            _selfDrive = _avoidance = false;
        }
        catch (Exception ex) { DecisionText.Text = "stop failed: " + ex.Message; return; }
        _pilot.Stop();
        DecisionText.Text = "robot stopped — self-drive and avoidance off";
    }

    async void OnSaveMap(object sender, RoutedEventArgs e)
    {
        try
        {
            var r = await _robot.SaveMapAsync();
            MemInfo.Text = $"map saved — {r.GetProperty("busy_cells").GetInt32()} cells on the robot";
        }
        catch (Exception ex) { MemInfo.Text = "save failed: " + ex.Message; }
    }

    async void OnClearMap(object sender, RoutedEventArgs e)
    {
        if (MessageBox.Show("Clear the robot's learned map and start the place frame again?",
                            "Clear map", MessageBoxButton.OKCancel) != MessageBoxResult.OK) return;
        try
        {
            await _robot.ClearMapAsync();
            _cells.Clear();
            MemInfo.Text = "map cleared on the robot";
        }
        catch (Exception ex) { MemInfo.Text = "clear failed: " + ex.Message; }
    }

    void OnFollow(object sender, RoutedEventArgs e)
    {
        _tracker.TargetHue = HueSlider.Value;
        _tracker.Enabled = true;
        _pilot.Start(AutoPilot.Mode.FollowObject);
        DecisionText.Text = $"follow-object engaged (hue {HueSlider.Value:0}°)";
    }

    static (byte r, byte g, byte b) HsvToRgb(double h, double s, double v)
    {
        double c = v * s, x = c * (1 - Math.Abs(h / 60.0 % 2 - 1)), m = v - c;
        double rr, gg, bb;
        if (h < 60) { rr = c; gg = x; bb = 0; }
        else if (h < 120) { rr = x; gg = c; bb = 0; }
        else if (h < 180) { rr = 0; gg = c; bb = x; }
        else if (h < 240) { rr = 0; gg = x; bb = c; }
        else if (h < 300) { rr = x; gg = 0; bb = c; }
        else { rr = c; gg = 0; bb = x; }
        return ((byte)((rr + m) * 255), (byte)((gg + m) * 255), (byte)((bb + m) * 255));
    }

    // ───────── memory map, as the robot holds it ─────────
    void DrawMemory()
    {
        double w = MemCanvas.ActualWidth, h = MemCanvas.ActualHeight;
        if (w < 40 || h < 40) return;
        MemCanvas.Children.Clear();
        double span = 2 * 6.0;                       // the grid is ±6 m
        double scale = Math.Min(w, h) / span * 0.95;

        foreach (var (mx, my, hits) in _cells)
        {
            byte a = (byte)Math.Min(230, 60 + hits * 18);
            var cell = new Rectangle
            {
                Width = Math.Max(2, scale * 0.10 + 1),
                Height = Math.Max(2, scale * 0.10 + 1),
                Fill = new SolidColorBrush(Color.FromArgb(a, 0xFF, 0x5A, 0x5A))
            };
            Canvas.SetLeft(cell, w / 2 + mx * scale);
            Canvas.SetTop(cell, h / 2 - my * scale);
            MemCanvas.Children.Add(cell);
        }

        // where the map says the robot is, facing where the map says it faces
        double rx = w / 2 + _poseX * scale, ry = h / 2 - _poseY * scale;
        MemCanvas.Children.Add(new Ellipse
        {
            Width = 9, Height = 9, Fill = Brushes.White
        });
        Canvas.SetLeft(MemCanvas.Children[^1], rx - 4.5);
        Canvas.SetTop(MemCanvas.Children[^1], ry - 4.5);
        MemCanvas.Children.Add(new Line
        {
            X1 = rx, Y1 = ry,
            X2 = rx + 18 * Math.Sin(_poseTheta), Y2 = ry - 18 * Math.Cos(_poseTheta),
            Stroke = Brushes.White, StrokeThickness = 2
        });

        MemInfo.Text = _busy < 0
            ? _fetchError
            : $"{_busy} cells remembered · robot at ({_poseX:0.0}, {_poseY:0.0}) m in the map · {_motion} · {_robot.State.LidarSummary}"
              + (_fetchError.Length > 0 ? "  ⚠ " + _fetchError : "");
    }
}
