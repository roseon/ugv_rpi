using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using CommandCenter.Core;

namespace CommandCenter;

public partial class MainWindow : Window
{
    readonly RobotClient _robot = new();

    public MainWindow()
    {
        InitializeComponent();
        ConnText.DataContext = _robot.State;
        ConnText.SetBinding(TextBlock.TextProperty, new System.Windows.Data.Binding("ConnText"));
        ConnText.SetBinding(TextBlock.ForegroundProperty, new System.Windows.Data.Binding("ConnBrush"));
        ConnDetail.DataContext = _robot.State;
        ConnDetail.SetBinding(TextBlock.TextProperty, new System.Windows.Data.Binding("ConnDetail"));

        // Pre-fill from the address the robot last answered on, so the app does
        // not auto-connect to a machine that moved networks months ago.
        HostBox.Text = AppSettings.LoadHost();

        DriveTab.Init(_robot);
        RadarTab.Init(_robot);
        CamerasTab.Init(_robot);
        LearningTab.Init(_robot);
        ChatTab.Init(_robot);
        Face.Init(_robot);

        _robot.State.PropertyChanged += (_, e) =>
        {
            if (e.PropertyName == nameof(RobotState.LidarHwText))
                Dispatcher.Invoke(() => LidarWarn.Text = _robot.State.LidarHw ? "" : "⚠ LIDAR silent — check sensor power/cable");
        };

        Closed += async (_, _) => { await _robot.DisposeAsync(); };
        ConnectBtn_Click(null, null);   // auto-connect to the remembered host
    }

    void OnConnect(object sender, RoutedEventArgs e) => ConnectBtn_Click(sender, e);

    void ConnectBtn_Click(object? sender, RoutedEventArgs? e)
    {
        var url = HostBox.Text.Trim();
        if (string.IsNullOrWhiteSpace(url)) return;
        _robot.Connect(url);
        ConnectBtn.Content = "Reconnect";
        _ = LoadVolumeAsync();
    }

    // ── the robot's output volume ────────────────────────────────────────────
    // The robot has one loudness, and it is the sink's, not this app's: the
    // slider asks /volume and shows what the robot reports back, so the two
    // never disagree.  Until the robot has answered, moving the slider means
    // nothing — otherwise the control would push its initial 0 and mute the
    // robot the moment the window opened.
    bool _volumeReady;
    int _volumeGen;

    async Task LoadVolumeAsync()
    {
        var now = await _robot.GetVolumeAsync();
        Dispatcher.Invoke(() =>
        {
            if (now is int pct) VolumeSlider.Value = pct;
            VolumeText.Text = now is int v ? $"{v}%" : "—";
            _volumeReady = true;
        });
    }

    void OnVolumeChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (!_volumeReady) return;
        int pct = (int)Math.Round(e.NewValue);
        VolumeText.Text = $"{pct}%";
        // A drag fires this per pixel; only the level it settles on is sent, so
        // one gesture is one command instead of forty.
        int gen = ++_volumeGen;
        _ = Task.Run(async () =>
        {
            await Task.Delay(200);
            if (Volatile.Read(ref _volumeGen) != gen) return;
            try { await _robot.SetVolumeAsync(pct); }
            catch { /* offline: the slider keeps the user's intent */ }
        });
    }
}
