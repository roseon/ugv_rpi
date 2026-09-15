using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using CommandCenter.Core;

namespace CommandCenter.Panels;

public partial class CamerasPanel : UserControl
{
    RobotClient _robot = null!;
    MjpegStreamer? _viewB;

    // One palette for the overlay, matching the rest of the HUD.
    static readonly SolidColorBrush Ink = new(Color.FromRgb(0x4F, 0xF5, 0xC0));
    static readonly SolidColorBrush Warn = new(Color.FromRgb(0xF5, 0xBD, 0x5F));
    static readonly SolidColorBrush Bad = new(Color.FromRgb(0xFF, 0x93, 0x93));

    public CamerasPanel() => InitializeComponent();

    public void Init(RobotClient robot)
    {
        _robot = robot;
        robot.VideoFrame += f => Dispatcher.Invoke(() =>
        {
            ViewA.Source = f;
            DrawDetections();      // the frame and its boxes are one picture; keep them in step
        });
        robot.CamerasChanged += () => Dispatcher.Invoke(() =>
        {
            CamList.ItemsSource = null;
            CamList.ItemsSource = robot.State.Cameras;
            var act = robot.State.Cameras.FirstOrDefault(c => c.InUse);
            CamStatus.Text = act != null ? $"ACTIVE: /dev/video{act.Index}" : "";
        });
        robot.EyesChanged += () => Dispatcher.Invoke(() =>
        {
            UpdateVision(robot.State.Eyes);
            DrawDetections();
        });

        // The boxes are fractions of the frame, so a resize moves them.
        ViewAHost.SizeChanged += (_, _) => DrawDetections();

        // second, independent MJPEG connection for VIEW B
        _viewB = new MjpegStreamer(robot.State.BaseUrl + "/video_feed2");
        _viewB.Frame += f => Dispatcher.Invoke(() => ViewB.Source = f);
        _viewB.Start();

        _ = robot.RefreshCamerasAsync();
        UpdateVision(robot.State.Eyes);
    }

    void UpdateVision(EyesStatus e)
    {
        VisionHealth.Text = e.Health;
        VisionHealth.Foreground = e.Healthy ? Ink : e.Available ? Warn : Bad;
        VisionCam.Text = e.CameraText;
        VisionCam.Foreground = e.ModelMissing ? Bad : Ink;
        VisionGaze.Text = e.GazeText;
        VisionLink.Text = e.LinkText;
        VisionLink.Foreground = e.Connected ? Ink : Warn;
    }

    void OnShowBoxesChanged(object sender, RoutedEventArgs e) => DrawDetections();

    /// <summary>
    /// Draw the robot's own detections on the live stream.
    ///
    /// The boxes come from the gaze's detector rather than a second pass here:
    /// the Pi is already running YOLO for the eyes, and a copy per viewer would
    /// be paid for on the robot.
    /// </summary>
    void DrawDetections()
    {
        // XAML applies IsChecked="True" while InitializeComponent() is still
        // running: the checkbox sits above the canvas in the document, so this
        // fires before ViewAOverlay's field exists, and again before Init has a
        // robot to ask.  Layout can raise SizeChanged that early as well.  One
        // guard at the top covers every caller.
        if (ViewAOverlay is null || _robot is null) return;
        ViewAOverlay.Children.Clear();
        var eyes = _robot.State.Eyes;
        if (ShowBoxes.IsChecked != true || !eyes.BoxesFresh) return;

        if (ViewA.Source is not BitmapSource frame) return;
        double cw = ViewAHost.ActualWidth, ch = ViewAHost.ActualHeight;
        if (cw < 20 || ch < 20 || frame.PixelWidth <= 0 || frame.PixelHeight <= 0) return;

        // The Image is Stretch=Uniform, so the frame is letterboxed inside the
        // host.  Boxes are fractions of the FRAME and must land on the rectangle
        // the frame actually occupies, not on the host's edges — otherwise every
        // box drifts by the letterbox margin, worst on a wide panel.
        double scale = Math.Min(cw / frame.PixelWidth, ch / frame.PixelHeight);
        double dw = frame.PixelWidth * scale, dh = frame.PixelHeight * scale;
        double ox = (cw - dw) / 2, oy = (ch - dh) / 2;

        foreach (var d in eyes.Detections)
        {
            double w = (d.X2 - d.X1) * dw, h = (d.Y2 - d.Y1) * dh;
            if (w < 2 || h < 2) continue;
            var ink = d.Person ? Ink : Warn;

            var rect = new Rectangle { Width = w, Height = h, Stroke = ink, StrokeThickness = 2 };
            Canvas.SetLeft(rect, ox + d.X1 * dw);
            Canvas.SetTop(rect, oy + d.Y1 * dh);
            ViewAOverlay.Children.Add(rect);

            var tag = new TextBlock
            {
                Text = $"{d.Name} {d.Conf:0.00}",
                Foreground = ink,
                FontSize = 11,
                FontWeight = FontWeights.Bold,
            };
            Canvas.SetLeft(tag, ox + d.X1 * dw + 3);
            Canvas.SetTop(tag, Math.Max(0, oy + d.Y1 * dh - 15));
            ViewAOverlay.Children.Add(tag);
        }
    }

    async void OnSwitch(object sender, RoutedEventArgs e)
    {
        if (CamList.SelectedItem is not CameraInfo cam) { MessageBox.Show("Select a camera in the list first."); return; }
        try
        {
            CamStatus.Text = $"Switching to /dev/video{cam.Index}…";
            await _robot.SelectCameraAsync(cam.Index);
            await Task.Delay(2500);            // give the server time to re-probe
            await _robot.RefreshCamerasAsync();
            CamStatus.Text = $"Switched to /dev/video{cam.Index}";
        }
        catch (Exception ex) { CamStatus.Text = ""; MessageBox.Show($"Switch failed: {ex.Message}"); }
    }

    async void OnRetry(object sender, RoutedEventArgs e)
    {
        try { await _robot.RetryCameraAsync(); CamStatus.Text = "Re-detecting…"; await Task.Delay(2000); await _robot.RefreshCamerasAsync(); }
        catch (Exception ex) { MessageBox.Show($"Retry failed: {ex.Message}"); }
    }

    void OnRefresh(object sender, RoutedEventArgs e) => _ = _robot.RefreshCamerasAsync();
}
