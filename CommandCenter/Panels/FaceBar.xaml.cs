using System;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;
using CommandCenter.Core;

namespace CommandCenter.Panels;

/// <summary>
/// The robot's face, docked above the tabs so it is on screen whatever page is
/// open, with the controls to make it talk.  It owns the display side of speech
/// and nothing else: what the mouth draws comes from the robot's own
/// /speech_status, and the words it shows are the words that state names.
///
/// When the robot starts talking the big face can come up by itself
/// (<see cref="AutoFace"/>) - the point of a face is that you see it while it
/// talks, not that you go looking for it.  It never opens more than once per
/// utterance, and the checkbox turns it off.
/// </summary>
public partial class FaceBar : UserControl
{
    RobotClient? _robot;
    FaceWindow? _face;
    bool _wasSpeaking;
    readonly DispatcherTimer _tick = new() { Interval = TimeSpan.FromMilliseconds(120) };

    public FaceBar()
    {
        InitializeComponent();
        _tick.Tick += (_, _) => Refresh();
    }

    public void Init(RobotClient robot)
    {
        _robot = robot;
        // The mouth pulls the robot's curve itself, at display rate, straight from
        // the newest status the poller stored - no per-frame dispatcher work.  UTC,
        // like the timestamp the status arrived with: the two are subtracted.
        Mouth.Jaw = () => robot.State.Speech.Mouth(DateTime.UtcNow);
        robot.SpeechChanged += () => Dispatcher.Invoke(OnSpeechChanged);
        robot.State.PropertyChanged += (_, e) =>
        {
            if (e.PropertyName is "Connected" or "Speech") Dispatcher.Invoke(Refresh);
        };
        _tick.Start();
        Refresh();
    }

    /// <summary>A new /speech_status arrived: update, and bring up the face if asked.</summary>
    void OnSpeechChanged()
    {
        Refresh();
        var speech = _robot?.State.Speech;
        bool speaking = speech?.SpeakingNow(DateTime.UtcNow) == true;
        bool started = speaking && !_wasSpeaking;
        _wasSpeaking = speaking;
        if (started && AutoFace.IsChecked == true && _robot?.State.Connected == true)
            // Visible, but not focused: a face that steals the keyboard mid-typing
            // is worse than one that is simply there.  It is topmost either way.
            ShowBig(activate: false);
    }

    void Refresh()
    {
        if (_robot is null) return;
        var speech = _robot.State.Speech;
        var now = DateTime.UtcNow;
        bool speaking = speech.SpeakingNow(now);
        double level = speech.Mouth(now).Open;      // the opening is the level, shaped

        LevelFill.Width = Math.Clamp(level, 0, 1) * 150;
        bool connected = _robot.State.Connected;
        SayBtn.IsEnabled = connected && speech.Available;
        SayBox.IsEnabled = connected && speech.Available;

        // One line for every reason the mouth can be still, so "nothing moved"
        // always reads as a cause rather than a mystery.
        if (!connected)
        {
            FaceLink.Text = "no robot — connect to drive the mouth";
            FaceLink.Foreground = Brushes.Salmon;
            SayBox.ToolTip = "connect to the robot first";
        }
        else if (!speech.Available)
        {
            FaceLink.Text = "robot has no speech status — " + speech.Error;
            FaceLink.Foreground = Brushes.Salmon;
            SayBox.ToolTip = "the robot's app.py does not publish /speech_status";
        }
        else if (speech.Error.Length > 0)
        {
            FaceLink.Text = "speech failed — " + speech.Error;
            FaceLink.Foreground = Brushes.Salmon;
        }
        else if (speaking)
        {
            FaceLink.Text = $"speaking  ({speech.Source})     {speech.Text}";
            FaceLink.Foreground = new SolidColorBrush(Color.FromRgb(0x4F, 0xF5, 0xC0));
        }
        else
        {
            FaceLink.Text = "idle — say something and the mouth will move";
            FaceLink.Foreground = new SolidColorBrush(Color.FromRgb(0x90, 0xA4, 0xAE));
        }
        if (speaking) LevelFill.Background = new SolidColorBrush(Color.FromRgb(0x4F, 0xF5, 0xC0));
        else LevelFill.Background = new SolidColorBrush(Color.FromRgb(0x37, 0x47, 0x4F));
        BigBtn.Content = _face?.IsVisible == true ? "⤡ Hide face" : "⤢ Big face";
        _face?.Follow(speech, now);
    }

    void ShowBig(bool activate)
    {
        try
        {
            if (_face is null)
            {
                _face = new FaceWindow(_robot!, System.Windows.Window.GetWindow(this));
                _face.Closed += (_, _) => { _face = null; Refresh(); };
                _face.Show();
                // A window first shown while the app's main window is minimised
                // comes up minimised with it (measured): the face would exist and
                // be invisible, which is exactly the failure this panel is for.
                if (_face.WindowState != System.Windows.WindowState.Normal)
                    _face.WindowState = System.Windows.WindowState.Normal;
            }
            else if (!_face.IsVisible)
            {
                _face.Show();
            }
            if (activate)
            {
                if (_face.WindowState == System.Windows.WindowState.Minimized)
                    _face.WindowState = System.Windows.WindowState.Normal;
                _face.Activate();
            }
        }
        catch (Exception ex)
        {
            // A face that cannot open must say so, not fail invisibly.
            SayNote.Text = "⚠ face window: " + ex.Message;
        }
        Refresh();
    }

    void OnToggleBig(object sender, System.Windows.RoutedEventArgs e)
    {
        if (_face?.IsVisible == true) _face.Hide();
        else if (_robot is not null) ShowBig(activate: true);
        Refresh();
    }

    private async void OnSay(object sender, System.Windows.RoutedEventArgs e) => await SayAsync(SayBox.Text);

    private async void OnQuick(object sender, System.Windows.RoutedEventArgs e)
    {
        if (sender is Button { Tag: string phrase }) await SayAsync(phrase);
    }

    private async void OnSayKey(object sender, KeyEventArgs e)
    {
        if (e.Key == Key.Enter) await SayAsync(SayBox.Text);
    }

    private async System.Threading.Tasks.Task SayAsync(string text)
    {
        if (_robot is null) return;
        text = text.Trim();
        if (text.Length == 0)
        {
            SayNote.Text = "type something to say";
            return;
        }
        try
        {
            SayNote.Text = "";
            SayBtn.IsEnabled = false;
            await _robot.SayAsync(text);
            SayBox.Clear();
            SayNote.Text = "said: " + text;
        }
        catch (Exception ex)
        {
            // The robot's own refusal reason, not a generic failure line.
            SayNote.Text = "⚠ " + ex.Message;
        }
        finally
        {
            Refresh();
        }
    }
}
