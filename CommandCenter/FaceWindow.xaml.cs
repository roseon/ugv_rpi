using System;
using System.Windows;
using System.Windows.Media;
using CommandCenter.Core;
using CommandCenter.Panels;

namespace CommandCenter;

/// <summary>
/// The robot's face, big, on top - and closed the moment it stops being wanted.
///
/// It holds no speech state of its own: the bar that owns the display calls
/// <see cref="Follow"/> with the robot's newest status, and this window only
/// draws it.  Sitting on the robot's other monitor while it talks is the whole
/// point, so it stays above other windows until it is hidden.
/// </summary>
public partial class FaceWindow : Window
{
    public FaceWindow(RobotClient robot, Window? hint)
    {
        InitializeComponent();
        Mouth.Jaw = () => robot.State.Speech.Mouth(DateTime.UtcNow);
        WindowStartupLocation = WindowStartupLocation.Manual;
        // Deliberately NOT owned by the Command Center.  An owned window is
        // hidden whenever its owner is minimised, and the whole point of the face
        // is that you see it talking while the app is out of the way - measured
        // on this desk, the face never appeared because the main window was
        // minimised.  Placement is taken from the main window instead.
        if (AppSettings.LoadFacePlacement() is { } place)
        {
            Left = place.Left;
            Top = place.Top;
        }
        else
        {
            (Left, Top) = PlaceNear(hint);
        }
        Closing += (_, _) => AppSettings.SaveFacePlacement(Left, Top);
    }

    /// <summary>
    /// Where a face that has never been moved belongs: beside the Command Center
    /// when it is on screen, otherwise right of centre on the primary screen.
    /// </summary>
    static (double Left, double Top) PlaceNear(Window? hint)
    {
        const double w = 600;
        if (hint is not null && Math.Abs(hint.Left) < 10000 && hint.Width > 400)
            return (hint.Left + hint.Width - w - 30, hint.Top + 60);
        // No usable main window (it can be minimised): the primary screen, since
        // WorkArea here spans the whole virtual desktop on this desk.
        return (SystemParameters.PrimaryScreenWidth - w - 40, 80);
    }

    /// <summary>Show the robot's newest speech state.  Called by the face bar.</summary>
    public void Follow(SpeechStatus speech, DateTime now)
    {
        bool speaking = speech.SpeakingNow(now);
        Caption.Text = speaking && speech.Text.Length > 0 ? "\u201C" + speech.Text + "\u201D" : "";
        double level = speech.Mouth(now).Open;
        LevelFill.Width = Math.Clamp(level, 0, 1) * 130;
        LevelText.Text = speaking ? $"level  {level:0.00}  ({speech.Source})" : "level  —";
        LevelFill.Background = new SolidColorBrush(speaking
            ? Color.FromRgb(0x4F, 0xF5, 0xC0)
            : Color.FromRgb(0x37, 0x47, 0x4F));
        FaceState.Text = speech.Summary(now);
    }
}
