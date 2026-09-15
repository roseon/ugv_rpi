using System;
using System.Text.Json;

namespace CommandCenter.Core;

/// <summary>
/// What the robot is saying, as the desktop needs it: the last second and a half
/// of the mouth curve its own model produced, plus the words.
///
/// There is no motion model here.  The curve is computed once, on the robot, by
/// `speech_face.py` - the same curve the panel on the robot's own screen reads -
/// because this class used to hold a second copy of it, character for character,
/// which is a description of how two faces drift apart.
///
/// The window is the recent past, so how far past is a matter of the local clock:
/// <see cref="Mouth"/> indexes back from the moment the status arrived.  A late
/// poll reads further back instead of jumping, and once the status is older than
/// the window the mouth is closed rather than pinned at whatever frame happened
/// to arrive last - so losing the robot can never leave it hanging open.
/// </summary>
public sealed class SpeechStatus
{
    /// <summary>True when the robot answered /speech_status at all.</summary>
    public bool Available;

    /// <summary>Why it did not, phrased for the user.  Empty while healthy.</summary>
    public string Error = "";

    /// <summary>The robot's own reading of its timeline - not re-derived here.</summary>
    public bool Speaking;

    public string Text = "";
    public string Source = "";                 // "audio" = measured from the WAV, "text" = derived

    /// <summary>The curve's rate, and one sample per frame of it.</summary>
    public double Sps;

    /// <summary>How far back the curve reaches, in seconds - the robot's own number.</summary>
    public double WindowS;
    public double[] Open = Array.Empty<double>();
    public double[] Wide = Array.Empty<double>();
    public double[] Smile = Array.Empty<double>();

    /// <summary>
    /// When this status arrived: the window is indexed back from here.  UTC, and
    /// every caller must hand <see cref="Mouth"/> the same clock - the two are
    /// subtracted, and a status stamped in UTC against a local "now" read as four
    /// hours in the future, which pinned the mouth to the newest frame of each poll
    /// and threw the rest of the curve away.
    /// </summary>
    public DateTime ReceivedAt = DateTime.MinValue;

    public static SpeechStatus NotPolled() => new() { Error = "waiting for the robot" };

    public static SpeechStatus NotDeployed() => new()
    {
        Error = "this app has no /speech_status - the robot is running a build from before the mouth"
    };

    public static SpeechStatus Unavailable(string why) => new() { Error = why };

    public static SpeechStatus Parse(JsonElement j)
    {
        if (!j.TryGetProperty("open", out _))
            // The endpoint exists but carries no curve: an older robot build.  Worth
            // its own sentence - "the robot stopped reporting" would send you to the
            // network for a problem that is a stale deploy.
            return new SpeechStatus
            {
                Error = "the robot's app has no mouth curve - it is running a build from before the shared mouth"
            };
        var s = new SpeechStatus { Available = true, Error = "", ReceivedAt = DateTime.UtcNow };
        s.Speaking = j.TryGetProperty("speaking", out var sp) && sp.ValueKind == JsonValueKind.True;
        s.Text = Str(j, "text");
        s.Source = Str(j, "source");
        s.Sps = Num(j, "sps");
        s.Open = Arr(j, "open");
        s.Wide = Arr(j, "wide");
        s.Smile = Arr(j, "smile");
        // The window is the robot's fact.  If a robot running an older build
        // omits it, the curve's own span is the same fact read off the data.
        s.WindowS = Num(j, "window_s");
        if (s.WindowS <= 0 && s.Sps > 0) s.WindowS = s.Open.Length / s.Sps;
        if (s.Speaking && (s.Open.Length == 0 || s.Sps <= 0))
        {
            // It claims to be speaking but published nothing to draw with.  The
            // mouth must not invent motion, and the panel should say so.
            s.Speaking = false;
            s.Error = "the robot reports speech with no mouth curve";
        }
        return s;
    }

    static string Str(JsonElement j, string name) =>
        j.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() ?? "" : "";

    static double Num(JsonElement j, string name) =>
        j.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.Number ? v.GetDouble() : 0.0;

    static double[] Arr(JsonElement j, string name)
    {
        if (!j.TryGetProperty(name, out var a) || a.ValueKind != JsonValueKind.Array) return Array.Empty<double>();
        var list = new double[a.GetArrayLength()];
        int i = 0;
        foreach (var v in a.EnumerateArray())
            list[i++] = v.ValueKind == JsonValueKind.Number ? v.GetDouble() : 0.0;
        return list;
    }

    /// <summary>Is this status recent enough to draw from?</summary>
    bool Live(DateTime now) => Available && ReceivedAt != DateTime.MinValue
                               && (now - ReceivedAt).TotalSeconds <= WindowS;

    /// <summary>Is the robot speaking, trusting only a status that is still current?</summary>
    public bool SpeakingNow(DateTime now) => Live(now) && Speaking;

    /// <summary>
    /// The mouth's three numbers at this instant.  All three are read from
    /// whichever frame this clock points at, so one frame's opening is never
    /// drawn with another frame's width.
    /// </summary>
    public (double Open, double Wide, double Smile) Mouth(DateTime now)
    {
        if (!Live(now)) return (0.0, 0.0, 0.0);
        int n = Math.Min(Open.Length, Math.Min(Wide.Length, Smile.Length));
        if (n == 0 || Sps <= 0) return (0.0, 0.0, 0.0);
        int back = (int)(Math.Max(0.0, (now - ReceivedAt).TotalSeconds) * Sps);
        int i = Math.Clamp(n - 1 - back, 0, n - 1);   // older than the window: hold its first frame
        return (Open[i], Wide[i], Smile[i]);
    }

    /// <summary>One line for the panels: what is happening, or why nothing is.</summary>
    public string Summary(DateTime now) => Error.Length > 0 ? Error
        : SpeakingNow(now) ? $"speaking ({Source})"
        : Live(now) ? "idle - say something and the mouth will move"
        : "the robot stopped reporting";
}
