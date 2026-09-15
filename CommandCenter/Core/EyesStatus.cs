using System.Text.Json;

namespace CommandCenter.Core;

/// <summary>
/// One poll of the robot's <c>/eyes_status</c>, turned into what a panel shows.
///
/// The eyes following a person is a chain, and every link fails the same way from
/// outside the robot — nothing moves. The links, in the order they break:
/// the gaze has a camera frame, a model turns it into boxes, the policy picks
/// one, and the Uno link carries it out. This keeps all of them so the panel can
/// name the one that is broken instead of showing a single unhelpful "eyes: off".
/// </summary>
public sealed class EyesStatus
{
    public record DetBox(double X1, double Y1, double X2, double Y2,
                         string Name, double Conf, bool Person);

    /// <summary>False when the robot did not answer the endpoint at all.</summary>
    public bool Available { get; private set; }

    /// <summary>Set when we could not even ask (offline, 404, bad JSON).</summary>
    public string UnavailableReason { get; private set; } = "";

    public List<DetBox> Detections { get; } = new();
    public bool Enabled { get; private set; }
    public string Reason { get; private set; } = "";
    public string Label { get; private set; } = "";

    /// <summary>Confidence of the box the gaze actually took, from the robot.
    /// Null when the robot predates the field: the label is then shown without a
    /// number, because guessing it from <see cref="Detections"/> picked the wrong
    /// box whenever two people were in frame.</summary>
    public double? ChosenConf { get; private set; }
    public int Persons { get; private set; }
    public int Hits { get; private set; }
    public double? Px { get; private set; }
    public double? Py { get; private set; }
    public string Port { get; private set; } = "";
    public bool Connected { get; private set; }
    public long Sent { get; private set; }
    public long Errors { get; private set; }
    public double? FrameAgeS { get; private set; }

    /// <summary>The robot's own verdict on its frame: this side never re-derives
    /// staleness from <see cref="FrameAgeS"/>, because the robot is the one that
    /// decides to stop detecting and to stop publishing detections.</summary>
    public bool FrameStale { get; private set; }
    public bool ModelReady { get; private set; }
    public string ModelError { get; private set; } = "";

    /// <summary>No model means no person can ever be found, whatever the frames show.</summary>
    public bool ModelMissing => ModelError.Length > 0 && !ModelReady;

    public static EyesStatus NotPolled() => new()
    {
        Available = false,
        UnavailableReason = "not polled yet",
    };

    public static EyesStatus Unavailable(string why) => new()
    {
        Available = false,
        UnavailableReason = why,
    };

    /// <summary>The robot answered, but has no gaze endpoint: the build is not deployed.</summary>
    public static EyesStatus NotDeployed() => new()
    {
        Available = false,
        UnavailableReason = "the robot has no /eyes_status - the gaze build is not deployed on the Pi",
    };

    static double? D(JsonElement o, string name) =>
        o.TryGetProperty(name, out var e) && e.ValueKind == JsonValueKind.Number
        && e.TryGetDouble(out var v) ? v : null;

    static bool B(JsonElement o, string name) =>
        o.TryGetProperty(name, out var e) && e.ValueKind == JsonValueKind.True;

    static string S(JsonElement o, string name) =>
        o.TryGetProperty(name, out var e) && e.ValueKind == JsonValueKind.String
            ? e.GetString() ?? "" : "";

    /// <summary>Parse one /eyes_status body. Never throws on a missing field: an
    /// older robot answers with fewer of them, and a missing one must read as
    /// "unknown", not as a hard failure.</summary>
    public static EyesStatus Parse(JsonElement j)
    {
        var s = new EyesStatus { Available = true };
        s.Enabled = B(j, "enabled");
        s.Reason = S(j, "reason");
        s.Label = S(j, "camera");
        s.ChosenConf = D(j, "camera_conf");
        s.Persons = (int)(D(j, "camera_persons") ?? 0);
        s.Hits = (int)(D(j, "camera_hits") ?? 0);
        s.Port = S(j, "port");
        s.Connected = B(j, "connected");
        s.Sent = (long)(D(j, "sent") ?? 0);
        s.Errors = (long)(D(j, "errors") ?? 0);
        s.FrameAgeS = D(j, "frame_age_s");
        s.FrameStale = B(j, "frame_stale");
        s.ModelReady = B(j, "model_ready");
        s.ModelError = S(j, "model_error");

        if (j.TryGetProperty("target", out var t) && t.ValueKind == JsonValueKind.Object)
        {
            s.Px = D(t, "px");
            s.Py = D(t, "py");
        }

        if (j.TryGetProperty("detections", out var ds) && ds.ValueKind == JsonValueKind.Array)
        {
            foreach (var d in ds.EnumerateArray())
            {
                if (d.ValueKind != JsonValueKind.Object) continue;
                if (!d.TryGetProperty("box", out var b) || b.ValueKind != JsonValueKind.Array
                    || b.GetArrayLength() < 4) continue;
                s.Detections.Add(new DetBox(
                    b[0].GetDouble(), b[1].GetDouble(), b[2].GetDouble(), b[3].GetDouble(),
                    S(d, "name") is { Length: > 0 } n ? n : "object",
                    D(d, "conf") ?? 0,
                    B(d, "person")));
            }
        }

        return s;
    }

    /// <summary>Boxes are only worth drawing while the frame behind them is live.
    /// The robot publishes none once its frame is stale or missing, so its own
    /// detections already carry that answer - no second staleness rule here.</summary>
    public bool BoxesFresh => Available && Detections.Count > 0;

    public string CameraText
    {
        get
        {
            if (!Available) return "CV  —";
            if (!Enabled) return "CV off — the gaze is not running";
            if (ModelMissing) return "CV off — the object model did not load";
            if (FrameStale) return $"CV — the camera stopped sending frames ({FrameAgeS:0.0} s)";
            if (FrameAgeS is null) return "CV — no camera frame is reaching the gaze";
            // "sees", not "is looking at": a close obstacle takes the gaze away
            // from what the camera found, and the CV line must not then read as
            // where the pupils point.  The GAZE line is that answer.
            var label = Label.Length == 0 ? ""
                : ChosenConf is { } conf ? $" · sees {Label} {conf:0.00}"
                : $" · sees {Label}";
            return $"CV {Hits} object(s) · {Persons} person{label} · frame {FrameAgeS:0.0} s old";
        }
    }

    public string GazeText => !Available ? "GAZE  —"
        : Px is null ? (Reason is "" or "idle" ? "GAZE idle" : $"GAZE {Reason}, no target")
        : $"GAZE {Reason} -> px {Px:0} py {Py:0}";

    public string LinkText => !Available ? "EYES  —"
        : !Enabled ? $"EYES off · {Sent} command(s) sent"
        : Connected ? $"EYES {Port} · {Sent} command(s) sent"
        : "EYES no Arduino on USB — the gaze is computed but nothing is sent";

    /// <summary>The one thing that is wrong, worst first; empty when the whole chain works.</summary>
    public string Health
    {
        get
        {            if (!Available) return UnavailableReason;
            // Off first: that is the state the user set, and it explains the
            // empty camera fields below instead of them reading as a dead
            // capture loop.  Nothing is wrong here, so nothing else is named.
            if (!Enabled) return "the gaze is off (enable it with POST /eyes)";
            if (ModelMissing) return $"cannot see people: {ModelError}";
            if (FrameStale)
                return $"camera frames are stale ({FrameAgeS:0.0} s) — the capture loop has stopped";
            if (FrameAgeS is null) return "no camera frame is reaching the gaze";
            if (!Connected) return "the gaze is running but no eye Arduino is on USB";
            return "";
        }
    }

    public bool Healthy => Available && Health.Length == 0;
}
