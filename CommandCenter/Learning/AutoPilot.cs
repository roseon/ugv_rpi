namespace CommandCenter.Learning;

/// <summary>
/// The operator's own tools for moving the robot: replay a taught path (aborting
/// on an obstacle the lidar can see) and follow a strongly-coloured object.
///
/// Reactive lidar self-driving is deliberately *not* here any more.  It was a
/// second driving brain: it scored headings against its own ObstacleMemory — an
/// occupancy grid in the robot's instantaneous frame, with no pose, so the same
/// wall landed in different cells as the robot drove and the operator's map
/// smeared into a blob around the robot — and it drove the robot over the
/// websocket, which pauses the robot's own avoider for 8 s.  The robot learns a
/// place-referenced map (spatial_memory.py) and drives itself from it
/// (self_drive.py, executed by the avoider in app.py), and that is proven on
/// hardware.  Two brains meant the panel drew one map while the robot drove on
/// another, which is why "it is not learning" was what the operator saw.
/// </summary>
public class AutoPilot
{
    public enum Mode { Off, Replay, FollowObject }

    readonly Func<double, double, Task> _drive;
    readonly Func<(double angle, double dist)[]> _getScan;
    readonly Func<System.Drawing.RectangleF[]> _getDetections;
    CancellationTokenSource? _cts;
    Task? _loop;

    public Mode CurrentMode { get; private set; } = Mode.Off;
    public string LastDecision { get; private set; } = "off";

    public double DangerM = 0.55;            // replay: abort below this clearance

    public AutoPilot(Func<double, double, Task> drive,
                     Func<(double angle, double dist)[]> getScan,
                     Func<System.Drawing.RectangleF[]> getDetections)
    {
        _drive = drive; _getScan = getScan; _getDetections = getDetections;
    }

    public void Start(Mode mode, PathPlan? replay = null)
    {
        Stop();
        CurrentMode = mode;
        _cts = new CancellationTokenSource();
        var plan = replay;
        _loop = Task.Run(() => mode switch
        {
            Mode.Replay when plan != null => ReplayLoop(plan, _cts.Token),
            Mode.FollowObject => FollowLoop(_cts.Token),
            _ => Task.CompletedTask
        });
    }

    public void Stop()
    {
        _cts?.Cancel();
        try { _drive(0, 0).Wait(400); } catch { }
        CurrentMode = Mode.Off;
        LastDecision = "stopped";
    }

    // ─────────── replay: follow a learned path, with obstacle aborts ───────────
    async Task ReplayLoop(PathPlan plan, CancellationToken ct)
    {
        LastDecision = $"replaying {plan.Steps.Count} steps ({plan.TotalSeconds:0.0}s)";
        foreach (var s in plan.Steps)
        {
            if (ct.IsCancellationRequested) break;
            double front = MinRange(_getScan(), -25, 25);
            if (front < DangerM)
            {
                LastDecision = $"replay aborted: obstacle {front:0.00}m ahead";
                await _drive(0, 0);
                return;
            }
            await _drive(s.L, s.R);
            await Task.Delay(TimeSpan.FromSeconds(s.Seconds), ct);
        }
        await _drive(0, 0);
        LastDecision = "replay finished";
    }

    // ─────────── object follow: steer toward largest CV box ───────────
    async Task FollowLoop(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            var boxes = _getDetections();
            if (boxes.Length == 0)
            {
                await _drive(0, 0);
                LastDecision = "no target — waiting";
                await Task.Delay(300, ct);
                continue;
            }
            var biggest = boxes.OrderByDescending(b => b.Width * b.Height).First();
            float cx = biggest.X + biggest.Width / 2;             // 0..1 of frame
            float err = cx - 0.5f;                                // <0 target left
            double turn = Math.Clamp(err * 2.0, -1, 1);
            double distProxy = biggest.Height;                    // bigger box = closer
            double fwd = Math.Clamp((0.45f - distProxy) * 2.0, -0.4, 0.6);
            double l = fwd - turn * 0.5, r = fwd + turn * 0.5;
            await _drive(Math.Clamp(l, -0.5, 0.5), Math.Clamp(r, -0.5, 0.5));
            LastDecision = $"follow box {biggest.Width * 100:0}%w err {err:+0.00;-0.00} → L {l:0.00} R {r:0.00}";
            await Task.Delay(150, ct);
        }
    }

    static double MinRange((double angle, double dist)[] scan, double a1, double a2)
    {
        double best = double.MaxValue;
        foreach (var (a, d) in scan)
        {
            double deg = (a + Math.PI) * 180 / Math.PI;         // normalise like the UI
            if (deg > a1 && deg < a2 && d > 0 && d < best) best = d / 1000.0;
        }
        return best == double.MaxValue ? 99 : best;
    }
}
