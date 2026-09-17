using System.Text.Json;

namespace CommandCenter.Core;

/// <summary>One poll of the robot's Wi-Fi health: parsed from /network_status.</summary>
public sealed class NetworkInfo
{
    /// <summary>true on the network, false off it, null = endpoint predates the feature.</summary>
    public bool? Connected;
    public string? Ssid;         // the connection name NM reports (the home SSID's profile)
    public int? Signal;          // 0..100 for the row in use
    public bool? SignalInUse;    // false = in range of networks but on none
    public string? DeviceState;  // the radio's own word: connected / disconnected / unavailable
    public WatchdogInfo? Watchdog;
    public string? Error;        // endpoint error text (module undeployed), null when healthy

    public sealed class WatchdogInfo
    {
        public bool Installed;
        public bool Active;
        public string? Profile;          // the pinned profile the watchdog rejoins
        public List<Reconnect> Reconnects = new();   // newest first
        public int Reboots;
    }

    public readonly struct Reconnect
    {
        public Reconnect(double t, double afterS) { T = t; AfterS = afterS; }
        public double T { get; }
        public double AfterS { get; }
    }

    /// <summary>The header's one line, or null when there is nothing to say.</summary>
    public string? HeaderLine
    {
        get
        {
            if (Error != null) return "wifi: " + Error;
            if (Connected == null) return null;
            if (Connected == true)
            {
                var sig = Signal is int s ? $" {s}%" : "";
                return $"📶 {Ssid ?? "wifi"}{sig}";
            }
            if (SignalInUse == false) return "📶 in range, not connected";
            var wd = Watchdog;
            if (wd is { Active: true })
            {
                if (wd.Reconnects.Count > 0)
                    return $"📶 offline — watchdog retrying (last reconnect +{wd.Reconnects[0].AfterS:0}s)";
                return "📶 offline — watchdog retrying";
            }
            return "📶 offline — no watchdog installed";
        }
    }

    /// <summary>Green on the network, amber working, grey nothing to say.</summary>
    public string HeaderBrushKey => Connected == true ? "ok" : Connected == false ? "warn" : "idle";

    public static NetworkInfo Parse(JsonElement j)
    {
        var info = new NetworkInfo();
        if (j.TryGetProperty("error", out var err) && err.ValueKind == JsonValueKind.String
            && err.GetString() is { Length: > 0 } e)
        {
            info.Error = e;
            return info;
        }
        info.Connected = j.TryGetProperty("connected", out var c) && c.ValueKind == JsonValueKind.True
            ? true
            : c.ValueKind == JsonValueKind.False ? false : null;
        if (j.TryGetProperty("ssid", out var ssid) && ssid.ValueKind == JsonValueKind.String)
            info.Ssid = ssid.GetString();
        if (j.TryGetProperty("signal", out var sig) && sig.ValueKind == JsonValueKind.Number)
            info.Signal = sig.GetInt32();
        if (j.TryGetProperty("state", out var ds) && ds.ValueKind == JsonValueKind.String)
            info.DeviceState = ds.GetString();   // the radio's word: connected/disconnected/...
        if (j.TryGetProperty("watchdog", out var wd) && wd.ValueKind == JsonValueKind.Object)
        {
            var w = new WatchdogInfo();
            if (wd.TryGetProperty("installed", out var inst)) w.Installed = inst.ValueKind == JsonValueKind.True;
            if (wd.TryGetProperty("active", out var act)) w.Active = act.ValueKind == JsonValueKind.True;
            if (wd.TryGetProperty("profile", out var prof) && prof.ValueKind == JsonValueKind.String)
                w.Profile = prof.GetString();
            if (wd.TryGetProperty("reconnects", out var recs) && recs.ValueKind == JsonValueKind.Array)
                foreach (var r in recs.EnumerateArray())
                {
                    double t = r.TryGetProperty("t", out var tv) && tv.ValueKind == JsonValueKind.Number ? tv.GetDouble() : 0;
                    double a = r.TryGetProperty("after_s", out var av) && av.ValueKind == JsonValueKind.Number ? av.GetDouble() : 0;
                    w.Reconnects.Add(new Reconnect(t, a));
                }
            if (wd.TryGetProperty("reboots", out var rb) && rb.ValueKind == JsonValueKind.Number)
                w.Reboots = rb.GetInt32();
            info.Watchdog = w;
        }
        return info;
    }

    public static NetworkInfo Unavailable(string why) =>
        new() { Error = why };
}
