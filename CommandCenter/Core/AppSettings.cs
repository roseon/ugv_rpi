using System.IO;

namespace CommandCenter.Core;

/// <summary>
/// The one place the Command Center remembers which robot it talks to.
///
/// The address used to be a literal in two files ("http://172.30.136.241:5000").
/// When the Pi moved networks the app kept auto-connecting to a machine that no
/// longer existed, and said nothing about why. The last address the robot
/// actually answered on is now remembered, and this is also the first-run
/// default.
/// </summary>
public static class AppSettings
{
    /// <summary>The robot's address as of the last verified network, used when nothing is remembered.</summary>
    public const string DefaultHost = "http://192.168.24.25:5000";

    static readonly string _dir = Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "UGVCommandCenter");
    static readonly string _file = Path.Combine(_dir, "host.txt");
    static readonly string _faceFile = Path.Combine(_dir, "face.txt");

    /// <summary>Remembered address, or the default when nothing is stored.</summary>
    public static string LoadHost()
    {
        try
        {
            if (File.Exists(_file))
            {
                var saved = File.ReadAllText(_file).Trim();
                if (!string.IsNullOrWhiteSpace(saved)) return saved;
            }
        }
        catch
        {
            // A settings file that cannot be read must not stop the app starting.
        }
        return DefaultHost;
    }

    /// <summary>Remember an address the robot answered on.</summary>
    public static void SaveHost(string baseUrl)
    {
        try
        {
            Directory.CreateDirectory(_dir);
            File.WriteAllText(_file, baseUrl);
        }
        catch
        {
            // Not being able to remember it is not worth failing a connection over.
        }
    }

    /// <summary>Where the face window was left, or null when it has never been moved.</summary>
    public static (double Left, double Top)? LoadFacePlacement()
    {
        try
        {
            if (File.Exists(_faceFile))
            {
                var parts = File.ReadAllText(_faceFile).Split(',');
                if (parts.Length == 2
                    && double.TryParse(parts[0], out var left)
                    && double.TryParse(parts[1], out var top))
                    return (left, top);
            }
        }
        catch
        {
            // As with the address: a settings file that cannot be read is not fatal.
        }
        return null;
    }

    /// <summary>Remember where the face window was left.</summary>
    public static void SaveFacePlacement(double left, double top)
    {
        try
        {
            Directory.CreateDirectory(_dir);
            File.WriteAllText(_faceFile, $"{left},{top}");
        }
        catch
        {
        }
    }
}
