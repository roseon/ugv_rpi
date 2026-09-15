using System;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;

namespace CommandCenter.Panels;

/// <summary>
/// The robot's mouth, drawn at display rate - and nothing more than drawn.
///
/// <see cref="Jaw"/> is the only input: the robot's own curve for this instant
/// (openness, corner width, smile), which is the same curve the panel on the
/// robot's screen reads.  The motion model lives in `speech_face.py` on the
/// robot, in one place, because it used to live here as well - character for
/// character - which is a description of how the two faces drift apart.
///
/// What is left here is the drawing plus the one thing that is genuinely local:
/// the pointer can move the mouth by hand.
/// </summary>
public sealed class MouthView : FrameworkElement
{
    /// <summary>The robot's curve at this instant; the panel wires it to /speech_status.</summary>
    public Func<(double Open, double Wide, double Smile)>? Jaw { get; set; }

    // ── what is on screen ──
    double _open, _wide, _smile;
    double _drawnOpen = double.NaN, _drawnWide, _drawnSmile;
    DateTime _last = DateTime.UtcNow;

    // ── manual control (dragging) ──
    bool _dragging;
    double _dragOpen, _dragWide;

    // The Minion mouth: yellow lips, a maroon interior, a row of white teeth
    // hanging from the top edge and a red tongue.  These names and values mirror
    // face_screen.py, which draws the same mouth on the robot's own panel.
    static readonly Color LipTop = Color.FromRgb(0xF7, 0xCE, 0x4A);
    static readonly Color LipBottom = Color.FromRgb(0xE3, 0xAA, 0x2E);
    static readonly Color LipEdge = Color.FromRgb(0xC9, 0x8B, 0x1E);
    static readonly Color MouthBack = Color.FromRgb(0x6E, 0x1B, 0x2A);
    static readonly Color MouthDeep = Color.FromRgb(0x45, 0x0E, 0x19);
    static readonly Color Tooth = Color.FromRgb(0xFF, 0xFA, 0xF2);
    static readonly Color Tongue = Color.FromRgb(0xE0, 0x6B, 0x6B);
    static readonly Color TongueDeep = Color.FromRgb(0xB8, 0x45, 0x4E);
    static readonly Color Gloss = Color.FromArgb(0x66, 0xFF, 0xE9, 0xA8);
    const int TeethTop = 7;                       // the reference shows a full row
    const int TeethBottom = 5;

    public MouthView()
    {
        ClipToBounds = true;
        Loaded += (_, _) => { _last = DateTime.UtcNow; CompositionTarget.Rendering += OnFrame; };
        Unloaded += (_, _) => CompositionTarget.Rendering -= OnFrame;
        MouseLeftButtonDown += OnDown;
        MouseMove += OnMove;
        MouseLeftButtonUp += OnUp;
        MouseLeave += (_, _) =>
        {
            // Dragging out of the mouth and letting go is still a release: a
            // captured drag that never ends would leave the mouth stuck open.
            if (_dragging) EndDrag();
        };
    }

    // ── the animation: what the model says, drawn ────────────────────────────
    void OnFrame(object? sender, EventArgs e)
    {
        var now = DateTime.UtcNow;
        if ((now - _last).TotalSeconds <= 0) return;
        _last = now;

        // No filter here: the curve arriving from the robot has already been
        // shaped at display rate, and easing it again is what made the pupils
        // lag their target.  A poll that arrives late simply indexes further
        // back into the window.
        (_open, _wide, _smile) = Jaw?.Invoke() ?? (0.0, 0.0, 0.0);
        if (_dragging)
        {
            _open = _dragOpen;
            _wide = _dragWide;
            _smile = 0.06;
        }

        if (Math.Abs(_open - _drawnOpen) > 2e-3 || Math.Abs(_wide - _drawnWide) > 2e-3
            || Math.Abs(_smile - _drawnSmile) > 2e-3)
        {
            _drawnOpen = _open; _drawnWide = _wide; _drawnSmile = _smile;
            InvalidateVisual();
        }
    }

    // ── pointer: move the mouth by hand ───────────────────────────────────────
    void OnDown(object sender, MouseButtonEventArgs e)
    {
        _dragging = true;
        CaptureMouse();
        StartDrag(e.GetPosition(this));
        e.Handled = true;
    }

    void StartDrag(Point p)
    {
        double w = Math.Max(1, ActualWidth), h = Math.Max(1, ActualHeight);
        double scale = Measure(w, h, out double cx, out double cy, out _);
        _dragOpen = Math.Clamp((p.Y - cy + scale * 0.30) / (scale * 0.85), 0, 1);
        _dragWide = Math.Clamp((p.X - cx) / (scale * 1.6), -0.35, 0.35);
        InvalidateVisual();
    }

    void OnMove(object sender, MouseEventArgs e)
    {
        Cursor = Cursors.SizeNS;
        if (_dragging) StartDrag(e.GetPosition(this));
    }

    void OnUp(object sender, MouseButtonEventArgs e) => EndDrag();

    void EndDrag()
    {
        if (!_dragging) return;
        _dragging = false;
        if (IsMouseCaptured) ReleaseMouseCapture();
        InvalidateVisual();
    }

    // ── drawing ───────────────────────────────────────────────────────────────
    double Measure(double w, double h, out double cx, out double cy, out double unit)
    {
        cx = w / 2;
        cy = h * 0.54;
        unit = Math.Min(w * 0.34, h * 0.60);            // mouth half-width
        return unit;
    }

    protected override void OnRender(DrawingContext dc)
    {
        double w = ActualWidth, h = ActualHeight;
        if (w < 20 || h < 20) return;

        double W = Measure(w, h, out double cx, out double cy, out _);
        double hw = W * (1 + 0.20 * _wide);              // half-width
        double openH = (0.06 + 0.94 * _open) * W * 0.60; // gap between the inner edges
        double innerUp = openH * 0.42;
        double innerDn = openH * 0.62;
        double lip = W * 0.20;                           // lip thickness
        double cornerY = cy - _smile * W * 0.16;         // a smile lifts the corners
        var left = new Point(cx - hw, cornerY);
        var right = new Point(cx + hw, cornerY);

        _drawnOpen = _open; _drawnWide = _wide; _drawnSmile = _smile;

        DrawGrounding(dc, cx, cy, W);
        var opening = OpeningGeometry(left, right, innerUp, innerDn);
        dc.DrawGeometry(InteriorBrush(), null, opening);

        // Teeth and tongue live inside the opening, so they are clipped to it -
        // nothing pokes through a lip corner however far the jaw is dropped.
        dc.PushClip(opening);
        DrawTeeth(dc, left, right, cornerY, innerUp, innerDn, openH, W);
        dc.Pop();

        DrawLips(dc, left, right, cx, hw, cornerY, innerUp, innerDn, lip);
    }

    /// <summary>`count` teeth with a 1 px gap, spanning 2*half around cx.</summary>
    static void TeethRow(DrawingContext dc, Brush brush, double cx, double half, double y, double h, int count)
    {
        if (h < 1.0 || half < 2.0) return;
        double width = Math.Max(2.0, (half * 2 - (count - 1)) / count);
        for (int k = 0; k < count; k++)
            dc.DrawRoundedRectangle(brush, null,
                new Rect(cx - half + k * (width + 1), y, width, h), 2, 2);
    }

    /// <summary>A soft shadow so the mouth sits on the panel instead of floating.</summary>
    static void DrawGrounding(DrawingContext dc, double cx, double cy, double W)
    {
        var shadow = new RadialGradientBrush
        {
            GradientOrigin = new Point(0.5, 0.5),
            Center = new Point(0.5, 0.5),
            RadiusX = 0.5,
            RadiusY = 0.5,
        };
        shadow.GradientStops.Add(new GradientStop(Color.FromArgb(0x00, 0, 0, 0), 0.55));
        shadow.GradientStops.Add(new GradientStop(Color.FromArgb(0x66, 0, 0, 0), 1.0));
        dc.DrawEllipse(shadow, null, new Point(cx, cy + W * 0.30), W * 1.35, W * 0.85);
    }

    static Brush InteriorBrush() =>
        new LinearGradientBrush(MouthBack, MouthDeep, new Point(0.5, 0), new Point(0.5, 1));

    /// <summary>The gap between the lips: two arcs meeting at the corners.</summary>
    static PathGeometry OpeningGeometry(Point left, Point right, double innerUp, double innerDn)
    {
        double dx = (right.X - left.X) * 0.32;
        var fig = new PathFigure { StartPoint = left, IsClosed = true, IsFilled = true };
        fig.Segments.Add(new BezierSegment(
            new Point(left.X + dx, left.Y - innerUp),
            new Point(right.X - dx, right.Y - innerUp), right, true));
        fig.Segments.Add(new BezierSegment(
            new Point(right.X - dx, right.Y + innerDn),
            new Point(left.X + dx, left.Y + innerDn), left, true));
        var g = new PathGeometry();
        g.Figures.Add(fig);
        return g;
    }

    void DrawTeeth(DrawingContext dc, Point left, Point right, double cornerY,
                   double innerUp, double innerDn, double openH, double W)
    {
        double cx = (left.X + right.X) / 2;
        double hw = (right.X - left.X) / 2;
        var toothBrush = new LinearGradientBrush(Tooth, Color.FromRgb(0xD6, 0xCE, 0xC2),
                                                new Point(0.5, 0), new Point(0.5, 1));
        double upperH = Math.Min(openH * 0.45, W * 0.11);
        if (upperH > 1.0)
            TeethRow(dc, toothBrush, cx, hw * 0.86, cornerY - innerUp * 0.85, upperH, TeethTop);

        double lowerH = Math.Min(openH * 0.26, W * 0.06);
        if (lowerH > 1.0 && openH > W * 0.16)
            TeethRow(dc, toothBrush, cx, hw * 0.80, cornerY + innerDn * 0.80 - lowerH, lowerH, TeethBottom);

        // The tongue only shows once the jaw is properly open.
        if (openH > W * 0.14)
        {
            var tongue = new LinearGradientBrush(Tongue, TongueDeep, new Point(0.5, 0), new Point(0.5, 1));
            dc.DrawEllipse(tongue, null, new Point(cx, cornerY + innerDn * 0.55), hw * 0.58, openH * 0.34);
        }
    }

    void DrawLips(DrawingContext dc, Point left, Point right, double cx, double hw,
                  double cornerY, double innerUp, double innerDn, double lip)
    {
        double dx = (right.X - left.X) * 0.32;
        var lips = new LinearGradientBrush(LipTop, LipBottom, new Point(0.5, 0), new Point(0.5, 1));
        var lipPen = new Pen(new SolidColorBrush(LipEdge), 1.4);

        // Upper lip: out from the left corner, over a cupid's bow, to the right,
        // then back along the inner edge - so the lip is exactly the band between
        // the outer shape and the opening.
        var upper = new PathFigure { StartPoint = left, IsClosed = true, IsFilled = true };
        upper.Segments.Add(new BezierSegment(
            new Point(left.X + hw * 0.45, cornerY - innerUp * 0.9 - lip * 2.8),
            new Point(cx - hw * 0.52, cornerY - innerUp * 1.05 - lip * 2.1),
            new Point(cx - hw * 0.12, cornerY - innerUp * 1.10 - lip * 1.5), true));
        upper.Segments.Add(new BezierSegment(
            new Point(cx + hw * 0.12, cornerY - innerUp * 1.10 - lip * 1.5),
            new Point(cx + hw * 0.52, cornerY - innerUp * 1.05 - lip * 2.1),
            right, true));
        upper.Segments.Add(new BezierSegment(
            new Point(right.X - dx, cornerY - innerUp),
            new Point(left.X + dx, cornerY - innerUp), left, true));
        var ug = new PathGeometry();
        ug.Figures.Add(upper);
        dc.DrawGeometry(lips, lipPen, ug);

        // Lower lip: the same construction under the opening, fuller in the middle.
        var lower = new PathFigure { StartPoint = left, IsClosed = true, IsFilled = true };
        lower.Segments.Add(new BezierSegment(
            new Point(left.X + dx, cornerY + innerDn),
            new Point(right.X - dx, cornerY + innerDn), right, true));
        lower.Segments.Add(new BezierSegment(
            new Point(right.X - hw * 0.45, cornerY + innerDn * 1.0 + lip * 3.0),
            new Point(cx + hw * 0.55, cornerY + innerDn * 1.05 + lip * 3.3),
            new Point(cx, cornerY + innerDn * 1.05 + lip * 2.4), true));
        lower.Segments.Add(new BezierSegment(
            new Point(cx - hw * 0.55, cornerY + innerDn * 1.05 + lip * 3.3),
            new Point(left.X + hw * 0.45, cornerY + innerDn * 1.0 + lip * 3.0),
            left, true));
        var lg = new PathGeometry();
        lg.Figures.Add(lower);
        dc.DrawGeometry(lips, lipPen, lg);

        // A wet highlight across the lower lip, and the dark mouth line at the
        // corners: two touches that make a flat shape read as a mouth.
        var gloss = new SolidColorBrush(Gloss);
        dc.DrawEllipse(gloss, null,
            new Point(cx - hw * 0.20, cornerY + innerDn * 1.05 + lip * 2.4),
            hw * 0.22, lip * 0.30);
        var corner = new SolidColorBrush(Color.FromArgb(0x99, 0x3A, 0x0B, 0x14));
        dc.DrawEllipse(corner, null, left, 2.2, 2.2);
        dc.DrawEllipse(corner, null, right, 2.2, 2.2);
    }
}
