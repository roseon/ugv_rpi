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
    // The factors below are the reference art's, and they are the same ones
    // face_screen.py draws the robot's own panel from: same half-width, jaw, lip
    // thickness and tooth rows, and the same two smile arcs.  Change them together.
    const double BendLift = 0.07;                  // corners ride this far above the middle
    const double BendSmile = 0.22;                 // ...plus this much more at a full smile
    const double LipBody = 0.84;                   // the body sits this far inside the rim

    double Measure(double w, double h, out double cx, out double cy, out double unit)
    {
        cx = w / 2;
        cy = h * 0.50;
        unit = Math.Min(w * 0.30, h * 0.55);            // mouth half-width
        return unit;
    }

    /// <summary>
    /// The mouth's two edges, and the shape between them.
    ///
    /// A Minion's mouth is a smile, not an oval: both edges are arcs through the
    /// same two corners - the upper one shallow, the lower one deep - so the shape
    /// is a banana with its corners lifted, and the lift grows with the jaw so it
    /// stays a smile when the mouth is wide open.  <see cref="Bend"/> is how far
    /// the corners ride above the middle; the shape a panel drew before this was
    /// an ellipse ring, which is a flat oval on the glass.
    ///
    /// Each edge is exactly a parabola, so each is one quadratic bezier: its
    /// control point is twice the arc's apex less the corners it joins.
    /// </summary>
    readonly struct Arcs
    {
        public readonly double Cx, Cy, Hw, Half, Bend;

        public Arcs(double cx, double cy, double hw, double half, double bend)
        {
            Cx = cx; Cy = cy; Hw = hw; Half = half; Bend = bend;
        }

        /// <summary>The same shape, moved `lip` outwards (or inwards, for the body).</summary>
        public Arcs Inset(double lip) => new Arcs(Cx, Cy, Hw + lip, Half + lip, Bend + lip);

        public double Up(double x) => Cy - Half - (Bend - Half) * U(x) * U(x);
        public double Down(double x) => Cy + Half - (Bend + Half) * U(x) * U(x);

        double U(double x) => (x - Cx) / Hw;

        /// <summary>The lune between the two arcs: two corners, two quadratic arcs.</summary>
        public Geometry Path
        {
            get
            {
                var left = new Point(Cx - Hw, Cy - Bend);
                var right = new Point(Cx + Hw, Cy - Bend);
                var fig = new PathFigure { StartPoint = left, IsClosed = true, IsFilled = true };
                fig.Segments.Add(new QuadraticBezierSegment(
                    new Point(Cx, Cy + Bend - 2 * Half), right, true));
                fig.Segments.Add(new QuadraticBezierSegment(
                    new Point(Cx, Cy - Bend + 2 * Half), left, true));
                var g = new PathGeometry();
                g.Figures.Add(fig);
                return g;
            }
        }
    }

    protected override void OnRender(DrawingContext dc)
    {
        double w = ActualWidth, h = ActualHeight;
        if (w < 20 || h < 20) return;

        double W = Measure(w, h, out double cx, out double cy, out _);
        double hw = W * (1 + 0.20 * _wide);              // half-width
        double openH = (0.06 + 0.94 * _open) * W * 1.05; // never zero: closed is a slit
        double lip = W * 0.14;                           // lip thickness
        double bend = openH * 0.5 + hw * (BendLift + BendSmile * _smile);
        var opening = new Arcs(cx, cy, hw, openH * 0.5, bend);

        _drawnOpen = _open; _drawnWide = _wide; _drawnSmile = _smile;

        DrawGrounding(dc, cx, cy, W);
        dc.DrawGeometry(InteriorBrush(), null, opening.Path);

        // Teeth and tongue live inside the opening, so they are clipped to it -
        // nothing pokes through a lip corner however far the jaw is dropped.
        dc.PushClip(opening.Path);
        DrawTeeth(dc, opening, openH, W);
        dc.Pop();

        DrawLips(dc, opening, lip);
    }

    /// <summary>`count` teeth with a 1 px gap, cut to the arc they hang from.
    ///
    /// Each tooth follows the smile's edge rather than crossing it flat, which is
    /// the shape a Minion's tooth row has - and it is what face_screen.py's
    /// teeth_row draws for the panel, from the same arcs.
    /// </summary>
    static void TeethRow(DrawingContext dc, Brush brush, Arcs arcs, double half, double h,
                         int count, bool lower)
    {
        if (h < 1.0 || half < 2.0) return;
        double width = Math.Max(2.0, (half * 2 - (count - 1)) / count);
        for (int k = 0; k < count; k++)
        {
            double xl = arcs.Cx - half + k * (width + 1), xr = xl + width;
            double top = Math.Ceiling(Math.Max(arcs.Up(xl), arcs.Up(xr))) + 1;
            double bottom = Math.Floor(Math.Min(arcs.Down(xl), arcs.Down(xr))) - 1;
            double y = lower ? Math.Max(top + 1, bottom - h) : top;
            double tall = (lower ? bottom : Math.Min(bottom, top + h)) - y;
            if (tall < 2)
                continue;
            dc.DrawRoundedRectangle(brush, null, new Rect(xl, y, width, tall), 2, 2);
        }
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

    void DrawTeeth(DrawingContext dc, Arcs opening, double openH, double W)
    {
        var toothBrush = new LinearGradientBrush(Tooth, Color.FromRgb(0xD6, 0xCE, 0xC2),
                                                new Point(0.5, 0), new Point(0.5, 1));
        // Upper teeth hang from the opening's top edge, the tongue sits in the
        // throat below them, and the lower row is drawn last so it is in front of
        // the tongue - the order face_screen.py paints them in.
        double upperH = Math.Min(openH * 0.45, W * 0.30);
        if (upperH > 1.0)
            TeethRow(dc, toothBrush, opening, opening.Hw * 0.88, upperH, TeethTop, false);

        // The tongue only shows once the jaw is properly open.
        if (openH > W * 0.14)
        {
            var tongue = new LinearGradientBrush(Tongue, TongueDeep, new Point(0.5, 0), new Point(0.5, 1));
            dc.DrawEllipse(tongue, null,
                           new Point(opening.Cx, opening.Cy + openH * 0.07), opening.Hw * 0.55, openH * 0.17);
        }

        double lowerH = Math.Min(openH * 0.30, W * 0.20);
        if (lowerH > 1.0 && openH > W * 0.16)
            TeethRow(dc, toothBrush, opening, opening.Hw * 0.80, lowerH, TeethBottom, true);
    }

    void DrawLips(DrawingContext dc, Arcs opening, double lip)
    {
        var lips = new LinearGradientBrush(LipTop, LipBottom, new Point(0.5, 0), new Point(0.5, 1));
        var lipPen = new Pen(new SolidColorBrush(LipEdge), 1.4);

        // The lips are one band around the opening, so the rim is drawn first and
        // the body over it: the band is then the same thickness all the way round,
        // which stacked fills are not.  They painted two lobes with a seam between
        // them, and a highlight that landed on the lip whatever the jaw was doing -
        // on the panel it read as a pale orb floating on the mouth.
        dc.DrawGeometry(lips, lipPen, opening.Inset(lip).Path);
        var body = opening.Inset(lip * LipBody);
        dc.DrawGeometry(lips, lipPen, body.Path);

        // The dark mouth line at the corners: one touch that makes the shape read
        // as a mouth.
        var corner = new SolidColorBrush(Color.FromArgb(0x99, 0x3A, 0x0B, 0x14));
        dc.DrawEllipse(corner, null, new Point(body.Cx - body.Hw, body.Cy - body.Bend), 2.2, 2.2);
        dc.DrawEllipse(corner, null, new Point(body.Cx + body.Hw, body.Cy - body.Bend), 2.2, 2.2);
    }
}
