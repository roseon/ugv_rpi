"""Lance's face on the robot's own screen (the Pi 5's DSI panel).

The mouth is drawn from the robot's mouth curve — the same curve the desktop's
Command Center draws — so the face on the robot and the face on the desktop move
together, syllable for syllable.

Why it polls instead of importing: the curve belongs to the app process
(``speech_face.FACE``), and this is a separate process because it owns a screen.
The app publishes that state at ``/speech_status``; this reads it a few times a
second and interpolates at 60 fps locally, exactly as the desktop does, so the
panel is smooth without the app doing any drawing work.

Run:  python3 face_screen.py            fullscreen on the panel
      python3 face_screen.py --windowed  in a window (for a desk test)
      python3 face_screen.py --selftest  print the drawn opening per frame, no display
"""

import argparse
import json
import math
import os
import threading
import time
import urllib.request

import pygame

# ── the same palette the desktop mouth uses ───────────────────────────────────
# The Minion mouth: yellow lips, a maroon interior, one row of white teeth
# hanging from the top edge and a red tongue, as in the reference art.  These
# names and values are mirrored in CommandCenter/Panels/MouthView.cs.
LIP_TOP = (0xF7, 0xCE, 0x4A)
LIP_BOTTOM = (0xE3, 0xAA, 0x2E)
LIP_EDGE = (0xC9, 0x8B, 0x1E)
MOUTH_BACK = (0x6E, 0x1B, 0x2A)
MOUTH_DEEP = (0x45, 0x0E, 0x19)
TOOTH = (0xFF, 0xFA, 0xF2)
TONGUE = (0xE0, 0x6B, 0x6B)
TONGUE_DEEP = (0xB8, 0x45, 0x4E)
BACKDROP = (0x12, 0x14, 0x1A)
CAPTION = (0xED, 0xE7, 0xE3)
HUD = (0x90, 0xA4, 0xAE)
TEETH_TOP = 7                                    # the reference shows a full row
TEETH_BOTTOM = 5

STATUS_URL = os.environ.get("FACE_STATUS_URL", "http://127.0.0.1:5000/speech_status")
POLL_HZ = 5.0


# ── drawing ──────────────────────────────────────────────────────────────────
# The motion these are drawn from belongs to speech_face.py, which both faces
# read: this file only turns (open, wide, smile) into pixels.
#
# The numbers are measured off the reference art (a lip band 0.14 of the mouth's
# half-width, a jaw that opens to 1.05 of it, two tooth rows with the tongue in
# the throat between them).  CommandCenter/Panels/MouthView.cs draws the same
# mouth from the same factors, so the two animate identically and differ only in
# their outline - a bezier with a cupid's bow there, an ellipse ring here.  This
# comment is the only thing that says so, so change the factors together.
def mouth_geometry(w, h, open_, wide, smile):
    """Every dimension of the mouth, from its openness and two shape terms."""
    unit = min(w * 0.30, h * 0.55)                   # mouth half-width
    cx = w / 2.0
    cy = h * 0.50 - smile * unit * 0.16              # a smile lifts the mouth
    hw = unit * (1 + 0.20 * wide)
    open_h = (0.06 + 0.94 * open_) * unit * 1.05     # never zero: closed is a slit
    return {"cx": cx, "cy": cy, "unit": unit, "hw": hw, "open_h": open_h,
            "lip": unit * 0.14}


def teeth_row(surface, cx, cy, hw, half_h, half, h, count, colour, lower=False):
    """A row of `count` teeth with a 1 px gap, inside the opening's ellipse.

    Every tooth's inner edge follows the ellipse, so a row placed at the curve
    stays in the mouth without a clip - and following the curve is the shape a
    Minion's tooth row has.  `lower` rows rise from the bottom edge instead of
    hanging from the top.  The rounding goes inward, so no pixel lands outside.
    """
    def edge(x):
        """Half-height of the opening at this x."""
        return half_h * math.sqrt(max(0.0, 1.0 - ((x - cx) / hw) ** 2))

    width = max(2, int((half * 2 - (count - 1)) / count))
    x0 = int(cx - half)
    for k in range(count):
        xl, xr = x0 + k * (width + 1), x0 + k * (width + 1) + width
        inner = min(edge(xl), edge(xr))
        if lower:
            y2, y1 = math.floor(cy + inner - 1), None
            y1 = math.ceil(max(cy - inner + 1, y2 - h))
        else:
            y1, y2 = math.ceil(cy - inner + 1), None
            y2 = math.floor(min(cy + inner - 1, y1 + h))
        if y2 - y1 < 2:
            continue
        pygame.draw.rect(surface, colour, pygame.Rect(xl, y1, width, y2 - y1))


def draw_mouth(surface, geo):
    """Paint the mouth: one lip ring, then the inside, clipped to the opening."""
    cx, cy, hw = geo["cx"], geo["cy"], geo["hw"]
    open_h, lip, unit = geo["open_h"], geo["lip"], geo["unit"]
    top = cy - open_h / 2

    def rect(x, y, w, h):
        return pygame.Rect(int(x), int(y), max(1, int(w)), max(1, int(h)))

    opening = rect(cx - hw, top, hw * 2, open_h)
    # The lips are one ring, so the band is the same thickness all the way round.
    # The stacked fills that used to draw this painted two lobes with a seam
    # between them and a fixed highlight that landed on the lip whatever the jaw
    # was doing; both were visible in a photograph of the panel.
    pygame.draw.ellipse(surface, LIP_EDGE,
                        rect(cx - hw - lip, top - lip, (hw + lip) * 2, open_h + lip * 2))
    body = rect(cx - hw - lip * 0.84, top - lip * 0.84, (hw + lip * 0.84) * 2,
                open_h + lip * 1.68)
    pygame.draw.ellipse(surface, LIP_TOP, body)
    # The lower lip is shaded, and the shading stays inside the band: a fill that
    # overshot it made the bottom lip 4 px thicker than the sides, which is the
    # kind of thing that reads as a separate lobe on a panel.
    pygame.draw.ellipse(surface, LIP_BOTTOM,
                        rect(body.x + body.w * 0.02, body.y + body.h * 0.40,
                             body.w * 0.96, body.h * 0.58))

    pygame.draw.ellipse(surface, MOUTH_BACK, opening)
    pygame.draw.ellipse(surface, MOUTH_DEEP,
                        rect(cx - hw * 0.90, top + open_h * 0.05, hw * 1.80, open_h * 0.90))

    # What is visible inside: the upper row hanging from the top of the opening,
    # the tongue in the throat below it, and the lower row in front of the tongue
    # once the jaw is open past a sliver.
    teeth_h = min(open_h * 0.45, unit * 0.30)
    if teeth_h > 1:
        teeth_row(surface, cx, cy, hw, open_h / 2, hw * 0.88, teeth_h, TEETH_TOP, TOOTH)
    if open_h > unit * 0.14:
        tongue = rect(cx - hw * 0.55, cy - open_h * 0.10, hw * 1.10, open_h * 0.34)
        pygame.draw.ellipse(surface, TONGUE, tongue)
        pygame.draw.ellipse(surface, TONGUE_DEEP,
                            rect(tongue.x + tongue.w * 0.20, tongue.y + tongue.h * 0.52,
                                 tongue.w * 0.60, tongue.h * 0.46))
    lower_h = min(open_h * 0.30, unit * 0.20)
    if lower_h > 1 and open_h > unit * 0.16:
        teeth_row(surface, cx, cy, hw, open_h / 2, hw * 0.80, lower_h, TEETH_BOTTOM,
                  TOOTH, lower=True)


class SpeechFeed:
    """The app's /speech_status, polled, as the face needs it."""

    def __init__(self, url=STATUS_URL, hz=POLL_HZ):
        self.url, self.hz = url, hz
        self._lock = threading.Lock()
        self._state = {"available": False, "speaking": False, "text": "", "curve": [],
                       "sps": 60.0, "received": 0.0, "error": "starting up"}
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def stop(self):
        self._stop.set()

    def _loop(self):
        while not self._stop.is_set():
            self.poll()
            time.sleep(max(0.05, 1.0 / self.hz))

    def poll(self):
        try:
            with urllib.request.urlopen(self.url, timeout=3) as r:
                d = json.loads(r.read())
        except Exception as e:                              # noqa: BLE001
            with self._lock:
                self._state = dict(self._state, available=False, speaking=False,
                                   error="no app on %s (%s)" % (self.url, type(e).__name__))
            return
        now = time.monotonic()
        # Zip the robot's three curves into what the drawing wants.  Lengths are
        # taken in step so a short one cannot shift the others.
        op = d.get("open") or []
        wd = d.get("wide") or []
        sm = d.get("smile") or []
        n = min(len(op), len(wd), len(sm))
        sps = float(d.get("sps", 60.0) or 60.0)
        window = float(d.get("window_s") or 0.0)
        state = {
            "available": True,
            "speaking": bool(d.get("speaking")),
            "text": d.get("text", "") or "",
            "source": d.get("source", ""),
            "curve": [(float(op[i]), float(wd[i]), float(sm[i])) for i in range(n)],
            "sps": sps,
            # How long the curve reaches back.  Published by the model; if a robot
            # running an older build omits it, the curve's own span is the same
            # fact read off the data.
            "window_s": window if window > 0 else (n / sps if sps > 0 else 0.0),
            "received": now,
            "error": "",
        }
        with self._lock:
            self._state = state

    def state(self):
        with self._lock:
            return dict(self._state)

    def mouth(self, now=None):
        """(open, wide, smile) for this instant, indexed into the robot's curve.

        The curve is the model's output, so what is drawn here is the same motion
        the desktop draws - only the pixels differ.  Polling at 5 Hz against a
        1.5 s window, the index is normally a few frames back, so a late poll reads
        slightly further back instead of jumping.  Once the status is older than the
        window the app has stopped reporting and the mouth rests closed: holding a
        frame from the middle of a sentence would leave a dead robot's mouth open.
        """
        st = self.state()
        curve = st.get("curve") or []
        if not curve or not st.get("available"):
            return 0.0, 0.0, 0.0
        now = time.monotonic() if now is None else now
        age = max(0.0, now - st["received"])
        if age > float(st.get("window_s") or 0.0):
            return 0.0, 0.0, 0.0
        sps = float(st.get("sps") or 60.0)
        if sps <= 0:
            return 0.0, 0.0, 0.0
        idx = len(curve) - 1 - int(age * sps)
        return curve[max(0, min(len(curve) - 1, idx))]


class FaceScreen:
    """Draws the mouth from the robot's curve.  No model lives here."""

    def __init__(self, feed, size=(800, 480), fullscreen=True):
        self.feed = feed
        self.size = size
        self.fullscreen = fullscreen
        # Built on the first frame, not here: a font is a display resource, and
        # this object is also constructed by the selftest, which has no display
        # ("font not initialized" - measured, at boot, before pygame.init ran).
        self.caption_font = None
        self.hud_font = None

    def step(self, now):
        """One frame: what the model says the mouth is doing, as geometry."""
        open_, wide, smile = self.feed.mouth(now)
        return mouth_geometry(self.size[0], self.size[1], open_, wide, smile)

    def render(self, surface, geo, st, now):
        surface.fill(BACKDROP)
        draw_mouth(surface, geo)
        if self.hud_font is None:
            self.caption_font = pygame.font.SysFont("dejavusans", 20, italic=True)
            self.hud_font = pygame.font.SysFont("dejavusans", 15)
        if st["speaking"] and st["text"]:
            words = "\u201C%s\u201D" % st["text"]
            if len(words) > 74:
                words = words[:71] + "\u2026\u201D"
            surface.blit(self.caption_font.render(words, True, CAPTION), (24, self.size[1] - 46))
        if not st["available"]:
            line = st["error"]
        elif st["speaking"]:
            line = "speaking (%s)" % (st.get("source", ""),)
        else:
            line = "idle - say something and the mouth will move"
        surface.blit(self.hud_font.render(line, True, HUD), (16, 12))


def run(size, fullscreen, fps):
    feed = SpeechFeed().start()
    screen_face = FaceScreen(feed, size=size, fullscreen=fullscreen)
    pygame.init()
    flags = pygame.FULLSCREEN if fullscreen else 0
    surface = pygame.display.set_mode(size, flags)
    # Which driver SDL actually chose, on the first line of the log: at boot this
    # is how a later reader tells "the session was not up yet" from "the face
    # drew somewhere nobody can see" without guessing.  Measured on this robot:
    # DISPLAY=:0 gives x11, an SSH-launched run with XDG_RUNTIME_DIR set gives
    # wayland, and both are the same panel.
    print("face_screen: %s driver on display %s, %dx%d"
          % (pygame.display.get_driver(), os.environ.get("DISPLAY", "(none)"),
             size[0], size[1]), flush=True)
    pygame.display.set_caption("Lance — face")
    pygame.mouse.set_visible(not fullscreen)
    clock = pygame.time.Clock()
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return 0
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    return 0
            now = time.monotonic()
            geo = screen_face.step(now)
            screen_face.render(surface, geo, feed.state(), now)
            pygame.display.flip()
            clock.tick(fps)
    finally:
        feed.stop()
        pygame.quit()


def selftest():
    """Drive the drawing from a synthetic curve: no display, no robot, no model.

    The model itself is `speech_face.py` and is checked by its own suite; what is
    checked here is that the panel turns the curve it is given into a mouth - and
    that it holds the closed end rather than jumping when a poll is missed.
    """
    print("== the panel draws the curve the robot publishes ==")
    sps = 100.0
    # Rising with the frame index, so a value says which frame it came from.
    ramp = [i / 200.0 for i in range(200)]
    feed = SpeechFeed()
    feed._state = {"available": True, "speaking": True, "text": "Bello! one two",
                   "source": "audio", "sps": sps, "window_s": 30.0, "received": 0.0,
                   "curve": [(v, 0.1 * v, 0.04) for v in ramp], "error": ""}
    face = FaceScreen(feed, size=(800, 480))
    drawn = [face.step(i / sps)["open_h"] for i in range(len(ramp))]
    peak, closed = max(drawn), min(drawn)
    print("   drawn opening px over the curve: peak %.1f, closed %.1f" % (peak, closed))
    print("   distinct openings: %d" % len(set(round(d, 1) for d in drawn)))
    assert len(set(round(d, 1) for d in drawn)) > 20, "the mouth did not follow the curve"
    assert peak > 100 and closed < 40, "the opening should track the curve at both ends"

    # a late poll reads further back into the window instead of jumping forward
    late, earlier = feed.mouth(1.0)[0], feed.mouth(0.5)[0]
    print("   a poll 1.0 s late reads %.2f where 0.5 s late reads %.2f" % (late, earlier))
    assert late < earlier, "a later poll must read an older frame of the curve"

    # the app stops reporting: the mouth rests closed rather than hanging open
    feed._state = dict(feed._state, received=-99.0)
    stale = [face.step(0.0)["open_h"] for _ in range(3)]
    print("   after the app stops reporting: %s px" % [round(s, 1) for s in stale])
    assert max(stale) < 20, "a status older than the window must rest closed"

    # nothing published yet (the app is down): a closed mouth, not a crash
    feed._state = dict(feed._state, available=False, curve=[], error="no app")
    print("   with no robot: opening %.1f px" % face.step(0.0)["open_h"])
    assert face.step(0.0)["open_h"] < 20, "the mouth should rest closed"

    # The shape itself, against the art it is drawn from.  A mouth built from
    # stacked filled ellipses passed every check above and still looked like two
    # lobes with a highlight floating on the lip in a photograph of the panel, so
    # the shape is checked in pixels, not just in formulas.
    print("== the mouth matches the reference art ==")
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    if not pygame.get_init():
        pygame.init()
    w, h = face.size
    shut = mouth_geometry(w, h, 0.0, 0.0, 0.0)
    open_geo = mouth_geometry(w, h, 1.0, 0.0, 0.0)
    mouth_w = shut["hw"] * 2 + shut["lip"] * 2
    print("   lip band is %.3f of the mouth's width (reference 0.04-0.08)"
          % (shut["lip"] / mouth_w))
    assert 0.04 < shut["lip"] / mouth_w < 0.09, "the lip band is not the reference's"
    print("   the mouth is %.3f of the panel wide (reference 0.65)" % (mouth_w / w))
    assert 0.55 < mouth_w / w < 0.75, "the mouth is the wrong size for the panel"
    # The reference's own mouth is nearly round, which a 16:9 panel cannot spend
    # the height on; what matters is that the jaw really drops instead of the
    # mouth staying a slit.
    tall = (open_geo["open_h"] + open_geo["lip"] * 2) / mouth_w
    print("   wide open it is %.2f as tall as it is wide (the reference's cell is ~1)" % tall)
    assert 0.5 < tall < 1.2, "a wide-open Minion mouth is not a slit"

    surf = pygame.Surface((w, h))
    surf.fill(BACKDROP)
    geo = mouth_geometry(w, h, 1.0, 0.25, 0.2)
    draw_mouth(surf, geo)
    cx, cy, hw, half_h = geo["cx"], geo["cy"], geo["hw"], geo["open_h"] / 2

    def band(axis, sign):
        """Lip pixels crossed walking out from the opening's edge, one radius."""
        if axis == "x":
            at = lambda k: (int(cx + sign * (hw + k)), int(cy))
        else:
            at = lambda k: (int(cx), int(cy + sign * (half_h + k)))
        n = 0
        for k in range(int(geo["lip"] * 2) + 4):
            x, y = at(k)
            if not (0 <= x < w and 0 <= y < h):
                break
            if surf.get_at((x, y))[:3] in (LIP_EDGE, LIP_TOP, LIP_BOTTOM):
                n += 1
            elif n:
                break
        return n

    bands = {"left": band("x", -1), "right": band("x", 1),
             "up": band("y", -1), "down": band("y", 1)}
    print("   lip band px at left/right/up/down: %s" % bands)
    assert min(bands.values()) >= int(geo["lip"] * 0.5), "there is no lip band somewhere"
    assert max(bands.values()) - min(bands.values()) <= 3, "the lip band is uneven (lobes)"

    palette = {BACKDROP, LIP_EDGE, LIP_TOP, LIP_BOTTOM, MOUTH_BACK, MOUTH_DEEP,
               TOOTH, TONGUE, TONGUE_DEEP}
    seen, strays, escapes = {}, {}, 0
    for y in range(h):
        for x in range(w):
            colour = surf.get_at((x, y))[:3]
            seen[colour] = seen.get(colour, 0) + 1
            if colour not in palette:
                strays[colour] = strays.get(colour, 0) + 1
            elif colour in (TOOTH, TONGUE, TONGUE_DEEP):
                if ((x - cx) / hw) ** 2 + ((y - cy) / half_h) ** 2 > 1.0:
                    escapes += 1
    print("   pixels: teeth %d, interior %d, tongue %d"
          % (seen.get(TOOTH, 0), seen.get(MOUTH_BACK, 0) + seen.get(MOUTH_DEEP, 0),
             seen.get(TONGUE, 0) + seen.get(TONGUE_DEEP, 0)))
    assert seen.get(TOOTH, 0) > 2000 and seen.get(MOUTH_BACK, 0) > 1000
    assert seen.get(TONGUE, 0) + seen.get(TONGUE_DEEP, 0) > 200, "no tongue when wide open"
    print("   colours outside the mouth's palette: %s" % (strays or "none"))
    assert not strays, "something that is not the mouth is being drawn"
    print("   teeth/tongue pixels outside the opening: %d" % escapes)
    assert escapes == 0, "the inside of the mouth is not clipped to the opening"
    print("   the shape, in pixels (Y lip, . throat, # tooth, t tongue):")
    for j in range(14):
        row = ""
        for i in range(52):
            colour = surf.get_at((int(w * (i + 0.5) / 52), int(h * (0.30 + 0.62 * (j + 0.5) / 14))))[:3]
            row += {LIP_TOP: "Y", LIP_BOTTOM: "Y", LIP_EDGE: "y"}.get(colour, "")\
                or {MOUTH_BACK: "m", MOUTH_DEEP: "."}.get(colour, "")\
                or ("#" if colour == TOOTH else "t" if colour in (TONGUE, TONGUE_DEEP) else " ")
        print("    " + row)
    print("FACE-SCREEN SELFTEST PASSED")


def main():
    ap = argparse.ArgumentParser(description="Lance's face on the robot's screen")
    ap.add_argument("--windowed", action="store_true", help="run in a window instead of fullscreen")
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--size", default="", help="WxH (default: the whole screen)")
    ap.add_argument("--selftest", action="store_true", help="check the animation without a display")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return 0
    if args.size:
        w, h = (int(v) for v in args.size.lower().split("x"))
        size = (w, h)
    else:
        # Info() only answers once the display subsystem is up; without this it
        # raises "video system not initialized" on an otherwise fine panel.
        pygame.display.init()
        info = pygame.display.Info()
        size = (info.current_w or 800, info.current_h or 480)
    return run(size, not args.windowed, args.fps)


if __name__ == "__main__":
    raise SystemExit(main())
