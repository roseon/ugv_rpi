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
GLOSS = (0xFF, 0xE9, 0xA8)
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
def mouth_geometry(w, h, open_, wide, smile):
    """Every dimension of the mouth, from its openness and two shape terms."""
    unit = min(w * 0.34, h * 0.60)
    cx, cy = w / 2.0, h * 0.54
    hw = unit * (1 + 0.20 * wide)
    open_h = (0.06 + 0.94 * open_) * unit * 0.60
    inner_up, inner_dn = open_h * 0.42, open_h * 0.62
    lip = unit * 0.20
    corner_y = cy - smile * unit * 0.16
    return {"cx": cx, "cy": cy, "unit": unit, "hw": hw, "open_h": open_h,
            "inner_up": inner_up, "inner_dn": inner_dn, "lip": lip, "corner_y": corner_y}


def teeth_row(surface, cx, half, y, h, count, colour):
    """`count` separate teeth with a 1 px gap, centred on cx, spanning 2*half."""
    y, h = int(y), int(h)
    width = max(2, int((half * 2 - (count - 1)) / count))
    x0 = int(cx - half)
    for k in range(count):
        pygame.draw.rect(surface, colour, pygame.Rect(x0 + k * (width + 1), y, width, h))


def draw_mouth(surface, geo):
    """Paint the mouth: lips first, then the opening, then what is inside it."""
    cx, hw = geo["cx"], geo["hw"]
    y0 = geo["corner_y"]
    up, dn, lip = geo["inner_up"], geo["inner_dn"], geo["lip"]

    def rect(x, y, w, h):
        return pygame.Rect(int(x), int(y), max(1, int(w)), max(1, int(h)))

    # lips: the whole mouth shape, then the opening on top of it — the band that
    # stays visible is the lip itself.
    outer = rect(cx - hw, y0 - up - lip * 2.3, hw * 2, (up + dn) + lip * 4.6)
    pygame.draw.ellipse(surface, LIP_EDGE, outer)
    inner = rect(cx - hw * 0.96, y0 - up - lip * 2.0, hw * 1.92, (up + dn) + lip * 4.0)
    pygame.draw.ellipse(surface, LIP_TOP, inner)
    lower = rect(cx - hw, y0, hw * 2, dn + lip * 4.2)
    pygame.draw.ellipse(surface, LIP_BOTTOM, lower)

    open_h = up + dn
    opening = rect(cx - hw, y0 - up, hw * 2, open_h)
    pygame.draw.ellipse(surface, MOUTH_BACK, opening)
    # The shading inside the opening must stay inside it.  An ellipse scaled and
    # nudged down within these factors is provably contained (0.72² + 0.032² < 1);
    # measured before, a taller one painted deep red over the lower lip.
    deep_w, deep_h = hw * 1.44, open_h * 0.68
    pygame.draw.ellipse(surface, MOUTH_DEEP,
                        rect(cx - deep_w / 2, y0 - up + (open_h - deep_h) * 0.55, deep_w, deep_h))

    # What is visible inside: the upper row of teeth hanging from the top edge,
    # the tongue below it, and a lower row once the jaw is open past a sliver.
    teeth_h = min(geo["open_h"] * 0.45, geo["unit"] * 0.11)
    if teeth_h > 1:
        teeth_row(surface, cx, hw * 0.86, y0 - up * 0.85, teeth_h, TEETH_TOP, TOOTH)
    lower_h = min(geo["open_h"] * 0.26, geo["unit"] * 0.06)
    if lower_h > 1 and geo["open_h"] > geo["unit"] * 0.16:
        teeth_row(surface, cx, hw * 0.80, y0 + dn * 0.80 - lower_h, lower_h, TEETH_BOTTOM, TOOTH)
    if geo["open_h"] > geo["unit"] * 0.14:
        tongue = rect(cx - hw * 0.58, y0 + dn * 0.55 - geo["open_h"] * 0.34,
                      hw * 1.16, geo["open_h"] * 0.68)
        pygame.draw.ellipse(surface, TONGUE, tongue)
        if tongue.h > 6:
            pygame.draw.ellipse(surface, TONGUE_DEEP, rect(tongue.x + tongue.w * 0.18,
                                                           tongue.y + tongue.h * 0.55,
                                                           tongue.w * 0.64, tongue.h * 0.42))
    pygame.draw.ellipse(surface, GLOSS, rect(cx - hw * 0.42, y0 + dn * 1.05 + lip * 2.2,
                                            hw * 0.44, lip * 0.6))


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
