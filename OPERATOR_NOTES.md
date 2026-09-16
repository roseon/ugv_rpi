# UGV Rover web UI — operator notes (Sep 2026)

## What is verified working (browser-driven against the live robot)
- Page load: telemetry live (CPU/RAM/voltage/RSSI/temp), zero console errors, all statics 200.
- D-pad: all 9 pads emit `T:1 {L,R}` frames with correct differential semantics
  (fwd `0.5/0.5`, spin `0.5/-0.5`, back `-0.5/-0.5`, release `0/0`), verified on the
  wire through the real socket.io connection.
- WASD: same drive path; keydown drives, keyup returns to `0/0`.
- Joystick: drags the gimbal (Pan/Tilt numbers move; gimbal frames sent).
- Robot-side safety clamp proven: an over-max `4.5/-4.5` emitted from the browser is
  clamped server-side to `0.5/-0.5` (verified in the previous pass via the deployed
  handler harness; the log line is `[drive] clamped ...` in ugv.log).
- LIDAR radar: live polling of /lidar_points + /lidar_status with the enable/disable
  toggle.
- Cameras (Sep 9): BOTH USB cameras verified producing frames — Microsoft LifeCam
  Studio (video2/3, used by the app, streams real JPEG via /video_feed) and Realtek
  "USB Camera" (video0/1, spare — captured a 614400-byte raw frame via v4l2-ctl).
  Only one camera is used by app.py at a time (first found, here video2).

## LIDAR D500 — RESOLVED: wrong baud, not hardware (Sep 9, late)
The D500 (STL-19P core) connected directly through its CP2102 adapter emits its
native stream at **921600 baud** — NOT the 230400 the vendor code (and every
earlier diagnosis) assumed. Reading a 921600 stream at 230400 yields exactly the
"saturated high-entropy garbage with zero 54 2C headers" signature that was
misdiagnosed as a half-seated ZH1.5T cable for most of the session.

Proof chain: multi-baud sweep on /dev/ttyUSB1 showed 639 `54 2C` headers in
30KB only at 921600; offline validation: 499/500 CRC8-valid 47-byte frames,
motor 3551 RPM, real distances 0-2098mm. The cable was fine all along.

FIX (base_ctrl.py): `_pick_lidar_baud(port)` returns 921600 for /dev/ttyUSB*
(direct CP2102 adapter) and 230400 for /dev/ttyACM* (ESP32 base-board relay).
DTR/RTS are asserted on open (the adapter drives the sensor's motor PWM via DTR;
asserted = motor runs at full speed; a floating PWM pin = internal 10Hz control).
Also: the ultrasonic `sensor_data_ser` no longer grabs the lidar's ttyUSB port.

VERIFIED LIVE: streaming true, ~1800 frames/s, full revolutions every ~0.1s,
/lidar_points serves 360/360 1-degree bins, WPF Radar tab reads "streaming
(1800 frames/s)", 356 pts, nearest 0.43m, avoidance ACTIVE and reacting.

KEY LESSON: never conclude hardware without sweeping the baud. Undersampling a
921600 stream at 230400 looks like saturated noise at the *right-looking* data
rate — the exact trap this session fell into.

## Operational gotchas learned the hard way
1. **WiFi flakiness is environmental.** The robot dropped off the network twice in one
   session (~1 min, then ~6+ min), with RSSI already reading `-32 dBm → unstable`
   beforehand. If the UI suddenly dead-ends, ping the robot before debugging software.
   The browser page will silently sit on a dead socket — that is the next UX gap to fix
   (reconnect banner + `socket.on('disconnect')` handling).
2. **Server prints are block-buffered under the cron autorun.** `print()` diagnostics
   (including the `[drive] clamped` receipt line) can sit unflushed in ugv.log. The
   deployed app.py now reconfigures stdout/stderr to line-buffered at startup, so the
   log is trustworthy for live debugging. Launching with `python -u` also works.
3. **Deploying**: back up first (`~/ugv_rpi/backup_20260908/` holds pass-1 and pass-2
   copies), scp app.py / templates, then restart:
   `kill $(pgrep -f 'ugv-env/bin/python /home/ws/ugv_rpi/app.py')` and relaunch via
   `setsid ... python -u app.py >> ~/ugv.log 2>&1 &` (or just reboot the robot —
   cron relaunches it via autorun.sh).

## Known-not-fixed (hardware or deferred)
- No camera detected (USB/CSI): video area shows the placeholder; feed returns
  "No camera detected" frames. Hardware.
- LIDAR serial opens but streams no data: hardware/power.
- cv_ctrl.py still contains a dead duplicate `execute_command` (second shadows first).

## PENDING DEPLOY (Sep 2026, latest pass)
The robot's WiFi degraded mid-session (66% packet loss; SSH cannot complete), so the
following files are **verified locally but not yet on the robot**:
- `app.py` — stdout/stderr line-buffering fix (log lines land instantly)
- `templates/index.html` — connection banner + video-state overlay + Retry button +
  button acknowledge states
- `templates/conn.js` — NEW: connection-health banner + camera-ready polling module
  (behavior-verified in a local browser harness; see below)

To finish the deploy when the link stabilizes, from the project root run:
    bash deploy.sh
It backs up current files to `~/ugv_rpi/backup_20260908/*.pass4`, uploads the three
files over one connection, and stops the app process (cron/autorun or a reboot then
relaunches it). It needs `/tmp/ap.sh` (the SSH askpass helper) to exist.

conn.js behavior verified in-browser (all transitions): no-camera → amber banner +
Retry visible; socket drop → red "reconnecting" + overlay; camera ready → banner
hidden; Retry → POST /retry_camera, button disables, stream re-polled.

## Self-drive subsystem structure (Sep 11 restructure)
The planner used to be one 630-line `self_drive.py` holding memory, detection
handling, pursuit and odometry. It is now six modules, each understandable
alone. Read this before changing anything in the chain:

| module | owns |
|---|---|
| `perception.py` | the robot-frame conventions — LIDAR `+180°` offset, camera bearing sign — plus pure scan helpers (`sector_min`, `turn_bias`, `arc_clearance`) and object-label matching |
| `robot_state.py` | reading live sensors: `LidarScan` (applies the angle fix **once**, at the boundary) and ESP32 wheel odometry |
| `spatial_memory.py` | the learned occupancy grid + object memory + `surroundings.json` (version 2; v1 maps were rotated 180° and are refused) |
| `detection_source.py` | which detections the planner sees: the live COCO stream plus a throttled `detect_world()` open-vocabulary top-up |
| `target_pursuit.py` | the target's state (`set`/`clear`/`observe`/`seen`/`halt`) and the pursuit rules (arrive, aim, veto, sweep) |
| `self_drive.py` | the thread lifecycle (`warm`/`enable`/`disable`/`pursue`), the tick, the heading choice + veto application, and the `status()` payload |

Rules that keep it that way:
- **Angles**: only `robot_state.LidarScan.read()` converts raw angles; everything
downstream (memory, planner, avoider helpers) works in the robot frame
(0 = forward, + = left). Never subtract `pi` anywhere else — that duplication is
what produced the 180°-inverted map and the inverted camera bearing.
- **Callers use transitions, not flag surgery**: `enable()` / `disable()` /
`pursue(name)` / `clear_target()` / `warm()`. `disable()` is pause + drop target
+ stop thread; the routes, the CLI, Lance and the boot path all call the same
methods, so "self-drive off" cannot drift between channels.
- **Memory is read through the planner's shortcuts** (`save_memory`,
`load_memory`, `clear_memory`) or `planner.memory` for reads.
- **Changing heading policy** (weights, veto, aim) lands in
`SelfDriver._choose_heading`; changing what a target *is* lands in
`target_pursuit.py`.

### The arrival stop, and the three rules it depends on
Three things had to agree before "drive to the chair" could end in a stop at
the chair. Each has exactly one home; change them there.

1. **The planner's `halt` outranks the avoider's forward motion** —
`LidarAvoider._decide` (app.py) checks `_planner_halted()` *before* the EVADE,
SLOW, post-evade-curve and CRUISE branches and reports the new `HOLD` state.
Only the `<250 mm` emergency reverse is exempt: it is the only way out of a
jam, and suppressing it would wedge the robot. Before this, `halt` was honored
in CRUISE alone, so the 450-700 mm band drove the robot on at 20-35% speed
*while the planner was saying stop*. `selfdrive_selftest.py` asserts the
ordering from app.py's syntax tree (it cannot be imported off the Pi).
2. **The arrival range is the object's own bearing** —
`target_pursuit.Target.observe(dets, scan)` measures the nearest LIDAR return
within `TARGET_CONE_DEG` (10°) of the detection's bearing, and treats *no
return* down that bearing as `OBJ_RANGE_MAX`. It deliberately does not fall
back to the forward cone: that cone measured whatever was nearest in front — on
this floor a wall 0.5 m away across +40°..+150° — so arrival used to fire at
the wall's distance and the robot stopped short. Collision avoidance is the
avoider's front cone, not this estimate.
3. **"drive to the X" is read, not guessed** — `lance.parse_pursuit_intent()`
routes verb + preposition + object straight to `_approach`, before the language
model is consulted. The model's own prompt advertises "drive towards the
chair" and it answered that phrase with a two-second forward drive, so the one
thing the user asked for was the one thing that did not happen. Extend the
verb/preposition lists there, not the prompt; anything the parser declines
still goes to the model. Not-objects (directions, measures) and clause breaks
("…the bin **and then** forward") are excluded on purpose.

4. **Arrival is sticky, and that lives in one place** —
`target_pursuit.PursuitPolicy.arrival_hold(target)` is the only arrival
question the planner asks. It returns True for the tick that first reaches the
object *and* for every tick after, including ones where the object has left the
camera view — which is exactly what happens once the robot is on top of it.
Deciding arrival from scratch each tick (`has_arrived`, removed) released halt
the moment the object left the frame, so the robot arrived, dropped `halt`, and
drove off again under the `looking for … — sweeping` path. It still releases
halt when the object is seen again beyond `arrive_m`, so a target that moves
away is re-approached rather than frozen forever. `Target.release_halt`
(unused) was deleted with it. Live proof: `arrived`, `halt=true`,
`avoidance_state=HOLD`, wheel odometry frozen for 20 s with `seen=false`.

Measured floor geometry at the last verification (robot stationary): straight
ahead open to 1439 mm; a wall 495-760 mm across +40°..+150°; ~1150 mm to the
right. That left wall is what held the old front-cone reading at ~580 mm, which
is also why the SLOW band was the state that mattered.

Note on `/send_command`: it is the **CLI**, not a JSON endpoint. A raw JSON POST
to it is silently dropped (`cmdline_ctrl` wants `command=base -c {"T":1,…}` in a
form field), which is how a previous pass concluded the chassis could not move.
With the correct form the wheels turn: a 4 s forward command moved the front cone
459 -> 179 mm and advanced the ESP32 odometry in the commanded direction.

## Mapping is short-horizon by design, and steering does not wobble

5. **The learned map must be able to forget.** Hit counts used to be capped at
65535 and decayed one per `DECAY_SEC`, so a cell observed for a minute held
hundreds of hits and could never clear. After a drive the grid read blocked in
every direction — measured `clearance = 0.000` for all 13 headings — so the
`W_MEM` term was a constant offset and the learned map contributed nothing to
steering. Now `HIT_MAX = 5`, the same ceiling applies to camera-fused object
disks (`observe_object` used to climb to 65535 independently), and `load()`
clamps an old file on read. An unobserved cell clears in ~30 s at
`DECAY_SEC = 6`. This is a short-horizon obstacle memory, which is the honest
shape for a robot-centric grid with no localisation. Live after the fix: max
hits 5, hit histogram spread 1..5, and the 13 heading scores spanning -0.15 to
+3.10 (they used to differ only by the live-LIDAR term).

   Decay only runs inside `observe_lidar`, i.e. while the planner ticks. A
   paused robot keeps its last map — deliberate — and the first tick after
   resume clears the stale guard (2768 -> 871 cells within 2 s, observed).

6. **Heading flicker is what "drove left and right" meant.** `HEADINGS` is a
discrete set and neighbouring scores differ by less than scan noise, so the
winner flipped (e.g. +0 <-> -15) every tick and `best / 90.0` answered each flip
with a full wheel differential. Three guards live in `self_drive.py`:
`HEADING_SMOOTHING` (EMA of the chosen heading), `TURN_DEADBAND` (residual turn
snaps to exactly 0 so straight really is equal wheel commands) and
`TURN_SLEW_PER_TICK` (rate limit). A hold-until-beaten-by-margin rule was tried
first and rejected: in a clear room straight beats a held 30 deg by only 0.10,
so the robot never straightened out at all. Live turn series over a 15 s drive:
sign held for long runs (`+0.00` x4, `+0.17` x5, `-0.33` x3, `+0.50` x5) with
rate-limited ramps, versus tick-by-tick sign flips before.

## Serial device identity has one owner (and the left-eye cause was wrong)

7. **Which USB serial node is which — never guess by glob prefix.** Four links
exist and only one is the eyes: the eyes' Arduino Uno is `/dev/ttyACM0`
(`2341:0043`, observed in dmesg as `cdc_acm 1-2:1.0: ttyACM0`), the D500 LIDAR
adapter is `/dev/ttyUSB0` (`10c4:ea60` CP2102N, confirmed live via
`/dev/serial/by-id/`), and the base controller is `/dev/ttyAMA0` (the Pi's GPIO
UART, not USB at all). `base_ctrl._pick_lidar_port()` used to fall back to
`acm[0]` whenever no ttyUSB bridge existed — that opens the **eyes' Uno** as the
lidar, holds the port, and `kick_lidar()` DTR-pulses it, which resets an Uno. A
second rule, `_pick_lidar_baud` choosing 921600 by `port.startswith('/dev/ttyUSB')`,
would then have driven the Uno at the lidar's rate. Identity now lives in one
owner, `serial_ports.py` (`uno_port()`, `lidar_port()`, `base_port()`,
`baud_for()`, `other_ports()`), read from `idVendor`/`idProduct` and
`/dev/serial/by-id`; both duplicate methods are deleted rather than fenced off,
and `base_ctrl` no longer globs at all. Regression harness:
`serial_ports_selftest.py` (35 checks on the Pi).

8. **An Uno that fails to configure makes no node at all — and the measured cause
is host-side, NOT the display load.** `lsusb` still lists the Uno, which reads as
"plugged in but broken", but the kernel says `usb 1-2: can't set config #1, error
-62` (with `xhci-hcd: Timeout while waiting for configure endpoint command`) and
`/dev/ttyACM0` never appears, so no gaze command reaches the eyes. In the same two
minutes the Pi's I2C host (`i2c_designware`, hosting the DSI touch panel) and the
**VideoCore firmware mailbox** clock (`fw-clk-arm`) also timed out, and
`get_throttled` is `0x0`. Three unrelated hosts on the Pi failing together is a
**Pi-side host event**: an Arduino 3.3 V rail cannot reach any of them, the burst
lasted 2 minutes then stopped for 36 (a steady overload is not a 2-minute event),
and `error -62` is `-ETIME` with the Uno's descriptors having read fine moments
earlier, so its bridge was alive. The I2C host answers again now (`EREMOTEIO` on
free addresses = transfer completed, not `ETIMEDOUT`), so the event cleared while
the Uno stayed stuck with zero interfaces. Recovery is a **port re-enumeration** —
physical replug, or root-only sysfs (`authorized`/`remove` and
`/dev/bus/usb/001/00N` are not writable by `ws`). The TFT 3.3 V budget is a real
precaution against *display* corruption but is NOT the diagnosis here. Cheapest
check: pull both displays' VCC+LED, replug the Uno, watch for `/dev/ttyACM0`.

9. **`base_ctrl.ReadLine.__init__` opened the lidar before its own state
existed.** `open_lidar_serial()` ran early in `__init__` and touched `lidar_ser`
and `_lbuf` before either was assigned, so the first open always raised
AttributeError, logged `[lidar] open failed ...`, and only recovered via the
reconnect that app.py's reader loop triggers. The open now happens at the end
of `__init__`; the log shows `lidar serial connected succeed on /dev/ttyUSB0 @
921600` on the first attempt with no failure line.

## The mouth: the robot's speech has one owner, and a desktop face (Sep 15)

| piece | owns |
|---|---|
| `speech_face.py` (`FACE`) | **the mouth's motion, once**: what is being said, when it started, the loudness measured from the very WAV bytes that go to `paplay`, and the model that turns that into the `(open, wide, smile)` curve both faces draw. Advanced lazily by whoever polls, at `SPS`, so the same syllable makes the same motion on the panel and on the desktop. |
| `cv_ctrl.speak_minion` / `play_speech` | brackets playback with `FACE.begin(text, wav)` / `FACE.end(seq)`, so the state follows the audio that actually plays, whether or not a speaker is attached. |
| `app.py` `GET /speech_status`, `POST /api/say` | the wire: `speaking`, `text`, `source`, `sps`, `window_s`, `seq`, and the last 1.5 s of the curve as `open`/`wide`/`smile`. `/api/say` takes JSON **or** a form body and returns at once (playback runs on its own thread); it answers `409` while the robot is already speaking. |
| `CommandCenter/Core/SpeechStatus.cs` | parses that payload and indexes into the curve against the local clock (`Mouth(now)`), so a late poll reads further back instead of jumping. One freshness rule (`Live`, using the robot's own `window_s`): past the window the mouth rests closed, because a dead robot's mouth must not stay open on its last frame. |
| `CommandCenter/Panels/MouthView.cs` | the drawing only, from that one curve — and the one thing genuinely local: the pointer can drag the mouth open by hand. It used to hold a second copy of the jaw timings, breathing and width, character for character with the panel's. |
| `CommandCenter/Panels/FaceBar.xaml(.cs)` | the strip on every tab: live mouth, the speaking/idle line, a say box, and the *open it when it talks* checkbox. Also the reason the mouth is not moving (no robot / no `/speech_status` / the robot's refusal / idle).
| `CommandCenter/FaceWindow.xaml(.cs)` | the big face, topmost and unowned (an owned window vanishes when the Command Center is minimised — exactly when a face is wanted). Opens itself on speech, remembers where you put it, and shows explicitly with `WindowState.Normal` because a window first shown while the app is minimised comes up minimised.

The retired **Mouth tab** is gone: its content is the bar, so the face is visible on every tab rather than only while you are looking at it. Measured live on the big face (window captured by `PrintWindow`, mouth measured in pixels): idle opening 6–9 px, speaking up to 77 px with per-syllable variation, rank correlation against the robot's own level **+0.75 at a 100 ms lag**; a real mouse drag held it at 77 px and it eased back to 9 px within 0.7 s after release.

### One owner of gaze smoothing
Two easing stages used to be stacked — `EyeGazer` walking the aim toward each
detection, and the Uno easing again with `GAZE_SPEED`. The firmware now draws
what it is sent (`GAZE_SPEED 1.0`); the only filter is `eyes_gaze.py`, with its
time constant **bounded by the detector interval** (`min(0.15 s, interval/3)`),
not chosen. `GAZE_STEP_MAX 5.5` stays, because a frame's cost is proportional to
the distance it paints (measured 189 scattered fills = 54 ms on the bit-banged
left panel), so a long move is spread over several cheap frames rather than
being a second filter. Measured on the Uno with `GAZE?` (drawn pupil vs the aim
it had been sent) while the aim ramped at a head's speed, flashing the same
sketch with and without the firmware's ease — **4.2 aim units (5.9 px) with it,
3.1 (4.3 px) without**,
which is one frame of travel. A head-sized step is unchanged either way (25 aim
units, 2.32 s vs 2.15 s to come within 1 unit), because the frame cap governs it.

### What the mouth's structure is now, after the deletions

One definition of the motion, two renderers:

```
speech_face.Speech (on the robot)   the model: level -> (open, wide, smile), at SPS,
   |                                plus `window_s` so nobody re-derives the window
   v
/speech_status                      the last 1.5 s of that curve, timestamped on arrival
   |
   +--> face_screen.py  (panel)     indexes the curve with time.monotonic
   +--> SpeechStatus.cs (desktop)   indexes it with DateTime.UtcNow
```

Both sides index the *same* way - newest frame at the payload's arrival time, and a
frame further back the longer ago that was - and both rest closed once the payload is
older than `window_s`, so a dead robot's mouth cannot stay open on its last frame.
The clock is UTC on both sides of the subtraction; a status stamped in UTC read
against a local `now` is four hours in the future, which pinned the desktop to the
newest frame of each poll and threw the rest of the curve away (measured: the drawn
peak went from 30 px to 79 px once both ends agreed).

Deleted this pass, having been added by earlier proving passes and reached by nothing:
`POST /voice` with four presets and a runtime `voice.json` (the voice is a definition,
not a choice), `Mouth.Speaking`/`Mouth.Level`, the desktop's own jaw timings and
envelope interpolation (`LevelAt`, `ElapsedS`, `DurationS`), and the `1.5` re-derived
in two renderers. The `TYPE 4` hardware-SPI firmware path, `start_face.sh`'s boot
installer and the single `voice.py` owner stay: each is something a later request
depends on.

## The Minion face: eyes and mouth from the reference art (Sep 15)

The eyes and the mouth are styled from the Minion goggle/mouth reference the user
supplied. What that changed, and what it cost:

| piece | now draws |
|---|---|
| `eyes_tft.ino` scene | one lens per panel: black strap knuckle at each side, then the goggle ring `r=114` (metal `#B9BEC2`, `#8A9095` outline), the yellow eyelid ring `r=105` (`#F5C842`), the white sclera `r=95`, and the brown iris `r=32` (`#9B4A24`) with a `r=16` pupil and a `r=6` white highlight up-left. RGB565 values were computed, not eyeballed (`0xBDF8`, `0x8C92`, `0xF648`, `0x9A44`). |
| `eyes_tft.ino` gaze | the iris travel limit is now `sclera - iris - 2 = 61 px` instead of 69, because the goggle's two bands shrank the white the iris is allowed on. A per-axis clamp would push it onto the yellow at diagonal gazes, so it stays the unit-vector rule. |
| `face_screen.py` + `MouthView.cs` | the mouth, from the reference's own proportions: yellow lips (`#F7CE4A` top, `#E3AA2E` lower, `#C98B1E` edge), maroon interior (`#6E1B2A`, `#450E19`), a lip band `0.14` of the mouth's half-width, a jaw opening to `1.05` of it, 7 white teeth with 1 px gaps hanging from the top of the opening, 5 more rising from the bottom once it is open, and a red tongue (`#E06B6B`, `#B84F4E`) in the throat between them. Both files carry these same factors **and draw the same two arcs**: the mouth is a smile, not an oval — its upper and lower edges are arcs through the same corners, the upper shallow and the lower deep, so the shape is a banana with the corners lifted, and the lift grows with the jaw (`bend = open_h/2 + hw * (0.07 + 0.22 * smile)`) so it stays a smile when the mouth is wide open. On the panel the arcs are sampled (`mouth_edges`, `mouth_polygon`); on the desktop each edge is one quadratic bezier, whose control point is twice the arc's apex less the corners it joins — the same curve, not a redrawing of it. |

Measured, since an Uno with no panel attached cannot be photographed:

- **The firmware's drawing is replicated and asserted** (`goggle sim`, run before
  deleting it): on the glass — metal ring 5,992 px, yellow eyelid 6,276 px, white
  sclera 25,136 px, iris 2,412 px, pupil 684 px, highlight 113 px, strap 4,154 px —
  and at the travel limit in all eight directions the furthest iris pixel is
  `+0.0 px` beyond the sclera edge, i.e. it never rides onto the yellow.
- **The panel mouth, live** (grim; note the session is labwc, so the face is an
  Xwayland client and the X root window scrot sees is black): the mouth is 539 px
  wide, the lip band is **even** — 33/33 px across and 33/32 px down — and it runs
  15 px closed → 187 px wide open, with teeth 4,118 → 45,455 px and a tongue
  0 → 11,804 px. The highlight colour the old drawing painted on the lip
  (`#FFE9A8`) appears **0 px** anywhere on the panel.
- **The desktop mouth, live** (face window captured across a sentence): 534–555 px
  wide and 179 px closed → 257 px open, against the panel's 539/253 for the same
  jaw; lip ~22,000 px, teeth 25,000 → 40,000 px, maroon 3,500 → 8,800 px, tongue
  0 → 8,100 px as it opens.
- **The smile, measured on the glass** (same grim capture, same palette classifier,
  before and after this change): before, the mouth was 547 × 90 px and the corners
  sat **36 px inside the middle's span on both edges** — 190..279 px at the middle
  against 226..243 px at the tips, symmetric, which is an ellipse. After, the mouth
  is 548 px wide × 116 tall and **the corners sit 27 px above the middle along the
  top edge and 109 px along the bottom edge** (at rest); mid-sentence, 22/109 px
  with a 2,478 px tooth row following the arc and the throat maroon beneath it. The same measurement on the desktop's big face, captured while the
  robot spoke through the app: a 474 × 103 px lip blob whose corners ride 16 px
  above the middle on the top edge and 101 px on the bottom — the panel's shape,
  drawn by the other renderer.
- **The shape is checked in pixels, not formulas** (`face_screen.py --selftest`,
  run on the robot: 0 failures). It asserts that both edges **rise** at the
  corners (an oval's are 0 and 0 — which is exactly what the panel drew for as
  long as this mouth existed, and every formula check passed), that the lip band is
  one thickness across the deep part (31–35 px of a 33 px lip), the palette inside
  the mouth, teeth and tongue never outside the opening, the reference's
  proportions, and prints an ASCII map of the result.

The panel's mouth used to be built from stacked filled ellipses — an outer ring, an
inner fill and a separate lower lobe — with a fixed highlight ellipse placed on the
lower lip by a formula that had nothing to do with the band. On the glass that read
as two yellow lobes with a pale orb floating on them, which is the photograph the
user sent. It is now one ring plus an opening, the tooth rows are cut to the arc
they hang from (so nothing needs clipping), and `face_screen.py --selftest` measures
all of it in pixels — band evenness across the mouth, the palette inside the mouth,
teeth and tongue never outside the opening, the mouth's share of the panel, the
rise of both edges at the corners, and an ASCII map of the shape — because the
formulas passed while the panel looked wrong, and kept passing while it drew a flat
oval where the reference draws a grin.

**The Uno is now at 87% of flash (28,082 bytes, 1,232 bytes of RAM free)**, down
from 99% (32,162 bytes, **94 bytes free**). The `ST7735` and `ILI9341` paths that
`avr-nm` had measured at 498 + 486 bytes of linked driver code — for panels this
robot does not have — are deleted, along with the `TAB` command that existed only
to probe them. The reclaimed build was then **flashed through the Pi's own
toolchain and proved on the device**, not in the link map alone:

| evidence | what it showed |
|---|---|
| `avrdude` | `Writing 28082 bytes to flash ... 100% 4.73s ... Avrdude done.` (the write, not just a compile) |
| boot banner | `EYES FW v5.3 - Minion goggle (GC9A01A)`, `EYE1/EYE2 init ... r=114 lid=105 sclera=95 iris=32`, `EYES READY` — v5.3 exists only in this sketch; `HEAD` has no such banner at all |
| command surface | `PING`→`PONG`; `TYPE 3 3`→`TYPE OK: LEFT=GC9A01A (bit-bang) RIGHT=GC9A01A (bit-bang)`; `GAZE?`→`GAZE aim 15.0 15.0  drawn -42.7 -42.7` |
| travel clamp | the drawn offset at `T 15 15` is `61 × (15−50)/50 = −42.7` — the **sclera-derived** radius (95−32−2), not the 69 the panel radius gave before the goggle, so the new art's geometry is running |
| repaint telemetry | `REPAINT n=25 avg=112..187ms fills≈335 px≈2600` — the iris is really being redrawn |
| live gaze | 60/60 and 59/59 polls with a head box, `sent` 41→69, `errors` 0, `port /dev/ttyACM0`, vertical residual ≤0.4 units |
| upload with the app running | `Error: protocol expects sync byte 0x14 but got 0x00` / `programmer is out of sync` / `unable to write flash`, exit 1 — the app owns the Uno, so it must be stopped first. Now written into the guide (§5), because the guide previously said only "upload" |

The goggle is on the glass. What no remote check can show is the picture itself:
the panels have no readback on this wiring (their MISO is unconnected by design),
so "what the eye looks like" is proved by the firmware's own geometry banner and
the drawn-offset values, not by pixels.

**Both gaze axes now obey one rule** — the mismatch this file used to record as an
open gap, closed in the pass that followed. The gaze's x went through an angle
(`box_bearing_deg()`, a 60° horizontal FOV spread over the panel's 120° cone, i.e.
0.83 aim units per degree) while y was the raw frame fraction
(`py = cy * 100 / height`), which works out at 2.14 units per degree — **2.56x as
sensitive**, so a head slightly above the axis swung the pupils far further up the
screen than the same offset to the side moved them. Measured on the live robot by
matching the chosen detection's head box against the aim the eyes were sent:

| over 50 samples | x | y |
|---|---|---|
| before, vs the angle rule | **0.22 units** | 16.91 |
| before, vs the pixel rule | 4.24 | **0.28** |
| after, vs the angle rule | 0.19 / 0.21 / 0.17 (three runs, 504 samples) | **0.30 / 0.24 / 0.25** |
| after, vs the pixel rule | 6.5 – 10.1 | 13.1 – 16.8 |

`perception.box_elevation_deg()` is the vertical counterpart of
`box_bearing_deg()`, taking the vertical FOV from the frame's aspect (640x480 at
60° across gives **46.8°** down, not 60°), and `eyes_gaze.elevation_to_py()` maps
it with the same cone `bearing_to_px()` uses. The camera dict handed to
`choose_gaze()` now carries two angles and no screen coordinate, so there is one
mapping rule in one place and no caller can smuggle in a second. A head at frame
centre still aims at (50, 50) — the two rules always agreed there — and the head
anchor (`head_box`) is untouched.

What the live sweep could not cover: the person in front of the robot never stood
past ~0.56 of the frame, so the largest right-hand excursion measured is +3.8°
(aim px 53) against 16° to the left, with y measured 4.3°–19.2° above the axis.
Both signs and the extremes are pinned in the harness (`elevation_to_py(±30)` ->
25/75, ±60 -> the edges, beyond the cone -> clamped). A live sample on the robot's
right needs either a person standing there while `/eyes_status` is sampled, or the
chassis pivoted ~20°, which this pass did not do: a LIDAR return sits 267 mm
behind the robot and nobody was watching the floor.

### The voice has one owner, and the robot's own screen has a face (Sep 15, later)

Every speech path used to synthesise for itself — Lance's replies through one
Azure voice, the UI's own lines ("Lights activated") through pyttsx3 — so the
robot spoke in two voices. There is one owner now:

| piece | owns |
|---|---|
| `voice.py` | the voice: the Azure credential, the SSML (pitch and rate that make the register), synthesis over REST (the SDK's WebSocket fails after a reboot while HTTPS works), `paplay`, the local fallback, and the robot-wide lock that stops two sentences talking over each other. A definition, not a choice: there were four presets with a `POST /voice` picker that no UI could reach and nobody asked for, so that surface is gone. |
| `minionese.py` | **what is said**: the words, trained at import from `minionese_corpus.txt` (see below). It used to be six hardcoded interjections glued in front of the caller's English here, which is English in a costume, not Minionese. |
| `face_screen.py` | the face on the robot's **own 800×480 panel**: it polls `/speech_status` a few times a second and indexes the same curve at 60 fps, so the app does no drawing and the two faces cannot drift. It is a second process (it owns a display), so it is not a module `app.py` imports. |
| `start_face.sh` | the whole lifecycle — `bash start_face.sh`, `--stop`, `--status`, `--selftest`. No caller may spell `face_screen.py` in the same shell as a `--stop`: the module's name in that shell's own command line makes `pkill -f` kill the shell, which looks exactly like a dead robot (no output, exit 255). Measured twice. |
| `deploy.sh` `EXTRA` | the two files above ship even though nothing imports them; the module set stays derived from `app.py`'s imports, and a file with no importer has to be named explicitly. The far-side `py_compile` is fed only the `.py` files — passing it the bash launcher printed a SyntaxError and, with no `set -e` remotely, still exited 0. |
| `deploy.sh` `CORPUS` / `ENGLISH` | `minionese_corpus.txt` and `minionese_english.txt` ship like the weights do, for the same reason: they are data the manifest cannot find. One deploy was watched writing **no** corpus while reporting every module present — the app then says the caller's English — so the far side now prints what the model trained to (`[check] minionese trained: (353, 107)`) and how much of it English reaches (`[check] english side covers: 100 of 100 meanings`) rather than only listing files. |

Measured on the panel (captured with **grim** — this session is labwc, so the X
root that `scrot` grabs is black while the composited output is not): idle
opening **16–18 px**, speaking **10 → 143 px across 63 distinct openings**, and
correlation against the robot's own published level **+0.89**; the widest frame
drew 143 px where the model predicts 136. The caption appears only while
speaking (0 → 1,405 caption pixels), so the panel shows the words as well. The
app's own command line reaches the same voice: `POST /send_command` with
`command=audio -s Lights activated` speaks **"Sos poohanan nonnichu"** — Minionese
through a second entry point, so the language is not wired into `/api/say` alone.

### Minionese: the language, trained from the user's corpus (Sep 15)

Six hardcoded interjections in front of the caller's English is English in a
costume. The user supplied the vocabulary and the dialogue, so the words are now
learned from it:

| piece | owns |
|---|---|
| `minionese_corpus.txt` | the user's own data: the film's dialogue, a Minion–English list and a Minion–French one. Shipped by `deploy.sh` like the weights, because the manifest cannot find a data file. |
| `minionese_english.txt` | the English side of the corpus's own 100 meanings, because part of the corpus is taught only in French (`Moka : S'il vous plaît`, `Grazi : Merci`) and an English speaker could not reach those at all. Data, shipped like the corpus; it supplies no words and no openings of its own. |
| `minionese.py` | the training pass, at import, and what it yields: the **table** (124 corpus pairs + 100 English meanings → 353 keys), the **parsed corpus** (`entries`) and its **English column** (`english`) — what makes the reachability check testable against the source rather than against its own keys — the **start distribution** (101 opening words, sampled from the corpus instead of a hardcoded list), and an **order-3 character model backed off to order 2** (107 Minionese words) that builds a word the table has never seen, seeded by that English word so it is stable. `voice.py` is the only caller. |
| `minionese_selftest.py` | 87 checks: every one of the corpus's 124 entries reachable from its own taught words, the English column covering every corpus meaning and reaching only Minionese the corpus taught, that a taught word means one thing alone and in a sentence, the numerals, the dropped articles and copulas, the character model's shape and stability, the openings, the edge cases, and that `voice.ssml` really speaks Minionese. |

Three parsing defects came out of testing rather than reading: the dash rule was
eating the hyphen in `Bi-do` and splitting it into a nonsense pair; the block's
French section headings are pasted with no separator, so they glued onto the
preceding translation and silently cost the numerals 4–10; and an unbounded
character walk produced twelve-character mush (`bonononjoutt`) until it was
bounded to 4–8 letters ending open.

Live on the robot — through the app's own path, Azure WAV played, and no fallback
in the log:

| asked | spoken |
|---|---|
| `The battery is low, please charge me.` | `Tulaliloo kissupay kapayego, piperwea dokapota me.` |
| `Hello! Thank you for the banana.` | `Matoka bello! Tank yu nananana banana.` |
| `I see a person three metres ahead.` | `Pika kononono kyupayee megamoku sae chocolok konnionn.` |
| `audio -s Lights activated` (app command) | `Sos poohanan nonnichu` |

The panel animated every one (mouth 87 px closed → 283 px open, caption ink on
screen), and `three` → `sae` shows the numerals surviving inside a sentence.
### The whole corpus is reachable from English (Sep 15)

Indexing only the corpus's translation side left words it plainly teaches
unreachable, so every entry is now reachable from **both** sides: the taught side
becomes a key (English *and* French, because the corpus teaches both), and a word
that exactly one entry uses resolves to that entry's Minionese — which is how
`sorry` (from `I'm sorry`) reaches `Bi-do` and `hungry` (`I'm hungry`) reaches
`Me want banana` instead of being invented. A word several entries would answer
with (`you`) is left to the invented path rather than resolved by whichever entry
came first, and the longest taught phrase always wins the match, so a phrase is
never read word by word.

Measured against the corpus itself, not against the table it built:

| measurement | result |
|---|---|
| corpus entries whose taught side failed to become a key | **0 / 124** |
| entries whose key translates to something other than that entry's Minionese | **0** |
| taught keys that drift between alone and inside a sentence | **0 / 228** (and 0 with the English column, which is checked in the suite) |
| taught words the corpus repeats for different Minionese (`bonjour`: Bello, Aloha, Konnichiwa) | 2, resolved by the first entry — deterministic, so a word still means one thing |

Live, several taught words in one sentence through `/api/say`:
`Hello! Sorry, I am hungry. Stop, thank you, goodbye. One apple, look!` →
**`Idiot bello! Bi-do, muakazik Me want banana. Stupa, Tank yu, Poopaye. Hana Bapple, Luk at tu!`**
— 9/9 taught words reached, panel animated 83 → 255 px with 2,588 caption pixels
of ink. The app's *other* entry point speaks it too: `POST /send_command` with
`command=audio -s Hello, sorry, goodbye, please` → **`Zzz bello, Bi-do, Poopaye,
laloopay`** (90 mouth frames published, peak open 0.78).

### The English side of the corpus's own meanings (Sep 15)

That paragraph was the gap the user then named: the corpus teaches some meanings
only in French, so an English speaker saying "please" got an invented word while
the corpus plainly teaches `Moka`. The fix is data, not a wider model —
`minionese_english.txt` gives every one of the corpus's 100 meanings its English
words, and `minionese.py` trains it the same way, merged so the corpus wins any
key it taught itself. The column supplies **no words and no openings**: its
Minionese side is the corpus's, and the suite proves it reaches nothing the corpus
did not teach.

Two defects came out of training it, both found by checks rather than by reading,
and both are regressions the column introduced:

| what happened | the fix |
|---|---|
| a colon in the column's own header parsed as an entry and was learned as a word (`french (moka`) | the coverage check compares the column's meanings to the corpus's as a **set**, so a stray line can no longer survive |
| the column's *fragments* claimed phrases — `battery` answered `Pip pip pip` from `low battery` — and the character model had nothing to invent | only the **corpus** indexes words (that is what makes `sorry` → `Bi-do`); the column indexes whole meanings |
| Minionese the corpus already speaks was re-translated — `Kiss kiss` came out `Kiss kiss Kiss kiss`, because `kiss` is now an English key | a one-word key that is already Minionese vocabulary is left alone |

Measured, live, on the robot through `/api/say` (12/12 taught words reached,
panel animating on every sentence, peak opening 0.83–0.84):

| asked | spoken |
|---|---|
| `Please, thank you, goodbye.` | `Hehehe moka, Tank yu, Poopaye.` |
| `Sorry! You're welcome. Let's go, follow me, listen.` | `Luk bi-do! Prego. Vamo, Chupa, Tara.` |
| `Hello! Ten apples, four bananas, low battery.` | `Loka bello! Ju bokamoka, Chari harazapo, Pip pip pip.` |

Named gaps: a **plural** is still invented (`apples` → `bokamoka`) because the
corpus teaches the singular — reaching plurals means stemming, which is a wider
model than the words the corpus supplies; and the caption shows the Minionese, so
the English meaning lives only where the caller asked for it.

Boot: the `@reboot` entry is **installed from the repo**, not hand-written.
`bash deploy.sh` ships `start_face.sh` and then asks it to `--install-boot`, so a
replaced Pi gets its face back on the next deploy. The launcher owns that line
(marked by a comment above it), which is what keeps the crontab and the script
from drifting:

| command | does |
|---|---|
| `--install-boot` | adds or repairs its own line, idempotently: a second run changes nothing, a duplicate or a hand-edited line collapses back to one canonical entry, and every line it does not own (the app, jupyter, ollama, the bluetooth sink) is left byte-for-byte alone. Backs the crontab up to `~/crontab_backup_<date>.txt` before writing and restores it if the install fails. Refuses to install if `face_screen.py` is not next to it, because a boot entry that cannot work is worse than none. |
| `--reload` | restarts the face **only if it is already running**, so shipping new bytes updates the screen and a face someone stopped on purpose stays stopped. `deploy.sh` calls it; nothing watches the face, so a deliberate `--stop` is not undone. |
| `--status` | running or not, and how many boot lines the crontab holds. |

The wait is for the session, not for the socket file: a display whose socket
exists with nothing listening **refuses the connection** (measured), so the
launcher connects to `/tmp/.X11-unix/X<n>` before starting and retries up to six
times with 10 s between, re-checking the display each time. Its first log line
names the driver it got (`face_screen: x11 driver on display :0, 800x480`) —
measured on this robot, `DISPLAY=:0` gives x11 while an SSH-launched run with
`XDG_RUNTIME_DIR` set gives wayland, and both are the same panel.

**No caller may write `face_screen.py` in the same shell as a `start_face.sh`
stop/reload.** That shell's own command line then contains the name the
`pkill -f` pattern matches, and the shell kills itself: no output, exit 255,
which looks exactly like a dead robot. Hit three times while building this —
including in `deploy.sh`, whose `[ -f face_screen.py ]` test made every deploy
print both "reloaded" and "could not reach the robot". The deploy now names only
the launcher.
