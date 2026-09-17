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

## What the map keeps, and why steering does not wobble

5. **The learned map must be able to forget.** Hit counts used to be capped at
65535 and decayed one per `DECAY_SEC`, so a cell observed for a minute held
hundreds of hits and could never clear. After a drive the grid read blocked in
every direction — measured `clearance = 0.000` for all 13 headings — so the
`W_MEM` term was a constant offset and the learned map contributed nothing to
steering. Now `HIT_MAX = 5`, the same ceiling applies to camera-fused object
disks (`observe_object` used to climb to 65535 independently), and `load()`
clamps an old file on read. Live after the fix: max hits 5, hit histogram spread
1..5, and the 13 heading scores spanning -0.15 to +3.10 (they used to differ only
by the live-LIDAR term).

   What decays is the right to *block*, not the memory: `MEM_FLOOR = 1` holds an
   observed cell in the map (and in `surroundings.json`) while dropping it below
   `MIN_HITS = 3`, so the place the robot has driven through is kept and a chair
   that has since moved cannot refuse a heading forever. Live after the floor:
   8,685 remembered cells, 3,006 of them beyond the LIDAR's own 6 m reach. Before
   it, 0 of 810 did — the map was a picture of the current view, which is what
   "it never learns the area" looked like from the outside.

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

### The robot names what it recognises, and the words are Minionese (Sep 16)

The request was a library of detected objects that "respond in Minions": people,
chairs, walls, flower.  What was missing was not a word list — `cv_ctrl` already
owns the vocabulary its detector can name (468 names: the COCO-80 its closed-set
model emits plus everything the seed list and `learn_object()` have taught) — but
any path from a detection to a spoken response.  `object_speech.py` is that path,
and it owns three things: the **reaction** a Minion gives an object of that kind,
the **name** (the object's own word), and **when** it may be said.  The vocabulary
is read from its owner, never copied, so a newly taught name is speakable at once.

Words still come only from `minionese.py`.  The corpus teaches a word for 10 of
the 468 names — `chair` is `Soka`, `banana` is `Banana`, and `person` is `Boss`,
which is why a person is greeted rather than invented.  The other 458 get an
invented word and the table says so:

| the robot sees | it says | source | why |
|---|---|---|---|
| `person` | `Bello boss` | taught | hello + the corpus's word for a person |
| `chair` | `Soka` | taught | the corpus's own word |
| `flower` | `Hmmokana` | invented | the corpus teaches no word for a flower |
| `dog` | `Kiss kiss atokanpa` | invented | the reaction is taught; the noun is not |
| `knife` | `Whaaa batokana` | invented | danger first, then the invented noun |
| `car` | `Matoka bokazilo` | invented | "look at that", then the invented noun |

That split is the whole reason the reaction layer exists: with 98 % of nouns
invented, a table of invented words would sound like a robot, and a Minion phrase
in front of it sounds like a Minion.  Reactions come from the corpus's own
phrases (`hello`, `muak muak muak`, `danger`, `look at that`) and the suite fails
if one of them is not a key the corpus taught.  Kinds are explicit name lists, not
substrings: `mouse` is a computer mouse in COCO and `fire hydrant` is not a fire.

**When** is a policy, not a chat log: one line per `GAP_S` (6 s), no object twice
in a row, and each object at most once per `LABEL_COOLDOWN_S` (90 s), all
suppressed while the robot is speaking.  When several things are in view a person
is named before the largest thing, because a room's biggest box is usually
furniture.  Measured live, on the robot's own camera (the gaze's 2 Hz pass is what
feeds it; `/object_speech` is the whole library and what it said last):

| time | what happened |
|---|---|
| 09:14:16 | camera sees a person; quiet — it was already named 81 s earlier |
| 09:14:43 | `person` re-named at **exactly 90 s** — the cooldown, on a person who never left the frame |
| 09:15:13–09:15:43 | a bottle is in view continuously and named **once**; the person stays quiet for its cooldown |
| 09:15:56 | `SPEAKING text='Wika soka'` — a **chair** detected, `Soka` spoken (`Wika` is the voice's opening word) |
| 09:16:13 | `SPEAKING text='Tulaliloo batokula'` — a **bottle**, its invented word `Batokula` |
| 09:24:55 | the same bottle again at 94 s — the cooldown again, after a deploy restart |

The spoken text is read from `/speech_status`, which publishes the words the voice
actually synthesised, so this is the words leaving the speaker and not the words
the code chose.

One seam had to be closed for this: the robot now speaks on its own, and
`listen_for_question()` leaves the microphone open for 5 s after the wake word,
so a line landing there would be transcribed as the user's question.  The voice is
held for exactly that window (`self.speaking`), and released before the answer is
spoken — `speak_minion` refuses to talk while that flag is set, so holding it
across the answer would have silenced Lance's replies.

Named gaps, all measured rather than assumed:

* **a wall cannot be spoken, because this robot cannot detect one.**  `wall`,
  `floor`, `door`, `window`, `ceiling` are not in the vocabulary the detector
  names, so they are not in the table either.  Adding them means widening what
  YOLO-World looks for (and re-embedding), a change to detection behaviour rather
  than to this library — and `learn_object("wall")` already exists for the moment
  it is wanted.
* **the detector's mistakes get named.**  A "motorcycle" at ≥ 0.40 confidence
  indoors was named `Matoka nonniono`.  `CONF_FLOOR` is the one knob for that.
* **the object-detection CV mode calls the same hook** (`cv_detect_objects`), but
  every live line above came through the gaze's camera pass — that mode is
  selected over socket.io, so it was not exercised this pass.

### The voice's speed is a measured choice, and it is no longer fast (Sep 16)

The user's report: "it talking way too fast".  The register was two knobs pushed
up together — `pitch="+40%"` **and** `rate="+28%"` — and only the pitch makes a
Minion; the rate was just speed.  The same sentence was synthesized on the robot
at five rates (the WAV length is the playback length, measured straight from
Azure's output at 24 kHz/16-bit):

| rate | the same sentence | per word |
|---|---|---|
| `+28%` (was) | 7.60 s | 0.51 s |
| `+10%` | 8.85 s | 0.59 s |
| `0%` | 9.60 s | 0.64 s |
| **`-10%` (now)** | **10.95 s** | **0.73 s** |
| `-20%` | 12.32 s | 0.82 s |

`RATE` in `voice.py` is now `-10%`: **1.44× slower** than it was, and the pitch is
untouched, because the character is in the pitch.  Nothing else had to change for
the face: the mouth is drawn from the very WAV that plays, so it stretched with
the audio — measured live through `POST /api/say`, the sentence played **11.35 s**
with the mouth peaking at 0.84 opening and the same 90-frame curve (`window_s`
1.5 s of curve at 60 sps).

The no-network fallback was fast for the same reason and is now matched to the
main voice (`espeak-ng -s 200` → `150`, `pyttsx3` rate 200 → 150), so a robot
without its Azure link does not sound like a different, rushed robot.
`minionese_selftest.py` now pins `rate <= 0` rather than naming one string, so a
later edit cannot quietly rush it again the way this one did.

One piece of scratch to delete rather than fix: `bt_synth.py` (git-ignored, so it
cannot be committed, but present in the tree) is a one-use Bluetooth test that
repeats the SSML **with the old `+28%` and with the subscription key hardcoded**.
Nothing imports it; it should go, and that key was in the repo long enough to be
worth rotating.

### The learned map is a place the robot keeps, and the volume has one owner (Sep 16)

The user's two asks: "lidar mapping — the area for learning the robot can keep
into memory; when it drives it will follow the mapping of the area", and "add
volume control on command center — it keeps blasting loud".

**The map was already place-referenced on paper and robot-centric in practice.**
`spatial_memory.py` had been rebuilt around a map frame, but nothing ever handed
it the wheel motion: the planner called `observe_lidar(angles, distances)` with no
odometry and no scan identity, so the pose sat at `(0, 0, 0)` for the life of the
process and every scan landed in the robot's instantaneous frame — the same wall
in different cells on every pass.  Now `robot_state.LidarScan` carries the
revolution's own timestamp, `self_drive` hands `WheelStep.step()` and that stamp
to the map on every tick, and the map moves the pose only when the scan agrees the
robot really went there.

Measured on the robot, driving it from the UI with self-drive off:

| | pose | map |
|---|---|---|
| standing | `(0.00, 0.00, 0.00)` | 235 busy, 646 free cells |
| 2.4 s backwards | `(-0.04, 0.65, -0.16)` | 687 busy — it learned the ground it covered |
| 2.4 s spin | held at `(-0.04, 0.65, -0.16)`, `motion: rejected` | the wheels turned, the scan denied it |

So both branches of the guard are visible live: real motion is accepted and
carries the frame, claimed motion the scan refuses is rejected and the map stays.
Asked at the current pose, **17 of 17 sampled bearings have the obstacle the
LIDAR sees right now remembered at the observed range** (0 missing) — the
coherence a robot-centric grid cannot have.  The saved `surroundings.json`
(version 3) now carries the pose with the grid, so the place survives a restart.

**Learning was the planner's child, which was the real bug.**  `disable()` tears
the planner thread down — that is what `/selfdrive` does when self-drive is
turned off — so the map stopped learning exactly when a person was driving by
hand, and started again only while self-drive ran.  A second, tiny sampler thread
(`SelfDriver._sample_loop`, started at `warm()` with the boot) keeps folding scans
into the map whenever the planner is idle, decides nothing, and owns the periodic
save.  The planner's own thread, its `stop()` contract and the `/selfdrive_status`
payload are unchanged apart from two added fields (`free_cells`, and `map` with
pose/motion).

**The map steers.**  A heading into remembered occupancy is now scored down at two
radii (0.55 m and 1.1 m) and refused outright when the near arc is solid — live,
with self-drive on and 1.36 m of clear floor ahead, the planner refused headings
`0, -15, -30, -110` on remembered occupancy alone (the live-scan veto only applies
while pursuing) and still chose `+80°`, so it went around instead of parking.  A
map that refuses *every* heading falls back to the live scan rather than holding
forever, because a bad pose must not freeze the robot.

A frontier term ("prefer the nearest unmapped space") was built and then deleted:
at any weight that made it matter it outranked the straight-ahead preference and
took the robot off the proven straight line in an open room, which is exploration
nobody asked for.  `explore_score()` went with it rather than sitting unused.

## The object detector is now `yolov8s` through OpenCV's DNN, and one module names it

**The model name lived in three places** — `cv_ctrl` loaded `yolov8n.pt`,
`eyes_gaze` defaulted to the same string, and `deploy.sh` carried a third copy to
decide what to ship — so the app and the eyes could drift on to two different
detectors with nothing failing.  `detector.py` is now the single owner, and
`deploy.sh` asks it rather than repeating the string; the measurements behind the
choice and the export line live in that module's header.

**Why the runtime moved rather than just the model.**  Torch costs the same on one
thread as on four on this CPU, while OpenCV's DNN — a dependency the app already
had — runs the same weights about three times faster.  That is what pays for a
bigger model: under the app's own load, `yolov8s` through the DNN takes **1337 ms**
per camera pass against the **986 ms** the old `yolov8n` took through torch (the
same `yolov8s` in torch would have cost 2282 ms).  Everything above `yolov8s`
pushes the pass past two seconds and the eyes' following is bounded by that rate,
so `yolov8x` — the most accurate, 10253 ms — was measured and rejected rather than
shipped.  Want the eyes more responsive instead of more accurate?  It is one
constant: export a smaller model the same way, name it in
`detector.MODEL_FILE`, and the pass drops to a measured 574 ms — inside the
gaze's declared 2 Hz (500 ms/frame) budget, which no torch model could meet.

**Boxes agree with the trained model.**  Over three live frames from the robot's
own camera, the new decode landed within 0–3 px of the same weights in ultralytics,
same classes — and it found two people where the old `yolov8n` found one at 0.32.
`detector_selftest.py` (17 checks) pins what the module owns without the weights:
the letterbox geometry, the decode back to frame pixels, per-class NMS, the
vocabulary `object_speech` builds its spoken table from, and that no module outside
the owner names a detector model again.

**The model file is deployed, not committed.**  It is 44 MB of weights derived from
`yolov8s.pt`, and every other weight in this repo lives on the robot rather than in
git (`.gitignore`), so this one does too: `deploy.sh` ships the exact bytes when the
tree has them, and when the robot has none it builds them there from the weights
with the export `detector.py`'s header carries.  That build runs under
`YOLO_AUTOINSTALL=false`, and it has to: ultralytics' exporter installs its own
`onnxruntime`/`onnxslim`, and the numpy wheel it pulled was 2.x, which this OpenCV
build cannot import — it left the robot unable to `import cv2` in any new process
until the stray `numpy/` was removed from the venv.  A model built this way is the
same detector but not the same bytes (the simplifier never runs): over one frame its
raw output was bit-identical to the shipped file (max absolute difference
0.000e+00), with the same box and confidence to six decimals.

## The frame follows places the room agrees with, and a held command is re-sent (Sep 16, later)

**The odometry guard could not see the steps this robot takes.**  `spatial_memory`
confirmed wheel travel against the scan before believing it, with an 80 mm
allowance. This LIDAR publishes a revolution every **0.101 s** (measured on the
robot: 1798 frames/s, scan stamps 0.101 s apart), so one revolution carries 20 mm
of travel at the slow speed and 35 mm at cruise — every step, at every speed this
chassis drives, was smaller than the allowance. On the robot's own scans the fit
scored 1.000 at the true pose and 0.933 for a phantom 35 mm step: above the
acceptance threshold, so phantom travel was believed. Driven headlessly with the
real code, 0.9 m of phantom travel in 70 ticks walked the pose and reported
`accepted` every time. That is the smearing the frame exists to stop, and live it
was visible as a pose the robot had carried **14.7 m** away from the origin of a
12 m grid.

**The frame is a bounded window, and the robot had driven out of it.**  
`GRID_SIZE = 121` is ±6 m and nothing brought the map back to the middle: live,
the pose had reached cell **(152, 143) of a 0..120 grid**, with **0 of 9,624
remembered cells within 3 m of the robot** and **0 of 267 live returns landing
inside the grid at all** — the robot was in territory its map could not
represent, so nothing the sensors saw could be stored or steer, which is what
"it never learns the area" looked like from the outside.  The window now slides
under the robot when it is more than `CENTRE_KEEP_M = 3` m from the middle: by a
whole number of cells, so no place is resampled, with the pose moved by the same
amount and the reference scan re-anchored, so the guard still judges motion (and
a file written while the pose was outside the array is re-anchored on load).
Live on two drives after it: **292/292** and **255/255** of the close returns sat
on a remembered cell and all of them projected inside the grid, with the pose
held at cells (60, 61) and (58, 73).  `selfdrive_selftest` drives 4 m in a room
big enough to drive in and checks the wall it approached is still ahead at its
place — the smaller synthetic room is 5 m deep and walking out of it is a
different test, which is how the first version of that check passed while the
pose never moved.

Now `FIT_TOL_M = 0.025` — on the same scans the truth scores 0.992, a phantom
35 mm step 0.525, 100 mm 0.042 and a phantom 10 cm + 5° turn 0.109 — and the
wheels' claim is *accumulated* to `FIT_EVERY_M = 0.10` before a scan judges it,
because one revolution is inside the sensor's own repeatability (stationary
scan-to-scan spread: median 0–1 mm, p90 13 mm). The pose only ever moves to a
place the room agrees about; held travel is one cell. Live after the fix, on a
30 s drive: 20 accepted / 4 rejected chunk verdicts, 100% of close LIDAR
returns (190/190) remembered within ±1 cell of where the scan puts them, and
81% of them recent enough to block. `selfdrive_selftest.py` now pins the case
the old harness missed — phantom travel a revolution at a time, 20 mm and a
claimed turn, which must never move the frame, against the same travel with the
room moving with it, which must be followed.

**A drive command expires on this chassis.**  `CMD_HEART_BEAT_SET`
(`tutorial_en/08`, default 3 s) stops the wheels when no motion frame arrives;
measured here, one frame at 0.08 turned the wheels for 0.248 m and then stopped
by themselves ~3.5 s later. `LidarAvoider._send` suppressed every repeat, so once
the chosen heading settled the avoider stopped talking — and the robot sat still
while the state machine still reported `CRUISE` (sampled live: avoidance
ACTIVE/CRUISE, front steady at ~1 m, wheel counters frozen), lurching only when
a new heading briefly changed the command. `SEND_KEEPALIVE_SEC = 1.0` re-sends a
held command; the web UI has always re-sent on a 2 s timer for the same reason.
Live after the fix: CRUISE → SLOW → EVADE → CRUISE as it met things, **5.08 m in
13.9 s at 0.364 m/s with no stall**, where a constant command used to die after
~3.5 s.  Two later drives measured **4.95 m in 14.8 s** and **5.53 m in 14.9 s**
at 0.33–0.37 m/s with no interval longer than the 0.25 s sampling gap.

**A pause could be overtaken by the tick already running.**  `pause(halt=True)`
clears `_active` and *then* sends its zeros, so a `_decide` call already in
flight could put a motion frame on the wire after them — and the chassis drives
on that frame for the whole heartbeat above.  Measured live: a halt was followed
by **0.570 m** of wheel travel, and by **0.000 m** once the zeros were the last
thing the chassis heard.  `_send` now refuses to emit a motion frame while the
avoider is inactive; the joystick does not come through `_send`, so manual
driving while paused is unaffected, and `selfdrive_selftest` runs the method
against a recording base — mutating the guard back reproduces the stray
`L=0.3 R=-0.3` frame after the halt.

A second gotcha this exposed: any client command over the websocket calls
`avoider.pause()` (8 s of manual control), so a browser or Command Center
sending frames — including a stray `L=0.5 R=-0.5` followed by zeros — takes the
wheels off the avoider while it is driving. That is by design for a joystick;
it is worth knowing when "self-drive did nothing" is observed.  The watchdog
that re-arms after those 8 s re-enables self-drive as well as avoidance
(`manual_control_watchdog`), so a robot that was switched off comes back on by
itself — observed live: self-drive off, then `active true` and driving with
wheel counters climbing, until the disable was repeated.  Disabling avoidance
(`/lidar_avoidance?enable=false`) clears the watchdog's stamp, which is what
makes a stop stick; `self_drive.enable()` in the watchdog re-arms driving, which
is the operator-facing surprise.

The map file built before this fix was moved aside on the robot as
`surroundings.json.pre-frame-fix` and the grid cleared, because a pose that had
walked 14.7 m makes its cells uninterpretable. The remembered area is bounded by
`GRID_SIZE = 121` (±6 m) — the robot had already driven outside it.

## The map keeps what it confirmed, and the Command Center shows the robot's map (Sep 16, later still)

**The memory floor made noise permanent.**  `MEM_FLOOR = 1` kept every cell that
had ever been seen, so stray returns accumulated forever: measured on the robot,
**7,380 of 12,449 remembered cells had exactly one hit**, with a median distance of
5.88 m — the edge of the sensor's range.  Distance said the same thing: **97% of
the cells within 2 m had three hits or more, while 86% of everything remembered
beyond 5.5 m was single-hit**.  A cell now earns the floor only by reaching
`MIN_HITS` (three revolutions); a cell that never did decays to nothing, as it
always used to.  A surface close enough to return several rays in one revolution —
3° of turn is 0.1 m at 1.9 m, so several rays land in one cell — is still confirmed
by that single sweep.  Live after the deploy: the loaded map fell from **12,406
cells to 3,573** within seconds (1,865 confirmed, 1,650 remembered but no longer
able to block), and a later drive settled at 9,953 cells with 6,127 confirmed.
The panel draws cells with two hits or more, so what it shows is the occupancy that
can still refuse a heading, not the whole history.

**Learning runs while avoidance is on, and follows the wheels exactly.**  With
*only* `/lidar_avoidance` enabled (self-drive off) and the robot driven by hand for
0.75 s, the map moved the robot **0.33 m against 0.33 m of commanded wheel travel**
and grew 3,574 → 3,868 cells, the cells it drove past climbing to the hit ceiling.
That is the operator's "when I enable avoidance it should record and learn
everything": the sampler is not the planner's job, so it runs whenever the app runs.

**The Command Center panel was running a second driving brain.**  The Learning tab
scored its own headings against its own `ObstacleMemory` — an occupancy grid in the
robot's *instantaneous* frame with no pose, so the same wall landed in different
cells as the robot drove — and drove the robot over the websocket, which pauses the
robot's own avoider for 8 s.  That is the blob of red the operator was reading as
"it is not learning" while the robot used a place-referenced map the panel could not
see.  `ObstacleMemory.cs` and the client's reactive loop are gone; the panel reads
`/surroundings`, `/selfdrive_status` and `/lidar_status`, draws the robot at its own
pose in the map frame, and its Engage button switches on avoidance and the robot's
self-drive together (Stop stops it; Save/Clear map go to `/selfdrive_map`).
Taught-path replay and hue-follow stay — they are operator tools, not a second
brain.  Proven through the running app with UI Automation: the panel read `6007
cells remembered · robot at (-0.1, 0.2) m in the map · accepted`, clicking Engage
produced the robot's own `heading -30° turn +0.49 cells 6834` and 7.0 m of travel,
and Stop parked it.

**Smoothness, measured.**  One self-drive run: **5.28 m in 14.9 s at 0.36 m/s with
0 stalled intervals**, the turn command stepping by at most 0.22 between samples
(range -0.69..0.54, so ramps rather than flips), and clearance falling to 350 mm
without the robot stopping.

**Object avoidance reads the camera pass the robot already runs (Sep 16, latest).**
This paragraph used to say the planner's object map only filled while the CV overlay
happened to be in an object mode, so live the camera saw one `surfboard` while the
map held **0 objects**.  The stream that was missing was the gaze's own: it already
runs the shared detector at `cam_hz` while the eyes are on and publishes every box
it saw (`/eyes_status.detections`, as fractions of the frame).  `DetectionSource`
now reads that pass first and the overlay's hits behind it, scaling the gaze's
fractions into the pixels `box_bearing_deg` wants (a bearing is a difference of
fractions, so the scale cancels), and `app.py` attaches the gaze to the planner's
detection source once it exists.  Nothing extra is computed: this is the pass the
eyes pay for anyway, not a second ~1.3 s/frame detector on a Pi already at load 3.7.

Freshness is the gaze's own verdict rather than a second rule: a status that is
disabled, has a frozen camera frame (`frame_stale`) or has stopped stepping (`age_s`
past `GAZE_MAX_AGE_S = 5 s`) feeds the planner nothing — so switching the eyes off
leaves the planner exactly as it was, and a wedged gaze cannot keep re-fusing its
last boxes ten times a second.  Double counting needs no code either: both streams
are read together and `observe_object` folds the same name, bearing and range into
one object.

Live on the robot's own camera, driving with the eyes on: the map held camera
objects for the whole run (**43 of 42 samples** — `toothbrush`, `oven`, `suitcase`,
`backpack`, `chair`, `person`) and the planner refused headings for them (`scores`
came back `None` at 0°, ±15°, ±30°, ±45°, ±60°, ±80°, ±110°) while the robot still
cruised — **42 samples over 15 s, 0 samples with under 0.01 m of wheel travel**,
mean 0.12 m/s (min 0.058 m/s, in a 0.33-0.5 m view), turn steps at most 0.24 per
0.25 s, which is the 0.08/tick slew cap, and **0.0000 m** of wheel travel after the
stop.  `selfdrive_selftest.py` §7b pins it: the overlay gate, the three freshness
gates, the both-streams dedupe, a person walking across the frame, and the LIDAR's
place surviving a person who walked away.

An object's grid disk is a claim about *now*, and the grid's own decay runs at the
LIDAR's 6 s timescale — far too slow for something that walks.  `observe_detections`
now remembers the cells its last batch covered and, when a new batch no longer covers
them, drops them: a cell only the camera claimed goes back to nothing, while a cell
the LIDAR has confirmed keeps its place and is put back below the blocking threshold
(the next revolution or two restores it).  An object therefore never makes a cell
*sure* — a place is a place because the LIDAR keeps seeing it.  Against the live map,
switching the eyes off moved object cells from **241 (171 blocked) to 241 (160
blocked)**: most object disks sit on LIDAR returns, and those are the LIDAR's places
to keep or forget, not the camera's.

**Where a detection really is: the LIDAR at its own bearing.**  The front cone used
to be applied to every detection — live, `oven` was stored twice at the same bearing
0.46 m and 1.7 m apart, and one view produced 20 entries for a handful of things —
so a chair 30° off to the side could refuse a heading it was nowhere near.  A
box's distance now comes from the 1-degree occupancy bins at the bearing the box
is actually at (`self_drive._detection_range`, robot frame = raw − 180°, per
`perception.LIDAR_ANGLE_OFFSET`), as wide as the box itself, freshness-gated and
clamped; only when that sector answers nothing does the front cone stay the
honest fallback.  Static on the robot (not driving, eyes publishing), a stored
object's range sits at its LIDAR sector within **0.01 m median, 18 of 19 within
0.2 m**; during a live drive the objects stay on what the camera names (`tv`,
`person`, `chair`, `train`, `clock`, `refrigerator`) with the planner refusing
the headings that point at them while still cruising.  The duplicate failure
itself had a second root: nothing ever expired an entry, so a drive's ghosts
were saved to `surroundings.json` and reloaded into every later session — the
count had reached 268 for a room holding a handful of things.  An entry no batch
has refreshed within `OBJ_TTL_S` (30 s) is now dropped, exactly like the cells:
the list is what the camera currently claims, and it fell live from 268 to 7–26
over one drive.  (One probe-measured artifact to not re-chase: while the robot
*drives*, stored ranges lag the live sector by ~0.4 m median — that is the robot
moving between fusion and sampling, not bad attribution; measure it static.)

**Volume.**  `set_audio_volume()` was pygame's music mixer, and the robot's speech
never goes through it — it is played by `paplay` to the Bluetooth sink, which sat
at 83% whatever the app asked for.  `voice.py` now owns the output level
(`sink_volume()`/`set_sink_volume()`, `pactl` with `amixer` behind it), `app.py`
exposes `GET|POST /volume`, and the Command Center's header carries a slider whose
value comes from the robot and is sent back debounced.  Proven live through the
slider itself (UI Automation): the panel opened at the robot's 83, `40`, `65`,
`25` all landed on the Pi, and `83` was restored.  The route clamps (`150` → 100)
and refuses nonsense (`level=loud` → 400 "is not a number").

**Cruise now behaves like a floor cleaner, not a compass needle.**  The complaint
"it keeps turning around" had a real policy root: the cruise goal was
`-abs(heading)/180` — "prefer the bumper" — with no memory of the course being
driven, so every block swung the robot toward whatever sector scored widest and
it pirouetted (headings of ±80°, ±110° measured live).  Three changes, one
owner each: the cruise goal holds a **course** (`self_drive._course_map`, the
map-frame bearing of recent actual travel read from the map's pose, so a pivot
doesn't rewrite it; scoring re-expresses it against the current heading and
rewards turning back within `COURSE_MAX_DEG` = Roomba's minimum-turn rule); a
gentle **novelty** term (`spatial_memory.novelty`, W_NOVEL = 0.5 vs safety's
combined 3.2) breaks *equal-clearance* ties toward unmapped floor so open room
gets covered instead of patrolled in circles — no-return rays (0 mm) leave floor
unmapped while real walls map theirs, which is the freshness gate that stopped
the old frontier wobble; and **covered floor** is counted (`covered_cells`, the
disk the robot itself has driven over, published in `/selfdrive_status`) so
mapping progress is one number the operator can watch grow.  Pinned in
`selfdrive_selftest.py` (course kept through pivots, unmapped side outscores the
mapped side at equal clearance through a real asymmetric scan, coverage grows
with travel); all suites green.

**Live proof is currently blocked by the battery, not the code.**  Deploying and
driving showed the planner commanding turns while the wheel counters sat frozen
to the last digit — 19 forward commands through the app's own
`/send_command` path moved odometry exactly 0.000 m, voltage read 9.04–9.06 V
(3.0 V/cell — empty for the 3S pack) and *rose* during the attempt, and ESP32
feedback stayed alive throughout.  That is undervoltage protection cutting
motor power while the Pi's rail keeps running.  Charge the pack, then re-run the
drive probe: the course/novelty/coverage behaviour is proven in the harness but
the on-floor Roomba run waits on a charged battery.

## The battery guard: park and say so before the cutoff says nothing (Sep 17)

The cutoff at ~9.0 V is not a shutdown anyone can act on — the Pi dies
mid-drive and the robot's state is unknown until someone walks over.  A
guard now rides the avoider's 10 Hz loop (`battery_guard.py`, ticked from
app.py): a pack that sags below 9.80 V for 6 s of driving — or touches
9.25 V once — parks the planner and the wheels through the same
`_selfdrive_off()` composite the /selfdrive route uses, shows
"BATTERY LOW - parked" on the video overlay, and speaks the corpus's own
alarm — "Bee do bee do bee do! Battery low. Banana time - me go home!" —
through the one Minionese voice path.  Recovery is latched: the pack must
hold 10.60 V at rest for 30 s before self-drive will trust it again.
Proven: the state machine in the harness (dip-vs-sustain, latch,
recover-by-rest, missing readings), the announcement through the real
voice on the robot, the wiring through a clean deploy and boot.  The
in-app trip itself has not been watched on a real sag, because the pack
reads 9.86 V at rest — nearly empty — and discharging it further to watch
a park is exactly what the guard is for the operator to decide.

## The fresh-map acceptance, and what it really showed (Sep 17, later)

Clearing the map did NOT restore the 99% near-0 heading discipline: on a
completely empty map the same cluttered room gave 47% (28/59 clear samples
near straight, 11 wide while clear, course re-formed 83 times in 61.7 m).
The refuted hypothesis: legacy busy cells fencing headings.  The real
story: the 99% run was the one where the odometry sign was still wrong —
the pose never moved, the memory never knew where the robot was, so it
could veto nothing.  That "discipline" was the memory being blind.  With
the pose tracking, the map genuinely knows the room and turns the robot
early — at corners and remembered walls, before the front cone sees them
(11 wide turns all happened while the front cone was clear).  Coverage
21 -> 574 cells, 118/119 samples moving, no stalls: the robot is doing
Roomba cornering, not pirouetting.  Judge heading discipline by travel
efficiency and coverage growth, not by straightness alone.

The same run caught a real defect in the guard's first wiring: the
manual-control watchdog re-enabled self-drive over the guard's park
(min 9.13 V measured under load), and the guard's announce-once flag had
become an enforce-once flag — it stayed silent while the robot drove
away at 9.1 V.  The guard now re-asserts its park on every tick while
latched, the watchdog stands down while latched, and announcing stays
once per latch.  Deployed; robot parked at 9.77 V resting.

## The wall-adjacent lurch was the map's veto, not flicker (Sep 17, evening)

A 5 Hz probe of a wall-adjacent drive (389 samples) split the complaint
in two: steering sign changed in only ~10% of samples — the EMA, slew
cap and deadband were already doing their job — while mean |turn| near
walls ran 0.359 vs 0.194 in the open, with +80..110 deg headings winning
while the front cone was clean.  The lurch was the memory:
MEM_BLOCK_CLEAR refused a heading on remembered wall-edges alone, so the
planner threw the robot at far-side gaps the live scan showed clear,
then corrected back — the "shifting left and right like crazy".

The veto now fires only when the live scan also refuses the heading;
otherwise the remembered obstacle steers by score (the blocked heading
ranks last).  With COURSE_BIAS 1.0 -> 1.8 and COURSE_MAX_DEG 90 -> 130,
the same 120 s acceptance moved straight-while-clear 47% -> 58%
(median clear-path turn 0.08 when it cruises) and coverage 21 -> 616
cells, peak 699 — best yet — with zero battery events (min 9.89 V).
The residual wide turns are -110 deg emergency pivots with the bumper
threatened: the avoidance layer's job, not course-keeping's.

## The park that did not hold: a missing `global` (Sep 17, evening)

The tuned run parked normally — and 4.79 m later the robot was still
cruising.  _selfdrive_off assigned _last_manual_cmd_time without a
`global` declaration: Python created a throwaway local, the module-level
watchdog timestamp kept its stale value, and the auto-resume watchdog
re-armed the drive the park was meant to end.  (Every earlier proof held
because the check window closed before the watchdog tick.)  Fixed at the
one line; proven live: 8 s drive, disable, worst odom delta across the
full 30 s watchdog window 0.185 m — coast-down, not cruise — then parked
solid through a 60 s watch.  The battery guard now also logs every state
change and a 30 s heartbeat (`[battery] v=... state=... latched=...`),
because a guard whose state is invisible reads as a guard that never
fired.

## The wall follower: matching the wall, not fighting it (Sep 17, night)

"Still swivelling beside walls" survived the veto fix because the
clearance score can never express "hold a line parallel to the wall":
any heading angled off the wall reads as safer clearance, so near a wall
the winner alternates between course-pull and clearance-pull.  The fix
is Roomba's actual trick: when a side wall is in reach (perception
.side_wall picks the most abeam solid return within 0.15..1.3 m), a P
regulator on the standoff error SUPPLIES the cruise course anchor —
"parallel at 0.85 m, tilted at most 20 deg" — and the existing
COURSE_BIAS/EMA/slew/deadband damping shapes it.  The anchor is
re-derived from the live scan every tick, so odometry drift never bends
the line; its goal weight is raised (WALL_GOAL_MULT 2.5) because W_LIDAR
and W_MEM both prefer headings angled off a wall in reach and at the
plain weight the robot drifted out of reach instead of reeling back to
its standoff.  A 1.2 s hold keeps the line through sensor gaps.

Pinned in selfdrive_selftest section 9 (both sides): level-at-standoff
aims along the wall, too-close steers away small, too-far steers in
small, a closing nose is caught, the turn never changes sign while
following, the gap-hold releases on expiry, open floor drops the anchor.
One harness lesson worth keeping: teleporting the wall between cases
while the pose stands still makes the place-frame map honestly record
two parallel walls — phantom cells a real drive never produces (the
odometry moves the pose with the robot) — so the wall cases clear the
map first.

Not yet proven: the robot dropped off this PC's Wi-Fi mid-pass (the
PC's own gateway unreachable — same link failure as before), so the
wall follower has never run on the floor.  The live acceptance to run
when the link returns: a wall-adjacent 120 s drive re-measuring mean
|turn| near walls against the 0.359 baseline (expect well under 0.2,
steady turn sign, coverage still climbing), robot parked after.

## Wi-Fi health in the Command Center (Sep 17, night)

The header strip no longer just goes grey when the robot drops off: it
draws /network_status (5 s poll, deliberately slow — nmcli costs the
Pi).  Robot side, wifi_status.py owns the reading (device state, SSID,
signal, the watchdog's report and service state) and the route answers
even when the module is not deployed, so an old build shows "update the
robot's app" instead of a failure.  The watchdog now publishes its
history (last reconnects with how long they took, recovery reboots,
pinned profile) in wifi_default_report.json — history in $HOME, which
survives a reboot, so "last reconnect" outlives the outage clock that
must not.  The strip shows: SSID + signal (green) when home; "offline —
watchdog retrying (last reconnect +Ns)" (amber) when the watchdog is
working; "offline — no watchdog installed" when it is not; the tooltip
carries the history.  Proven: both python suites offline (wifi_status
8 probes, watchdog decision machine 9 probes, C# parse 8 probes against
the exact payload wifi_status.assemble emits, dotnet build clean).
Not yet proven: any of it against real NetworkManager or a real drop —
the robot is still off this PC's dead Wi-Fi link.
