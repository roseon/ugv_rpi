# Robot Eyes — wiring 2× TFT displays (Arduino Uno ← Raspberry Pi 5)

This guide wires **two SPI TFT displays** (one per "eye") to an **Arduino Uno**, and the Uno to the **Raspberry Pi 5** over USB serial. The Pi runs YOLO person detection and streams the detected person's screen position to the Uno, which moves the pupils toward them — that's what makes the eyes "look" at people.

> Open **`schematic.svg`** for the picture version of this table.

---

## 1. Parts

| Part | Notes |
|---|---|
| 2× SPI TFT display | **The sketch in this repo drives GC9A01A round 240×240 panels only** — the ST7735 and ILI9341 drivers were deleted to reclaim flash (§5). An **ST7735 1.8″** (128×160) or **ILI9341 2.4″/2.8″** (240×320) board wires up the same way, but its Adafruit driver has to be restored in the sketch first, so a garbage or blank panel on one of those is expected and not a wiring fault. |
| 1× Arduino Uno | The Uno's ATmega16U2/CH340 gives the Pi a serial port over USB. |
| 1× Raspberry Pi 5 | Runs the robot app, which owns the eyes' serial link and drives the gaze from LIDAR + camera. |
| 8× male–female jumper wires | For display → Uno signal wires. |
| 1× USB A→B cable | Pi 5 USB-A port → Uno USB-B port (power + serial). |
| (optional) 1× 8-channel level shifter | `TXS0108E` (or 2× `74AHCT125`). Recommended if your display board is strictly 3.3 V logic (see §4). |

### How to confirm which TFT you have (check your photos/boards)
- **1.8″ board, red PCB, 8-pin header, tiny 1.8″ screen** → ST7735.
- **2.4″/2.8″ board, often black PCB with a micro-SD slot, larger screen** → ILI9341.
- Read the white silkscreen near the pins: it will say `VCC GND CS RESET DC SDA SCK LED` (ST7735) or `VCC GND CS RESET DC MOSI SCK BLK` (ILI9341). **Base your wiring on your board's own silkscreen** — the schematic uses the standard labels.

---

## 2. Wiring — Arduino Uno → both TFTs

The two displays are **fully independent** — **no signal wires are shared**. The **left** eye runs on its own software-driven port (**SCLK=D4, MOSI=D3**) with its own CS/DC/RST; the **right** eye runs on the Uno's hardware SPI (**SCK=D13, MOSI=D11**) with its own CS/DC/RST. Each display gets five dedicated signal wires. Only power (VCC/LED) and GND are common (you can separate those too if you prefer).

| Signal | Uno pin | TFT #1 (LEFT eye) | TFT #2 (RIGHT eye) |
|---|---|---|---|
| Power (logic) | **3.3 V** | `VCC` | `VCC` |
| Power (backlight) | **3.3 V** | `LED` | `LED` |
| Ground | **GND** | `GND` | `GND` |
| Clock | **D4** | `SCK` / `SCL` | — |
| Data | **D3** | `SDA` / `MOSI` | — |
| Clock | **D13** | — | `SCK` / `SCL` |
| Data | **D11** | — | `SDA` / `MOSI` |
| Chip select | **D10** | `CS` | — |
| Chip select | **D7** | — | `CS` |
| Data/command | **D9** | `DC` | — |
| Data/command | **D6** | — | `DC` |
| Reset | **D8** | `RESET` | — |
| Reset | **D5** | — | `RESET` |

Notes:
- **No shared signal wires** — each display is on its own port, so you never have to tie SCK/MOSI together.
- The left eye runs the firmware's **fast direct-port bit-bang SPI** (mosi=D3, sclk=D4) — a full 240×240 fill takes ~1.08 s on the left against ~0.225 s on the right (~9.4 µs/byte vs ~2 µs/byte, measured at boot: `FILL L: 1079ms R: 225ms`). The right eye uses hardware SPI (SCK=D13, MOSI=D11).
- **What that costs the moving gaze, measured.** A tracking frame repaints only the pixels the iris move changes — ~345 spans, ~1210 px — and takes **~102 ms** end to end (~10 frames/s): ~46 ms of panel writes (the bit-banged left eye is most of it) plus ~52 ms of computing the row-by-row difference on the Uno. Before the incremental repaint, each frame erased and redrew both full irises (~12,900 px) and took **~341 ms** (2.9 frames/s), which is what the eyes were reported as "refreshing". The firmware prints `REPAINT n=25 avg=…ms fills=… px=…` so you can see that on the serial line.
- **If you want the gaze genuinely smooth (25–30 frames/s), put the left panel on the same SPI bus as the right one**: `SDA/MOSI → D11`, `SCL/SCK → D13`, keep its own `CS`/`DC`/`RST`, then `echo 'TYPE 4 3' > /dev/ttyACM0` (no re-flash). The two panels share the bus and take turns by `CS`; the left eye then paints at the right eye's byte rate, which removes ~24 ms from every frame. It also **frees `D3` and `D4`** — and `D3` is a PWM pin (see the L298P note below).
- `MISO` (D12) is **not connected** — the TFTs don't need it for writing.
- **VCC → 3.3 V** is the safe rule for GC9A01A round modules (they have no onboard regulator — 5 V on VCC destroys them). If your board has an **onboard 3.3 V regulator** (common on ILI9341 2.4″ boards, identifiable by the small regulator + jumper near the header), you may feed `VCC` from the Uno's **5 V** instead — check the board's datasheet/silkscreen first.
- **Power budget — a precaution against display corruption, *not* the diagnosis for a vanished Uno.** The Uno's `3.3V` pin is fed by an on-board LP2985 regulator rated **~50 mA recommended / 150 mA absolute maximum**. A 1.28″ GC9A01A round module draws **~10–20 mA** for logic *and* **~20–40 mA** for the backlight at full brightness, so **two of them on the Uno's 3.3 V pin can exceed that budget** and can brown out a *display*: white screen, random lines, or one eye dying while the other keeps working. Worth fixing on its own merits — give the displays their own **3.3 V supply** (VCC and LED both), tie its GND to the Uno's GND, and leave the Uno's 3.3 V pin for logic only. **It does not explain the Uno disappearing from USB**, for the reasons measured in §3a.
- **Pin budget — and the L298P conflict.** The eyes already use 10 of the Uno's 14 digital pins: `D3 D4 D5 D6 D7 D8 D9 D10 D11 D13`. `D0`/`D1` are the Uno's own USB-serial pair and must stay free, so the only spare digital pins are **`D2`** and **`D12`** (the optional auto-detect MISO). **An L298P motor shield claims `D10`, `D11`, `D12` and `D13`** — `D10`/`D11` are the PWM speed pins for motors A and B, `D12`/`D13` the direction pins (the board inverts one of each direction pair, so it needs only four pins rather than six). That set collides head-on with this build: **`D10` is the LEFT eye's `CS`, and `D11` + `D13` are the RIGHT eye's `MOSI` and `SCK`.** So one Uno cannot carry both:
  - **Stacked as a shield**, those four pins are physically occupied by its headers and no eye wire can reach them — the two cannot be stacked at all.
  - **As a separate module jumpered to the same pins**, the shield's control pins are high-impedance inputs, so panel signals still arrive, but the motors twitch with every display transaction, and any variant that buffers or inverts those lines corrupts the display data outright.
  - **If both must live on one Uno — what is actually free.** `D2` and `D12` as digital I/O (*`D12` only if you do not wire the optional auto-detect `MISO`*), and `A0`–`A5` (= digital `14`–`19`) **as digital only: the analog pins cannot `analogWrite`**. `D0`/`D1` stay reserved for the Uno's USB-serial pair.
  - **There is no free PWM pin, so the analog header cannot give the module speed control.** The Uno's PWM-capable pins are `D3 D5 D6 D9 D10 D11` — six of them — and this build holds **all six**: `D3` is the LEFT eye's `MOSI`, `D5`/`D6` its `RST`/`DC`, `D9`/`D10` its `DC`/`CS`, and `D11` is the RIGHT eye's `MOSI`. An L298P's `EN` (speed) inputs need one of those, so moving the eyes to `A0`–`A5` does not create the speed pin the module wants.
  - **Which eye signals need no PWM at all:** `CS`, `DC` and `RST`. The sketch drives all three with plain `digitalWrite` and takes them as constants (`L_CS`, `L_DC`, `L_RST`, `R_CS`, `R_DC`, `R_RST`), so moving one to `A0`–`A5` is a one-line change to that constant.
  - **The one move that both frees a PWM pin and speeds the eyes up:** put the left panel's two data wires on the SPI bus — `MOSI D3 → D11`, `SCLK D4 → D13`, keeping its own `CS`/`DC`/`RST` — and set `TYPE 4 3`. That returns **`D3` (PWM-capable) and `D4`** to the free list, so the L298P's `EN` can have `D3`, and the left eye stops being the slow half of every frame. This is the only eye-wire change worth making: the firmware supports both wirings (type 3 = bit-bang on D3/D4, type 4 = bus on D11/D13), so nothing needs re-flashing to switch.
  - **The honest options:** (1) drive the motors from the **ESP32 base controller** — the chassis motors are already that controller's job on this robot, so a second driver on the Uno buys nothing; (2) **accept full-speed-only direction control** — tie the L298P's `EN` pins `HIGH` and put `IN1`/`IN2` on `D2`/`D12` or on `A0`–`A5` (digital only), giving forward/reverse/stop and no speed adjustment; or (3) **free a PWM pin deliberately** by moving one eye's `CS`, `DC` or `RST` to the analog header and redefining it in the sketch, which frees that pin's PWM — e.g. `D10` or `D9` — for the module's `EN`. Prefer `D9`/`D10`: `D5`/`D6` are the only other movable PWM pins here, and they sit on **Timer 0**, which is where `millis()` comes from — the same clock this firmware times its fills and gaze loop with.
- **A stacked shield can also back-power the Uno.** On the documented L298P shield the motor supply can be jumpered onto the Arduino's `Vin` (its `OPT` jumper), and with that jumper fitted but no motor supply connected it will try to run the motors from USB. If the displays' `VCC`/`LED` are also fed from the Uno's `3.3V` pin, that rail is a shared weak point. Keep the display supply separate.
- Backlight: tie `LED` to **3.3 V** on both displays. **Do not move the left eye's `LED` to D3** — D3 is that eye's data line (MOSI), and a PWM backlight on it will fight the SPI data and kill the left eye. There is genuinely no spare PWM pin for the backlights on an Uno with this pin map — the eyes hold all six PWM-capable pins, and `D2` (the one free digital pin) cannot `analogWrite` either, so **no** Uno pin can dim a backlight here. If you need to dim, dim on the **supply side** (a series resistor, or a separate adjustable 3.3 V rail), not from the Uno.

---

## 3. Wiring — Arduino Uno ↔ Raspberry Pi 5

| From | To | Purpose |
|---|---|---|
| Pi 5 **USB-A** port | Uno **USB-B** port | Serial link + power |

- **Which `/dev` node is which — never guess by position.** The robot has three serial links, and only one of them is the eyes:

  | Link | Node | USB identity |
  |---|---|---|
  | **Eyes' Arduino Uno** | **`/dev/ttyACM0`** | Arduino SA `2341:0043` (CDC-ACM) — `/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_<serial>-if00` |
  | D500 LIDAR adapter | `/dev/ttyUSB0` | Silicon Labs `10c4:ea60` CP2102N bridge |
  | Base controller (ESP32) | `/dev/ttyAMA0` | the Pi's GPIO UART — **not USB at all** |

  A CH340-clone Uno appears on `/dev/ttyUSB0` instead. To see which is which, run `python pi_eyes.py --list`, which prints every node with its USB identity.
- **`/dev/ttyACM0` only exists while the Uno is configured.** If the Uno fails to configure (`dmesg` shows `usb 1-2: can't set config #1, error -62` and `xhci-hcd …: Timeout while waiting for configure endpoint command`) there is **no `ttyACM*` node at all**, and no gaze command can reach the eyes. `lsusb` still lists the Uno in that state, which is why it is easy to misread as "plugged in but broken". The measurements in §3a show this is a **host-side event on the Pi**, not a display or sketch fault.
- **The robot app must never claim the Uno.** `base_ctrl._pick_lidar_port()` used to fall back to `/dev/ttyACM*` whenever no `/dev/ttyUSB*` bridge was present — that would open the eyes' Uno as if it were the lidar, hold the port, and reset the Uno via its `kick_lidar()` DTR pulse. It now identifies devices through `serial_ports.py` and refuses to return an Arduino.

---

## 3a. "The Uno vanished from the Pi" — what was actually measured

Symptom: `lsusb` still lists the Uno, but there is **no `/dev/ttyACM0`**, so nothing can reach the eyes.

Measured on 2026-09-14 (Pi 5 at `192.168.24.25`):

```
[09:04:45] i2c_designware 1f00088000.i2c: controller timed out
[09:04:45] raspberrypi-clk …: Failed to change fw-clk-arm frequency: -110
[09:04:45] xhci-hcd xhci-hcd.0: Timeout while waiting for configure endpoint command
[09:04:45] usb 1-2: can't set config #1, error -62      <- 1st failure
[09:06:42] usb 1-2: USB disconnect, device number 4
[09:06:46] usb 1-2: new full-speed USB device number 5 <- re-enumerated
[09:06:56] usb 1-2: can't set config #1, error -62      <- 2nd failure
```

Then **nothing for the next 36 minutes**: the `can't set config` count is still `2`, `vcgencmd get_throttled` is `0x0`, and the Uno sits on the bus with **zero interfaces** (`lsusb -t` lists only the CP210x on bus 1).

**What that shows — the fault is on the Pi's side, not the Arduino's:**

- `error -62` is `-ETIME`, and the failing message is the *host controller* not receiving its own *configure endpoint* command. The Uno's descriptors were read fine moments earlier (`New USB device found, idVendor=2341 …`), so its USB bridge was alive and answering.
- In the same two minutes the Pi's **I2C host** (`i2c_designware`, hosting the DSI touch panel) and the **VideoCore firmware mailbox** (`fw-clk-arm`) timed out too. An Arduino's 3.3 V rail cannot reach either.
- The Pi never flagged undervoltage (`get_throttled=0x0`), and the burst lasted 2 minutes and then stopped — a steady overload is not a 2-minute event.

**The host recovered; the Uno did not.** Probing the I2C host now returns `EREMOTEIO` (transfer completed, no device present) on free addresses — *not* `ETIMEDOUT` — so the controller is healthy again. The Uno is simply **stuck in the failed state**: USB core gave up after two configure attempts and never retries by itself.

**The fix is to make the host re-enumerate the port** — unplug the Uno's USB cable and plug it back in (a hub port toggle or a reboot does the same). If `/dev/ttyACM0` then appears, that was the whole problem. Doing it without touching the cable needs root: `/sys/bus/usb/devices/1-2/authorized`, `bConfigurationValue` and `remove` are all `root root`, and `/dev/bus/usb/001/00N` is not writable by `ws`:

```bash
# from a root shell on the Pi, instead of a physical replug:
echo 0 | sudo tee /sys/bus/usb/devices/1-2/authorized
echo 1 | sudo tee /sys/bus/usb/devices/1-2/authorized
ls /dev/ttyACM0
```

**What is *not* proven: the display load.** Two modules on the Uno's 3.3 V pin can exceed its ~50 mA budget (see §2) and that is worth fixing, but it cannot explain this failure — an Arduino 3.3 V rail cannot make the Pi's host controller, I2C bus or firmware mailbox time out, and the failure was a single 2-minute burst rather than a steady condition.

**The cheapest check that confirms or clears it:** with everything running, pull **both displays' `VCC` and `LED` wires off the Uno** (leave the Uno on USB), replug the Uno's USB, and watch `ls /dev/ttyACM0`. Node appears → the load is implicated; next measure the displays' own `VCC`–`GND` (≈3.3 V steady = no sag, <3.0 V = a real one). Node still absent → the load is cleared and the fault is the Uno's USB bridge, that cable, or that host port.
- The Pi 5 USB-A port supplies ~600 mA — enough for the Uno + both TFTs' logic, but **both backlights at full brightness can draw close to that**. If the eyes flicker or a screen dims under load, power the Uno from a powered USB hub or a 5 V / 2 A supply via its DC jack (never feed the Uno's 5V pin while USB is plugged in). Treat this as a robustness improvement, not as the diagnosis for a vanished Uno — see §3a.

---

## 4. Level shifting (the one real gotcha)

The Uno's outputs are **5 V**; TFT controller chips (ST7735, ILI9341) are rated **3.3 V logic**. The boards usually survive direct 5 V in practice, but it's out of spec.

- **Recommended:** put an **8-channel level shifter** (TXS0108E) between the Uno and the TFTs on the 8 signal lines: SCK, MOSI, CS1, CS2, DC1, DC2, RST1, RST2. HV side = Uno 5 V, LV side = 3.3 V (tied to the same 3.3 V that feeds VCC).
- **"It just works" shortcut (common with ST7735 1.8″ boards):** wire direct as in §2. Many boards clamp fine at 5 V. If a display glitches or gets warm, add the shifter.
- If you choose the shifter, it goes inline on the signal wires only — power and GND stay direct.

---

## 5. Software — what goes where

### Arduino Uno — `eyes_tft/eyes_tft.ino`
1. Open `eyes_tft/eyes_tft.ino` in the Arduino IDE (it's in its own folder, as Arduino requires).
2. Install libraries: **Adafruit GFX** and **Adafruit GC9A01A** (round 1.28″ 240×240). The sketch no longer includes the ST7735/ILI9341 drivers, so installing them changes nothing.
3. `DEFAULT_TYPE` is already `3` — GC9A01A on the bit-bang port (D3/D4). Its only other value is `4`, the same panel with its two data wires moved to the hardware SPI bus; `TYPE 4 3` over serial does the same without a re-flash. Confirm the pin constants match §2, and confirm what is actually on the board from the boot banner ("Boot behavior", below).
4. **Upload — stop the app first.** The app owns `/dev/ttyACM0`, and an upload attempted while it runs dies in the bootloader handshake (`Error: protocol expects sync byte 0x14 but got 0x00`, `programmer is out of sync`, `unable to write flash`) and silently leaves the old firmware in place. Measured on the robot, the whole sequence is:

   ```bash
   ssh ws@<robot>
   pkill -f '[a]pp\.py'; sleep 2          # the app has to release the Uno
   ~/arduino-cli/arduino-cli compile --fqbn arduino:avr:uno ~/ugv_rpi/eyes/eyes_tft
   ~/arduino-cli/arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:uno ~/ugv_rpi/eyes/eyes_tft
   #   Writing | ################################################## | 100% ... Avrdude done.
   cd ~/ugv_rpi && nohup ./ugv-env/bin/python app.py >> app.log 2>&1 </dev/null &
   ```

   That build is **28,082 bytes (87%) of the Uno's 32,256**, with 1,232 bytes of RAM free — it was at 99% (94 bytes free) until the legacy panel drivers were deleted. Once it is flashed, each screen shows one big cartoon eye in the Minion goggle design: a
grey metal ring around the panel edge, a yellow eyelid ring inside it, a large
white sclera, and a small brown iris with a black pupil and a white highlight —
plus the black strap knuckle entering at each side. The reference art this comes
from is attached to the request that introduced it (Sept 2026); the proportions
are the firmware's own: ring r=114, eyelid r=105, sclera r=95, iris r=32.

### Raspberry Pi 5 — the app owns the gaze
`app.py` drives the eyes through `eyes_gaze.py`. It looks at whatever the LIDAR reports inside `prox_mm` (default **1200 mm**), takes over for anything inside `close_mm` (default **450 mm**) — the "something is getting too close" case — and otherwise **follows the people the camera sees**: a `person` detection is preferred over any other object, and only when no person is in frame does the largest object drive the gaze. It is the **only** writer of `T <px> <py>`; a second writer would interleave pupil positions, so nothing else may open the Uno while it runs.

```bash
cd ~/ugv_rpi && source ugv-env/bin/activate
python app.py                                            # the gaze owner (on at boot)
curl -X POST 'localhost:5000/eyes?enable=true'           # on / off
curl 'localhost:5000/eyes_status'                        # what it is looking at, and why
curl -X POST 'localhost:5000/eyes?prox_mm=800&close_mm=300'   # retune the distances
```

**"Are the eyes following me?"** Stand in front of the robot and read the status. `camera` should become `person`, `camera_persons` should be at least `1`, and `target` should move as you walk across — that is the whole check:

```bash
curl -s localhost:5000/eyes_status
# {... "reason": "camera", "target": {"px": 62, "py": 44}, "camera": "person",
#      "camera_persons": 1, "camera_hits": 2, "sent": 117, "errors": 0, ...}
```

The three ways to see nothing move, in the order worth checking:

| status says | what it means |
|---|---|
| `sent: 0`, no `port` | the gaze never opened the Uno — `serial_ports.uno_port()` did not find it (`ls /dev/serial/by-id/`) |
| `camera: null`, `camera_persons: 0`, `reason: idle` | the person was not detected: no frame yet, or `[eyes] object model unavailable` in the app log (run the app from `~/ugv_rpi`, where the detector's model file lives — `detector.py` names it, and `bash deploy.sh` reports whether the robot has it) |
| `connected: false` | nothing is running the gaze at all — check the app is this version: `grep -c eyes_gaze ~/ugv_rpi/app.py` must not be `0` |

`eyes/pi_eyes.py` is now a **read-only** status tool for that same link — it opens neither the Uno nor the camera, so running it cannot fight the app for either device:

```bash
python eyes/pi_eyes.py            # one-shot state of the gaze
python eyes/pi_eyes.py --watch    # follow it until Ctrl-C
python eyes/pi_eyes.py --list     # which node is which (touches no hardware)
```

### Serial protocol (Pi → Uno, ASCII lines, 115200 baud)
| Command | Meaning |
|---|---|
| `T <px> <py>` | Look toward point; `px`,`py` in **0–100** (50,50 = frame center). Both pupils converge on it. |
| `T -1 -1` | No person in view — pupils return to center. |
| `PING` | Uno replies `PONG` — wiring/link sanity check. |
| `TYPE <l> <r>` | Set which port each eye is on at runtime: `3` = GC9A01A on the bit-bang port (as wired, e.g. `TYPE 3 3`), `4` = the same panel moved to the hardware SPI bus. Re-initializes both displays and re-runs the color test. No re-flash needed. Anything else prints the two valid values and changes nothing. |

### Boot behavior (this is your main diagnostic)
When the Uno powers up (or resets) it prints the pin map and controller IDs over serial, then **paints LEFT eye RED and RIGHT eye GREEN for 4 seconds** — the color test — then switches to eye mode. What you see during those 4 seconds tells us exactly what's wrong:

| You see | Meaning | Fix |
|---|---|---|
| Left RED, right GREEN | Wiring + both inits correct | nothing — eyes follow |
| One solid color, other blank/garbage | That eye's wiring or controller type is wrong | re-check that eye's 5 wires; try `TYPE` for that eye |
| Neither shows color | Power problem (VCC/GND/LED/backlight), or the panel is not the GC9A01A the sketch drives | check power first, then confirm the panel is a round 240×240 GC9A01A — nothing else has a driver in this sketch |
| Scrolling lines/garbage | Controller mismatch (ST7735 init on ILI9341 panel or vice-versa), CS/DC swapped, or a display browned out (see the power budget in §2) | try the other `TYPE`; swap CS↔DC; then measure the display's `VCC`–`GND` |
| The Uno **vanishes from the Pi** (`lsusb` lists it, no `/dev/ttyACM0`) | A **host-side** event on the Pi that left the port stuck unconfigured — measured, see §3a. The display load is *not* the cause | replug the Uno's USB (or reset the port) and re-check |

---

## 6. Bring-up test sequence (with the color test)

1. Upload `eyes_tft/eyes_tft.ino`.
2. Watch the screens during boot: **LEFT should flash RED, RIGHT should flash GREEN** for ~4 s, then both show the goggle eye — grey ring at the panel edge, yellow eyelid inside it, white sclera, brown iris with a black pupil and a white highlight, black strap knuckle at each side. A board still running the old firmware shows a flat white circle with a blue iris and no yellow or grey at all, which is the quickest way to tell whether this sketch is actually flashed.
3. If the colors are wrong/blank/garbage, use the table above, and switch controller types from the Pi **without re-flashing**:
   ```bash
   echo 'TYPE 3 3' > /dev/ttyACM0   # the eyes' Uno, see the node table in §3; or TYPE 1 1 / TYPE 2 2
   ```
4. `PING` should print `PONG` on the Pi. With this firmware the boot fills take milliseconds-to-sub-second (the Uno prints `FILL L: …ms R: …ms`), and during tracking it prints `REPAINT n=25 avg=…ms` every 25 iris moves so you can see the repaint cost on the serial line.

1. **Power only:** plug the Uno into the Pi. Both TFTs should light up (backlight on) and the Arduino sketch draws the two eyes.
2. **Serial check:** start the app and run `python eyes/pi_eyes.py` — `connected=True` with `port=/dev/ttyACM0` means the app holds the eyes' Uno (see the node table in §3). If it reports `connected=False`, run `python eyes/pi_eyes.py --list` and check `dmesg | tail`.
3. **Manual gaze test:** release the port from the app first — `curl -X POST 'localhost:5000/eyes?enable=false'` (the gaze loop would otherwise overwrite you) — then `echo "T 90 50" > /dev/ttyACM0` → both pupils drift right. `echo "T -1 -1"` → center. Re-enable with `?enable=true`.
4. **Full system:** stand in front of the camera and wave — the eyes should track you left/right/up/down as you move across the frame.
5. If a display stays blank: swap its `CS`/`DC`/`RST` wires with the other display's to isolate a bad pin vs. a bad display; re-check that `SCK`→`SCK` and `MOSI`→`SDA` (a reversed SDA/SCK is the #1 wiring mistake).

---

## 7. The most common wiring mistake (read this if you used the old table)

An earlier version of this guide shared one SPI bus (SCK→D13, MOSI→D11 for **both** displays) without saying so, and the firmware's default for the left eye is the bit-bang driver:

- LEFT eye, default: `SCL/SCK → D4`, `SDA/MOSI → D3` — the firmware's `TYPE 3`
- RIGHT eye, always: `SCK → D13`, `SDA/MOSI → D11`

**Tying both displays to D13/D11 is now supported** — it is the faster wiring — but you must tell the firmware: `echo 'TYPE 4 3' > /dev/ttyACM0`, and the left eye needs its own `CS`/`DC`/`RST` as always. If you wire the left display to the bus and leave the firmware on `TYPE 3`, that eye stays blank or shows garbage. Which is which:

| Left eye wired to | Set with | Left-eye frame cost |
| --- | --- | --- |
| `SCLK D4`, `MOSI D3` | `TYPE 3 3` (default) | ~9.4 µs/byte, the slow half |
| `SCK D13`, `MOSI D11` (shared bus) | `TYPE 4 3` | ~2 µs/byte, same as the right eye |

## 8. How the eye movement works (data flow)

```
Pi 5 camera → YOLOv8n "person" box → the head box (top 16% x 34%), its centre
            → bearing   = (0.5 - cx)*60 deg             (perception.box_bearing_deg)
              elevation = (cy - 0.5)*46.8 deg           (perception.box_elevation_deg;
                                                         46.8 comes from the aspect:
                                                         640x480 at 60 deg across)
            → px = 50 - bearing/120*100,  py = 50 + elevation/120*100   (one cone)
            → serial "T <px> <py>" (or "T -1 -1" when lost)
Arduino Uno → parses line → maps px,py to pupil offset → redraws both TFTs
```

Both axes are angles in the robot frame and both are mapped with the same 120° cone
(`DEFAULT_SCREEN_FOV_DEG`), so a target 30° up and one 30° to the side move the
pupils equally far. That was not always true — y used the raw frame fraction
(`py = cy*100/frame_h`), which is 2.56x as sensitive per degree — and it was
measurable on the robot: matching the head box against the aim the eyes were sent
left a residual of **16.9 aim units on y** before the fix and **0.25 after**, with
x unchanged at 0.2. A head at frame centre aims at (50,50) either way.

The eye firmware is in `eyes_tft/eyes_tft.ino`; the Pi side is `eyes_gaze.py`, driven by the app's `/eyes` endpoint. The current firmware's boot banner is the fastest way to report back: screenshot the serial output (`PING`, probe IDs, per-eye `init:` lines) and tell me which colors each screen showed in the test phase.

### Quick fault-finding checklist (run these in order)
1. **Backlight** — is the white LED behind the screen on? If not, check the `LED` pin is powered (3.3 V) and its wire isn't swapped with another pin. A lit backlight with no image = init/wiring problem; a dark screen = power problem.
2. **VCC + GND** — measure 3.3 V between `VCC` and `GND` at each display's header.
3. **Color test** — reboot the Uno and note each screen's color during the 4 s test (RED = left, GREEN = right).
4. **Which port the left eye is on** — on the bit-bang port as wired (`TYPE 3 3`) unless its two data wires were moved to the SPI bus (`TYPE 4 3`). The wrong one of the two leaves the left eye blank or garbled and never touches the right eye, so it is the first thing to rule out.
5. **Per-eye wiring** — left eye: D4(clock) D3(data) D10(CS) D9(DC) D8(RST). Right eye: D13(clock) D11(data) D7(CS) D6(DC) D5(RST). A blank eye usually means its CS/DC/RST wires are on the wrong Uno pins.
5b. **Left eye blank/white while the right eye works** — the left eye is the only display driven by the firmware's own bit-bang driver, and before **v5.1** that driver never asserted chip-select while it sent the init sequence. A GC9A01A ignores every byte received with CS high, so `SLPOUT`/`COLMOD`/`DISPON` were all discarded, the panel stayed asleep, and the eye showed nothing however much pixel data followed. Fixed in **v5.1**. Confirm which firmware is actually on the board from the boot banner — the first serial line prints `EYES FW v5.3 - Minion goggle (GC9A01A)` for the current sketch, `v5.1` for the chip-select fix alone, and `v5` or `v5.2` for anything older; the per-eye `init:` line then prints `r=114 lid=105 sclera=95 iris=32`. If the banner is older than `v5.3`, this design is not flashed yet. **Left blank *and* right dead together** points somewhere else entirely — something is holding `D10`, `D11` or `D13`; see the L298P note in §2.
6. **MISO/D12** — the header still allows for it, but nothing reads it: there is no panel-ID probe in the firmware, so adding that wire changes nothing.