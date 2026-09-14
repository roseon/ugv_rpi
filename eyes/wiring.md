# Robot Eyes — wiring 2× TFT displays (Arduino Uno ← Raspberry Pi 5)

This guide wires **two SPI TFT displays** (one per "eye") to an **Arduino Uno**, and the Uno to the **Raspberry Pi 5** over USB serial. The Pi runs YOLO person detection and streams the detected person's screen position to the Uno, which moves the pupils toward them — that's what makes the eyes "look" at people.

> Open **`schematic.svg`** for the picture version of this table.

---

## 1. Parts

| Part | Notes |
|---|---|
| 2× SPI TFT display | Most common: **ST7735 1.8″** (128×160) or **ILI9341 2.4″/2.8″** (240×320). Any SPI TFT (ST7789, ILI9488, SSD1331…) works — same wiring, different Arduino library init. |
| 1× Arduino Uno | The Uno's ATmega16U2/CH340 gives the Pi a serial port over USB. |
| 1× Raspberry Pi 5 | Runs `pi_eyes.py` (already has `ultralytics` + `pyserial` installed). |
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
- The left eye runs the firmware's **fast direct-port bit-bang SPI** (mosi=D3, sclk=D4) — a full 240×240 fill takes ~0.4 s instead of ~10 s with the Adafruit software-SPI path, so the eyes track gaze smoothly. The right eye uses hardware SPI (SCK=D13, MOSI=D11).
- `MISO` (D12) is **not connected** — the TFTs don't need it for writing.
- **VCC → 3.3 V** is the safe rule for GC9A01A round modules (they have no onboard regulator — 5 V on VCC destroys them). If your board has an **onboard 3.3 V regulator** (common on ILI9341 2.4″ boards, identifiable by the small regulator + jumper near the header), you may feed `VCC` from the Uno's **5 V** instead — check the board's datasheet/silkscreen first.
- **Power budget — a precaution against display corruption, *not* the diagnosis for a vanished Uno.** The Uno's `3.3V` pin is fed by an on-board LP2985 regulator rated **~50 mA recommended / 150 mA absolute maximum**. A 1.28″ GC9A01A round module draws **~10–20 mA** for logic *and* **~20–40 mA** for the backlight at full brightness, so **two of them on the Uno's 3.3 V pin can exceed that budget** and can brown out a *display*: white screen, random lines, or one eye dying while the other keeps working. Worth fixing on its own merits — give the displays their own **3.3 V supply** (VCC and LED both), tie its GND to the Uno's GND, and leave the Uno's 3.3 V pin for logic only. **It does not explain the Uno disappearing from USB**, for the reasons measured in §3a.
- Backlight: tie `LED` to **3.3 V** on both displays. **Do not move the left eye's `LED` to D3** — D3 is that eye's data line (MOSI), and a PWM backlight on it will fight the SPI data and kill the left eye. There is genuinely no spare PWM pin for two backlights on an Uno with this pin map: D2 is the only free pin, and it can only dim *one* eye. If you need to dim, dim on the **supply side** (a series resistor, or a separate adjustable 3.3 V rail), not from the Uno.

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
2. Install libraries: **Adafruit GFX**, plus **Adafruit ST7735** (1.8″), **Adafruit ILI9341** (2.4″/2.8″) and **Adafruit GC9A01A** (round 1.28″ 240×240).
3. Set `DEFAULT_TYPE` at the top to match your boards (`1`=ST7735, `2`=ILI9341, `3`=GC9A01A round — the default) and confirm the pin constants match §2.
4. Upload. Each screen shows one big cartoon eye (sclera + colored iris + pupil).

### Raspberry Pi 5 — `pi_eyes.py`
```bash
cd ~/ugv_rpi/eyes
source ~/ugv_rpi/ugv-env/bin/activate
python pi_eyes.py            # auto-detects the eyes' Uno by USB identity
python pi_eyes.py --list     # show each USB serial device and what it is
```
It grabs the robot's USB camera, runs YOLOv8n (`person` class, conf 0.35), and streams the person's position to the Uno at ~20 Hz.

### Serial protocol (Pi → Uno, ASCII lines, 115200 baud)
| Command | Meaning |
|---|---|
| `T <px> <py>` | Look toward point; `px`,`py` in **0–100** (50,50 = frame center). Both pupils converge on it. |
| `T -1 -1` | No person in view — pupils return to center. |
| `PING` | Uno replies `PONG` — wiring/link sanity check. |
| `TYPE <l> <r>` | Set each eye's controller at runtime: `1` = ST7735, `2` = ILI9341, `3` = GC9A01A round (e.g. `TYPE 3 3`). Re-initializes both displays and re-runs the color test. No re-flash needed. |

### Boot behavior (this is your main diagnostic)
When the Uno powers up (or resets) it prints the pin map and controller IDs over serial, then **paints LEFT eye RED and RIGHT eye GREEN for 4 seconds** — the color test — then switches to eye mode. What you see during those 4 seconds tells us exactly what's wrong:

| You see | Meaning | Fix |
|---|---|---|
| Left RED, right GREEN | Wiring + both inits correct | nothing — eyes follow |
| One solid color, other blank/garbage | That eye's wiring or controller type is wrong | re-check that eye's 5 wires; try `TYPE` for that eye |
| Neither shows color | Power problem (VCC/GND/LED/backlight) or wrong controller on both | check power first, then `TYPE 3 3` (GC9A01A) vs `TYPE 2 2` / `TYPE 1 1` |
| Scrolling lines/garbage | Controller mismatch (ST7735 init on ILI9341 panel or vice-versa), CS/DC swapped, or a display browned out (see the power budget in §2) | try the other `TYPE`; swap CS↔DC; then measure the display's `VCC`–`GND` |
| The Uno **vanishes from the Pi** (`lsusb` lists it, no `/dev/ttyACM0`) | A **host-side** event on the Pi that left the port stuck unconfigured — measured, see §3a. The display load is *not* the cause | replug the Uno's USB (or reset the port) and re-check |

---

## 6. Bring-up test sequence (with the color test)

1. Upload `eyes_tft/eyes_tft.ino`.
2. Watch the screens during boot: **LEFT should flash RED, RIGHT should flash GREEN** for ~4 s, then both show one cartoon eye filling the round panel (white sclera circle edge-to-edge, colored iris + dark pupil + highlight).
3. If the colors are wrong/blank/garbage, use the table above, and switch controller types from the Pi **without re-flashing**:
   ```bash
   echo 'TYPE 3 3' > /dev/ttyACM0   # the eyes' Uno, see the node table in §3; or TYPE 1 1 / TYPE 2 2
   ```
4. `PING` should print `PONG` on the Pi. With this firmware the boot fills take milliseconds-to-sub-second (the Uno prints `FILL L: …ms R: …ms`), and during tracking it prints `REPAINT n=25 avg=…ms` every 25 iris moves so you can see the repaint cost on the serial line.

1. **Power only:** plug the Uno into the Pi. Both TFTs should light up (backlight on) and the Arduino sketch draws the two eyes.
2. **Serial check:** run `python pi_eyes.py` (it auto-detects the Uno) — the script sends `PING` first and prints `Uno alive: PONG`. If it reports that it cannot identify the Arduino, run `python pi_eyes.py --list` and check `dmesg | tail`.
3. **Manual gaze test:** with the Uno on `/dev/ttyACM0` (see the node table in §3), `echo "T 90 50" > /dev/ttyACM0` → both pupils drift right. `echo "T -1 -1"` → center.
4. **Full system:** stand in front of the camera and wave — the eyes should track you left/right/up/down as you move across the frame.
5. If a display stays blank: swap its `CS`/`DC`/`RST` wires with the other display's to isolate a bad pin vs. a bad display; re-check that `SCK`→`SCK` and `MOSI`→`SDA` (a reversed SDA/SCK is the #1 wiring mistake).

---

## 7. The most common wiring mistake (read this if you used the old table)

An earlier version of this guide shared one SPI bus (SCK→D13, MOSI→D11 for **both** displays). That is **wrong for the current firmware**. The left eye must be on **its own port**:

- LEFT eye: `SCL/SCK → D4`, `SDA/MOSI → D3`
- RIGHT eye: `SCK → D13`, `SDA/MOSI → D11`

If you wired **both** displays' clock to D13 and data to D11, the left eye will be blank or show garbage — move its two wires from D13→D4 and D11→D3. Do **not** tie both displays' SCK or MOSI together.

## 8. How the eye movement works (data flow)

```
Pi 5 camera → YOLOv8n "person" box → center (cx,cy)
            → px = cx*100/frame_w, py = cy*100/frame_h
            → serial "T <px> <py>" (or "T -1 -1" when lost)
Arduino Uno → parses line → smooths gaze → maps px,py to pupil offset
            → redraws both TFTs (iris+pupil move toward the target)
```

The eye firmware is in `eyes_tft/eyes_tft.ino`; the Pi side is `pi_eyes.py`. The current firmware's boot banner is the fastest way to report back: screenshot the serial output (`PING`, probe IDs, per-eye `init:` lines) and tell me which colors each screen showed in the test phase.

### Quick fault-finding checklist (run these in order)
1. **Backlight** — is the white LED behind the screen on? If not, check the `LED` pin is powered (3.3 V) and its wire isn't swapped with another pin. A lit backlight with no image = init/wiring problem; a dark screen = power problem.
2. **VCC + GND** — measure 3.3 V between `VCC` and `GND` at each display's header.
3. **Color test** — reboot the Uno and note each screen's color during the 4 s test (RED = left, GREEN = right).
4. **Controller type** — try `TYPE 1 1` then `TYPE 2 2` and repeat the color test; one of them should light up solid if the wiring is right.
5. **Per-eye wiring** — left eye: D4(clock) D3(data) D10(CS) D9(DC) D8(RST). Right eye: D13(clock) D11(data) D7(CS) D6(DC) D5(RST). A blank eye usually means its CS/DC/RST wires are on the wrong Uno pins.
6. **Optional:** wire the right eye's MISO pin to **D12** — the firmware then auto-detects ST7735 vs ILI9341 and prints the panel ID at boot.