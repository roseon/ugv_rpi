"""Which serial device is which — the single owner of node and baud identity.

Four links exist on this robot and they must never be confused:

    /dev/ttyAMA0   base controller (ESP32) — the Pi's GPIO UART, *not* USB
    /dev/ttyUSB*   D500 LIDAR adapter      — a CP210x / CH34x USB bridge
    /dev/ttyACM*   eyes' Arduino Uno       — an Arduino CDC-ACM device

Every question of the form "what is this port?" and "what baud does it speak?"
is answered here, and every caller asks here.  Guessing by glob or path prefix is
what broke the eyes, in three separate places:

  * ``base_ctrl`` fell back to ``/dev/ttyACM*`` whenever no ``/dev/ttyUSB*``
    existed, so it could open the Uno as though it were the lidar: its
    ``kick_lidar()`` DTR pulse resets an Uno (DTR is wired to RESET), and holding
    the port open meant the eye bridge could never open it either.
  * the lidar baud came from ``port.startswith('/dev/ttyUSB')`` — a second,
    independent rule about what a port is, which would have driven the Uno at
    the lidar's relay rate.
  * ``eyes/pi_eyes.py`` hardcoded ``/dev/ttyACM0``, so it would happily open the
    lidar adapter whenever the Uno was not the device at that node.

Identity comes from USB ids (``idVendor``/``idProduct``) and the
``/dev/serial/by-id`` name — never from position.
"""

import glob
import os

# --- test seams: point these at a fixture tree to exercise this without hardware
DEV_DIR = "/dev"
SYSFS_TTY = "/sys/class/tty"
BY_ID_DIR = "/dev/serial/by-id"

# --- the node each device is expected to occupy
UNO_NODE = "/dev/ttyACM0"        # Arduino Uno R3 (CDC-ACM, 16U2) = the eyes
LIDAR_NODE = "/dev/ttyUSB0"      # D500 CP2102 adapter (a CH340 clone lands on ttyUSBn)
BASE_NODE_PI5 = "/dev/ttyAMA0"   # Pi 5: the GPIO UART header carries the ESP32
BASE_NODE_OTHER = "/dev/serial0" # older Pis: the GPIO UART alias (ttyAMA0 = Bluetooth there)

# --- USB identities
ARDUINO_VID = "2341"             # Arduino SA — the Uno R3
BRIDGE_VIDS = ("10c4", "1a86")   # Silicon Labs CP210x, QinHeng CH34x — USB bridges
_ARDUINO_HINT = "arduino"        # appears in the USB strings and in by-id names

# --- baud: the sensor's native rate through its own bridge, or the base relay
BRIDGE_BAUD = 921600
BASE_RELAY_BAUD = 230400


def usb_identity(dev):
    """(vid, pid, product, manufacturer) for a /dev/ttyX node, or None.

    Reads sysfs by walking up from the tty's device link until the USB device
    directory (the one holding ``idVendor``) is found.
    """
    try:
        path = os.path.realpath(os.path.join(SYSFS_TTY, os.path.basename(dev), "device"))
    except OSError:
        return None
    for _ in range(6):
        if os.path.exists(os.path.join(path, "idVendor")):

            def read(name):
                try:
                    with open(os.path.join(path, name)) as fh:
                        return fh.read().strip()
                except OSError:
                    return ""

            return read("idVendor"), read("idProduct"), read("product"), read("manufacturer")
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None


def by_id_name(dev):
    """The /dev/serial/by-id name pointing at `dev`, or ''."""
    try:
        want = os.path.realpath(dev)
    except OSError:
        return ""
    for link in glob.glob(os.path.join(BY_ID_DIR, "*")):
        try:
            if os.path.realpath(link) == want:
                return os.path.basename(link)
        except OSError:
            continue
    return ""


def is_arduino(dev):
    """True if `dev` is an Arduino (the eyes' Uno) rather than a bridge.

    Matched on vendor id 2341 or an "arduino" hint in the USB strings / by-id
    name.  A CH340 clone cannot be told from a CH340-anything by vendor id, so
    those are reported as unidentified rather than guessed at — pass ``--port``
    explicitly for a clone.
    """
    ident = usb_identity(dev)
    if ident:
        vid, _pid, product, manufacturer = ident
        if vid == ARDUINO_VID:
            return True
        if _ARDUINO_HINT in ("%s %s" % (product, manufacturer)).lower():
            return True
    return _ARDUINO_HINT in by_id_name(dev).lower()


def is_bridge(dev):
    """True for a USB serial bridge — the class the lidar's adapter belongs to."""
    ident = usb_identity(dev)
    if ident and ident[0] in BRIDGE_VIDS:
        return True
    name = by_id_name(dev).lower()
    return "cp210" in name or "ch340" in name or "silicon_labs" in name


def candidates():
    """Every USB serial node present, ttyUSB* first (the bridge namespace)."""
    return (sorted(glob.glob(os.path.join(DEV_DIR, "ttyUSB*"))) +
            sorted(glob.glob(os.path.join(DEV_DIR, "ttyACM*"))))


def uno_port():
    """The eyes' Arduino Uno node, or None if it cannot be identified.

    Never falls back to a position: an unidentifiable port is reported as None so
    the caller can show what was found instead of opening the wrong device.
    """
    for dev in candidates():
        if is_arduino(dev):
            return dev
    return None


def lidar_port():
    """The D500 lidar adapter node, or None.

    Prefers an actual USB bridge (CP210x/CH34x); otherwise falls back to any
    non-Arduino port, so kits that relay the stream through the ESP32 base board
    still work.  Never returns an Arduino, so the eye hardware can never be
    claimed as the lidar and DTR-reset by ``kick_lidar()``.
    """
    ports = [dev for dev in candidates() if not is_arduino(dev)]
    for dev in ports:
        if is_bridge(dev):
            return dev
    return ports[0] if ports else None


def other_ports(exclude=()):
    """Bridge-namespace ports that are neither an Arduino nor excluded.

    This is where the optional ultrasonic sensor's adapter is found — it lives on
    a second ttyUSB adapter when one is fitted.  Centralised so no caller needs a
    glob of its own.
    """
    skip = set(exclude)
    return [dev for dev in candidates()
            if dev not in skip
            and not is_arduino(dev)
            and os.path.basename(dev).startswith("ttyUSB")]


def base_port(pi5=True):
    """The base controller's node — the Pi's GPIO UART, never USB.

    ``pi5`` selects the platform's flavour: a Pi 5 exposes the header directly,
    older Pis use the ``/dev/serial0`` alias (their ``ttyAMA0`` is Bluetooth).
    """
    return BASE_NODE_PI5 if pi5 else BASE_NODE_OTHER


def baud_for(dev):
    """The baud the device on `dev` speaks.

    The D500's own adapter is a USB bridge carrying the sensor's native stream at
    921600; kits wired through the ESP32 base board relay it at 230400.  Keyed on
    identity, not on the path, so a port that is not a bridge — in particular an
    Arduino — can never be driven at the lidar's rate by accident.  The basename
    check is only a fallback for when sysfs cannot be read.
    """
    if dev is None:
        return BASE_RELAY_BAUD
    if is_bridge(dev):
        return BRIDGE_BAUD
    if "cp210" in by_id_name(dev).lower():
        return BRIDGE_BAUD
    return BASE_RELAY_BAUD if os.path.basename(dev).startswith("ttyACM") else BRIDGE_BAUD


def describe(dev):
    """One human-readable line naming what `dev` is (for error messages)."""
    ident = usb_identity(dev)
    name = by_id_name(dev)
    if is_arduino(dev):
        kind = "Arduino (eyes' Uno)"
    elif is_bridge(dev):
        kind = "serial bridge (lidar adapter?)"
    else:
        kind = "unidentified"
    detail = ""
    if ident:
        detail = "vid:pid %s:%s %s" % (ident[0], ident[1], ident[2])
    elif name:
        detail = name
    return "%-16s %s%s" % (dev, kind, ("  [%s]" % detail) if detail else "")


def inventory():
    """Multi-line string describing every USB serial node currently present."""
    devs = candidates()
    if not devs:
        return "no /dev/ttyACM* or /dev/ttyUSB* nodes present"
    return "\n".join("  " + describe(d) for d in devs)
