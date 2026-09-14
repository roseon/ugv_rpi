#!/usr/bin/env python3
"""Regression tests for serial_ports — the single owner of node and baud identity.

The failures these lock down, all of which handed the eyes' Uno to the LIDAR:

  * ``base_ctrl._pick_lidar_port()`` fell back to ``/dev/ttyACM*`` when no
    ``/dev/ttyUSB*`` existed, so it opened the Uno as the lidar, held the port,
    and DTR-reset it via ``kick_lidar()``;
  * the lidar baud was chosen by ``port.startswith('/dev/ttyUSB')`` — a second
    rule about what a port is, which would drive the Uno at 230400;
  * ``pi_eyes.py`` hardcoded ``/dev/ttyACM0``.

Run:  python3 serial_ports_selftest.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial_ports as sp

PASS = 0
FAIL = 0

# (idVendor, idProduct, product, manufacturer)
UNO = ("2341", "0043", "Arduino Uno", "Arduino (www.arduino.cc)")
CP210X = ("10c4", "ea60", "CP2102N USB to UART Bridge Controller", "Silicon Labs")
CH340 = ("1a86", "7523", "USB Serial", "QinHeng Electronics")
GENERIC = ("1234", "5678", "Mystery Widget", "Acme")


def check(label, got, want):
    global PASS, FAIL
    if got == want:
        PASS += 1
        print("PASS  " + label)
    else:
        FAIL += 1
        print("FAIL  " + label)
        print("        got  %r" % (got,))
        print("        want %r" % (want,))


def make_fixture(root, nodes):
    """Build a fake /dev + sysfs + by-id tree.  Returns (dev, sysfs, byid)."""
    dev = os.path.join(root, "dev")
    sysfs = os.path.join(root, "sys")
    byid = os.path.join(root, "by-id")
    for d in (dev, sysfs, byid):
        os.makedirs(d, exist_ok=True)
    for name, (vid, pid, product, mfr) in nodes.items():
        open(os.path.join(dev, name), "w").close()
        # Mirror the real layout: <sysfs>/<tty>/device/ is the link target and
        # the USB device dir holding idVendor is its parent.
        base = os.path.join(sysfs, name)
        os.makedirs(os.path.join(base, "device"), exist_ok=True)
        for field, value in (("idVendor", vid), ("idProduct", pid),
                             ("product", product), ("manufacturer", mfr)):
            with open(os.path.join(base, field), "w") as fh:
                fh.write(value + "\n")
    return dev, sysfs, byid


def scenario(nodes):
    root = tempfile.mkdtemp(prefix="serialports-")
    sp.DEV_DIR, sp.SYSFS_TTY, sp.BY_ID_DIR = make_fixture(root, nodes)
    return root


def node(name):
    """Expected path for a fixture node, with the platform's separator."""
    return os.path.join(sp.DEV_DIR, name)


def main():
    global PASS, FAIL

    # --- 1. both devices present ---------------------------------------------
    scenario({"ttyACM0": UNO, "ttyUSB0": CP210X})
    check("uno_port() identifies the Arduino", sp.uno_port(), node("ttyACM0"))
    check("lidar_port() identifies the CP210x bridge", sp.lidar_port(), node("ttyUSB0"))
    check("the two ports are different devices", sp.uno_port() != sp.lidar_port(), True)
    check("is_arduino(Uno) is True", sp.is_arduino(node("ttyACM0")), True)
    check("is_arduino(bridge) is False", sp.is_arduino(node("ttyUSB0")), False)
    check("is_bridge(bridge) is True", sp.is_bridge(node("ttyUSB0")), True)
    check("is_bridge(Uno) is False", sp.is_bridge(node("ttyACM0")), False)

    # --- 2. REGRESSION: only the Uno present ---------------------------------
    # The original failure: no ttyUSB bridge at all, so the old code returned
    # acm[0] and handed the Uno to the lidar reader.
    scenario({"ttyACM0": UNO})
    check("REGRESSION: no bridge -> lidar_port() must not take the Uno",
          sp.lidar_port(), None)
    check("REGRESSION: and uno_port() still finds the eyes",
          sp.uno_port(), node("ttyACM0"))

    # --- 3. REGRESSION: baud is decided by identity, not by path -------------
    scenario({"ttyACM0": UNO, "ttyUSB0": CP210X})
    check("REGRESSION: the Uno is never driven at the lidar's 921600",
          sp.baud_for(node("ttyACM0")), sp.BASE_RELAY_BAUD)
    check("the CP210x bridge is driven at 921600",
          sp.baud_for(node("ttyUSB0")), sp.BRIDGE_BAUD)
    check("baud for no port is the safe relay rate",
          sp.baud_for(None), sp.BASE_RELAY_BAUD)

    # --- 4. only the lidar present -------------------------------------------
    scenario({"ttyUSB0": CP210X})
    check("no Uno -> uno_port() must not take the lidar", sp.uno_port(), None)
    check("no Uno -> lidar_port() still finds the bridge", sp.lidar_port(), node("ttyUSB0"))

    # --- 5. a CH340 clone is reported honestly, not guessed at ---------------
    scenario({"ttyUSB0": CH340})
    check("a CH340 clone is not claimed as the eyes' Uno", sp.uno_port(), None)
    check("a CH340 clone is offered as the lidar bridge", sp.lidar_port(), node("ttyUSB0"))
    check("a CH340 clone uses the bridge baud", sp.baud_for(node("ttyUSB0")), sp.BRIDGE_BAUD)

    # --- 6. lidar_port() prefers an actual bridge ----------------------------
    # An unidentified ttyUSB device sorts first; the bridge must still win.
    scenario({"ttyUSB0": GENERIC, "ttyUSB1": CP210X})
    check("lidar_port() skips a non-bridge ttyUSB for the real bridge",
          sp.lidar_port(), node("ttyUSB1"))

    # --- 7. an Arduino sharing the ttyUSB namespace --------------------------
    scenario({"ttyUSB0": UNO, "ttyUSB1": CP210X})
    check("an Arduino on ttyUSB is never returned as the lidar",
          sp.lidar_port(), node("ttyUSB1"))
    check("an Arduino on ttyUSB is still found as the Uno",
          sp.uno_port(), node("ttyUSB0"))

    # --- 8. other_ports(): the ultrasonic's spare adapter --------------------
    scenario({"ttyUSB0": CP210X, "ttyUSB1": GENERIC, "ttyACM0": UNO})
    check("other_ports() excludes the lidar",
          node("ttyUSB0") in sp.other_ports(exclude=(sp.lidar_port(),)), False)
    check("other_ports() offers the spare bridge",
          sp.other_ports(exclude=(sp.lidar_port(),)), [node("ttyUSB1")])
    check("other_ports() never offers the Arduino",
          node("ttyACM0") in sp.other_ports(), False)
    check("other_ports() with nothing spare is empty",
          scenario({"ttyUSB0": CP210X}) and
          sp.other_ports(exclude=(sp.lidar_port(),)), [])

    # --- 9. the base controller's node ---------------------------------------
    check("base_port(pi5=True) is the GPIO UART header",
          sp.base_port(True), "/dev/ttyAMA0")
    check("base_port(pi5=False) is the serial0 alias",
          sp.base_port(False), "/dev/serial0")

    # --- 10. nothing plugged in ---------------------------------------------
    scenario({})
    check("no devices -> uno_port() is None", sp.uno_port(), None)
    check("no devices -> lidar_port() is None", sp.lidar_port(), None)
    check("no devices -> other_ports() is empty", sp.other_ports(), [])
    check("no devices -> inventory() says so",
          "no /dev/tty" in sp.inventory(), True)

    # --- 11. by-id name fallback (needs symlink support) ---------------------
    scenario({"ttyACM0": UNO})
    link = os.path.join(sp.BY_ID_DIR, "usb-Arduino__www.arduino.cc__0043-if00")
    try:
        os.symlink(node("ttyACM0"), link)
    except (OSError, NotImplementedError, AttributeError):
        print("SKIP  by-id symlink checks (symlinks unavailable here)")
    else:
        check("by_id_name() resolves the Arduino link", sp.by_id_name(node("ttyACM0")),
              "usb-Arduino__www.arduino.cc__0043-if00")
        check("by-id name marks it an Arduino", sp.is_arduino(node("ttyACM0")), True)
        check("describe() names the eyes' Uno",
              "Arduino" in sp.describe(node("ttyACM0")), True)

    # --- 12. inventory() lists each node once --------------------------------
    scenario({"ttyACM0": UNO, "ttyUSB0": CP210X})
    lines = [ln for ln in sp.inventory().splitlines() if ln.strip()]
    check("inventory() lists both nodes", len(lines), 2)
    check("inventory() mentions the Uno", any("Arduino" in ln for ln in lines), True)

    print()
    print("%d checks, %d failures" % (PASS + FAIL, FAIL))
    if FAIL:
        print("SERIAL PORTS SELFTEST FAILED")
        return 1
    print("SERIAL PORTS SELFTEST PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
