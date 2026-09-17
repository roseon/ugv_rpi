"""wifi_status.py — the robot's Wi-Fi health, as one JSON payload.

app.py's /network_status serves snapshot(); the Command Center's header strip
draws from it.  Everything nmcli-shaped lives here so app.py stays out of the
network-stack business.

The watchdog half comes from wifi_default (its report file and service name),
so there is one owner of each fact:
    connection state + signal   -> parsed here, from nmcli
    reconnect history + reboots -> wifi_default's report (wifi_default.json)
    watchdog installed/running  -> systemctl is-active (absent = not installed)

Parse-only helpers are separated from the nmcli calls so the selftest runs
the parsing against recorded output without a radio.
"""

import json
import os
import subprocess

import wifi_default

SERVICE = "wifi-default-watch.service"


def _run(args, timeout=15):
    try:
        return subprocess.run(args, capture_output=True, text=True,
                              timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def parse_device_status(text):
    """`nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status` -> per-device
    info for wifi devices only: {device: {state, connection}}."""
    out = {}
    for line in text.splitlines():
        parts = line.rsplit(":", 3)          # DEVICE:TYPE:STATE:CONNECTION
        if len(parts) != 4 or parts[1] != "wifi":
            continue
        device, _, state, conn = parts
        out[device] = {"state": state or "unknown", "connection": conn or None}
    return out


def parse_signal(text):
    """`nmcli -t -f IN-USE,SIGNAL device wifi list` -> (in_use, signal_pct)
    for the row the robot is actually on (terse mode marks it `*`); with no
    such row, the strongest listed signal and False — the robot is in range
    of its network but not on it, which is exactly what the UI should say."""
    in_use_sig, best = None, None
    for line in text.splitlines():
        used, _, sig = line.rpartition(":")
        if used.startswith("*"):
            in_use_sig = sig
        elif best is None and sig.isdigit():
            best = sig
    if in_use_sig is not None:
        try:
            return (True, int(in_use_sig))
        except ValueError:
            return (True, None)
    try:
        return (False, int(best)) if best is not None else (None, None)
    except ValueError:
        return (False, None)


def watchdog_state(report_path=None):
    """The watchdog half: installed/running from systemctl, history from the
    report file.  Everything is best-effort — a missing piece reads as
    "not installed", never as an error the endpoint would raise on."""
    active = _run(["systemctl", "is-active", SERVICE]).strip()
    report = {"profile": None, "reconnects": [], "reboots": 0}
    try:
        with open(report_path or wifi_default.REPORT_FILE) as fh:
            loaded = json.load(fh)
        report.update({k: loaded[k] for k in report if k in loaded})
    except (OSError, ValueError):
        pass
    return {
        "installed": active != "",
        "active": active == "active",
        "profile": report.get("profile"),
        "reconnects": report.get("reconnects") or [],
        "reboots": int(report.get("reboots") or 0),
    }


def assemble(devices, signal, watchdog):
    """The endpoint's payload from parsed pieces.  `connected` is the wifi
    device's own state — the thing the user means by 'has it got wifi'."""
    wifi = next(((d, i) for d, i in devices.items()
                 if i["state"].startswith("connected")), None)
    if wifi:
        device, info = wifi
        return {"connected": True, "device": device, "state": info["state"],
                "ssid": info["connection"], "signal": signal,
                "watchdog": watchdog, "error": None}
    # Not connected: name the state the radio reports, and how long it has
    # been down if the watchdog is keeping score.
    device = next(iter(devices), None)
    state = devices[device]["state"] if device else "no wifi device"
    outage = None
    try:
        outage = os.path.getmtime(wifi_default.STATE_FILE)
    except OSError:
        pass
    return {"connected": False, "device": device, "state": state,
            "ssid": None, "signal": None, "outage_since": outage,
            "watchdog": watchdog, "error": None}


def snapshot():
    devices = parse_device_status(_run(["nmcli", "-t", "-f",
                                        "DEVICE,TYPE,STATE,CONNECTION",
                                        "device", "status"]))
    used, sig = parse_signal(_run(["nmcli", "-t", "-f", "IN-USE,SIGNAL",
                                   "device", "wifi", "list"]))
    return assemble(devices, sig, watchdog_state())


def selftest():
    fails = []

    def check(name, ok, detail=""):
        print(("PASS " if ok else "FAIL ") + name
              + (("  <- %r" % (detail,)) if detail != "" else ""))
        if not ok:
            fails.append(name)

    devices = parse_device_status(
        "wlan0:wifi:connected:preconfigured\n"
        "eth0:ethernet:unmanaged:\n"
        "wlan1:wifi:disconnected:--\n")
    check("device rows: wifi only, state kept",
          set(devices) == {"wlan0", "wlan1"}
          and devices["wlan0"]["state"] == "connected"
          and devices["wlan0"]["connection"] == "preconfigured"
          and devices["wlan1"]["state"] == "disconnected", devices)

    used, sig = parse_signal("*:78\n :41\ngarbage\n")
    check("signal: the in-use row's percentage", (used, sig) == (True, 78),
          (used, sig))
    used2, sig2 = parse_signal(" :41\n")
    check("signal: none in use -> the strongest listed, flagged False",
          (used2, sig2) == (False, 41), (used2, sig2))
    used3, sig3 = parse_signal("")
    check("signal: no rows at all -> nothing claimed",
          (used3, sig3) == (None, None), (used3, sig3))

    wd = {"installed": True, "active": True, "profile": "home",
          "reconnects": [{"t": 1.0, "after_s": 12.0}], "reboots": 0}
    ok = assemble({"wlan0": {"state": "connected",
                             "connection": "home"}}, 78, wd)
    check("payload: connected, with ssid and signal",
          ok["connected"] and ok["ssid"] == "home" and ok["signal"] == 78
          and ok["error"] is None and ok["watchdog"]["active"], ok)

    down = assemble({"wlan0": {"state": "disconnected", "connection": None}},
                    None, wd)
    check("payload: disconnected names the state and keeps the watchdog",
          not down["connected"] and down["state"] == "disconnected"
          and down["watchdog"]["reconnects"] == [{"t": 1.0, "after_s": 12.0}],
          down)

    print()
    print("RESULT: %d failures" % len(fails))
    if fails:
        print("FAILED: " + ", ".join(fails))
        raise SystemExit(1)
    print("WIFI STATUS PROBES PASSED")


if __name__ == "__main__":
    selftest()
