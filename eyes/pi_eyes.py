#!/usr/bin/env python3
"""Eyes link + status tool (Raspberry Pi side) — READ ONLY.

The gaze is owned by the robot app.  ``app.py``'s ``EyeGazer`` (eyes_gaze.py)
fuses LIDAR proximity with what the camera sees and is the only thing that ever
writes ``T <px> <py>`` to the Uno.  Two writers on one serial port interleave
into garbled pupil positions, and a second ``cv2.VideoCapture`` fights the frame
loop for the camera — so this tool does neither.  It reports what the eyes are
doing and which node the link is on.

    python pi_eyes.py                  # one-shot status from the running app
    python pi_eyes.py --watch          # follow it until Ctrl-C
    python pi_eyes.py --list           # which /dev node is which (no hardware)
    python pi_eyes.py --url http://192.168.24.25:5000

Exit status is 0 when the gaze link is up, 1 otherwise, so it is scriptable.
"""

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request

DEFAULT_URL = "http://127.0.0.1:5000"

# The gaze reasons eyes_gaze.choose_gaze() can report, in plain words.
REASON_TEXT = {
    "lidar_close": "something is too close",
    "camera": "an object the camera recognises",
    "lidar_near": "something near",
    "hold": "holding the last gaze",
    "idle": "nothing worth looking at - centred",
    "stopped": "gaze switched off",
}


def fetch_status(base_url, timeout=3.0):
    """The app's own view of the eyes.  Raises on no app / bad reply."""
    with urllib.request.urlopen(base_url.rstrip("/") + "/eyes_status",
                                timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8", "replace"))


def describe(status):
    """One short block a human can read at the console."""
    target = status.get("target")
    if target:
        where = f"px={target['px']:>3} py={target['py']:>3}"
    else:
        where = "centred (T -1 -1)"

    lidar = ""
    if status.get("lidar_mm") is not None:
        lidar = f"  lidar {status['lidar_mm']:.0f} mm at {status['lidar_bearing_deg']:+.1f} deg"
    cam = status.get("camera")
    cam_txt = f"  camera={cam}" if cam else ""

    lines = [
        "eyes:  enabled={}  connected={}  port={}".format(
            status.get("enabled"), status.get("connected"), status.get("port")),
        "gaze:  {}  ({})".format(
            where, REASON_TEXT.get(status.get("reason"), status.get("reason"))),
        "rules: look at <= {:.0f} mm, take over <= {:.0f} mm, {:.0f} Hz".format(
            status.get("prox_mm") or 0, status.get("close_mm") or 0,
            status.get("hz") or 0),
        "bus:   sent={} errors={} age={}s   hits={}".format(
            status.get("sent"), status.get("errors"), status.get("age_s"),
            status.get("camera_hits")),
    ]
    if lidar or cam_txt:
        lines.insert(2, "input: " + (lidar + cam_txt).strip())
    return "\n".join(lines)


def list_devices():
    """Which /dev node is which.  Reads sysfs only; opens nothing."""
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import serial_ports
    print("USB serial devices:")
    print(serial_ports.inventory())
    print()
    print(f"  eyes' Uno:  {serial_ports.uno_port() or 'not detected'}")
    print(f"  lidar port: {serial_ports.lidar_port() or 'not detected'}")
    print(f"  base port:  {serial_ports.base_port()}")
    print()
    print("The app opens the Uno; a node listed here being correct does not mean")
    print("the app has it - check `python pi_eyes.py` for `connected=True`.")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Report the robot's eye-gaze state (read-only).")
    ap.add_argument("--list", action="store_true",
                    help="list the USB serial devices and what each one is, then exit")
    ap.add_argument("--url", default=DEFAULT_URL,
                    help=f"the robot app's base URL (default {DEFAULT_URL})")
    ap.add_argument("--watch", action="store_true",
                    help="poll until Ctrl-C, printing only changes")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="seconds between polls in --watch mode (default 1.0)")
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="per-request timeout in seconds (default 3.0)")
    args = ap.parse_args()

    if args.list:
        list_devices()
        return 0

    def once():
        try:
            status = fetch_status(args.url, args.timeout)
        except urllib.error.URLError as e:
            print(f"cannot reach the robot app at {args.url}: {e}")
            print("The eyes are driven by the app - start it (or pass --url).")
            return None
        except (ValueError, json.JSONDecodeError) as e:
            print(f"the app replied with something that is not JSON: {e}")
            return None
        print(describe(status))
        return status

    first = once()
    if first is None:
        return 1
    if not first.get("connected"):
        print()
        print("The gaze link is NOT open. The eyes are probably not enumerating -")
        print("run `python eyes/pi_eyes.py --list` and `dmesg | tail` on the Pi.")

    if not args.watch:
        return 0 if first.get("connected") else 1

    last = first
    try:
        while True:
            time.sleep(max(0.2, args.interval))
            try:
                status = fetch_status(args.url, args.timeout)
            except urllib.error.URLError as e:
                print(f"lost the app: {e}")
                return 1
            except ValueError as e:      # includes json.JSONDecodeError
                print(f"the app replied with something that is not JSON: {e}")
                continue
            if (status.get("reason"), status.get("target"),
                    status.get("connected")) != (last.get("reason"),
                                                 last.get("target"),
                                                 last.get("connected")):
                print("---")
                print(describe(status))
                last = status
    except KeyboardInterrupt:
        print("\nstopped")
    return 0 if last.get("connected") else 1


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass
    sys.exit(main())
