#!/usr/bin/env python3
"""wifi_default.py — return the robot to its default network when it drops.

Runs under systemd (wifi_default.sh --install), ticking every
RECONNECT_INTERVAL_S:

    * nothing active        -> nmcli connection up <the pinned profile>
    * disconnected 15 min   -> systemctl reboot, so a hung radio comes back too

The pinned profile is the first wifi profile whose settings carry the pins
`wifi_default.sh --apply` sets (autoconnect yes, top priority) — matched on
settings rather than the name, so a renamed profile still matches.  The
outage clock lives in /run (tmpfs): it cannot survive a reboot, which is
exactly what a recovery clock should do.
"""

import os
import subprocess
import sys
import time

RECONNECT_INTERVAL_S = 30
RECOVERY_MAX_MIN = 15          # disconnected this long -> reboot; 0 disables
STATE_FILE = "/run/wifi_default.state"   # the outage clock: tmpfs, dies at reboot
REPORT_FILE = os.path.join(os.path.expanduser("~"),
                           "wifi_default_report.json")  # history: survives
HEARTBEAT_TICKS = 20           # one status line a minute while unchanged

OUTAGE_LIMIT_S = RECOVERY_MAX_MIN * 60


def log(msg):
    print("[wifi] %s" % msg, flush=True)


def _nm(args, timeout=30):
    """One nmcli command.  Every argument is a fixed string — the profile
    name arrives from nmcli itself, never from outside."""
    try:
        return subprocess.run(["nmcli"] + args, capture_output=True,
                              text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None


def active_connections():
    """Active connections on real devices (loopback excluded)."""
    r = _nm(["-t", "-f", "NAME,DEVICE", "connection", "show", "--active"])
    if r is None or r.returncode != 0:
        return []
    out = []
    for line in r.stdout.splitlines():
        name, _, device = line.partition(":")
        if device and device != "lo":
            out.append(name)
    return out


def pinned_profile():
    """The profile --apply pinned: the wifi profile with autoconnect yes and
    the highest priority.  A profile name may itself contain a colon, so
    each `-t` line is split from the END (name first, fields after)."""
    r = _nm(["-t", "-f", "NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY",
             "connection", "show"])
    if r is None or r.returncode != 0:
        return None
    best_name, best_prio = None, None
    for line in r.stdout.splitlines():
        parts = line.rsplit(":", 3)
        if len(parts) != 4:
            continue
        name, type_, auto, prio = parts
        if type_ != "802-11-wireless" or auto != "yes":
            continue
        try:
            p = int(prio)
        except ValueError:
            continue
        if best_prio is None or p > best_prio:
            best_name, best_prio = name, p
    return best_name


def outage_started_s():
    try:
        return os.path.getmtime(STATE_FILE)
    except OSError:
        return 0.0


def mark_outage():
    if outage_started_s() == 0.0:
        try:
            open(STATE_FILE, "w").close()
        except OSError:
            pass


def clear_outage():
    try:
        os.remove(STATE_FILE)
    except OSError:
        pass


def _write_report(profile, reconnects, reboots):
    """The history /network_status reads.  Best effort: the watchdog's own job
    (rejoining) must never depend on whether the report file is writable."""
    try:
        import json
        with open(REPORT_FILE, "w") as fh:
            json.dump({"profile": profile, "reconnects": reconnects,
                       "reboots": reboots}, fh)
    except OSError:
        pass


class _Report:
    """The watchdog's published history: the last reconnects (newest first,
    capped) and how many times the reboot arm fired.  The *live* outage is
    STATE_FILE's mtime (/run, dies at reboot — which is what a recovery clock
    should do); history is kept next to the app so it survives one."""

    def __init__(self):
        self.profile = None
        self.reconnects = []      # [{"t": epoch, "after_s": s}] newest first
        self.reboots = 0
        self._load()

    def _load(self):
        try:
            import json
            with open(REPORT_FILE) as fh:
                d = json.load(fh)
            self.profile = d.get("profile")
            self.reconnects = d.get("reconnects") or []
            self.reboots = int(d.get("reboots") or 0)
        except (OSError, ValueError):
            pass

    def save(self):
        _write_report(self.profile, self.reconnects, self.reboots)

    def note_reconnected(self, after_s):
        self.reconnects.insert(0, {"t": time.time(), "after_s": round(after_s, 1)})
        del self.reconnects[5:]
        self.save()

    def note_reboot(self):
        self.reboots += 1
        self.save()


REPORT = None


def _report():
    global REPORT
    if REPORT is None:
        REPORT = _Report()
    return REPORT


def tick(counter):
    """One daemon step: decide, act, report.  Returns True when connected."""
    rep = _report()
    active = active_connections()
    if active:
        if outage_started_s():
            after_s = int(time.time() - outage_started_s())
            log("back on %s after %d s" % (", ".join(active), after_s))
            clear_outage()
            rep.note_reconnected(after_s)   # NM's own autoconnect counts too
        return True
    mark_outage()
    prof = pinned_profile()
    if prof is None:
        if counter % HEARTBEAT_TICKS == 0:
            log("no connection and no pinned wifi profile - nothing to bring up")
        return False
    down_s = int(time.time() - outage_started_s())
    if counter % HEARTBEAT_TICKS == 0:
        log("disconnected %d s - trying '%s'" % (down_s, prof))
    r = _nm(["connection", "up", prof], timeout=90)
    if r is not None and r.returncode == 0:
        log("reconnected via '%s' after %d s" % (prof, down_s))
        clear_outage()
        rep.profile = prof
        rep.note_reconnected(down_s)
        return True
    # A profile that exists but cannot connect for RECOVERY_MAX_MIN means the
    # radio or the stack is wedged: a reboot rejoins at boot (autoconnect).
    # With no pinned profile at all a reboot would only loop — never taken.
    if OUTAGE_LIMIT_S and down_s >= OUTAGE_LIMIT_S:
        log("no connection for %d min - rebooting to recover the radio"
            % RECOVERY_MAX_MIN)
        clear_outage()
        rep.profile = prof
        rep.note_reboot()
        subprocess.run(["systemctl", "reboot"])
    return False


def selftest():
    """The decision logic, offline: nmcli and systemd are stubbed, the state
    machine and the file clock are the real ones."""
    global STATE_FILE, REPORT_FILE
    import tempfile
    handle = tempfile.NamedTemporaryFile(delete=False)
    handle.close()
    os.remove(handle.name)
    STATE_FILE = handle.name
    # The report path must be unique to this run: %TEMP% is stable, so a fixed
    # name there would load the previous run's history into case 1 and every
    # count-based check would be a coin flip.
    REPORT_FILE = STATE_FILE + ".report.json"
    global REPORT
    REPORT = None               # each run of the decision machine starts fresh
    import atexit
    atexit.register(lambda: [os.remove(p) for p in (STATE_FILE, REPORT_FILE)
                             if os.path.exists(p)])

    class FakeNM:
        def __init__(self, active, profile, up_ok=True):
            self.active, self.profile, self.up_ok = active, profile, up_ok
            self.up_calls = 0

        def __call__(self, args, timeout=30):
            class R:
                def __init__(self, rc, out):
                    self.returncode, self.stdout = rc, out
            if "--active" in args:
                return R(0, self.active)
            if "show" in args and "-t" in args:
                return R(0, self.profile)
            if args[:2] == ["connection", "up"]:
                self.up_calls += 1
                return R(0 if self.up_ok else 4, "")
            return R(0, "")

    fails = []

    def check(name, ok, detail=""):
        print(("PASS " if ok else "FAIL ") + name
              + (("  <- %s" % (detail,)) if detail != "" else ""))
        if not ok:
            fails.append(name)

    HOME = "home:802-11-wireless:yes:100\n"

    def arm(fake):
        globals()["_nm"] = fake

    # 1. a healthy link does nothing at all
    fake = FakeNM("home:wlan0\n", HOME)
    arm(fake)
    clear_outage()
    tick(0)
    check("a healthy link does nothing",
          fake.up_calls == 0 and outage_started_s() == 0.0, fake.up_calls)

    # 2. loopback-only "active" is not a connection: bring the wifi up
    fake = FakeNM("lo:lo\n", HOME, up_ok=True)
    arm(fake)
    clear_outage()
    tick(0)
    check("loopback alone counts as offline and reconnects",
          fake.up_calls == 1 and outage_started_s() == 0.0, fake.up_calls)

    # 3. disconnected, activation works: reconnect and clear the clock
    fake = FakeNM("", HOME, up_ok=True)
    arm(fake)
    clear_outage()
    tick(0)
    check("a drop triggers the rejoin",
          fake.up_calls == 1 and outage_started_s() == 0.0, fake.up_calls)
    last = _report().reconnects[0] if _report().reconnects else None
    check("...and the reconnect lands in the report",
          bool(last) and set(last) == {"t", "after_s"}
          and last["after_s"] >= 0, last)

    # 4. disconnected, activation fails, outage young: keep trying, no reboot
    fake = FakeNM("", HOME, up_ok=False)
    arm(fake)
    clear_outage()
    tick(0)
    tick(1)
    check("a failing rejoin retries without the reboot arm",
          fake.up_calls == 2 and outage_started_s() > 0.0, fake.up_calls)

    # 5. disconnected past the limit: reboot, and the clock dies first
    fake = FakeNM("", HOME, up_ok=False)
    arm(fake)
    clear_outage()
    mark_outage()
    old = time.time() - (RECOVERY_MAX_MIN + 1) * 60
    os.utime(STATE_FILE, (old, old))
    rebooted = []
    real_run = subprocess.run
    subprocess.run = lambda cmd, **kw: rebooted.append(list(cmd))
    tick(0)
    subprocess.run = real_run
    check("an outage past the limit reboots the robot",
          bool(rebooted) and rebooted[0][:2] == ["systemctl", "reboot"]
          and outage_started_s() == 0.0, rebooted)
    check("...and the reboot lands in the report",
          _report().reboots == 1, _report().reboots)
    check("...and the history survives a restart (new instance reads it back)",
          _Report().reboots == 1 and len(_Report().reconnects) >= 1,
          (_Report().reboots, len(_Report().reconnects)))

    # 6. no pinned profile: no rejoin attempt and never a reboot loop
    fake = FakeNM("", "wired:ethernet:yes:0\n", up_ok=False)
    arm(fake)
    clear_outage()
    mark_outage()
    old = time.time() - (RECOVERY_MAX_MIN + 5) * 60
    os.utime(STATE_FILE, (old, old))
    rebooted.clear()
    subprocess.run = lambda cmd, **kw: rebooted.append(list(cmd))
    tick(0)
    subprocess.run = real_run
    check("no pinned profile: no rejoin, no reboot",
          fake.up_calls == 0 and not rebooted, (fake.up_calls, rebooted))

    clear_outage()
    print()
    print("RESULT: %d failures" % len(fails))
    if fails:
        print("FAILED: " + ", ".join(fails))
        raise SystemExit(1)
    print("WIFI WATCHDOG PROBES PASSED")


def main():
    log("watchdog up (interval %ds, reboot after %s min)"
        % (RECONNECT_INTERVAL_S, RECOVERY_MAX_MIN or "never"))
    counter = 0
    while True:
        try:
            tick(counter)
        except Exception as e:      # one bad tick must not kill the daemon
            log("tick failed: %r" % (e,))
        counter += 1
        time.sleep(RECONNECT_INTERVAL_S)


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        main()
