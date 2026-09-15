#!/bin/bash
# Start, stop, check, install or reload Lance's face on the robot's own screen.
#
#   bash start_face.sh                 start it (if it is not already up)
#   bash start_face.sh --stop          stop it
#   bash start_face.sh --status        is it up, which display, what is it saying
#   bash start_face.sh --selftest      check the animation, no display needed
#   bash start_face.sh --reload        restart it only if it is running
#   bash start_face.sh --install-boot  install/repair the @reboot crontab entry
#
# The face needs the desktop session's display, so it is started from the
# @reboot crontab rather than from the app, which runs headless.  This script
# owns that crontab line: `deploy.sh` only asks it to install or reload, so the
# entry cannot drift from the launcher and a replaced Pi gets it back on the
# next deploy.
#
# Everything about the process lives here on purpose.  Calling `pkill -f
# face_screen.py` from another shell kills that shell too whenever the pattern
# is also written in its own command line - measured, and it looks exactly like
# a dead robot (no output, exit 255).  The pattern below is bracketed so it
# cannot match itself, and callers never need to name the process at all.
cd "$(dirname "$0")" || exit 1
HERE="$(pwd)"
PATTERN='[f]ace_screen\.py'
LOG="${FACE_LOG:-$HOME/lance_face.log}"
PY="${FACE_PYTHON:-$(command -v python3 || echo /usr/bin/python3)}"

# The line this script owns, and the comment that marks it.  The marker lets the
# installer recognise its own work instead of counting on the wording of the
# line, and the comment line is what makes the entry visible to a human reading
# the crontab.
MARK="# lance-face: installed by start_face.sh --install-boot - do not edit by hand"
ENTRY="@reboot sh -c \"sleep 20; /bin/bash $HERE/start_face.sh >> \$HOME/lance_face.log 2>&1\""

# At boot the display and the session come up at their own pace: wait for the
# socket, then try again a few times, because a socket that exists is not yet a
# session that accepts a connection.
DISPLAY_WAIT_S="${FACE_DISPLAY_WAIT_S:-120}"
START_ATTEMPTS="${FACE_ATTEMPTS:-6}"
RETRY_S="${FACE_RETRY_S:-10}"

running() { pgrep -f "$PATTERN" >/dev/null; }
pid_of() { pgrep -f "$PATTERN" | head -1; }

# The crontab lines this script owns.  Comments are excluded: the marker comment
# above names this file, so counting lines that merely mention it made the
# installer see two stale entries and never reach its idempotent path.
boot_lines() { crontab -l 2>/dev/null | grep -v '^[[:space:]]*#' | grep 'start_face\.sh'; }

# Pick up a display that appears after this process started, and the X cookies
# that go with it: cron has neither.
refresh_display() {
  local d
  for d in /tmp/.X11-unix/X*; do
    [ -e "$d" ] && export DISPLAY=":${d##*/X}"
  done
  export DISPLAY="${DISPLAY:-:0}"
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
}

display_socket() { echo "/tmp/.X11-unix/X${DISPLAY#:}"; }

# A socket file is not a session.  Measured: a display whose socket exists but
# has nothing listening refuses the connection, and waiting for the socket alone
# would let the face start against a display that is not up yet - which is the
# @reboot failure mode.  Connect to it instead and wait until it answers.
display_ready() {
  local sock
  sock="$(display_socket)"
  [ -S "$sock" ] || return 1
  "$PY" -c 'import socket,sys
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(3)
s.connect(sys.argv[1])' "$sock" >/dev/null 2>&1
}

RELOAD=0
case "${1:-start}" in
  --selftest|selftest)
    # The only place the module's name appears, so no caller has to type it:
    # a shell that names it and also runs --stop kills itself with its own pkill.
    exec "$PY" face_screen.py --selftest
    ;;
  --stop|stop)
    if ! running; then echo "face: not running"; exit 0; fi
    pkill -f "$PATTERN" 2>/dev/null
    sleep 1
    if running; then
      echo "face: still running (pids: $(pgrep -f "$PATTERN" | tr '\n' ' '))" >&2
      exit 1
    fi
    echo "face: stopped"
    exit 0
    ;;
  --status|status)
    if running; then
      echo "face: running (pid $(pid_of))${DISPLAY:+ on display $DISPLAY}"
    else
      echo "face: not running"
    fi
    echo "boot entry: $(boot_lines | grep -c .) line(s) in the crontab"
    echo "log (last 3):"; tail -3 "$LOG" 2>/dev/null | sed 's/^/  /'
    exit 0
    ;;
  --install-boot|install-boot)
    # A boot entry for a script whose module is not here would fail at every
    # boot, which is worse than no entry: say so instead of installing it.
    if [ ! -f "$HERE/face_screen.py" ]; then
      echo "face: face_screen.py is not next to this script - not installing a boot entry" >&2
      exit 1
    fi
    CUR="$(crontab -l 2>/dev/null || true)"
    ours="$(printf '%s\n' "$CUR" | grep -v '^[[:space:]]*#' | grep 'start_face\.sh' || true)"
    if [ "$(printf '%s\n' "$ours" | grep -c .)" = "1" ] && [ "$ours" = "$ENTRY" ]; then
      echo "face: boot entry already installed and correct - nothing changed"
      exit 0
    fi
    BACKUP="$HOME/crontab_backup_$(date +%Y%m%d_%H%M%S).txt"
    if ! printf '%s\n' "$CUR" > "$BACKUP"; then
      echo "face: could not back up the crontab - not touching it" >&2
      exit 1
    fi
    # Ours by marker, or ours by content for a line someone added by hand.  Every
    # other line is left exactly as it is, which is what makes this safe to run
    # on a crontab shared with the app, the audio sink and jupyter.
    NEW="$(printf '%s\n' "$CUR" | grep -v -e 'start_face\.sh' -e 'lance-face: installed by' || true)"
    TMP="$(mktemp)"
    { printf '%s\n' "$NEW"; printf '%s\n' "$MARK"; printf '%s\n' "$ENTRY"; } > "$TMP"
    if ! crontab "$TMP"; then
      echo "face: installing the boot entry failed" >&2
      rm -f "$TMP"
      if crontab "$BACKUP"; then echo "face: crontab restored from $BACKUP" >&2
      else echo "face: COULD NOT RESTORE the crontab - it is in $BACKUP" >&2; fi
      exit 1
    fi
    rm -f "$TMP"
    echo "face: boot entry installed (replaced $(printf '%s\n' "$ours" | grep -c .) stale line(s)); previous crontab kept in $BACKUP"
    exit 0
    ;;
  --reload|reload)
    if ! running; then echo "face: not running - nothing to reload"; exit 0; fi
    RELOAD=1
    ;;
esac

refresh_display

if running && [ "$RELOAD" = 0 ]; then
  echo "face: already running (pid $(pid_of)) - nothing to do"
  exit 0
fi

if [ "$RELOAD" = 1 ]; then
  pkill -f "$PATTERN" 2>/dev/null
  sleep 1
  if running; then                       # same escalation the app restart needs
    pkill -9 -f "$PATTERN" 2>/dev/null
    sleep 2
  fi
  running && { echo "face: could not stop the old one" >&2; exit 1; }
fi

waited=0
while ! display_ready && [ "$waited" -lt "$DISPLAY_WAIT_S" ]; do
  sleep 2; waited=$((waited + 2)); refresh_display
done
if ! display_ready; then
  echo "face: no display answering at $(display_socket) after ${DISPLAY_WAIT_S}s - trying anyway" >&2
fi

attempt=1
while [ "$attempt" -le "$START_ATTEMPTS" ]; do
  nohup "$PY" face_screen.py >> "$LOG" 2>&1 </dev/null &
  sleep 5
  if running; then
    if [ "$RELOAD" = 1 ]; then echo "face: reloaded (pid $(pid_of)) on display $DISPLAY"
    else echo "face: started (pid $(pid_of)) on display $DISPLAY (attempt $attempt)"; fi
    exit 0
  fi
  echo "face: attempt $attempt/$START_ATTEMPTS did not stay up; retrying in ${RETRY_S}s" >&2
  attempt=$((attempt + 1))
  refresh_display
  # The session may have gone away (or not arrived) mid-retry: wait for it to
  # answer again before spending another attempt.
  waited=0
  while ! display_ready && [ "$waited" -lt 30 ]; do sleep 2; waited=$((waited + 2)); refresh_display; done
  [ "$attempt" -le "$START_ATTEMPTS" ] && sleep "$RETRY_S"
done

echo "face: FAILED to start after $START_ATTEMPTS attempts - see $LOG" >&2
tail -5 "$LOG" 2>/dev/null >&2
exit 1
