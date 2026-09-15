#!/bin/bash
# Deploy the app's Python modules to the robot, and restart it.
#
#   bash deploy.sh                     # ship to $ROBOT_HOST, verify, restart the app
#   bash deploy.sh --manifest          # print the module set that WOULD be shipped; touch nothing
#   bash deploy.sh --no-restart        # ship and verify, but leave the running app alone
#   ROBOT_HOST=ws@192.168.24.31 bash deploy.sh
#   ROBOT_PASS='...' bash deploy.sh    # password auth (creates an askpass helper)
#
# The file list is NOT hand-written: it is the transitive closure of the local
# modules app.py imports, read out of the source with `ast`.  eyes_gaze.py was
# absent from the Pi precisely because the old hand-written list did not mention
# it, so the app ran without the gaze and the eyes never moved.  A new module
# cannot be forgotten now: if app.py imports it, or anything it imports does, it
# ships.
#
# Not modules, and deliberately not in this set: config.yaml and templates/,
# which the app reads at runtime and which are already on the robot.
#
# These two ARE shipped, though nothing imports them: the face on the robot's
# own screen is a second process (it owns a display, so it cannot live in the
# app) and start_face.sh is what launches it at boot.  Kept as an explicit short
# list rather than a glob, because the manifest's whole point is that the set is
# knowable - a file with no importer has to be named.
#
# The detector's weights ship too when they are here.  They are not importable
# code, so the manifest cannot find them, and ultralytics would fetch them from
# GitHub on first use - which a robot with no route there cannot do.
set -u

ROBOT_HOST="${ROBOT_HOST:-ws@192.168.24.25}"
REMOTE_DIR="${REMOTE_DIR:-/home/ws/ugv_rpi}"
WEIGHTS="${WEIGHTS:-yolov8n.pt}"      # eyes_gaze's default detector, also cv_ctrl's
# Fail in seconds on a wrong or sleeping host rather than hanging on a TCP SYN.
SSH_OPTS=(-o ConnectTimeout=15 -o ServerAliveInterval=10 -o ServerAliveCountMax=3)
PY="${PYTHON:-python}"
HERE="$(cd "$(dirname "$0")" && pwd)"
# The manifest holds repo-relative names, so everything below must run from the
# repo root even when the script is invoked as `bash ../deploy.sh`.
cd "$HERE" || exit 1

MODE=deploy
RESTART=1
for arg in "$@"; do
  case "$arg" in
    --manifest|--dry-run) MODE=manifest ;;
    --no-restart)         RESTART=0 ;;
    -h|--help)            sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg  (--manifest, --no-restart, --help)" >&2; exit 2 ;;
  esac
done

command -v "$PY" >/dev/null 2>&1 || { echo "'$PY' not found; set PYTHON=/path/to/python" >&2; exit 1; }

# ── the module set, derived from what app.py actually imports ────────────────
FILES="$("$PY" - "$HERE" <<'PY'
import ast
import os
import sys

root = os.path.abspath(sys.argv[1])
ENTRY = "app.py"


def local_path(dotted):
    """Repo path for a dotted module name (or a plain file name), else None.

    Only names that exist as a file here count, so stdlib and third-party
    imports (os, cv2, flask, ultralytics, serial...) drop out by construction.
    """
    if dotted.endswith(".py"):              # the entry point arrives as app.py
        dotted = dotted[:-3]
    parts = dotted.split(".")
    as_file = os.path.join(root, *parts) + ".py"
    if os.path.isfile(as_file):
        return as_file
    as_pkg = os.path.join(root, *parts, "__init__.py")
    if os.path.isfile(as_pkg):
        return as_pkg
    return None


def imported_names(path):
    """Every module name imported in the file, including inside functions."""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            tree = ast.parse(fh.read())
    except SyntaxError as e:
        print(f"[manifest] cannot parse {path}: {e}", file=sys.stderr)
        return []
    names = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            if node.level:                    # relative import: package-internal
                continue
            if node.module:
                names.append(node.module)
                names += [f"{node.module}.{a.name}" for a in node.names]
    return names


seen, out = set(), []
stack = [ENTRY]
while stack:
    name = stack.pop()
    path = local_path(name)
    if path is None:
        continue
    full = os.path.abspath(path)
    if full in seen:
        continue
    seen.add(full)
    out.append(os.path.relpath(full, root).replace(os.sep, "/"))
    stack += imported_names(full)

# Written as bytes, not with print(): on Windows print() emits CRLF, and a
# trailing \r on each name makes `tar` unable to stat the file and makes the
# far-side `py_compile` look for a name that does not exist.  The list is
# consumed by shell word-splitting, which does not strip \r.
sys.stdout.buffer.write(("\n".join(sorted(out)) + "\n").encode("utf-8"))
PY
)"

if [ -z "$FILES" ]; then
  echo "derived an empty module set - is app.py here?" >&2
  exit 1
fi

EXTRA=("face_screen.py" "start_face.sh")
for extra in "${EXTRA[@]}"; do
  [ -f "$extra" ] && FILES="$FILES
$extra"
done

COUNT="$(printf '%s\n' "$FILES" | wc -l | tr -d ' ')"
LIST="${FILES//$'\n'/ }"          # one line, for the places that need arguments
# py_compile is for Python.  Feeding it start_face.sh made the far-side check
# print a SyntaxError and then still exit 0 (the remote shell had no `set -e`),
# so a failure and a success looked the same from here.  Compile the modules.
PY_LIST="$(printf '%s\n' "$FILES" | grep -E '\.py$' | tr '\n' ' ')"

if [ "$MODE" = "manifest" ]; then
  echo "# $COUNT local modules app.py reaches (transitively), from $HERE"
  printf '%s\n' "$FILES"
  # Weights are a blob, not a module, so this list cannot show them.
  if [ -f "$WEIGHTS" ]; then echo "# weights: $WEIGHTS would ship too"
  else echo "# weights: no $WEIGHTS here - the robot must have its own, or nobody is detected"; fi
  exit 0
fi

if [ -n "${ROBOT_PASS:-}" ]; then
  printf '#!/bin/sh\necho "%s"\n' "$ROBOT_PASS" > /tmp/deploy_askpass.sh
  chmod +x /tmp/deploy_askpass.sh
  export SSH_ASKPASS=/tmp/deploy_askpass.sh SSH_ASKPASS_REQUIRE=force DISPLAY=:
fi

echo "[deploy] $COUNT modules -> $ROBOT_HOST:$REMOTE_DIR"
printf '  %s\n' $FILES

# ── keep the version being replaced, with nothing but POSIX shell ────────────
printf '%s\n' "$FILES" | ssh "${SSH_OPTS[@]}" "$ROBOT_HOST" "
  cd '$REMOTE_DIR' || exit 1
  rm -rf backup_deploy_prev && mkdir -p backup_deploy_prev
  while read -r f; do
    if [ -f \"\$f\" ]; then
      mkdir -p \"backup_deploy_prev/\$(dirname \"\$f\")\"
      cp -p \"\$f\" \"backup_deploy_prev/\$f\"
    fi
  done
  echo '[deploy] previous copies kept in backup_deploy_prev/'
" || { echo "[deploy] could not connect to $ROBOT_HOST or cd into $REMOTE_DIR - wrong host, robot asleep, or wrong REMOTE_DIR?" >&2; exit 1; }

# ── ship ─────────────────────────────────────────────────────────────────────
tar czf - $FILES | ssh "${SSH_OPTS[@]}" "$ROBOT_HOST" "cd '$REMOTE_DIR' && tar xzf - && echo '[deploy] files extracted'" \
  || { echo "[deploy] transfer failed" >&2; exit 1; }

# ── the detector's weights ───────────────────────────────────────────────────
# A blob, not a module: its own tar, never passed to py_compile.  Absent locally
# it is simply not shipped, and the far-side check below says so out loud.
if [ -f "$WEIGHTS" ]; then
  tar czf - "$WEIGHTS" | ssh "${SSH_OPTS[@]}" "$ROBOT_HOST" "cd '$REMOTE_DIR' && tar xzf - && echo '[deploy] weights shipped: $WEIGHTS'" \
    || { echo "[deploy] weights transfer failed" >&2; exit 1; }
fi

# ── check the far side, and say plainly whether the gaze is there ────────────
printf '%s\n' "$FILES" | ssh "${SSH_OPTS[@]}" "$ROBOT_HOST" "
  cd '$REMOTE_DIR' || exit 1
  P=./ugv-env/bin/python
  if [ ! -x \"\$P\" ]; then echo '[check] no interpreter at' \"\$P\" '- is the venv there?'; exit 1; fi
  missing=0
  while read -r f; do
    [ -f \"\$f\" ] || { echo \"[check] MISSING on the robot: \$f\" >&2; missing=1; }
  done
  [ \$missing -eq 0 ] && echo '[check] every module present'
  \$P -m py_compile $PY_LIST || { echo '[check] a module does not compile on the robot' >&2; exit 1; }
  echo '[check] all modules compile on the robot'
  \$P -c 'import eyes_gaze; print(\"[check] gaze module importable:\", eyes_gaze.__file__)'
  if [ -f '$WEIGHTS' ]; then
    echo '[check] gaze weights present: $WEIGHTS'
  else
    echo '[check] gaze weights MISSING: no person can be detected - put $WEIGHTS in $REMOTE_DIR'
  fi
  if [ \$missing -ne 0 ]; then exit 1; fi
" || { echo "[deploy] the far side is missing modules or cannot import the gaze - not restarting" >&2; exit 1; }

# ── restart as exactly one process ───────────────────────────────────────────
# pgrep -f matches the whole command text, and this whole command text travels
# in the remote shell's argv: measured on the robot, 'pgrep -af' listed its own
# 'bash -c ...' line right beside the app.  The bracket in '[a]pp\.py' hides only
# the pattern itself, so the launch line below must not spell the name either -
# otherwise pkill kills the very shell that was going to do the restart.
if [ "$RESTART" = 1 ]; then
  ssh "${SSH_OPTS[@]}" "$ROBOT_HOST" "
    cd '$REMOTE_DIR' || exit 1
    script=\"ap\"\"p.py\"
    pkill -f '[a]pp\.py' 2>/dev/null
    sleep 2
    # Measured: the app does not exit on SIGTERM (still on port 5000 six seconds
    # later), so a plain pkill leaves the old process owning the camera while the
    # new one starts against a busy device.  Escalate instead of stopping short.
    if pgrep -f '[a]pp\.py' >/dev/null; then
      echo '[deploy] the running app ignored SIGTERM - escalating to SIGKILL'
      pkill -9 -f '[a]pp\.py' 2>/dev/null
      sleep 3
    fi
    if ! pgrep -f '[a]pp\.py' >/dev/null; then
      nohup ./ugv-env/bin/python \"\$script\" >> app.log 2>&1 </dev/null &
      sleep 4
    fi
    n=\$(pgrep -cf '[a]pp\.py')
    echo \"[deploy] app processes: \$n\"
    pgrep -af '[a]pp\.py'
    if [ \"\$n\" != 1 ]; then echo '[deploy] expected exactly one app process' >&2; exit 1; fi
  " || { echo "[deploy] restart did not settle on one process" >&2; exit 1; }
  echo "[deploy] restart done. Gaze check: curl -s localhost:5000/eyes_status"

  # ── the face on the robot's own screen ────────────────────────────────────
  # The launcher owns the @reboot crontab line, so shipping these two files and
  # asking it to install is what makes a replaced robot get its face back.  It
  # is idempotent, and it only reloads a face that is already running, so a
  # screen someone stopped on purpose stays stopped.
  # Deliberately names only the launcher.  The launcher stops the face with
  # `pkill -f` on its module's name, so a shell whose own command line spells
  # that name kills itself: measured, this block used to test for the module with
  # `[ -f ... ]`, and the reload then killed the ssh shell mid-block - the deploy
  # printed both "reloaded" and "could not reach the robot", and its exit status
  # was 255.  The launcher checks for its own module instead.
  ssh "${SSH_OPTS[@]}" "$ROBOT_HOST" "
    cd '$REMOTE_DIR' || exit 1
    if [ -f start_face.sh ]; then
      /bin/bash start_face.sh --install-boot || echo '[face] could not install the boot entry' >&2
      /bin/bash start_face.sh --reload || echo '[face] could not reload the face' >&2
    else
      echo '[face] start_face.sh not shipped: the screen face will not start at boot' >&2
    fi
  " || echo "[deploy] the face step could not reach the robot - its boot entry is unchanged" >&2
else
  echo "[deploy] --no-restart: the running app still has the old code"
  echo "[deploy] --no-restart: the face's boot entry was left alone too (run start_face.sh --install-boot on the robot to repair it)"
fi
