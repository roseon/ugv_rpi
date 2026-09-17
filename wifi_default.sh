#!/bin/bash
# wifi_default.sh — make the robot come back to its home network on its own.
#
# What this owns:
#   bash wifi_default.sh --status      what is pinned, and what the watchdog saw
#   bash wifi_default.sh --apply       pin the default SSID's profile: autoconnect
#                                      on, top priority, Wi-Fi power-save off
#   bash wifi_default.sh --install     also install+enable the reconnect watchdog
#                                      service (needs root; it is the part that
#                                      acts when nothing is connected)
#   bash wifi_default.sh --selftest    check the watchdog's decision logic offline
#
# The failure this answers: the robot is carried out of range (or the access
# point reboots), and NetworkManager does not always rejoin on its own —
# autoconnect can be off on the profile (one `nmcli con down` leaves it that
# way), and Wi-Fi power saving can drop an idle link that never comes back
# without a nudge.  The robot then sits unreachable until a human walks over.
# Two layers fix that without a second network ever being involved:
#   1. the profile itself: autoconnect yes, highest priority, powersave off;
#   2. wifi_default.py under systemd: whenever NO connection is active, bring
#      the default profile up; if the link has been dead for 15 minutes,
#      reboot — a Pi that reboots into a working autoconnect is reachable
#      again, a Pi hung in the Wi-Fi stack is not.
#
# The SSID is deliberately not hardcoded here: the DEFAULT connection —
# whichever profile the robot's radio is bound to, activated or not — is what
# gets pinned, so a moved robot re-homes by changing its network once.

cd "$(dirname "$0")" || exit 1
HERE="$(pwd)"
UNIT_DIR="/etc/systemd/system"
UNIT="wifi-default-watch.service"
UNIT_PATH="$UNIT_DIR/$UNIT"
UNIT_MARK="# Managed by wifi_default.sh --install - edits by hand are overwritten"
PYTHON="${PYTHON:-python3}"

# ── the connection profile bound to the robot's home SSID ────────────────────
# Matched by NAME (case-insensitive) among wifi profiles: NetworkManager names
# a newly joined network after its SSID, and duplicates come out as "SSID 1".
default_profile() {
  local ssid line name type
  ssid="$("$PYTHON" - <<'PY'
import subprocess
try:
    out = subprocess.run(["nmcli", "-t", "-f", "ACTIVE,SSID", "device", "wifi", "list"],
                         capture_output=True, text=True, timeout=30).stdout
except Exception:
    ssid = ""
else:
    for line in out.splitlines():
        f = line.split(":")
        if len(f) >= 2 and f[0] == "yes" and f[1]:
            ssid = f[1].strip()
            break
print(ssid)
PY
)"
  if [ -n "$ssid" ]; then
    nmcli -g NAME,TYPE connection show 2>/dev/null |
    while IFS= read -r line; do
      name="${line%%:*}"
      type="${line#*:}"; type="${type%%:*}"
      [ "$type" = "802-11-wireless" ] || continue
      if [ "${name,,}" = "${ssid,,}" ]; then printf '%s\n' "$name"; break; fi
    done
    return 0
  fi
  # No scan results (radio off, or this is running over ssh while the radio
  # is busy): fall back to the profile of the last wifi connection known.
  nmcli -g NAME,TYPE,TIMESTAMP connection show 2>/dev/null |
  while IFS= read -r line; do
    type="${line#*:}"; type="${type%%:*}"
    [ "$type" = "802-11-wireless" ] && printf '%s\n' "${line%%:*}"
  done | tail -1
}

show_settings() {
  nmcli -f connection.autoconnect,connection.autoconnect-priority,wifi.powersave \
        connection show "$1" 2>/dev/null | sed 's/^/  /'
}

case "${1:---status}" in
  --apply|apply)
    PROF="$(default_profile)"
    if [ -z "$PROF" ]; then
      echo "wifi: no wifi profile to pin (has the robot ever joined its home network?)" >&2
      exit 1
    fi
    if ! nmcli connection modify "$PROF" \
           connection.autoconnect yes \
           connection.autoconnect-priority 100 \
           802-11-wireless.powersave 2; then
      echo "wifi: could not modify $PROF" >&2
      exit 1
    fi
    echo "wifi: pinned '$PROF' (autoconnect yes, priority 100, powersave off)"
    # If the robot is sitting disconnected right now, bring it home at once.
    if [ -z "$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null |
               awk -F: '$2 != "lo" && $2 != ""')" ]; then
      echo "wifi: nothing connected - activating '$PROF' now"
      nmcli connection up "$PROF" || echo "wifi: activation failed (the watchdog will keep trying)" >&2
    fi
    show_settings "$PROF"
    ;;

  --install|install)
    PROF="$(default_profile)"
    [ -n "$PROF" ] || { echo "wifi: no wifi profile to pin" >&2; exit 1; }
    # 1. the settings (no root needed)
    bash "$HERE/$(basename "$0")" --apply || exit 1
    # 2. the watchdog (root: it must act before anyone logs in)
    if [ ! -f "$HERE/wifi_default.py" ]; then
      echo "wifi: wifi_default.py is not next to this script - not installing the watchdog" >&2
      exit 1
    fi
    if [ -e "$UNIT_PATH" ] && ! grep -q "Managed by wifi_default.sh" "$UNIT_PATH" 2>/dev/null; then
      echo "wifi: $UNIT_PATH exists and is not ours - leaving it alone" >&2
      exit 1
    fi
    if ! cat > "$UNIT_PATH" <<UNIT
$UNIT_MARK
[Unit]
Description=Return the robot to its default Wi-Fi when it drops (wifi_default.sh)
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=simple
ExecStart=$PYTHON $HERE/wifi_default.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
UNIT
    then
      echo "wifi: could not write $UNIT_PATH (run with sudo)" >&2
      exit 1
    fi
    systemctl daemon-reload || { echo "wifi: daemon-reload failed" >&2; exit 1; }
    systemctl enable --now "$UNIT" || { echo "wifi: could not enable $UNIT" >&2; exit 1; }
    echo "wifi: watchdog installed and running ($UNIT)"
    ;;

  --selftest|selftest)
    exec "$PYTHON" "$HERE/wifi_default.py" --selftest
    ;;

  --status|status|*)
    PROF="$(default_profile)"
    echo "default profile: ${PROF:-none found}"
    [ -n "$PROF" ] && show_settings "$PROF"
    echo "active connections:"
    nmcli -t -f NAME,DEVICE,TYPE connection show --active 2>/dev/null | sed 's/^/  /'
    if systemctl is-active "$UNIT" >/dev/null 2>&1; then
      echo "watchdog: active ($(systemctl show -p MainPID --value "$UNIT"))"
      journalctl -u "$UNIT" -n 5 --no-pager 2>/dev/null | sed 's/^/  /'
    elif [ -e "$UNIT_PATH" ]; then
      echo "watchdog: installed but not active"
    else
      echo "watchdog: not installed (bash wifi_default.sh --install, needs sudo once)"
    fi
    ;;
esac
