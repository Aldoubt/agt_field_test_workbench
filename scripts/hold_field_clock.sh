#!/usr/bin/env bash
# Hold the already synchronized host clock during one field session. Never set
# the time or rewrite sensor stamps. Restore only the service stopped here.
set -euo pipefail
if [[ $# -eq 0 ]]; then
  echo 'Usage: hold_field_clock.sh <field launch command...>' >&2
  exit 2
fi
if [[ "$(timedatectl show -p NTPSynchronized --value)" != yes ]]; then
  echo 'Host clock is not synchronized; let NTP synchronize before holding it.' >&2
  exit 2
fi
if systemctl is-active --quiet chrony.service chronyd.service ntp.service ntpd.service ptp4l.service phc2sys.service; then
  echo 'Another clock synchronization service is active; this helper only manages systemd-timesyncd.' >&2
  exit 2
fi
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/agt-field-clock-${UID}.lock"
flock -n 9 || { echo 'Another field session owns the host-clock hold.' >&2; exit 2; }
restore=false
cleanup() {
  if "$restore"; then
    echo 'Restoring systemd-timesyncd after the field session.' >&2
    sudo systemctl start systemd-timesyncd.service || {
      echo 'Restore failed; run: sudo systemctl start systemd-timesyncd.service' >&2
      return 1
    }
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if systemctl is-active --quiet systemd-timesyncd.service; then
  # This prompt, if needed, belongs to the operator's explicit --hold-system-clock.
  sudo -v
  restore=true
  sudo -n systemctl stop systemd-timesyncd.service
fi
echo 'Host clock held: NTP stepping paused for this session; sensor timestamps unchanged.' >&2
"$@"
