#!/usr/bin/env bash
set -euo pipefail

PORT="$DROIDCAM_PORT"
ADB_BIN="$ADB"
SERIAL="$ANDROID_SERIAL"
URL="http://127.0.0.1:${PORT}/video"

if ! command -v "$ADB_BIN" >/dev/null 2>&1; then
    cat >&2 <<'EOF'
ERROR: adb was not found.

Install Android platform tools in WSL, then rerun this command:
  sudo apt update
  sudo apt install android-tools-adb

If the phone is connected to Windows, attach it to WSL first from Windows
PowerShell:
  usbipd list
  usbipd bind --busid <BUSID>
  usbipd attach --wsl --busid <BUSID>
EOF
    exit 1
fi

"$ADB_BIN" start-server >/dev/null

if [[ -z "$SERIAL" ]]; then
    mapfile -t DEVICES < <("$ADB_BIN" devices | awk 'NR > 1 && $2 == "device" {print $1}')
    if [[ "${#DEVICES[@]}" -eq 0 ]]; then
        cat >&2 <<'EOF'
ERROR: no authorized Android device was found by adb.

Check these items:
  1. Enable Developer options and USB debugging on the phone.
  2. Accept the phone's USB debugging authorization prompt.
  3. If using WSL, attach the phone with usbipd from Windows PowerShell.
  4. Start the DroidCam app on the phone.
EOF
        "$ADB_BIN" devices >&2 || true
        exit 1
    fi
    SERIAL="${DEVICES[0]}"
    if [[ "${#DEVICES[@]}" -gt 1 ]]; then
        echo "Multiple Android devices found; using ${SERIAL}. Set ANDROID_SERIAL to choose." >&2
    fi
fi

"$ADB_BIN" -s "$SERIAL" wait-for-device
"$ADB_BIN" -s "$SERIAL" forward --remove "tcp:${PORT}" >/dev/null 2>&1 || true
"$ADB_BIN" -s "$SERIAL" forward "tcp:${PORT}" "tcp:${PORT}" >/dev/null

echo "DroidCam USB forward is active for device ${SERIAL}."
echo "Use camera_source: ${URL}"

if ! timeout 5 python3 - "$URL" <<'PY'
import sys
import urllib.request

url = sys.argv[1]
try:
    request = urllib.request.Request(url, headers={"User-Agent": "SOP-Robot"})
    with urllib.request.urlopen(request, timeout=3) as response:
        response.read(64)
        print(f"Verified DroidCam video endpoint: {url}")
except Exception as exc:
    print(
        f"WARNING: adb forwarding is active, but {url} did not return video yet: {exc}",
        file=sys.stderr,
    )
    print("Start DroidCam on the phone and select USB mode, then rerun this check.", file=sys.stderr)
PY
then
    echo "WARNING: endpoint verification timed out, but adb forwarding is active." >&2
fi
