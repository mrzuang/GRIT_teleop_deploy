#!/usr/bin/env bash
set -euo pipefail

SERVICE_DIR="${XROBO_SERVICE_DIR:-/opt/apps/roboticsservice}"
VIDEO_PORT="${XROBO_VIDEO_PORT:-12345}"
PICO_PACKAGE="com.xrobotoolkit.client"
PICO_ACTIVITY="$PICO_PACKAGE/com.unity3d.player.UnityPlayerActivity"

adb_status=$(adb devices -l 2>&1 || true)
if grep -q "no permissions" <<<"$adb_status"; then
    rule='SUBSYSTEM=="usb", ATTR{idVendor}=="2d40", MODE="0660", GROUP="plugdev", TAG+="uaccess"'
    echo "Installing PICO USB permissions (sudo required)..."
    printf '%s\n' "$rule" | sudo tee /etc/udev/rules.d/51-pico.rules >/dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    adb kill-server >/dev/null 2>&1 || true
    sleep 1
fi

echo "Waiting for PICO over USB..."
timeout 15 adb wait-for-device || {
    adb devices -l
    echo "PICO ADB is unavailable; reconnect USB or accept USB debugging in the headset."
    exit 1
}

if ! pgrep -f '(^|/)RoboticsServiceProcess$|\./RoboticsServiceProcess$' >/dev/null; then
    echo "Starting XRoboToolkit PC Service..."
    mkdir -p "${XDG_CACHE_HOME:-$HOME/.cache}/xrobotoolkit"
    bash "$SERVICE_DIR/runService.sh" \
        >"${XDG_CACHE_HOME:-$HOME/.cache}/xrobotoolkit/pc-service.log" 2>&1 &
    sleep 2
fi

service_port=$(ss -H -ltnp 2>/dev/null | awk '/RoboticsService/ && $4 ~ /^\*:/ { print $4; exit }' | sed 's/.*://')
test -n "$service_port" || {
    echo "Could not detect PC Service port. Trying default 60061..."
    service_port=60061
}

echo "PC Service port: $service_port"

# Headset -> workstation tracking data.
adb reverse "tcp:$service_port" "tcp:$service_port" >/dev/null

# Workstation -> headset Remote Vision video.
adb forward --remove "tcp:$VIDEO_PORT" >/dev/null 2>&1 || true
adb forward "tcp:$VIDEO_PORT" "tcp:$VIDEO_PORT" >/dev/null

echo "Restarting XRoboToolkit on PICO..."
adb shell am force-stop "$PICO_PACKAGE"
adb shell input keyevent KEYCODE_HOME
adb shell am start -n "$PICO_ACTIVITY" >/dev/null

echo
echo "XRoboToolkit USB tunnels ready:"
echo "  tracking: PICO 127.0.0.1:$service_port -> USB -> PC :$service_port"
echo "  video:    PC 127.0.0.1:$VIDEO_PORT -> USB -> PICO :$VIDEO_PORT"
echo "  app:      XRoboToolkit restarted"
echo
echo "Reverse mappings:"
adb reverse --list
echo "Forward mappings:"
adb forward --list
echo
echo "Set PC Service IP in XRoboToolkit to 127.0.0.1"
echo "Run Remote Vision with: --pico-host 127.0.0.1"
