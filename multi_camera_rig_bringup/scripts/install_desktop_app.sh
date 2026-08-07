#!/bin/bash
# Installs the "Multi-Camera Demo App" as a clickable icon (GNOME app menu +
# Desktop) that runs `ros2 launch multi_camera_rig_bringup app.launch.py`.
# Safe to re-run any time (e.g. after moving the repo) to refresh the install.
set -e

BRINGUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCH_SCRIPT="$BRINGUP_DIR/scripts/launch_camera_rig_app.sh"
ICON_SRC="$BRINGUP_DIR/desktop/multi-camera-rig-icon.svg"

ICON_DEST_DIR="$HOME/.local/share/icons/hicolor/scalable/apps"
ICON_DEST="$ICON_DEST_DIR/multi-camera-rig.svg"
APPS_DIR="$HOME/.local/share/applications"
DESKTOP_FILE_NAME="multi-camera-rig.desktop"

mkdir -p "$ICON_DEST_DIR" "$APPS_DIR"
cp "$ICON_SRC" "$ICON_DEST"

DESKTOP_ENTRY_CONTENT="[Desktop Entry]
Type=Application
Name=Multi-Camera Demo App
Comment=Launch the camera rig, GUI, and RViz together; closing GUI or RViz stops everything
Exec=x-terminal-emulator -e $LAUNCH_SCRIPT
Icon=$ICON_DEST
Terminal=false
Categories=Development;
"

echo "$DESKTOP_ENTRY_CONTENT" > "$APPS_DIR/$DESKTOP_FILE_NAME"
chmod +x "$APPS_DIR/$DESKTOP_FILE_NAME"

if [ -d "$HOME/Desktop" ]; then
    echo "$DESKTOP_ENTRY_CONTENT" > "$HOME/Desktop/$DESKTOP_FILE_NAME"
    chmod +x "$HOME/Desktop/$DESKTOP_FILE_NAME"
    # GNOME requires marking a Desktop launcher as trusted before it's clickable.
    gio set "$HOME/Desktop/$DESKTOP_FILE_NAME" "metadata::trusted" true 2>/dev/null || true
fi

command -v update-desktop-database >/dev/null && update-desktop-database "$APPS_DIR" 2>/dev/null || true
command -v gtk-update-icon-cache >/dev/null && gtk-update-icon-cache -f "$HOME/.local/share/icons/hicolor" 2>/dev/null || true

echo "Installed: $APPS_DIR/$DESKTOP_FILE_NAME"
[ -d "$HOME/Desktop" ] && echo "Installed: $HOME/Desktop/$DESKTOP_FILE_NAME"
echo "If GNOME still shows it as untrusted or the icon/name looks stale, right-click the Desktop icon and choose 'Allow Launching', or log out and back in to refresh the app menu."
