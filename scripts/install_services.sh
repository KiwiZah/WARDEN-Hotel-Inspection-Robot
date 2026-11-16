#!/usr/bin/env bash
set -euo pipefail
# Run as the target (non-root) Pi user; sudo is needed only to install service files.
PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_NAME="$(id -un)"
HOME_DIR="$HOME"
mkdir -p "$HOME_DIR/.config/warden"
if [[ ! -f "$HOME_DIR/.config/warden/warden.env" ]]; then
  cp "$PROJECT/config/warden.env.example" "$HOME_DIR/.config/warden/warden.env"
  chmod 600 "$HOME_DIR/.config/warden/warden.env"
fi
for name in warden-ros warden-joy warden-host warden-teleop; do
  sed -e "s|@USER@|$USER_NAME|g" -e "s|@HOME@|$HOME_DIR|g" \
      -e "s|@PROJECT@|$PROJECT|g" "$PROJECT/deploy/systemd/$name.service.in" | \
      sudo tee "/etc/systemd/system/$name.service" >/dev/null
done
# Disable duplicate historical stacks to prevent competing motor publishers.
sudo systemctl disable --now turtlebot.service warden-demo.service 2>/dev/null || true
sudo systemctl daemon-reload
sudo systemctl enable warden-ros.service warden-joy.service warden-host.service warden-teleop.service
printf 'Installed (not started). Check environment/ports and bench test before starting.\n'
