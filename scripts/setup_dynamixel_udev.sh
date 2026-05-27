#!/usr/bin/env bash
set -euo pipefail

RULE_URL="https://raw.githubusercontent.com/ROBOTIS-GIT/dynamixel-workbench/master/99-dynamixel-workbench-cdc.rules"
RULE_DEST="/etc/udev/rules.d/99-dynamixel-workbench-cdc.rules"

curl -fsSL "$RULE_URL" | sudo tee "$RULE_DEST" >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger
