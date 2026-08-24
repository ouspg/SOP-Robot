#!/usr/bin/env bash
# Launch all subsystems in separate tabs within a single GNOME Terminal window

gnome-terminal \
  --tab --title="Robot Hardware" -- bash -c "pixi run robot; exec bash" \
  --tab --title="Chatbot" -- bash -c "pixi run chatbot; exec bash" \
  --tab --title="Face Tracker" -- bash -c "pixi run face-tracker; exec bash" \
  --tab --title="Demo" -- bash -c "echo 'Waiting 5s for controllers to start...'; sleep 5; pixi run demo; exec bash"
