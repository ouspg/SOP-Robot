#!/usr/bin/env bash
# Launch all subsystems in separate tabs within a single GNOME Terminal window


gnome-terminal  --tab -- bash -c "pixi run robot-arm; exec bash"
sleep 5
gnome-terminal   --tab  -- bash -c "pixi run chatbot; exec bash"
gnome-terminal   --tab -- bash -c "pixi run face-tracker; exec bash"
gnome-terminal   --tab -- bash -c "echo 'Waiting 5s for controllers to start...'; sleep 5; pixi run demo; exec bash"
