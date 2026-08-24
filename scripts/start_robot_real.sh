#!/usr/bin/bash

gnome-terminal --tab -- bash -c "pixi run robot-real; exec bash"

gnome-terminal --tab -- bash -c "pixi run chatbot; exec bash"

gnome-terminal --tab -- bash -c "pixi run face-tracker; exec bash"
