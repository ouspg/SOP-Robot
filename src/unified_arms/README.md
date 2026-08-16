# Unified arms

This ROS 2 package coordinates arm actions and hand gestures. It uses
`/dev/ttyACM0` for the physical arm controller and falls back to fake-hardware
behavior when that serial port is unavailable.

Run it directly with:

```console
pixi run unified-arms
```

The complete real and fake robot demos launch this node automatically.
