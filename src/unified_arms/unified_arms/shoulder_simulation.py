"""Convert the physical shoulder controller's commands to URDF joint angles."""

from math import isfinite, radians


def joint_positions(servo_commands, calibration):
    """Convert commanded servo IDs, preserving Arduino integer and clamp behavior."""
    positions = {}
    for joint, settings in calibration['joints'].items():
        if not settings.get('enabled', True):
            continue
        servo_id = settings.get('servo_id')
        if servo_id not in servo_commands:
            continue
        positions[joint] = calibrated_angle(servo_commands[servo_id], settings)
    return positions


def pose_positions(pose, calibration, hand='both'):
    """Convert full arm poses, optionally including wrists, to joint targets."""
    positions = {}
    for joint, angle, settings in pose_targets(pose, calibration, hand):
        positions[joint] = calibrated_angle(angle, settings)
    return positions


def servo_commands(pose, calibration, hand='both'):
    """Select physical channels and emit constrained integer angles in ID order."""
    commands = {}
    for _joint, angle, settings in pose_targets(pose, calibration, hand):
        if 'servo_id' in settings:
            commands[settings['servo_id']] = constrained_angle(angle, settings)
    return dict(sorted(commands.items()))


def pose_targets(pose, calibration, hand):
    """Select pose entries in firmware order, omitting unchanged joints."""
    if len(pose) not in (6, 8, 10):
        raise ValueError('Arm poses need 6 servo values, 8 with shoulder-out, or 10 with wrists')
    if hand not in ('right', 'left', 'both'):
        raise ValueError("Arm side must be 'right', 'left', or 'both'")
    for joint, settings in calibration['joints'].items():
        if not settings.get('enabled', True):
            continue
        index = settings['pose_index']
        if index >= len(pose) or (hand != 'both' and not joint.startswith(hand[0] + '_')):
            continue
        angle = pose[index]
        if angle != -1:
            yield joint, angle, settings


def constrained_angle(angle, settings):
    """Match the firmware's String.toInt() and per-servo constrain() calls."""
    if not isfinite(angle):
        raise ValueError('Servo angles must be finite')
    return min(settings['max_degrees'], max(settings['min_degrees'], int(angle)))


def calibrated_angle(angle, settings):
    """Apply integer servo travel, limits, neutral offset and linkage direction."""
    angle = constrained_angle(angle, settings)
    return radians(
        (angle - settings['neutral_degrees'])
        * settings['direction']
        * settings['joint_degrees_per_servo_degree']
    )
