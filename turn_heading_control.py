TURN_LEFT_PROTOCOL_VALUE = 6660
TURN_RIGHT_PROTOCOL_VALUE = 5550


def choose_turn_direction(current_heading, target_heading):
    """Return the shortest turn direction and its positive angle in degrees."""
    current = float(current_heading) % 360.0
    target = float(target_heading) % 360.0
    clockwise = (target - current) % 360.0
    if clockwise == 0.0:
        return "none", 0.0
    if clockwise <= 180.0:
        return "right", clockwise
    return "left", 360.0 - clockwise


def plan_turn_timeout_recovery(current_heading, target_heading, rtk_fixed,
                               tolerance_degrees=2.0):
    """Plan what to do after one RTK turn attempt reaches its time limit.

    A timeout is only a checkpoint.  With no fixed/fresh RTK heading the
    vehicle must remain stopped and wait.  With healthy RTK, recalculate the
    shortest turn from the *current* heading instead of failing the route or
    blindly repeating the original turn.
    """
    if not rtk_fixed or current_heading is None:
        return {
            "action": "wait_rtk",
            "direction": "none",
            "relativeAngle": None,
        }

    direction, relative_angle = choose_turn_direction(
        current_heading,
        target_heading,
    )
    if direction == "none" or relative_angle <= float(tolerance_degrees):
        return {
            "action": "complete",
            "direction": direction,
            "relativeAngle": relative_angle,
        }
    return {
        "action": "retry",
        "direction": direction,
        "relativeAngle": relative_angle,
    }
