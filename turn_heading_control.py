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
