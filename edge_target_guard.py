DEFAULT_EDGE_TARGET_TOLERANCE_M = 0.10


class EdgeTriggerLatch(object):
    """Emit one trigger for each asserted edge-sensor cycle."""

    def __init__(self):
        self._armed = True

    def consume(self, edge_state):
        asserted = str(edge_state) == "0"
        if not asserted:
            self._armed = True
            return False
        if not self._armed:
            return False
        self._armed = False
        return True


def should_accept_edge_stop(distance_to_target_m, tolerance_m=DEFAULT_EDGE_TARGET_TOLERANCE_M):
    """Return True only when an edge alarm happens close to the segment target."""
    if distance_to_target_m is None:
        return False
    try:
        return float(distance_to_target_m) <= float(tolerance_m)
    except (TypeError, ValueError):
        return False


def should_recover_from_edge_stop(distance_to_target_m, tolerance_m=DEFAULT_EDGE_TARGET_TOLERANCE_M):
    """Return True when an edge alarm is too far from the segment target."""
    if distance_to_target_m is None:
        return False
    try:
        return float(distance_to_target_m) > float(tolerance_m)
    except (TypeError, ValueError):
        return False
