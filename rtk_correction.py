# coding=utf-8

"""Pure RTK correction math shared by the runtime and tests."""


def compute_heading_error(target_heading, current_heading):
    """Return the shortest signed heading error in degrees."""
    raw_error = float(target_heading) - float(current_heading)
    return (raw_error + 180.0) % 360.0 - 180.0


def compute_linear_steering(heading_error, cte, heading_gain=10.0, cte_gain=1000.0):
    """Legacy linear RTK correction: heading term minus cross-track term."""
    return float(heading_error) * heading_gain - int(float(cte_gain) * float(cte))
