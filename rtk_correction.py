# coding=utf-8

"""Pure RTK correction math shared by the runtime and tests."""


def compute_heading_error(target_heading, current_heading):
    """Return the shortest signed heading error in degrees."""
    raw_error = float(target_heading) - float(current_heading)
    return (raw_error + 180.0) % 360.0 - 180.0


def compute_cte_dot(cte, last_cte, dt):
    """Return cross-track error velocity in meters per second."""
    try:
        dt = float(dt)
    except (TypeError, ValueError):
        return 0.0
    if dt <= 0.0:
        return 0.0
    return (float(cte) - float(last_cte)) / dt


def low_pass_filter(previous_value, new_value, alpha=0.8):
    """Smooth a noisy sample; alpha is the retained previous-value weight."""
    alpha = float(alpha)
    if alpha < 0.0:
        alpha = 0.0
    elif alpha > 1.0:
        alpha = 1.0
    return alpha * float(previous_value) + (1.0 - alpha) * float(new_value)


def compute_linear_steering(
    heading_error,
    cte,
    heading_gain=10.0,
    cte_gain=1000.0,
    cte_dot=0.0,
    cte_d_gain=0.0,
):
    """Linear RTK correction with reversed heading and cross-track directions."""
    return (
        -float(heading_error) * heading_gain
        + int(float(cte_gain) * float(cte))
        + float(cte_d_gain) * float(cte_dot)
    )
