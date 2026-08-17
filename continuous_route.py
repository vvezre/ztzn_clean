# coding=utf-8
"""Helpers for executing soft recorded boundary points as one continuous run.

The route planner still stores one task per recorded sub-segment so the
frontend can draw every real boundary point.  The executor, however, must not
stop and restart the lower machine at every soft point.  These helpers group
adjacent tasks that belong to the same continuous path and attach the
remaining sub-segments to the first task as runtime-only look-ahead data.
"""


CONTINUATION_KEY = "_continuousSegments"


def collect_continuous_run(segments, start_index):
    """Return the executable run beginning at ``start_index``.

    A following task is part of the same uninterrupted run only when:

    * both tasks have the same non-empty ``continuousPathId``;
    * the preceding task explicitly says not to stop at its end;
    * the following task explicitly says not to turn at its start; and
    * both tasks use the same movement mode.

    Missing flags keep their historical safe behaviour (stop and turn), so
    old task files are never silently merged.
    """
    if not isinstance(segments, (list, tuple)):
        return []
    if start_index < 0 or start_index >= len(segments):
        return []

    first = segments[start_index]
    if not isinstance(first, dict):
        return [first]

    run = [first]
    path_id = first.get("continuousPathId")
    mode = first.get("mode")
    if not path_id:
        return run

    cursor = start_index + 1
    while cursor < len(segments):
        previous = run[-1]
        candidate = segments[cursor]
        if not isinstance(candidate, dict):
            break
        if previous.get("stopAtEnd") is not False:
            break
        if candidate.get("turnAtStart") is not False:
            break
        if candidate.get("continuousPathId") != path_id:
            break
        if candidate.get("mode") != mode:
            break
        run.append(candidate)
        cursor += 1
    return run


def attach_continuations(run):
    """Copy a run into one runtime segment with look-ahead targets attached."""
    if not run:
        return None
    root = dict(run[0])
    root[CONTINUATION_KEY] = [dict(segment) for segment in run[1:]]
    # Cleanup after the combined run must follow the last sub-segment, not the
    # first one.  This keeps the vehicle moving through intermediate points
    # and still brakes at a real corner or at the end of the polyline.
    root["stopAtEnd"] = run[-1].get("stopAtEnd") is not False
    return root
