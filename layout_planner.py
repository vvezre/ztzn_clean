# coding=utf-8
import math


def _as_int(value, default=0):
    if value is None or value == "":
        return default
    return int(float(value))


def _range_bounds(region, axis):
    single = region.get(axis)
    start = region.get(axis + "Start", single)
    end = region.get(axis + "End", single)
    start = _as_int(start)
    end = _as_int(end, start)
    if start > end:
        start, end = end, start
    return start, end


def _iter_region_cells(region):
    row_start, row_end = _range_bounds(region, "row")
    col_start, col_end = _range_bounds(region, "col")
    for row in range(row_start, row_end + 1):
        for col in range(col_start, col_end + 1):
            yield row, col


def expand_panel_cells(layout):
    cells = set()
    layout = layout or {}

    for region in layout.get("areas", []) or []:
        cells.update(_iter_region_cells(region))
    for region in layout.get("extras", []) or []:
        cells.update(_iter_region_cells(region))
    for region in layout.get("holes", []) or []:
        cells.difference_update(_iter_region_cells(region))

    return cells


def build_panel_segments(cells):
    rows = {}
    for row, col in cells:
        rows.setdefault(row, []).append(col)

    result = {}
    for row in sorted(rows):
        cols = sorted(rows[row])
        if not cols:
            result[row] = []
            continue

        segments = []
        start = cols[0]
        prev = cols[0]
        for col in cols[1:]:
            if col == prev + 1:
                prev = col
                continue
            segments.append((start, prev))
            start = col
            prev = col
        segments.append((start, prev))
        result[row] = segments

    return result


def _connector_length(connector):
    return _as_int(connector.get("length"), 0)


def _connector_row_match(connector, row):
    row_start, row_end = _range_bounds(connector, "row")
    return row_start <= row <= row_end


def _connector_col_match(connector, col):
    col_start, col_end = _range_bounds(connector, "col")
    return col_start <= col <= col_end


def panel_point_xy(row, col, step_x, step_y, connectors=None):
    connectors = connectors or []
    x = col * step_x
    y = row * step_y

    for connector in connectors:
        connector_type = connector.get("type")
        if connector_type == "col" and _connector_row_match(connector, row):
            after_col = _as_int(connector.get("afterCol"))
            if col > after_col:
                x += _connector_length(connector)
        elif connector_type == "row" and _connector_col_match(connector, col):
            after_row = _as_int(connector.get("afterRow"))
            if row > after_row:
                y += _connector_length(connector)

    return int(round(x)), int(round(y))


def _axis_move(current, target, mode, turn_back_len, back_len):
    start_x, start_y = current
    end_x, end_y = target
    dx = end_x - start_x
    dy = end_y - start_y

    if dx == 0 and dy == 0:
        return None
    if dx != 0 and dy != 0:
        raise ValueError("axis move requires either x or y to stay unchanged")

    if dx > 0:
        angle = 90
        length = dx
    elif dx < 0:
        angle = 270
        length = -dx
    elif dy > 0:
        angle = 0
        length = dy
    else:
        angle = 180
        length = -dy

    return {
        "angle": angle,
        "mode": mode,
        "length": int(round(length)),
        "turn_back_len": turn_back_len,
        "back_len": back_len,
        "startX": int(round(start_x)),
        "startY": int(round(start_y)),
        "endX": int(round(end_x)),
        "endY": int(round(end_y)),
    }


def _append_move(tasks, current, target, mode, turn_back_len, back_len):
    move = _axis_move(current, target, mode, turn_back_len, back_len)
    if move is not None and move["length"] > 0:
        tasks.append(move)
        return target
    return current


def _append_transition(tasks, current, target, turn_back_len):
    current_x, current_y = current
    target_x, target_y = target

    if current_y != target_y:
        current = _append_move(
            tasks,
            current,
            (current_x, target_y),
            mode=2,
            turn_back_len=turn_back_len,
            back_len=0,
        )
    if current[0] != target_x:
        current = _append_move(
            tasks,
            current,
            (target_x, target_y),
            mode=2,
            turn_back_len=turn_back_len,
            back_len=0,
        )

    return current


def create_task_by_panel_layout(
    layout,
    areaNumber=1,
    startX=0,
    startY=0,
    direction="left",
    isLastArea=True,
    lineCount=0,
    goBackLen=5,
    goLeftOrRightBackLen=15,
    turnBackLen=10,
    panelWidth=113,
    panelHeight=226,
    leftOrRightBridgeLen=150,
    gap=3,
    angle_radians=0,
    angle_to="y",
    gapX=None,
    gapY=None,
    angle_radians_x=None,
    angle_radians_y=None,
    returnToOrigin=False,
):
    if gapX is None:
        gapX = gap
    if gapY is None:
        gapY = gap

    if angle_radians_x is None:
        angle_radians_x = angle_radians if angle_to != "y" else 0
    if angle_radians_y is None:
        angle_radians_y = angle_radians if angle_to == "y" else 0

    x_projection = math.cos(angle_radians_x or 0)
    y_projection = math.cos(angle_radians_y or 0)
    step_x = int(round((_as_int(panelWidth) + _as_int(gapX)) * x_projection))
    step_y = int(round((_as_int(panelHeight) + _as_int(gapY)) * y_projection))
    connectors = (layout or {}).get("connectors", []) or []
    cells = expand_panel_cells(layout)
    segments_by_row = build_panel_segments(cells)
    rows = sorted(segments_by_row.keys(), reverse=True)

    tasks = []
    current = (int(round(startX)), int(round(startY)))
    left_to_right = direction != "right"

    for row in rows:
        row_segments = segments_by_row[row]
        if not row_segments:
            continue

        if left_to_right:
            ordered_segments = row_segments
        else:
            ordered_segments = list(reversed(row_segments))

        for col_start, col_end in ordered_segments:
            if left_to_right:
                clean_start_col = col_start
                clean_end_col = col_end
            else:
                clean_start_col = col_end
                clean_end_col = col_start

            clean_start = panel_point_xy(row, clean_start_col, step_x, step_y, connectors)
            clean_end = panel_point_xy(row, clean_end_col, step_x, step_y, connectors)

            current = _append_transition(tasks, current, clean_start, turnBackLen)
            current = _append_move(
                tasks,
                current,
                clean_end,
                mode=1,
                turn_back_len=turnBackLen,
                back_len=goLeftOrRightBackLen,
            )

        left_to_right = not left_to_right

    if returnToOrigin:
        current = _append_transition(tasks, current, (0, 0), turnBackLen)

    for index, task in enumerate(tasks):
        task["id"] = index + 1
        task["areaNumber"] = areaNumber

    return tasks
