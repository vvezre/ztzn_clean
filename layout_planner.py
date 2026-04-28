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


def _build_cells_by_row(cells):
    rows = {}
    for row, col in cells:
        rows.setdefault(row, []).append(col)
    for row in rows:
        rows[row] = sorted(set(rows[row]))
    return rows


def _build_cells_by_col(cells):
    cols = {}
    for row, col in cells:
        cols.setdefault(col, []).append(row)
    for col in cols:
        cols[col] = sorted(set(cols[col]))
    return cols


def _connector_length(connector):
    return _as_int(connector.get("length"), 0)


def _connector_row_match(connector, row):
    row_start, row_end = _range_bounds(connector, "row")
    return row_start <= row <= row_end


def _connector_col_match(connector, col):
    col_start, col_end = _range_bounds(connector, "col")
    return col_start <= col <= col_end


def _connector_edges(cells, connectors):
    cells = set(cells)
    rows = _build_cells_by_row(cells)
    cols = _build_cells_by_col(cells)
    edges = []

    for connector in connectors or []:
        connector_type = connector.get("type")
        if connector_type == "col":
            row_start, row_end = _range_bounds(connector, "row")
            after_col = _as_int(connector.get("afterCol"))
            for row in range(row_start, row_end + 1):
                row_cols = rows.get(row, [])
                left_cols = [col for col in row_cols if col <= after_col]
                right_cols = [col for col in row_cols if col > after_col]
                if left_cols and right_cols:
                    edges.append(((row, max(left_cols)), (row, min(right_cols))))
        elif connector_type == "row":
            col_start, col_end = _range_bounds(connector, "col")
            after_row = _as_int(connector.get("afterRow"))
            for col in range(col_start, col_end + 1):
                col_rows = cols.get(col, [])
                top_rows = [row for row in col_rows if row <= after_row]
                bottom_rows = [row for row in col_rows if row > after_row]
                if top_rows and bottom_rows:
                    edges.append(((max(top_rows), col), (min(bottom_rows), col)))

    return edges


def connected_panel_components(cells, connectors=None):
    cells = set(cells)
    adjacency = {}
    for cell in cells:
        adjacency[cell] = set()

    for row, col in cells:
        for neighbor in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
            if neighbor in cells:
                adjacency[(row, col)].add(neighbor)
                adjacency[neighbor].add((row, col))

    for left, right in _connector_edges(cells, connectors or []):
        if left in cells and right in cells:
            adjacency[left].add(right)
            adjacency[right].add(left)

    components = []
    seen = set()
    for cell in sorted(cells):
        if cell in seen:
            continue
        stack = [cell]
        component = set()
        seen.add(cell)
        while stack:
            current = stack.pop()
            component.add(current)
            for neighbor in adjacency.get(current, []):
                if neighbor not in seen:
                    seen.add(neighbor)
                    stack.append(neighbor)
        components.append(component)

    return sorted(components, key=lambda component: (-max(row for row, col in component),
                                                     min(col for row, col in component)))


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


def _task_total_length(tasks):
    return sum(_as_int(task.get("length")) for task in tasks)


def _plan_component_tasks(component_cells, current, left_to_right, step_x, step_y, connectors,
                          turn_back_len, go_left_or_right_back_len):
    segments_by_row = build_panel_segments(component_cells)
    rows = sorted(segments_by_row.keys(), reverse=True)
    tasks = []

    for row in rows:
        row_segments = segments_by_row[row]
        if not row_segments:
            continue

        ordered_segments = row_segments if left_to_right else list(reversed(row_segments))

        for col_start, col_end in ordered_segments:
            if left_to_right:
                clean_start_col = col_start
                clean_end_col = col_end
            else:
                clean_start_col = col_end
                clean_end_col = col_start

            clean_start = panel_point_xy(row, clean_start_col, step_x, step_y, connectors)
            clean_end = panel_point_xy(row, clean_end_col, step_x, step_y, connectors)

            current = _append_transition(tasks, current, clean_start, turn_back_len)
            current = _append_move(
                tasks,
                current,
                clean_end,
                mode=1,
                turn_back_len=turn_back_len,
                back_len=go_left_or_right_back_len,
            )

        left_to_right = not left_to_right

    return tasks, current


def _append_return_to_origin(tasks, current, turn_back_len):
    before_count = len(tasks)
    current = _append_transition(tasks, current, (0, 0), turn_back_len)
    for task in tasks[before_count:]:
        task["action"] = "return_origin"
    return current


def create_return_to_origin_tasks(current, turnBackLen=10, areaNumber=None, start_id=1):
    tasks = []
    _append_return_to_origin(tasks, current, turnBackLen)
    for index, task in enumerate(tasks):
        task["id"] = start_id + index
        if areaNumber is not None:
            task["areaNumber"] = areaNumber
    return tasks


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
    layout = layout or {}
    connectors = layout.get("connectors", []) or []
    cells = expand_panel_cells(layout)
    components = connected_panel_components(cells, connectors)
    visit_strategy = layout.get("visitStrategy", "nearest")

    tasks = []
    current = (int(round(startX)), int(round(startY)))
    preferred_left_to_right = direction != "right"
    remaining_components = list(components)

    while remaining_components:
        if visit_strategy == "row":
            component = remaining_components.pop(0)
            component_tasks, current = _plan_component_tasks(
                component, current, preferred_left_to_right, step_x, step_y,
                connectors, turnBackLen, goLeftOrRightBackLen
            )
        else:
            best_index = None
            best_tasks = None
            best_current = None
            best_score = None
            for component_index, component in enumerate(remaining_components):
                for left_to_right in (preferred_left_to_right, not preferred_left_to_right):
                    candidate_tasks, candidate_current = _plan_component_tasks(
                        component, current, left_to_right, step_x, step_y,
                        connectors, turnBackLen, goLeftOrRightBackLen
                    )
                    orientation_penalty = 0 if left_to_right == preferred_left_to_right else 1
                    score = (_task_total_length(candidate_tasks), orientation_penalty, component_index)
                    if best_score is None or score < best_score:
                        best_score = score
                        best_index = component_index
                        best_tasks = candidate_tasks
                        best_current = candidate_current

            component_tasks = best_tasks or []
            current = best_current if best_current is not None else current
            del remaining_components[best_index]

        tasks.extend(component_tasks)

    if returnToOrigin:
        current = _append_return_to_origin(tasks, current, turnBackLen)

    for index, task in enumerate(tasks):
        task["id"] = index + 1
        task["areaNumber"] = areaNumber

    return tasks
