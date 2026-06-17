# coding=utf-8
import math


def _as_int(value, default=0):
    if value is None or value == "":
        return default
    return int(float(value))


def _round_like_js(value):
    return int(math.floor(value + 0.5))


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


def build_panel_segments(cells, segment_key=None):
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
            same_segment = col == prev + 1
            if same_segment and segment_key is not None:
                same_segment = segment_key(row, col) == segment_key(row, prev)
            if same_segment:
                prev = col
                continue
            segments.append((start, prev))
            start = col
            prev = col
        segments.append((start, prev))
        result[row] = segments

    return result


def _cell_sources_from_layout(layout):
    if not isinstance(layout, dict):
        return None

    sources = {}
    source_id = 0
    for group_name in ("areas", "extras"):
        for region in layout.get(group_name, []) or []:
            for cell in _iter_region_cells(region):
                sources.setdefault(cell, set()).add(source_id)
            source_id += 1

    for region in layout.get("holes", []) or []:
        for cell in _iter_region_cells(region):
            sources.pop(cell, None)

    return sources


def _same_layout_source(left, right, cell_sources):
    if cell_sources is None:
        return True
    return bool(cell_sources.get(left, set()) & cell_sources.get(right, set()))


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


def connected_panel_components(cells, connectors=None, layout=None):
    cells = set(cells)
    cell_sources = _cell_sources_from_layout(layout)
    adjacency = {}
    for cell in cells:
        adjacency[cell] = set()

    for row, col in cells:
        for neighbor in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
            if neighbor in cells and _same_layout_source((row, col), neighbor, cell_sources):
                adjacency[(row, col)].add(neighbor)
                adjacency[neighbor].add((row, col))

    if cell_sources is None:
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
    x_gaps = {}
    y_gaps = {}

    for connector in connectors:
        connector_type = connector.get("type")
        if connector_type == "col":
            after_col = _as_int(connector.get("afterCol"))
            if col > after_col:
                x_gaps[after_col] = max(x_gaps.get(after_col, 0), _connector_length(connector))
        elif connector_type == "row":
            after_row = _as_int(connector.get("afterRow"))
            if row > after_row:
                y_gaps[after_row] = max(y_gaps.get(after_row, 0), _connector_length(connector))

    x += sum(x_gaps.values())
    y += sum(y_gaps.values())

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


def _append_transition(tasks, current, target, turn_back_len, horizontal_first=False):
    current_x, current_y = current
    target_x, target_y = target

    if horizontal_first and current_x != target_x:
        current = _append_move(
            tasks,
            current,
            (target_x, current_y),
            mode=2,
            turn_back_len=turn_back_len,
            back_len=0,
        )
    if current[1] != target_y:
        current = _append_move(
            tasks,
            current,
            (current[0], target_y),
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


def _normalize_sweep_direction(value):
    if value in ("bottom_to_top", "bottomToTop", "up", "upward"):
        return "bottom_to_top"
    return "top_to_bottom"


def _sort_components_for_sweep(components, sweep_direction):
    if sweep_direction == "bottom_to_top":
        return sorted(components, key=lambda component: (min(row for row, col in component),
                                                         min(col for row, col in component)))
    return sorted(components, key=lambda component: (-max(row for row, col in component),
                                                    min(col for row, col in component)))


def _lane_ratios_for_sweep(sweep_direction):
    if sweep_direction == "bottom_to_top":
        return (0.25, 0.75)
    return (0.75, 0.25)


def _clean_line_point(row, col, step_x, step_y, projected_panel_width,
                      projected_panel_height, connectors, sweep_direction, lane_ratio=None):
    x, y = panel_point_xy(row, col, step_x, step_y, connectors)
    ratio = _lane_ratios_for_sweep(sweep_direction)[0] if lane_ratio is None else lane_ratio
    return _round_like_js(x + projected_panel_width * 0.5), _round_like_js(y + projected_panel_height * ratio)


def _row_segment_contains_x(row, row_segments, x, step_x, step_y, projected_panel_width,
                            projected_panel_height, connectors, sweep_direction):
    for col_start, col_end in row_segments:
        start_x, _ = _clean_line_point(
            row, col_start, step_x, step_y, projected_panel_width,
            projected_panel_height, connectors, sweep_direction
        )
        end_x, _ = _clean_line_point(
            row, col_end, step_x, step_y, projected_panel_width,
            projected_panel_height, connectors, sweep_direction
        )
        if min(start_x, end_x) <= x <= max(start_x, end_x):
            return True
    return False


def _row_segment_covers_x_range(row, row_segments, start_x, end_x,
                                step_x, step_y, projected_panel_width,
                                projected_panel_height, connectors, sweep_direction):
    min_target_x = min(start_x, end_x)
    max_target_x = max(start_x, end_x)
    for col_start, col_end in row_segments:
        segment_start_x, _ = _clean_line_point(
            row, col_start, step_x, step_y, projected_panel_width,
            projected_panel_height, connectors, sweep_direction
        )
        segment_end_x, _ = _clean_line_point(
            row, col_end, step_x, step_y, projected_panel_width,
            projected_panel_height, connectors, sweep_direction
        )
        if min(segment_start_x, segment_end_x) <= min_target_x and max(segment_start_x, segment_end_x) >= max_target_x:
            return True
    return False


def _row_clean_y(row, row_segments, step_x, step_y, projected_panel_width,
                 projected_panel_height, connectors, sweep_direction):
    if not row_segments:
        return None
    col_start, _ = row_segments[0]
    _, y = _clean_line_point(
        row, col_start, step_x, step_y, projected_panel_width,
        projected_panel_height, connectors, sweep_direction
    )
    return y


def _find_clean_line_row(point, segments_by_row, step_x, step_y, projected_panel_width,
                         projected_panel_height, connectors, sweep_direction):
    x, y = point
    for row, row_segments in segments_by_row.items():
        for col_start, col_end in row_segments:
            for lane_ratio in _lane_ratios_for_sweep(sweep_direction):
                start_x, start_y = _clean_line_point(
                    row, col_start, step_x, step_y, projected_panel_width,
                    projected_panel_height, connectors, sweep_direction, lane_ratio
                )
                end_x, end_y = _clean_line_point(
                    row, col_end, step_x, step_y, projected_panel_width,
                    projected_panel_height, connectors, sweep_direction, lane_ratio
                )
                if start_y == y and end_y == y and min(start_x, end_x) <= x <= max(start_x, end_x):
                    return row
    return None


def _find_covered_row(point, segments_by_row, step_x, step_y, projected_panel_height, connectors):
    x, y = point
    for row, row_segments in segments_by_row.items():
        for col_start, col_end in row_segments:
            start_x, start_base_y = panel_point_xy(row, col_start, step_x, step_y, connectors)
            end_x, end_base_y = panel_point_xy(row, col_end, step_x, step_y, connectors)
            if start_base_y != end_base_y:
                continue
            min_x = min(start_x, end_x)
            max_x = max(start_x, end_x)
            min_y = min(start_base_y, start_base_y + projected_panel_height)
            max_y = max(start_base_y, start_base_y + projected_panel_height)
            if min_x <= x <= max_x and min_y <= y <= max_y:
                return row
    return None


def _should_transition_horizontal_first(current, target, segments_by_row, step_x, step_y,
                                        projected_panel_width, projected_panel_height, connectors, sweep_direction):
    if current[0] == target[0] or current[1] == target[1]:
        return False

    current_row = _find_clean_line_row(
        current, segments_by_row, step_x, step_y, projected_panel_width,
        projected_panel_height, connectors, sweep_direction
    ) or _find_covered_row(
        current, segments_by_row, step_x, step_y, projected_panel_height, connectors
    )
    target_row = _find_clean_line_row(
        target, segments_by_row, step_x, step_y, projected_panel_width,
        projected_panel_height, connectors, sweep_direction
    ) or _find_covered_row(
        target, segments_by_row, step_x, step_y, projected_panel_height, connectors
    )
    if current_row is None or target_row is None:
        return False

    current_row_covers_target_x = _row_segment_contains_x(
        current_row, segments_by_row[current_row], target[0],
        step_x, step_y, projected_panel_width, projected_panel_height, connectors, sweep_direction
    )
    target_row_covers_current_x = _row_segment_contains_x(
        target_row, segments_by_row[target_row], current[0],
        step_x, step_y, projected_panel_width, projected_panel_height, connectors, sweep_direction
    )
    return current_row_covers_target_x and not target_row_covers_current_x


def _rows_between_cover_x(start_row, end_row, x, segments_by_row, step_x, step_y,
                          projected_panel_width, projected_panel_height, connectors, sweep_direction):
    for row in range(min(start_row, end_row), max(start_row, end_row) + 1):
        if row not in segments_by_row:
            return False
        if not _row_segment_contains_x(
            row, segments_by_row[row], x, step_x, step_y, projected_panel_width,
            projected_panel_height, connectors, sweep_direction
        ):
            return False
    return True


def _find_same_row_detour_y(current, target, segments_by_row, step_x, step_y,
                            projected_panel_width, projected_panel_height, connectors, sweep_direction):
    if current[1] != target[1] or current[0] == target[0]:
        return None

    current_row = _find_clean_line_row(
        current, segments_by_row, step_x, step_y, projected_panel_width,
        projected_panel_height, connectors, sweep_direction
    )
    target_row = _find_clean_line_row(
        target, segments_by_row, step_x, step_y, projected_panel_width,
        projected_panel_height, connectors, sweep_direction
    )
    if current_row is None or current_row != target_row:
        return None
    if _row_segment_covers_x_range(
        current_row, segments_by_row[current_row], current[0], target[0],
        step_x, step_y, projected_panel_width, projected_panel_height, connectors, sweep_direction
    ):
        return None

    candidates = []
    for row, row_segments in segments_by_row.items():
        if row == current_row:
            continue
        if not _row_segment_covers_x_range(
            row, row_segments, current[0], target[0], step_x, step_y,
            projected_panel_width, projected_panel_height, connectors, sweep_direction
        ):
            continue
        if not _rows_between_cover_x(
            current_row, row, current[0], segments_by_row, step_x, step_y,
            projected_panel_width, projected_panel_height, connectors, sweep_direction
        ):
            continue
        if not _rows_between_cover_x(
            current_row, row, target[0], segments_by_row, step_x, step_y,
            projected_panel_width, projected_panel_height, connectors, sweep_direction
        ):
            continue
        detour_y = _row_clean_y(
            row, row_segments, step_x, step_y, projected_panel_width,
            projected_panel_height, connectors, sweep_direction
        )
        if detour_y is None:
            continue
        candidates.append((abs(detour_y - current[1]), row, detour_y))

    if not candidates:
        return None
    return min(candidates)[2]


def _append_transition_safely(tasks, current, target, turn_back_len, segments_by_row,
                              step_x, step_y, projected_panel_width,
                              projected_panel_height, connectors, sweep_direction):
    detour_y = _find_same_row_detour_y(
        current, target, segments_by_row, step_x, step_y, projected_panel_width,
        projected_panel_height, connectors, sweep_direction
    )
    if detour_y is not None:
        current = _append_move(tasks, current, (current[0], detour_y), mode=2, turn_back_len=turn_back_len, back_len=0)
        current = _append_move(tasks, current, (target[0], detour_y), mode=2, turn_back_len=turn_back_len, back_len=0)
        return _append_move(tasks, current, target, mode=2, turn_back_len=turn_back_len, back_len=0)

    horizontal_first = _should_transition_horizontal_first(
        current, target, segments_by_row, step_x, step_y,
        projected_panel_width, projected_panel_height, connectors, sweep_direction
    )
    return _append_transition(tasks, current, target, turn_back_len, horizontal_first)


def _component_segments_by_row(component_cells, step_x, step_y, connectors):
    return build_panel_segments(
        component_cells,
        lambda row, col: panel_point_xy(row, col, step_x, step_y, connectors)[1]
    )


def _connector_cell_point(cell, other_cell, step_x, step_y, projected_panel_width,
                          projected_panel_height, connectors, sweep_direction):
    row, col = cell
    other_row, other_col = other_cell
    if row != other_row:
        lane_ratio = 0.75 if row < other_row else 0.25
    else:
        lane_ratio = _lane_ratios_for_sweep(sweep_direction)[0]
    return _clean_line_point(
        row, col, step_x, step_y, projected_panel_width,
        projected_panel_height, connectors, sweep_direction, lane_ratio
    )


def _connector_routes_between_components(all_cells, connectors, from_component, to_component):
    if from_component is None or to_component is None or from_component == to_component:
        return []
    from_cells = set(from_component)
    to_cells = set(to_component)
    routes = []
    for left, right in _connector_edges(all_cells, connectors or []):
        if left in from_cells and right in to_cells:
            routes.append((left, right))
        elif right in from_cells and left in to_cells:
            routes.append((right, left))
    return routes


def _plan_connector_transition(current, current_component, target_component, all_cells,
                               connectors, turn_back_len, step_x, step_y,
                               projected_panel_width, projected_panel_height, sweep_direction):
    routes = _connector_routes_between_components(all_cells, connectors, current_component, target_component)
    if not routes:
        return [], current

    current_segments = _component_segments_by_row(current_component, step_x, step_y, connectors)
    best_tasks = None
    best_current = None
    best_score = None

    for source_cell, target_cell in routes:
        source_point = _connector_cell_point(
            source_cell, target_cell, step_x, step_y, projected_panel_width,
            projected_panel_height, connectors, sweep_direction
        )
        target_point = _connector_cell_point(
            target_cell, source_cell, step_x, step_y, projected_panel_width,
            projected_panel_height, connectors, sweep_direction
        )
        route_tasks = []
        route_current = _append_transition_safely(
            route_tasks, current, source_point, turn_back_len, current_segments,
            step_x, step_y, projected_panel_width, projected_panel_height, connectors, sweep_direction
        )
        route_current = _append_transition(route_tasks, route_current, target_point, turn_back_len)
        score = (_task_total_length(route_tasks), source_cell, target_cell)
        if best_score is None or score < best_score:
            best_tasks = route_tasks
            best_current = route_current
            best_score = score

    return best_tasks or [], best_current if best_current is not None else current


def _find_point_component(point, components, step_x, step_y, projected_panel_width,
                          projected_panel_height, connectors, sweep_direction):
    for component in components:
        segments_by_row = _component_segments_by_row(component, step_x, step_y, connectors)
        if _find_clean_line_row(
            point, segments_by_row, step_x, step_y, projected_panel_width,
            projected_panel_height, connectors, sweep_direction
        ) is not None:
            return component
        if _find_covered_row(point, segments_by_row, step_x, step_y, projected_panel_height, connectors) is not None:
            return component
    return None


def _plan_component_tasks(component_cells, current, left_to_right, step_x, step_y, connectors,
                          turn_back_len, go_left_or_right_back_len, sweep_direction="top_to_bottom",
                          projected_panel_width=0, projected_panel_height=0):
    segments_by_row = _component_segments_by_row(component_cells, step_x, step_y, connectors)
    rows = sorted(segments_by_row.keys(), reverse=sweep_direction != "bottom_to_top")
    tasks = []

    for row in rows:
        row_segments = segments_by_row[row]
        if not row_segments:
            continue

        for lane_ratio in _lane_ratios_for_sweep(sweep_direction):
            ordered_segments = row_segments if left_to_right else list(reversed(row_segments))

            for col_start, col_end in ordered_segments:
                if left_to_right:
                    clean_start_col = col_start
                    clean_end_col = col_end
                else:
                    clean_start_col = col_end
                    clean_end_col = col_start

                clean_start = _clean_line_point(
                    row, clean_start_col, step_x, step_y, projected_panel_width, projected_panel_height,
                    connectors, sweep_direction, lane_ratio
                )
                clean_end = _clean_line_point(
                    row, clean_end_col, step_x, step_y, projected_panel_width, projected_panel_height,
                    connectors, sweep_direction, lane_ratio
                )

                current = _append_transition_safely(
                    tasks, current, clean_start, turn_back_len, segments_by_row,
                    step_x, step_y, projected_panel_width, projected_panel_height, connectors, sweep_direction
                )
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
    projected_panel_width = int(round(_as_int(panelWidth) * x_projection))
    projected_panel_height = int(round(_as_int(panelHeight) * y_projection))
    step_x = int(round((_as_int(panelWidth) + _as_int(gapX)) * x_projection))
    step_y = int(round((_as_int(panelHeight) + _as_int(gapY)) * y_projection))
    layout = layout or {}
    sweep_direction = _normalize_sweep_direction(layout.get("sweepDirection", "top_to_bottom"))
    connectors = layout.get("connectors", []) or []
    cells = expand_panel_cells(layout)
    components = _sort_components_for_sweep(connected_panel_components(cells, connectors, layout), sweep_direction)
    visit_strategy = layout.get("visitStrategy", "nearest")

    tasks = []
    current = (int(round(startX)), int(round(startY)))
    preferred_left_to_right = direction != "right"
    remaining_components = list(components)
    current_component = _find_point_component(
        current, components, step_x, step_y, projected_panel_width,
        projected_panel_height, connectors, sweep_direction
    )

    while remaining_components:
        if visit_strategy == "row":
            component = remaining_components.pop(0)
            transition_tasks, transition_current = _plan_connector_transition(
                current, current_component, component, cells, connectors, turnBackLen,
                step_x, step_y, projected_panel_width, projected_panel_height, sweep_direction
            )
            component_tasks, current = _plan_component_tasks(
                component, transition_current, preferred_left_to_right, step_x, step_y,
                connectors, turnBackLen, goLeftOrRightBackLen, sweep_direction,
                projected_panel_width, projected_panel_height
            )
            component_tasks = transition_tasks + component_tasks
            current_component = component
        else:
            best_index = None
            best_tasks = None
            best_current = None
            best_score = None
            for component_index, component in enumerate(remaining_components):
                transition_tasks, transition_current = _plan_connector_transition(
                    current, current_component, component, cells, connectors, turnBackLen,
                    step_x, step_y, projected_panel_width, projected_panel_height, sweep_direction
                )
                for left_to_right in (preferred_left_to_right, not preferred_left_to_right):
                    candidate_tasks, candidate_current = _plan_component_tasks(
                        component, transition_current, left_to_right, step_x, step_y,
                        connectors, turnBackLen, goLeftOrRightBackLen, sweep_direction,
                        projected_panel_width, projected_panel_height
                    )
                    candidate_tasks = transition_tasks + candidate_tasks
                    orientation_penalty = 0 if left_to_right == preferred_left_to_right else 1
                    score = (_task_total_length(candidate_tasks), orientation_penalty, component_index)
                    if best_score is None or score < best_score:
                        best_score = score
                        best_index = component_index
                        best_tasks = candidate_tasks
                        best_current = candidate_current

            component_tasks = best_tasks or []
            current = best_current if best_current is not None else current
            current_component = remaining_components[best_index]
            del remaining_components[best_index]

        tasks.extend(component_tasks)

    if returnToOrigin:
        current = _append_return_to_origin(tasks, current, turnBackLen)

    for index, task in enumerate(tasks):
        task["id"] = index + 1
        task["areaNumber"] = areaNumber

    return tasks
