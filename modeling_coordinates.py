# coding=utf-8
"""Coordinate helpers shared by modeling capture and route generation.

All areas that belong to one modeling session must use one coordinate frame.
The first valid point of area one is the model origin.  Every area point and
connection point is projected from RTK latitude/longitude into centimeters
relative to that origin.  In particular, creating area two or area three must
not create another local ``(0, 0)``.
"""
import copy
import math


EARTH_RADIUS_M = 6371000.0


def _number(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if not math.isnan(number) and not math.isinf(number) else None


def find_model_origin(draft):
    """Return the first RTK point from the first recorded modeling area."""
    if not isinstance(draft, dict):
        return None
    for group in draft.get("groups") or []:
        for point in group.get("points") or []:
            lat = _number(point.get("lat"))
            lon = _number(point.get("lon"))
            if lat is None or lon is None:
                continue
            return {
                "pointId": point.get("id"),
                "groupId": group.get("id"),
                "lat": lat,
                "lon": lon,
                "x": 0.0,
                "y": 0.0,
            }
    return None


def lat_lon_to_model_xy_cm(origin, lat, lon):
    """Project an RTK point into the model-wide east/north centimeter frame."""
    lat = _number(lat)
    lon = _number(lon)
    if not isinstance(origin, dict) or lat is None or lon is None:
        return None
    origin_lat = _number(origin.get("lat"))
    origin_lon = _number(origin.get("lon"))
    if origin_lat is None or origin_lon is None:
        return None
    mean_lat = math.radians((origin_lat + lat) / 2.0)
    x_m = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(mean_lat)
    y_m = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return x_m * 100.0, y_m * 100.0


def _normalize_point(point, origin, force=False):
    item = dict(point or {})
    xy = lat_lon_to_model_xy_cm(origin, item.get("lat"), item.get("lon"))
    has_xy = _number(item.get("x")) is not None and _number(item.get("y")) is not None
    if xy is not None and (force or not has_xy):
        # RTK is the source of truth.  Recomputing even an existing x/y repairs
        # legacy drafts in which every area used its own first point as (0, 0).
        item["x"] = round(xy[0], 3)
        item["y"] = round(xy[1], 3)
    return item


def normalize_draft_coordinates(draft, force=False):
    """Return a draft whose area and bridge points share one model origin.

    The function intentionally keeps point ids, recording order, recognition
    roles and area/link topology unchanged.  Only x/y and coordinate-frame
    metadata are updated, so it is safe to run on both new and legacy drafts.
    """
    if not isinstance(draft, dict):
        return draft
    normalized = copy.deepcopy(draft)
    origin = find_model_origin(normalized)
    if origin is None:
        return normalized

    groups = []
    for group in normalized.get("groups") or []:
        item = dict(group)
        item["points"] = [
            _normalize_point(point, origin, force=force)
            for point in (group.get("points") or [])
        ]
        groups.append(item)
    normalized["groups"] = groups

    links = []
    for link in normalized.get("groupLinks") or []:
        item = dict(link)
        item["points"] = [
            _normalize_point(point, origin, force=force)
            for point in (link.get("points") or [])
        ]
        links.append(item)
    normalized["groupLinks"] = links

    normalized["coordinateFrame"] = {
        "type": "model_origin",
        "unit": "cm",
        "originPointId": origin.get("pointId"),
        "originGroupId": origin.get("groupId"),
        "originLat": round(origin["lat"], 10),
        "originLon": round(origin["lon"], 10),
    }
    return normalized
