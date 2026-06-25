#!/usr/bin/env python3
"""
AlphaRobot per-map location storage helper.

Each saved navigation map receives a separate location file:

~/ros2_ws/maps/locations/<map_name>.yaml

Example:
~/ros2_ws/maps/locations/home_map_final.yaml
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict

import yaml


WORKSPACE = Path.home() / "ros2_ws"
MAP_DIR = WORKSPACE / "maps"
LOCATION_DIR = MAP_DIR / "locations"


def clean_map_name(name: Any) -> str:
    clean = str(name).strip()

    if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", clean):
        raise ValueError(
            "Map name can use only letters, numbers, underscore, and hyphen."
        )

    return clean


def clean_location_name(name: Any) -> str:
    clean = str(name).strip().lower()
    clean = re.sub(r"[^a-z0-9\s_-]", " ", clean)
    clean = re.sub(r"[\s-]+", "_", clean)
    clean = re.sub(r"_+", "_", clean).strip("_")

    if not clean or len(clean) > 48:
        raise ValueError(
            "Location name can use letters, numbers, spaces, underscore, or hyphen."
        )

    return clean


def map_yaml_path(map_name: Any) -> Path:
    clean = clean_map_name(map_name)
    return MAP_DIR / f"{clean}.yaml"


def location_file_path(map_name: Any) -> Path:
    clean = clean_map_name(map_name)
    return LOCATION_DIR / f"{clean}.yaml"


def available_maps() -> list[str]:
    MAP_DIR.mkdir(parents=True, exist_ok=True)

    maps = []

    for yaml_file in sorted(MAP_DIR.glob("*.yaml")):
        pgm_file = yaml_file.with_suffix(".pgm")

        if pgm_file.is_file():
            maps.append(yaml_file.stem)

    return maps


def ensure_location_file(map_name: Any) -> Path:
    LOCATION_DIR.mkdir(parents=True, exist_ok=True)

    location_file = location_file_path(map_name)

    if not location_file.is_file():
        location_file.write_text("locations:\n")

    return location_file


def load_locations(map_name: Any) -> Dict[str, Dict[str, float]]:
    location_file = ensure_location_file(map_name)

    try:
        raw = yaml.safe_load(location_file.read_text()) or {}
    except Exception as exc:
        raise RuntimeError(
            f"Could not read location file '{location_file.name}': {exc}"
        ) from exc

    raw_locations = raw.get("locations", {})

    if not isinstance(raw_locations, dict):
        return {}

    locations: Dict[str, Dict[str, float]] = {}

    for raw_name, raw_pose in raw_locations.items():
        if not isinstance(raw_pose, dict):
            continue

        try:
            name = clean_location_name(raw_name)

            locations[name] = {
                "x": float(raw_pose["x"]),
                "y": float(raw_pose["y"]),
                "yaw_deg": float(raw_pose.get("yaw_deg", 0.0)),
            }

        except (KeyError, TypeError, ValueError):
            continue

    return locations


def save_location(
    map_name: Any,
    location_name: Any,
    x: Any,
    y: Any,
    yaw_deg: Any,
) -> Dict[str, float]:
    clean_map = clean_map_name(map_name)
    clean_location = clean_location_name(location_name)

    if clean_map not in available_maps():
        raise ValueError(
            f"Map '{clean_map}' is not saved. Save its YAML and PGM first."
        )

    pose = {
        "x": round(float(x), 4),
        "y": round(float(y), 4),
        "yaw_deg": round(float(yaw_deg), 2),
    }

    locations = load_locations(clean_map)
    locations[clean_location] = pose

    location_file = ensure_location_file(clean_map)

    location_file.write_text(
        yaml.safe_dump(
            {"locations": locations},
            sort_keys=False,
            default_flow_style=False,
        )
    )

    return pose


def migrate_legacy_locations() -> bool:
    """
    Copy the old config/named_locations.yaml into the new per-map location
    file for home_map_final, only if the new file does not already exist.
    """
    legacy_file = (
        WORKSPACE
        / "src"
        / "articubot_one"
        / "config"
        / "named_locations.yaml"
    )

    target_file = location_file_path("home_map_final")

    if target_file.is_file() or not legacy_file.is_file():
        return False

    try:
        raw = yaml.safe_load(legacy_file.read_text()) or {}
        raw_locations = raw.get("locations", {})

        if not isinstance(raw_locations, dict):
            return False

        LOCATION_DIR.mkdir(parents=True, exist_ok=True)

        target_file.write_text(
            yaml.safe_dump(
                {"locations": raw_locations},
                sort_keys=False,
                default_flow_style=False,
            )
        )

        return True

    except Exception:
        return False


if __name__ == "__main__":
    migrated = migrate_legacy_locations()

    print("Available saved maps:")

    for map_name in available_maps():
        locations = load_locations(map_name)

        print(
            f"  - {map_name}: "
            f"{', '.join(sorted(locations.keys())) or 'no locations saved'}"
        )

    if migrated:
        print("\nMigrated old named_locations.yaml into home_map_final.")
