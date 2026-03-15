#!/usr/bin/env python3
import difflib
import json
import os
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ObjectQueryNode(Node):
    def __init__(self):
        super().__init__("object_query_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("query_topic", "/task/object_query")
        self.declare_parameter("result_topic", "/task/object_result")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("object_memory_file", "")
        self.declare_parameter("min_match_score", 0.58)
        self.declare_parameter("reload_memory_on_each_query", False)
        self.declare_parameter("use_live_detections", True)
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("prefer_live_detections", True)
        self.declare_parameter("live_detection_ttl_sec", 2.5)
        self.declare_parameter("min_detection_score", 0.4)
        self.declare_parameter("query_alias_map_json", "{}")
        self.declare_parameter("intent_to_labels_json", "{}")
        self.declare_parameter("live_target_policy", "balanced")  # highest_score|closest_center|balanced

        self.query_topic = str(self.get_parameter("query_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.object_memory_file = str(self.get_parameter("object_memory_file").value).strip()
        self.min_match_score = float(self.get_parameter("min_match_score").value)
        self.reload_memory_on_each_query = bool(
            self.get_parameter("reload_memory_on_each_query").value
        )
        self.use_live_detections = bool(self.get_parameter("use_live_detections").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.prefer_live_detections = bool(self.get_parameter("prefer_live_detections").value)
        self.live_detection_ttl_sec = float(self.get_parameter("live_detection_ttl_sec").value)
        self.min_detection_score = float(self.get_parameter("min_detection_score").value)
        self.query_alias_map = self._parse_query_alias_map(
            str(self.get_parameter("query_alias_map_json").value)
        )
        self.intent_to_labels = self._parse_intent_map(
            str(self.get_parameter("intent_to_labels_json").value)
        )
        self.live_target_policy = (
            str(self.get_parameter("live_target_policy").value).strip().lower().replace("-", "_")
            or "balanced"
        )

        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.query_sub = self.create_subscription(String, self.query_topic, self._on_query, 10)
        self.live_det_sub = self.create_subscription(
            String, self.detections_topic, self._on_detections, 10
        )

        self.memory_entries = self._load_memory()
        self.live_entries: List[Dict[str, Any]] = []
        self.get_logger().info(
            "Object query ready. "
            f"memory_entries={len(self.memory_entries)} "
            f"live_detections={self.use_live_detections} "
            f"topic={self.detections_topic} "
            f"intent_keys={len(self.intent_to_labels)} "
            f"target_policy={self.live_target_policy}"
        )

    @staticmethod
    def _normalize(text: str) -> str:
        return " ".join((text or "").strip().lower().split())

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_result(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.result_pub.publish(msg)

    def _parse_query_alias_map(self, payload: str) -> Dict[str, str]:
        try:
            data = json.loads(payload)
            if not isinstance(data, dict):
                return {}
        except Exception:
            return {}

        out = {}
        for k, v in data.items():
            key = self._normalize(str(k))
            val = self._normalize(str(v))
            if key and val:
                out[key] = val
        return out

    def _parse_intent_map(self, payload: str) -> Dict[str, List[str]]:
        try:
            data = json.loads(payload)
            if not isinstance(data, dict):
                return {}
        except Exception:
            return {}

        out: Dict[str, List[str]] = {}
        for k, v in data.items():
            key = self._normalize(str(k))
            if not key:
                continue

            labels: List[str] = []
            if isinstance(v, list):
                labels = [self._normalize(str(item)) for item in v if self._normalize(str(item))]
            else:
                one = self._normalize(str(v))
                if one:
                    labels = [one]

            if labels:
                out[key] = labels
        return out

    def _resolve_query_alias(self, query: str) -> str:
        if not query:
            return query
        if query in self.query_alias_map:
            return self.query_alias_map[query]
        for alias, canonical in self.query_alias_map.items():
            if alias and alias in query:
                return canonical
        return query

    def _load_memory(self) -> List[Dict[str, Any]]:
        if not self.object_memory_file:
            self.get_logger().warn("object_memory_file not set; using empty memory")
            return []
        if not os.path.isfile(self.object_memory_file):
            self.get_logger().warn(
                f'object_memory_file does not exist: "{self.object_memory_file}"'
            )
            return []

        try:
            with open(self.object_memory_file, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            self.get_logger().error(f"Failed to load object memory JSON: {exc}")
            return []

        if not isinstance(data, list):
            self.get_logger().error("Object memory JSON root must be a list")
            return []

        normalized: List[Dict[str, Any]] = []
        for item in data:
            if not isinstance(item, dict):
                continue
            label = self._normalize(str(item.get("label", "")))
            if not label:
                continue
            aliases = item.get("aliases", [])
            aliases_norm = []
            if isinstance(aliases, list):
                aliases_norm = [self._normalize(str(v)) for v in aliases if self._normalize(str(v))]

            pose = item.get("pose", {})
            if not isinstance(pose, dict):
                pose = {}

            normalized.append(
                {
                    "label": label,
                    "aliases": aliases_norm,
                    "frame_id": str(item.get("frame_id", "map")),
                    "pose": {
                        "x": float(pose.get("x", 0.0)),
                        "y": float(pose.get("y", 0.0)),
                        "yaw": float(pose.get("yaw", 0.0)),
                    },
                    "pickup": {
                        "height_m": float(item.get("pickup", {}).get("height_m", 0.75))
                        if isinstance(item.get("pickup", {}), dict)
                        else 0.75
                    },
                }
            )
        return normalized

    def _entry_aliases(self, entry: Dict[str, Any]) -> List[str]:
        return [entry["label"]] + list(entry.get("aliases", []))

    @staticmethod
    def _token_overlap(a: str, b: str) -> float:
        aset = set(a.split())
        bset = set(b.split())
        if not aset or not bset:
            return 0.0
        return len(aset & bset) / max(1, len(aset | bset))

    def _match_entry(self, query: str) -> Tuple[Optional[Dict[str, Any]], float, str]:
        best_entry = None
        best_score = 0.0
        best_alias = ""

        for entry in self.memory_entries:
            for alias in self._entry_aliases(entry):
                if not alias:
                    continue
                if query == alias or query in alias or alias in query:
                    score = 1.0
                else:
                    ratio = difflib.SequenceMatcher(None, query, alias).ratio()
                    overlap = self._token_overlap(query, alias)
                    score = 0.75 * ratio + 0.25 * overlap

                if score > best_score:
                    best_score = score
                    best_entry = entry
                    best_alias = alias

        return best_entry, best_score, best_alias

    def _prune_live_entries(self) -> None:
        if not self.live_entries:
            return
        now = time.monotonic()
        self.live_entries = [
            e
            for e in self.live_entries
            if now - float(e.get("seen_at_mono", 0.0)) <= self.live_detection_ttl_sec
        ]

    def _entry_matches_intent_label(self, entry: Dict[str, Any], label: str) -> bool:
        if not label:
            return False
        for alias in self._entry_aliases(entry):
            if alias == label:
                return True
            if label in alias or alias in label:
                return True
        return False

    def _live_target_rank(self, entry: Dict[str, Any]) -> float:
        det_score = float(entry.get("det_score", 0.0))
        bbox = entry.get("bbox", {})
        center_score = 0.5
        if isinstance(bbox, dict):
            cx = float(bbox.get("x_center_norm", 0.5))
            center_score = max(0.0, 1.0 - min(1.0, abs(cx - 0.5) * 2.0))

        policy = self.live_target_policy
        if policy == "highest_score":
            return det_score
        if policy == "closest_center":
            return 0.85 * center_score + 0.15 * det_score
        # balanced
        return 0.70 * det_score + 0.30 * center_score

    def _intent_labels_for_query(self, query: str) -> List[str]:
        labels: List[str] = []
        if query in self.intent_to_labels:
            labels.extend(self.intent_to_labels[query])
        for key, vals in self.intent_to_labels.items():
            if key and key in query:
                labels.extend(vals)

        deduped: List[str] = []
        seen = set()
        for label in labels:
            if label and label not in seen:
                deduped.append(label)
                seen.add(label)
        return deduped

    def _match_live_entry_by_intent(
        self, query: str
    ) -> Tuple[Optional[Dict[str, Any]], float, str]:
        self._prune_live_entries()
        if not self.live_entries:
            return None, 0.0, ""

        intent_labels = self._intent_labels_for_query(query)
        if not intent_labels:
            return None, 0.0, ""

        best_entry = None
        best_score = 0.0
        best_alias = ""
        for entry in self.live_entries:
            det_score = float(entry.get("det_score", 0.0))
            if det_score < self.min_detection_score:
                continue
            for label in intent_labels:
                if not self._entry_matches_intent_label(entry, label):
                    continue

                # Keep some lexical contribution and prioritize configured target policy.
                ratio = difflib.SequenceMatcher(None, query, label).ratio()
                overlap = self._token_overlap(query, label)
                text_score = 0.7 * ratio + 0.3 * overlap
                rank = self._live_target_rank(entry)
                score = 0.80 * rank + 0.20 * text_score

                if score > best_score:
                    best_score = score
                    best_entry = entry
                    best_alias = label
        return best_entry, best_score, best_alias

    def _match_live_entry(self, query: str) -> Tuple[Optional[Dict[str, Any]], float, str]:
        self._prune_live_entries()
        if not self.live_entries:
            return None, 0.0, ""

        best_entry = None
        best_score = 0.0
        best_alias = ""
        for entry in self.live_entries:
            det_score = float(entry.get("det_score", 0.0))
            if det_score < self.min_detection_score:
                continue
            for alias in self._entry_aliases(entry):
                if not alias:
                    continue
                if query == alias or query in alias or alias in query:
                    text_score = 1.0
                else:
                    ratio = difflib.SequenceMatcher(None, query, alias).ratio()
                    overlap = self._token_overlap(query, alias)
                    text_score = 0.75 * ratio + 0.25 * overlap

                # Blend lexical match with detector confidence.
                score = 0.75 * text_score + 0.25 * det_score
                if score > best_score:
                    best_score = score
                    best_entry = entry
                    best_alias = alias
        return best_entry, best_score, best_alias

    def _on_detections(self, msg: String) -> None:
        if not self.use_live_detections:
            return

        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().warn("detections payload is not valid JSON")
            return

        if isinstance(payload, dict):
            objects = payload.get("objects", [])
        elif isinstance(payload, list):
            objects = payload
        else:
            objects = []

        if not isinstance(objects, list):
            return

        now = time.monotonic()
        parsed: List[Dict[str, Any]] = []
        for item in objects:
            if not isinstance(item, dict):
                continue
            label = self._normalize(str(item.get("label", "")))
            if not label:
                continue
            aliases = item.get("aliases", [])
            aliases_norm = []
            if isinstance(aliases, list):
                aliases_norm = [self._normalize(str(v)) for v in aliases if self._normalize(str(v))]

            pose = item.get("pose", {})
            if not isinstance(pose, dict):
                pose = {}
            pickup = item.get("pickup", {})
            if not isinstance(pickup, dict):
                pickup = {}

            parsed.append(
                {
                    "label": label,
                    "aliases": aliases_norm,
                    "frame_id": str(item.get("frame_id", "map")),
                    "pose": {
                        "x": float(pose.get("x", 0.0)),
                        "y": float(pose.get("y", 0.0)),
                        "yaw": float(pose.get("yaw", 0.0)),
                    },
                    "pickup": {
                        "height_m": float(pickup.get("height_m", 0.75)),
                    },
                    "det_score": float(item.get("score", 1.0)),
                    "seen_at_mono": now,
                    "bbox": item.get("bbox", {}) if isinstance(item.get("bbox", {}), dict) else {},
                }
            )
        self.live_entries = parsed

    def _on_query(self, msg: String) -> None:
        raw_query = self._normalize(msg.data)
        if not raw_query:
            return
        query = self._resolve_query_alias(raw_query)

        if self.reload_memory_on_each_query:
            self.memory_entries = self._load_memory()

        if self.use_live_detections and self.prefer_live_detections:
            entry, score, alias = self._match_live_entry_by_intent(query)
            if entry is not None and score >= self.min_match_score:
                payload = {
                    "ok": True,
                    "query": raw_query,
                    "resolved_query": query,
                    "matched_alias": alias,
                    "label": entry["label"],
                    "frame_id": entry.get("frame_id", "map"),
                    "pose": entry.get("pose", {"x": 0.0, "y": 0.0, "yaw": 0.0}),
                    "pickup": entry.get("pickup", {"height_m": 0.75}),
                    "score": score,
                    "source": "live_detection_intent",
                }
                self._publish_result(payload)
                self._publish_status(
                    f'Live intent match "{raw_query}" -> "{entry["label"]}" at ({payload["pose"]["x"]:.2f}, {payload["pose"]["y"]:.2f})'
                )
                self.get_logger().info(
                    f'Live intent match: query="{raw_query}" resolved="{query}" label="{entry["label"]}" alias="{alias}" score={score:.2f}'
                )
                return

            entry, score, alias = self._match_live_entry(query)
            if entry is not None and score >= self.min_match_score:
                payload = {
                    "ok": True,
                    "query": raw_query,
                    "resolved_query": query,
                    "matched_alias": alias,
                    "label": entry["label"],
                    "frame_id": entry.get("frame_id", "map"),
                    "pose": entry.get("pose", {"x": 0.0, "y": 0.0, "yaw": 0.0}),
                    "pickup": entry.get("pickup", {"height_m": 0.75}),
                    "score": score,
                    "source": "live_detection",
                }
                self._publish_result(payload)
                self._publish_status(
                    f'Live object match "{query}" -> "{entry["label"]}" at ({payload["pose"]["x"]:.2f}, {payload["pose"]["y"]:.2f})'
                )
                self.get_logger().info(
                    f'Live match: query="{raw_query}" resolved="{query}" label="{entry["label"]}" alias="{alias}" score={score:.2f}'
                )
                return

        entry, score, alias = self._match_entry(query)
        if entry is None or score < self.min_match_score:
            payload = {
                "ok": False,
                "query": raw_query,
                "resolved_query": query,
                "reason": "not_found",
                "score": score,
            }
            self._publish_result(payload)
            self._publish_status(f'Object not found for "{raw_query}" (score={score:.2f})')
            self.get_logger().warn(
                f'Object not found: query="{raw_query}" resolved="{query}" score={score:.2f}'
            )
            return

        payload = {
            "ok": True,
            "query": raw_query,
            "resolved_query": query,
            "matched_alias": alias,
            "label": entry["label"],
            "frame_id": entry.get("frame_id", "map"),
            "pose": entry.get("pose", {"x": 0.0, "y": 0.0, "yaw": 0.0}),
            "pickup": entry.get("pickup", {"height_m": 0.75}),
            "score": score,
            "source": "memory",
        }
        self._publish_result(payload)
        self._publish_status(
            f'Object match "{query}" -> "{entry["label"]}" at ({payload["pose"]["x"]:.2f}, {payload["pose"]["y"]:.2f})'
        )
        self.get_logger().info(
            f'Object match: query="{query}" label="{entry["label"]}" alias="{alias}" score={score:.2f}'
        )


def main() -> None:
    rclpy.init()
    node = ObjectQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
