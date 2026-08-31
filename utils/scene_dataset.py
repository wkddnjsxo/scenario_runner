import json
import os
import re
import shutil
from datetime import datetime, timezone


class SceneDatasetManager:
    def __init__(self, root_dir, scene_prefix="scene", scene_digits=2):
        self.root_dir = os.path.abspath(root_dir)
        self.scene_prefix = str(scene_prefix)
        self.scene_digits = max(1, int(scene_digits))

    def save_success(
        self,
        route_payload,
        meta_payload,
        files_source_dir=None,
        preferred_scene_name=None,
        existing_scene_dir=None,
    ):
        if existing_scene_dir:
            scene_dir = os.path.abspath(existing_scene_dir)
            scene_name = os.path.basename(scene_dir)
            index = self._parse_scene_index(scene_name) or self._next_scene_index()
            os.makedirs(scene_dir, exist_ok=True)
        else:
            index, scene_name, scene_dir = self._create_next_scene_dir(preferred_scene_name)

        meta = dict(meta_payload)
        meta.update(
            {
                "scene_index": index,
                "scene_name": scene_name,
                "saved_at": datetime.now(timezone.utc).isoformat(),
            }
        )

        moved_file_count = self._move_scene_files(files_source_dir, scene_dir)
        meta["moved_file_count"] = moved_file_count

        self._write_json(os.path.join(scene_dir, "meta.json"), self._json_safe(meta))
        self._write_json(
            os.path.join(scene_dir, "route.json"),
            self._json_safe(route_payload),
        )
        return scene_dir

    def reserve_next_scene(self):
        return self._create_next_scene_dir()

    def peek_next_scene(self):
        index = self._next_scene_index()
        return index, self._format_scene_name(index)

    def _format_scene_name(self, index):
        return f"{self.scene_prefix}{index:0{self.scene_digits}d}"

    def _create_next_scene_dir(self, preferred_scene_name=None):
        os.makedirs(self.root_dir, exist_ok=True)
        if preferred_scene_name:
            index = self._parse_scene_index(preferred_scene_name) or self._next_scene_index()
            scene_dir = os.path.join(self.root_dir, preferred_scene_name)
            try:
                os.makedirs(scene_dir)
                return index, preferred_scene_name, scene_dir
            except FileExistsError:
                pass

        index = self._next_scene_index()

        while True:
            scene_name = self._format_scene_name(index)
            scene_dir = os.path.join(self.root_dir, scene_name)
            try:
                os.makedirs(scene_dir)
                return index, scene_name, scene_dir
            except FileExistsError:
                index += 1

    def _next_scene_index(self):
        max_index = 0
        if not os.path.isdir(self.root_dir):
            return 1

        for name in os.listdir(self.root_dir):
            path = os.path.join(self.root_dir, name)
            if not os.path.isdir(path):
                continue
            index = self._parse_scene_index(name)
            if index is not None:
                max_index = max(max_index, index)

        return max_index + 1

    def _parse_scene_index(self, name):
        patterns = [r"^" + re.escape(self.scene_prefix) + r"(\d+)$"]
        if self.scene_prefix.rstrip("_") == "scene":
            patterns.append(r"^scene_?(\d+)$")

        for pattern in patterns:
            match = re.match(pattern, name)
            if match:
                return int(match.group(1))
        return None

    def _write_json(self, path, payload):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)

    def _move_scene_files(self, source_dir, scene_dir):
        if not source_dir or not os.path.isdir(source_dir):
            return 0

        source_dir = os.path.abspath(source_dir)
        scene_dir = os.path.abspath(scene_dir)
        if source_dir == scene_dir:
            return sum(
                1
                for name in os.listdir(scene_dir)
                if os.path.isfile(os.path.join(scene_dir, name))
                and name not in ("meta.json", "route.json")
            )

        moved = 0
        for name in sorted(os.listdir(source_dir)):
            src = os.path.join(source_dir, name)
            if not os.path.isfile(src):
                continue
            dst = os.path.join(scene_dir, name)
            shutil.move(src, dst)
            moved += 1

        shutil.rmtree(source_dir, ignore_errors=True)
        return moved

    def _json_safe(self, value):
        if isinstance(value, dict):
            return {str(k): self._json_safe(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._json_safe(item) for item in value]
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        return str(value)
