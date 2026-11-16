import ast
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RlBundleTests(unittest.TestCase):
    def test_python_sources_parse_without_optional_training_dependencies(self):
        for path in (ROOT / "rl").rglob("*.py"):
            ast.parse(path.read_text(), filename=str(path))

    def test_primary_assets_are_relocatable(self):
        paths = [
            ROOT / "models/car_body.urdf", ROOT / "models/patrol_car.urdf",
            ROOT / "models/patrol_car.sdf", ROOT / "models/car_properties.json",
            ROOT / "worlds/map_20260817_154310_filtered.sdf",
            ROOT / "worlds/map_20260817_154310_filtered.json",
        ]
        for path in paths:
            self.assertNotIn("/home/a/ros2_ws/gz_sim", path.read_text(), str(path))

    def test_metadata_relative_paths_resolve(self):
        metadata = ROOT / "worlds" / "map_20260817_154310_filtered.json"
        data = json.loads(metadata.read_text())
        self.assertFalse(Path(data["map"]).is_absolute())
        self.assertTrue((metadata.parent / data["map"]).resolve().is_file())
        self.assertTrue((metadata.parent / data["world"]).resolve().is_file())


if __name__ == "__main__":
    unittest.main()
