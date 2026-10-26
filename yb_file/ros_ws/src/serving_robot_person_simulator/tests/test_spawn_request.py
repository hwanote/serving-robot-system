"""Ensure Gazebo gets SDF bytes, not a disappearing temporary file."""
import json
import sys
from pathlib import Path
from unittest import TestCase
from unittest.mock import patch
from xml.etree import ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from serving_robot_person_simulator import gazebo_api


class SpawnRequestTests(TestCase):
    def test_inline_sdf_and_exact_name(self):
        with patch.object(gazebo_api, "_run_service", return_value="data: true") as call:
            gazebo_api.spawn("cafe", "test_person_01", 0.0, -2.3, 0.22, 1.55)
        args = call.call_args.args
        self.assertEqual(args[0], "cafe")
        self.assertEqual(args[1], "create")
        self.assertEqual(args[3], "gz.msgs.EntityFactory")
        request = args[2]
        self.assertNotIn("sdf_filename:", request)
        self.assertNotIn("/tmp/person_sim_", request)
        self.assertIn('name: "test_person_01", allow_renaming: false', request)
        xml = json.loads(request.removeprefix("sdf: ").split(', name: ', 1)[0])
        model = ET.fromstring(xml).find("model")
        self.assertEqual(model.get("name"), "test_person_01")
        self.assertEqual(model.findtext("pose"), "0.000000 -2.300000 0 0 0 0")

    def test_name_ownership(self):
        with self.assertRaises(ValueError):
            gazebo_api.spawn("cafe", "robot_01", 0, 0, 0.22, 1.55)

    def test_service_rejection_not_silent(self):
        with patch.object(gazebo_api.subprocess, "run", return_value=type("Result", (), {"returncode": 0, "stdout": "data: false", "stderr": ""})()):
            with self.assertRaisesRegex(RuntimeError, "rejected"):
                gazebo_api.spawn("cafe", "test_person_01", 0, 0, 0.22, 1.55)
