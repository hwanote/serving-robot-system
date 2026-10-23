import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from serving_robot_person_simulator.pose_cli import parse_pose


class GazeboPoseParserTest(unittest.TestCase):
    def test_model_named_pose(self):
        p = '''pose {
  name: "robot_01"
  id: 17
  position {
    x: -3.2
    y: -2.25
    z: 0.02
  }
  orientation { w: 1 }
}'''
        self.assertEqual(parse_pose(p), ('robot_01', -3.2, -2.25))

    def test_zero_coordinates_omitted(self):
        p = '''pose {
  name: "test_person_01"
  position { z: 0.0 }
}'''
        self.assertEqual(parse_pose(p), ('test_person_01', 0.0, 0.0))

    def test_unnamed_cannot_be_treated_as_pose(self):
        p = '''pose {
  position { x: 3 y: 4 }
}'''
        self.assertIsNone(parse_pose(p))

    def test_invalid_position(self):
        p = '''pose {
  name: "robot_01"
  position { x: nan y: 0 }
}'''
        self.assertIsNone(parse_pose(p))


if __name__ == '__main__':
    unittest.main()
