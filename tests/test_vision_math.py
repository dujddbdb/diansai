import importlib.util
import pathlib
import unittest


MODULE = pathlib.Path(__file__).parents[1] / "k230" / "vision_math.py"


def _baseline_center(corners):
    return (
        sum(point[0] for point in corners) / 4.0,
        sum(point[1] for point in corners) / 4.0,
    )


if MODULE.exists():
    spec = importlib.util.spec_from_file_location("vision_math", MODULE)
    vision_math = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vision_math)
    quadrilateral_center = vision_math.quadrilateral_center
else:
    quadrilateral_center = _baseline_center


class VisionMathTest(unittest.TestCase):
    def test_uses_diagonal_intersection_for_oblique_target(self):
        corners = [(0.0, 0.0), (10.0, 0.0), (8.0, 10.0), (2.0, 10.0)]
        x, y = quadrilateral_center(corners)
        self.assertAlmostEqual(x, 5.0, places=4)
        self.assertAlmostEqual(y, 6.25, places=4)

    def test_explicit_perspective_mapping_supports_plane_calibration(self):
        corners = [(0.0, 0.0), (10.0, 0.0), (8.0, 10.0), (2.0, 10.0)]
        self.assertTrue(hasattr(vision_math, "perspective_map_point"))
        x, y = vision_math.perspective_map_point(corners, 0.5, 0.5)
        self.assertAlmostEqual(x, 5.0, places=4)
        self.assertAlmostEqual(y, 6.25, places=4)
        right_x, right_y = vision_math.perspective_map_point(corners, 0.75, 0.5)
        self.assertGreater(right_x, x)
        self.assertAlmostEqual(right_y, y, places=4)

    def test_laser_origin_relative_error_uses_calibrated_origin(self):
        corners = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        self.assertTrue(hasattr(vision_math, "relative_control_error"))
        dx, dy = vision_math.relative_control_error(corners, 0.5, 0.5, 3.0, 7.0)
        self.assertAlmostEqual(dx, 2.0, places=4)
        self.assertAlmostEqual(dy, -2.0, places=4)

    def test_coordinate_packet_contains_only_xy(self):
        self.assertTrue(hasattr(vision_math, "pack_coordinate_packet"))
        packet = vision_math.pack_coordinate_packet(0, 0)
        self.assertEqual(packet, bytes((0x00, 0x00, 0x00, 0x00)))
        self.assertEqual(len(packet), 4)

    def test_coordinate_packet_keeps_signed_twos_complement(self):
        packet = vision_math.pack_coordinate_packet(-1, 2)
        self.assertEqual(packet, bytes((0xFF, 0xFF, 0x00, 0x02)))
        self.assertEqual(len(packet), 4)


if __name__ == "__main__":
    unittest.main()
