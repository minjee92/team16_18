"""Geometry and preservation checks; these do not replace Gazebo rendering tests."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml

SIM = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('texture', SIM / 'tools/add_ground_texture.py')
TEXTURE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TEXTURE)


class GroundTextureTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        Image.new('L', (20, 10), 255).save(self.root / 'map.pgm')
        # Deliberately non-square image and asymmetric physical alignment.
        Image.new('RGB', (100, 50), 'white').save(self.root / 'course.png')
        self.meta = {'image': 'map.pgm', 'resolution': 0.1, 'origin': [-1, -2, 0]}
        (self.root / 'map.yaml').write_text(yaml.safe_dump(self.meta))
        self.calibration = {'texture_size': [100, 50], 'landmarks': [
            {'texture_px': [0, 0], 'map_px': [2, 1]},
            {'texture_px': [100, 0], 'map_px': [18, 1]},
            {'texture_px': [100, 50], 'map_px': [18, 9]},
            {'texture_px': [0, 50], 'map_px': [2, 9]},
        ]}
        (self.root / 'calibration.json').write_text(json.dumps(self.calibration))

    def transform(self, calibration=None):
        return TEXTURE.fit_transform(calibration or self.calibration,
                                     (100, 50), (20, 10), self.meta)[0]

    def test_top_left_origin_and_scale(self):
        vertices = TEXTURE.world_vertices(self.transform(), (100, 50), (20, 10), self.meta)
        np.testing.assert_allclose(vertices, [[-0.8, -1.1], [0.8, -1.1],
                                             [0.8, -1.9], [-0.8, -1.9]])

    def test_map_origin_rotation(self):
        self.meta['origin'][2] = math.pi / 2
        vertices = TEXTURE.world_vertices(self.transform(), (100, 50), (20, 10), self.meta)
        np.testing.assert_allclose(vertices[0], [-1.9, -1.8])

    def test_inconsistent_landmark_rejected(self):
        self.calibration['landmarks'][3]['map_px'] = [8, 9]
        with self.assertRaisesRegex(ValueError, 'Landmark error'):
            self.transform()

    def test_degenerate_landmarks_rejected(self):
        self.calibration['landmarks'] = self.calibration['landmarks'][:2]
        with self.assertRaises(ValueError):
            self.transform()

    def test_reflected_course_rejected(self):
        for point in self.calibration['landmarks']:
            point['map_px'][0] = 20 - point['map_px'][0]
        with self.assertRaisesRegex(ValueError, 'reflects'):
            self.transform()

    def test_wrong_image_dimensions_rejected(self):
        self.calibration['texture_size'] = [99, 50]
        with self.assertRaisesRegex(ValueError, 'texture_size'):
            self.transform()

    def test_grid_anchor_uv_and_triangle_winding(self):
        calibration = {'texture_size': [100, 50], 'grid': {
            'x': [[0, 2], [30, 6], [100, 18]],
            'y': [[0, 1], [20, 7], [50, 9]],
        }}
        vertices, uv, triangles = TEXTURE.grid_mesh(calibration, (100, 50), (20, 10), self.meta)
        np.testing.assert_allclose(vertices[4], [-0.4, -1.7])
        np.testing.assert_allclose(uv[4], [0.3, 0.6])
        self.assertEqual(len(triangles), 8)
        for a, b, c in triangles:
            ab, ac = vertices[b] - vertices[a], vertices[c] - vertices[a]
            self.assertGreater(ab[0]*ac[1] - ab[1]*ac[0], 0)

    def test_grid_fold_rejected(self):
        calibration = {'texture_size': [100, 50], 'grid': {
            'x': [[0, 2], [30, 18], [100, 6]], 'y': [[0, 1], [50, 9]]}}
        with self.assertRaisesRegex(ValueError, 'increase strictly'):
            TEXTURE.grid_mesh(calibration, (100, 50), (20, 10), self.meta)

    def test_existing_walls_blocks_and_input_preserved(self):
        original = SIM / 'worlds/mission4_3_clean_1cm.sdf'
        before = original.read_bytes()
        output = self.root / 'textured.sdf'
        TEXTURE.add_texture(original, self.root / 'map.yaml', self.root / 'course.png',
                            self.root / 'calibration.json', output)
        self.assertEqual(original.read_bytes(), before)
        source = ET.fromstring(before).find('world')
        target = ET.parse(output).getroot().find('world')
        for model in source.findall('model'):
            retained = target.find(f"model[@name='{model.get('name')}']")
            self.assertEqual(ET.tostring(model), ET.tostring(retained))
        added = target.find("model[@name='course_ground_texture']")
        self.assertEqual(added.findall('.//collision'), [])
        self.assertEqual((self.root / 'textured_assets/course.png').read_bytes(),
                         (self.root / 'course.png').read_bytes())
        ET.parse(self.root / 'textured_assets/ground.dae')
        with self.assertRaises(FileExistsError):
            TEXTURE.add_texture(original, self.root / 'map.yaml', self.root / 'course.png',
                                self.root / 'calibration.json', output)


if __name__ == '__main__':
    unittest.main()
