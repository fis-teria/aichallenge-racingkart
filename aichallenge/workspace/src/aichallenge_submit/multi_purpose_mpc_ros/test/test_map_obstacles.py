import sys
from types import ModuleType

import numpy as np

skimage_morphology = ModuleType("skimage.morphology")
skimage_morphology.remove_small_holes = lambda data, **_: data
skimage_draw = ModuleType("skimage.draw")
skimage_draw.line_aa = lambda *_: ([], [], [])
sys.modules.setdefault("skimage.morphology", skimage_morphology)
sys.modules.setdefault("skimage.draw", skimage_draw)

from multi_purpose_mpc_ros.core.map import Map, Obstacle


def make_map(width=20, height=20, resolution=1.0):
    map_obj = Map.__new__(Map)
    map_obj.data = np.ones((height, width), dtype=np.int8)
    map_obj.data_backup = map_obj.data.copy()
    map_obj.width = width
    map_obj.height = height
    map_obj.resolution = resolution
    map_obj.origin = [0.0, 0.0, 0.0]
    map_obj.obstacles = []
    map_obj.boundaries = []
    return map_obj


def test_add_obstacle_clips_at_left_edge():
    map_obj = make_map()

    map_obj.add_obstacles([Obstacle(0.0, 10.0, 8.0)])

    assert map_obj.data.shape == (20, 20)
    assert np.count_nonzero(map_obj.data == 0) > 0


def test_add_obstacle_clips_at_right_edge():
    map_obj = make_map()

    map_obj.add_obstacles([Obstacle(19.0, 10.0, 8.0)])

    assert map_obj.data.shape == (20, 20)
    assert np.count_nonzero(map_obj.data == 0) > 0


def test_add_obstacle_clips_at_vertical_edges():
    map_obj = make_map()

    map_obj.add_obstacles([Obstacle(10.0, 19.0, 8.0),
                           Obstacle(10.0, 0.0, 8.0)])

    assert map_obj.data.shape == (20, 20)
    assert np.count_nonzero(map_obj.data == 0) > 0
