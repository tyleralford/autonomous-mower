import importlib.util
from pathlib import Path


def _load_guard_module():
    script_path = Path('/home/tyler/mower_ws/src/mower_navigation/scripts/map_bounds_guard.py')
    spec = importlib.util.spec_from_file_location('map_bounds_guard', str(script_path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_in_bounds_math():
    mod = _load_guard_module()
    bounds = (0.0, 10.0, 0.0, 5.0)

    # Inside corners
    assert mod.in_bounds(bounds, 0.0, 0.0)
    assert mod.in_bounds(bounds, 10.0, 5.0)

    # Inside
    assert mod.in_bounds(bounds, 5.0, 2.5)

    # Outside
    assert not mod.in_bounds(bounds, -0.01, 2.5)
    assert not mod.in_bounds(bounds, 10.01, 2.5)
    assert not mod.in_bounds(bounds, 5.0, -0.01)
    assert not mod.in_bounds(bounds, 5.0, 5.01)


def test_map_bounds_from_meta():
    mod = _load_guard_module()
    b = mod.map_bounds_from_meta(0.05, (1.0, 2.0), 100, 200)
    assert b == (1.0, 1.0 + 100 * 0.05, 2.0, 2.0 + 200 * 0.05)
