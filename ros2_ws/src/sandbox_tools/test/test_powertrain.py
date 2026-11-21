import math

from sandbox_tools.powertrain import REGEN_LIMIT, ediff_factors, split_torque


def test_straight_throttle_is_equal():
    assert split_torque(0.5, 0.0, 0.0, 5.0) == (0.5, 0.5)


def test_regen_is_20_percent_of_forward_power():
    left, right = split_torque(0.0, 1.0, 0.0, 10.0)
    assert math.isclose(left, -0.2) and math.isclose(right, -0.2)
    assert REGEN_LIMIT == 0.2


def test_regen_never_exceeds_limit_even_in_a_turn():
    for steer in (-1.0, 0.0, 1.0):
        for t in split_torque(0.0, 1.0, steer, 10.0):
            assert -0.2 - 1e-9 <= t <= 0.0


def test_regen_fades_at_low_speed():
    assert split_torque(0.0, 1.0, 0.0, 0.0) == (0.0, 0.0)
    assert math.isclose(split_torque(0.0, 1.0, 0.0, 1.0)[0], -0.1)


def test_brake_cancels_throttle():
    assert split_torque(1.0, 0.5, 0.0, 10.0)[0] < 0.0


def test_ediff_right_turn_gives_less_torque_to_right_wheel():
    left, right = split_torque(0.5, 0.0, 1.0, 5.0)         # + = right
    assert right < 0.5 < left
    assert math.isclose(left + right, 1.0)                  # total unchanged


def test_ediff_left_turn_mirrors():
    left, right = split_torque(0.5, 0.0, -1.0, 5.0)
    assert left < 0.5 < right


def test_ediff_off_is_locked():
    assert split_torque(0.5, 0.0, 1.0, 5.0, ediff=False) == (0.5, 0.5)


def test_outer_wheel_is_clipped_at_full_torque():
    left, right = split_torque(1.0, 0.0, 1.0, 5.0)
    assert left == 1.0 and right < 1.0


def test_ediff_factors_match_geometry():
    d = math.radians(20)
    l, r = ediff_factors(d, 1.627, 1.2)
    radius = 1.627 / math.tan(d)
    assert math.isclose(r / l, (radius + 0.6) / (radius - 0.6))
