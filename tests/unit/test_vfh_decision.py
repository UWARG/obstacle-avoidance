"""
Test for VFHDecision module.
"""

import pytest
from modules import polar_obstacle_density
from modules.vfh_decision import vfh_decision

DENSITY_THRESHOLD = 0.5
MIN_CONSEC_SECTORS = 3
WIDE_VALLEY_THRESHOLD = 6
TARGET_ANGLE = 0.0  # Target directly ahead

# pylint: disable=redefined-outer-name


@pytest.fixture()
def decision_maker() -> vfh_decision.VFHDecision:
    """Creates a VFHDecision instance with predefined parameters."""
    return vfh_decision.VFHDecision(
        DENSITY_THRESHOLD, MIN_CONSEC_SECTORS, WIDE_VALLEY_THRESHOLD, TARGET_ANGLE
    )


def create_polar_obstacle_density(
    densities: list[float],
) -> polar_obstacle_density.PolarObstacleDensity:
    """Utility to create a PolarObstacleDensity object given a list of densities."""
    sectors = []
    for i, density in enumerate(densities):
        angle_start = -90 + i * 5
        angle_end = angle_start + 5
        result, sector = polar_obstacle_density.SectorObstacleDensity.create(
            angle_start, angle_end, density
        )
        assert result and sector is not None
        sectors.append(sector)
    result, polar_density = polar_obstacle_density.PolarObstacleDensity.create(sectors)
    assert result and polar_density is not None
    return polar_density


@pytest.fixture()
def open_space_target_ahead() -> polar_obstacle_density.PolarObstacleDensity:
    """Creates a PolarObstacleDensity where the target direction is completely open."""
    densities = [0.0] * 36  # All sectors have 0 density (open space)
    return create_polar_obstacle_density(densities)


@pytest.fixture()
def target_completely_blocked() -> polar_obstacle_density.PolarObstacleDensity:
    """Creates a PolarObstacleDensity where the target direction is fully blocked."""
    densities = [0.8] * 36  # All sectors have high density (completely blocked)
    return create_polar_obstacle_density(densities)


@pytest.fixture()
def target_open_ne_direction_nw() -> polar_obstacle_density.PolarObstacleDensity:
    """Creates a PolarObstacleDensity where NE is open but NW is blocked."""
    densities = []
    for i in range(36):
        angle_start = -90 + i * 5
        if -45 >= angle_start >= -90:
            densities.append(0.8)  # High density for NW direction
        else:
            densities.append(0.0)  # Open space elsewhere

    densities[27] = 0.2  # NE (45 degrees) is open, has density value lower than threshold
    return create_polar_obstacle_density(densities)


@pytest.fixture()
def target_open_narrow_valley() -> polar_obstacle_density.PolarObstacleDensity:
    """Creates a PolarObstacleDensity with a narrow valley around the target."""
    densities = [0.8] * 36
    densities[17] = 0.4  # -5 to 0 degrees
    densities[18] = 0.4  # 0 to +5 degrees
    return create_polar_obstacle_density(densities)


@pytest.fixture()
def target_obstructed_front_sector() -> polar_obstacle_density.PolarObstacleDensity:
    """Creates a PolarObstacleDensity where the front sector (centered at 0°) is obstructed."""
    densities = []

    for i in range(36):  # Generate densities for -90° to +90° in 5° sectors
        angle_start = -90 + i * 5
        if angle_start <= 0 < angle_start + 5:
            densities.append(0.8)  # Obstruction at the sector containing 0°
        else:
            densities.append(0.0)  # Open space elsewhere

    return create_polar_obstacle_density(densities)


@pytest.fixture()
def fragmented_open_space() -> polar_obstacle_density.PolarObstacleDensity:
    """
    Creates a PolarObstacleDensity with open sectors of 10 degrees (2 consecutive sectors),
    separated by individual blocked sectors.
    """
    densities = []
    for i in range(36):  # Generate 36 sectors for -90° to +90°
        if (i % 3) < 2:  # Two consecutive open sectors
            densities.append(0.0)  # Open sector
        else:  # Single blocked sector
            densities.append(0.8)  # Blocked sector

    return create_polar_obstacle_density(densities)


class TestVFHDecision:
    """Test for the VFHDecision.run() method."""

    def test_open_space_target_ahead(
        self,
        decision_maker: vfh_decision.VFHDecision,
        open_space_target_ahead: polar_obstacle_density.PolarObstacleDensity,
    ) -> None:
        """Tests decision-making when the target direction is completely open."""
        result, steering_angle = decision_maker.run(open_space_target_ahead)
        assert not result
        assert steering_angle is None

    def test_target_completely_blocked(
        self,
        decision_maker: vfh_decision.VFHDecision,
        target_completely_blocked: polar_obstacle_density.PolarObstacleDensity,
    ) -> None:
        """Tests decision-making when the target direction is completely blocked."""
        result, steering_angle = decision_maker.run(target_completely_blocked)
        assert result
        assert steering_angle == "REVERSE"

    def test_target_open_ne_direction_nw(
        self,
        decision_maker: vfh_decision.VFHDecision,
        target_open_ne_direction_nw: polar_obstacle_density.PolarObstacleDensity,
    ) -> None:
        """Tests decision-making when NE is open but NW is blocked."""
        result, steering_angle = decision_maker.run(target_open_ne_direction_nw)
        assert not result
        assert steering_angle is None

    def test_target_open_narrow_valley(
        self,
        decision_maker: vfh_decision.VFHDecision,
        target_open_narrow_valley: polar_obstacle_density.PolarObstacleDensity,
    ) -> None:
        """Tests decision-making when a narrow valley surrounds the target."""
        result, steering_angle = decision_maker.run(target_open_narrow_valley)
        assert result
        assert steering_angle == "REVERSE"

    def test_front_obstructed(
        self,
        decision_maker: vfh_decision.VFHDecision,
        target_obstructed_front_sector: polar_obstacle_density.PolarObstacleDensity,
    ) -> None:
        """Tests decision making when front sector obstructed, rest open"""
        result, steering_angle = decision_maker.run(target_obstructed_front_sector)
        assert result
        assert steering_angle == -45

    def test_fragmented_open_space(
        self,
        decision_maker: vfh_decision.VFHDecision,
        fragmented_open_space: polar_obstacle_density.PolarObstacleDensity,
    ) -> None:
        """Tests the decision module with fragmented open spaces of 10 degrees each"""

        result, steering_angle = decision_maker.run(fragmented_open_space)
        assert result  # Obstacle avoidance is required
        assert steering_angle == "REVERSE"
