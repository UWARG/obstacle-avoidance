"""
Classes to store object densities as a histogram.
"""


class SectorObstacleDensity:
    """
    Class to represent the obstacle density for a specific angular sector.
    """

    __create_key = object()

    @classmethod
    def create(
        cls, angle_start: float, angle_end: float, density: float
    ) -> "tuple[bool, SectorObstacleDensity | None]":
        """
        Create a new SectorObstacleDensity object.

        Parameters:
        - angle_start: The starting angle of the sector cluster in degrees
        - angle_end: The ending angle of the sector cluster in degrees
        - density: The calculated obstacle density for this sector. (0 <= density <= 1)
        """
        return True, SectorObstacleDensity(cls.__create_key, angle_start, angle_end, density)

    def __init__(
        self, class_private_create_key: object, angle_start: float, angle_end: float, density: float
    ) -> None:
        """
        Private constructor, use the create() method.
        """
        assert (
            class_private_create_key is SectorObstacleDensity.__create_key
        ), "Use the create() method"

        self.angle_start = angle_start
        self.angle_end = angle_end
        self.density = density

    def __str__(self) -> str:
        """
        String Representation.
        """
        return f"Sector [{self.angle_start}° - {self.angle_end}°]: Density = {self.density}"


class PolarObstacleDensity:
    """
    Class to represent a collection of obstacle densities across angular sectors.
    """

    __create_key = object()

    @classmethod
    def create(
        cls, sector_densities: list[SectorObstacleDensity]
    ) -> "tuple[bool, PolarObstacleDensity | None]":
        """
        Create a new PolarObstacleDensity object from a list of SectorObstacleDensity objects.

        Parameters:
        - sector_densities: List of SectorDensity objects.
        """
        return True, PolarObstacleDensity(cls.__create_key, sector_densities)

    def __init__(
        self, class_private_create_key: object, sector_densities: list[SectorObstacleDensity]
    ) -> None:
        """
        Private constructor, use create() method.
        """
        assert (
            class_private_create_key is PolarObstacleDensity.__create_key
        ), "Use the create() method"

        self.sector_densities = sector_densities

    def __str__(self) -> str:
        """
        String Representation.
        """
        density_strs = [str(sector) for sector in self.sector_densities]
        formatted_densities = ", ".join(density_strs)
        return f"PolarObstacleDensity: {len(self.sector_densities)} sectors, Densities: {formatted_densities}."
