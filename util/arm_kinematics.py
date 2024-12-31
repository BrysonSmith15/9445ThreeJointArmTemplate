from enum import Enum
from math import acos, atan2, sin, cos

from wpimath.units import meters, kilograms, newton_meters
from wpimath.geometry import Rotation2d


class RotationRange:
    min: Rotation2d
    max: Rotation2d

    def __init__(self, min: Rotation2d, max: Rotation2d):
        self.min = min
        self.max = max

    def in_range(self, angle: Rotation2d) -> bool:
        return (
            self.min.radians() <= angle.radians()
            and angle.radians() <= self.max.radians()
        )


class ArmResolution(Enum):
    HIGH = 1
    LOW = 2
    FORCE = 3


class ArmKinematics3:
    """
    This class calculates the kinematics for an arm which has a shoulder, elbow, and wrist.
    One limitation is that the class considers each joint to be uniformly dense when preforming calculations in force mode.
    """

    l1: meters
    l2: meters
    a1_angle_range: RotationRange
    a2_angle_range: RotationRange
    resolution_strategy: ArmResolution
    a1_mass: kilograms
    a2_mass: kilograms
    a1_gear_ratio: float
    a2_gear_ratio: float

    def __init__(
        self,
        l1: meters,
        l2: meters,
        a1_angle_range: RotationRange,
        a2_angle_range: RotationRange,
        resolution_strategy: ArmResolution,
        a1_mass: kilograms = 0,
        a2_mass: kilograms = 0,
        a1_gear_ratio: float = 1,
        a2_gear_ratio: float = 1,
    ):
        """
        Constructs the arm kinematics class
        @param l1: meters  -- this is the length of the arm from the shoulder joint to the elbow.
        @param l2: meters  -- this is the length of the arm from the elbow joint to the wrist
        @param a1_angle_range: RotationRange  -- this is the range that the shoulder joint is able to rotate through
        @param a2_angle_range: RotationRange  -- this is the range that the elbow joint is able to rotate through
        @param resolution_strategy: ArmResolution  -- this is how the solution to the inverse kinematics problem will be solved. Refer to ```ArmResolution``` docs for information
        @param a1_mass: kilograms  -- The mass of the arm in kilograms of the length from the shoulder to elbow joint. Only required if ```FORCE``` is used to solve kinematics
        @param a2_mass: kilograms  -- The mass of the arm in kilograms of the length from the shoulder to elbow joint. Only required if ```FORCE``` is used to solve kinematics
        @param a1_gear_ratio: float  -- The gear ratio between the driving motor and the shoulder joint. Can be divided further by the number of motors if applicable. Only required if ```FORCE``` is used to solve kinematics
        @param a1_gear_ratio: float  -- The gear ratio between the driving motor and the elbow joint. Can be divided further by the number of motors if applicable. Only required if ```FORCE``` is used to solve kinematics
        """
        self.l1 = l1
        self.l2 = l2
        self.a1_angle_range = a1_angle_range
        self.a2_angle_range = a2_angle_range
        self.resolution_strategy = resolution_strategy
        self.a1_mass = a1_mass
        self.a2_mass = a2_mass
        self.a1_gear_ratio = a1_gear_ratio
        self.a2_gear_ratio = a2_gear_ratio

    def get_angles(
        self, x: meters, y: meters, wrist_angle: Rotation2d
    ) -> tuple[Rotation2d, Rotation2d, Rotation2d]:
        # theta is the angle of the shoulder arm. phi is the elbow angle
        try:
            phi = Rotation2d(
                acos((x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2))
            )
        except ValueError:
            raise ValueError(
                f"The elbow angle will be unreachable with position ({x}, {y})"
            )

        if not self.a2_angle_range.in_range(phi):
            raise ValueError(
                f"The elbow angle will be unreachable with position ({x}, {y})"
            )
        try:
            theta_part1 = atan2(y, x)
            theta_part2 = atan2(
                self.l2 * sin(phi.radians()), self.l1 + self.l2 * cos(phi.radians())
            )
        except ValueError:
            raise ValueError(
                f"The shoulder angle will be unreachable with position ({x}, {y})"
            )

        if self.resolution_strategy == ArmResolution.HIGH:
            phi = -phi
            if not self.a2_angle_range.in_range(phi):
                raise ValueError(
                    f"Neither HIGH or LOW resolution will make the elbow fit in its range at ({x}, {y})"
                )
            theta = Rotation2d(theta_part1 + theta_part2)
            if not self.a1_angle_range.in_range(theta):
                theta = Rotation2d(theta_part1 - theta_part2)
                if not self.a1_angle_range.in_range(theta):
                    raise ValueError(
                        f"Neither HIGH or LOW resolution will make the shoulder fit in its range at ({x}, {y})"
                    )
        elif self.resolution_strategy == ArmResolution.LOW:
            theta = Rotation2d(theta_part1 - theta_part2)
            if not self.a1_angle_range.in_range(theta):
                theta = Rotation2d(theta_part1 + theta_part2)
                if not self.a1_angle_range.in_range(theta):
                    raise ValueError(
                        f"Neither HIGH or LOW resolution will make the shoulder fit in its range at ({x}, {y})"
                    )
        else:
            low_phi = phi
            high_phi = -phi
            low_theta = Rotation2d(theta_part1 - theta_part2)
            high_theta = Rotation2d(theta_part1 + theta_part2)
            low_force = self._calculate_force(
                low_theta, low_phi, self.a1_gear_ratio, self.a2_gear_ratio
            )
            high_force = self._calculate_force(
                high_theta, high_phi, self.a1_gear_ratio, self.a2_gear_ratio
            )
            if abs(low_force[0] - low_force[1]) <= abs(high_force[0] - high_force[1]):
                phi = low_phi
                theta = high_theta
                if not (self.a1_angle_range.in_range(theta)) or (
                    not self.a2_angle_range.in_range(phi)
                ):
                    phi = high_phi
                    theta = high_theta
                    if not (self.a1_angle_range.in_range(theta)) or (
                        not self.a2_angle_range.in_range(phi)
                    ):
                        raise ValueError(
                            f"Neither HIGH or LOW resolution will make the shoulder fit in its range at ({x}, {y}) with FORCE settings"
                        )
            else:
                phi = high_phi
                theta = high_theta
                if not (self.a1_angle_range.in_range(theta)) or (
                    not self.a2_angle_range.in_range(phi)
                ):
                    phi = low_phi
                    theta = low_theta
                    if not (self.a1_angle_range.in_range(theta)) or (
                        not self.a2_angle_range.in_range(phi)
                    ):
                        raise ValueError(
                            f"Neither HIGH or LOW resolution will make the shoulder fit in its range at ({x}, {y}) with FORCE settings"
                        )

        wrist = (phi + theta).rotateBy(-wrist_angle)
        return (theta, phi, wrist)

    def _calculate_force(
        self,
        theta: Rotation2d,
        phi: Rotation2d,
        theta_gear_ratio: float,
        phi_gear_ratio: float,
    ) -> tuple[newton_meters, newton_meters]:
        return (
            self.a1_mass * cos(theta.radians()) * (self.l1 / 2) * theta_gear_ratio,
            self.a2_mass
            * cos(phi.radians())
            * (cos(theta.radians()) * self.l1 + cos(phi.radians()) * (self.l2 / 2))
            * phi_gear_ratio,
        )
