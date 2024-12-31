from wpimath.geometry import Rotation2d
from util.arm_kinematics import ArmKinematics3, ArmResolution, RotationRange

from math import sqrt


def pretty_close(value: float, compare_to: float, max_diff: float = 0.1) -> bool:
    return abs(value - compare_to) <= max_diff


kinematics = ArmKinematics3(
    1,
    1,
    RotationRange(Rotation2d(0), Rotation2d.fromDegrees(90)),
    RotationRange(Rotation2d.fromDegrees(-80), Rotation2d.fromDegrees(80)),
    RotationRange(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90)),
    ArmResolution.HIGH,
)

assert not RotationRange(Rotation2d(0), Rotation2d.fromDegrees(90)).in_range(
    Rotation2d.fromDegrees(-1)
)

assert RotationRange(Rotation2d(0), Rotation2d.fromDegrees(90)).in_range(
    Rotation2d.fromDegrees(45)
)


assert not RotationRange(Rotation2d(0), Rotation2d.fromDegrees(90)).in_range(
    Rotation2d.fromDegrees(91)
)

# this is (x position, y position, wrist angle, whether the test should cause an error)
options = [
    (0, 2, Rotation2d.fromDegrees(0), False),
    (2, 0, Rotation2d.fromDegrees(0), False),
    (sqrt(2) - 0.1, sqrt(2) - 0.1, Rotation2d.fromDegrees(0), False),
    (sqrt(2), sqrt(2), Rotation2d.fromDegrees(0), False),
    (sqrt(2) - 0.1, sqrt(2) - 0.1, Rotation2d.fromDegrees(45), False),
    (sqrt(2) + 0.1, sqrt(2) + 0.1, Rotation2d.fromDegrees(45), True),
    (25, 25, Rotation2d(0), True),
]

for x, y, wrist, should_err in options:
    try:
        out = kinematics.get_angles(x, y, wrist)
        print(
            f"({x}, {y}, {wrist.degrees()} deg) ",
            out[0].degrees(),
            out[1].degrees(),
            out[2].degrees(),
        )

        assert pretty_close(
            kinematics.l1 * out[0].cos()
            + kinematics.l2 * out[1].rotateBy(out[0]).cos(),
            x,
        )

        assert pretty_close(
            kinematics.l1 * out[0].sin()
            + kinematics.l2 * out[1].rotateBy(out[0]).sin(),
            y,
        )

        assert pretty_close(
            (out[0].degrees() + out[1].degrees() + out[2].degrees()), wrist.degrees(), 3
        )
    except ValueError as e:
        print(f'({x}, {y}) threw an error: "{e}"')
        assert should_err
