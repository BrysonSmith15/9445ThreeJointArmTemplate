from wpimath.geometry import Rotation2d
from util.arm_kinematics import ArmKinematics3, ArmResolution, RotationRange

import math

kinematics = ArmKinematics3(
    1,
    1,
    RotationRange(Rotation2d(0), Rotation2d.fromDegrees(90)),
    RotationRange(Rotation2d(0), Rotation2d.fromDegrees(90)),
    ArmResolution.HIGH,
)

print(
    kinematics.get_angles(
        math.sqrt(2) - 0.1, math.sqrt(2) - 0.1, Rotation2d.fromDegrees(0)
    )
)

assert True
