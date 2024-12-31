from wpilib import TimedRobot, run
from wpimath.geometry import Rotation2d
from util.arm_kinematics import ArmKinematics3, ArmResolution, RotationRange

from math import sqrt


class Robot(TimedRobot):
    # Initialize Robot
    def robotInit(self):
        print("\033[1;91m")
        kinematics = ArmKinematics3(
            1,
            1,
            RotationRange(Rotation2d(0), Rotation2d.fromDegrees(90)),
            RotationRange(Rotation2d(0), Rotation2d.fromDegrees(90)),
            ArmResolution.LOW,
        )
        options = [
            (0, 2, Rotation2d.fromDegrees(0)),
            (2, 0, Rotation2d.fromDegrees(0)),
            (sqrt(2) - 0.1, sqrt(2) - 0.1, Rotation2d.fromDegrees(0)),
            (sqrt(2) - 0.1, sqrt(2) - 0.1, Rotation2d.fromDegrees(45)),
            (sqrt(2) + 0.1, sqrt(2) + 0.1, Rotation2d.fromDegrees(45)),
        ]
        for x, y, wrist in options:
            try:
                out = kinematics.get_angles(x, y, wrist)
                print(
                    f"({x}, {y}, {wrist.degrees()} deg) ",
                    out[0].degrees(),
                    out[1].degrees(),
                    out[2].degrees(),
                )
            except ValueError as e:
                print(f'({x}, {y}) threw an error: "{e}"')

        print("\033[0;0m")
        exit(0)

    def robotPeriodic(self) -> None:
        pass

    # Autonomous Robot Functions
    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        # self.m_autonomousCommand.cancel()
        ...

    # Teleop Robot Functions
    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        pass

    def teleopExit(self):
        pass

    # Test Robot Functions
    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass

    # Disabled Robot Functions
    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def SimulationPeriodic(self) -> None:
        pass


# Start the Robot when Executing Code
if __name__ == "__main__":
    run(Robot)
