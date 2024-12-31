from wpilib import TimedRobot, run
from wpimath.geometry import Rotation2d
from util.arm_kinematics import ArmKinematics3, ArmResolution, RotationRange

from math import sqrt


class Robot(TimedRobot):
    # Initialize Robot
    def robotInit(self):
        pass

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
