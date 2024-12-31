import wpilib

from ntcore import NetworkTable, NetworkTableInstance, Event, EventFlags, ValueEventData

from commands2 import Subsystem

from rev import CANSparkMax, CANSparkLowLevel, SparkPIDController, SparkRelativeEncoder

from phoenix6.hardware import CANcoder
from phoenix6.configs import MagnetSensorConfigs
from phoenix6.signals import AbsoluteSensorRangeValue, SensorDirectionValue


class Arm(Subsystem):
    shoulder_motor: CANSparkMax
    shoulder_absolute_encoder: CANcoder
    shoulder_relative_encoder: SparkRelativeEncoder
    shoulder_pid: SparkPIDController

    # if you have more than one shoulder motor, define them here.
    # they can be named as you want, but later examples will use the standard shoulder_motor2, 3, etc.
    # you will still only use one PID and encoder unless something is really wacky (it should not be)

    elbow_motor: CANSparkMax
    elbow_absolute_encoder: CANcoder
    elbow_relative_encoder: SparkRelativeEncoder
    elbow_pid: SparkPIDController

    # same here with multiple wrist motors

    wrist_motor: CANSparkMax
    wrist_absolute_encoder: CANcoder
    wrist_relative_encoder: SparkRelativeEncoder
    wrist_pid: SparkPIDController

    # same with multiple wrist motors as with elbow and shoulder

    nettable: NetworkTable

    # these are to take actions every mod_max iterations over periodic.
    mod_counter: int
    mod_max: int

    def __init__(self) -> None:
        super().__init__()
        self.nettable = NetworkTableInstance.getDefault().getTable("Arm")
        """shoulder constructor"""
        # ID Must change
        # If using Neos/other brushless, these can stay the same.
        # CIM motors or similar will be CANSparkLowLevel.MotorType.kBrushed
        self.shoulder_motor = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
        self.shoulder_pid = self.shoulder_motor.getPIDController()
        self.shoulder_relative_encoder = self.shoulder_motor.getEncoder()
        # ID must change
        # canbus should only change if another canbus is installed on the robot other than the roborio
        self.shoulder_absolute_encoder = CANcoder(11)
        # if you have multiple shoulder motors, create them here.
        """
        self.shoulder_motor2 = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)
        # this will make the shoulder_motor2 follow the first shoulder_motor. If you need to invert the motor, there is a parameter for that.
        _ = self.shoulder_motor2.follow(self.shoulder_motor2)
        # follow the same example for the elbow and wrist motors if applicable. They will not have examples present
        """
        """elbow constructor"""
        # ID Must change
        # If using Neos/other brushless, these can stay the same.
        # CIM motors or similar will be CANSparkLowLevel.MotorType.kBrushed
        self.elbow_motor = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)
        self.elbow_pid = self.elbow_motor.getPIDController()
        self.elbow_relative_encoder = self.elbow_motor.getEncoder()
        # ID must change
        # canbus should only change if another canbus is installed on the robot other than the roborio
        self.elbow_absolute_encoder = CANcoder(13)
        """wrist constructor"""
        # ID Must change
        # If using Neos/other brushless, these can stay the same.
        # CIM motors or similar will be CANSparkLowLevel.MotorType.kBrushed
        self.wrist_motor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)
        self.wrist_pid = self.elbow_motor.getPIDController()
        self.wrist_relative_encoder = self.elbow_motor.getEncoder()
        # ID must change
        # canbus should only change if another canbus is installed on the robot other than the roborio
        self.wrist_absolute_encoder = CANcoder(15)

        """misc member variables"""
        self.mod_counter = 0
        # if you do not want to do encoder checks every second, change this.
        # self.periodic runs 50 times per second.
        # This variable represents how many periodic loops must run for the checks to happen
        self.mod_max = 50

        """configure CANCoders"""
        # if you change any of these, you must fix the later code based on the changes. Only do this if you have a good reason
        status = self.shoulder_absolute_encoder.configurator.apply(
            MagnetSensorConfigs()
            .with_absolute_sensor_range(AbsoluteSensorRangeValue.UNSIGNED_0_TO1)
            .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        )

        if status.is_warning():
            wpilib.reportWarning(
                f"Creating shoulder CANcoder on Arm generated {status.name} with description {status.description}"
            )
        elif status.is_error():
            wpilib.reportError(
                f"Creating shoulder CANcoder on Arm generated {status.name} with description {status.description}"
            )

        # if you change any of these, you must fix the later code based on the changes. Only do this if you have a good reason
        status = self.elbow_absolute_encoder.configurator.apply(
            MagnetSensorConfigs()
            .with_absolute_sensor_range(AbsoluteSensorRangeValue.UNSIGNED_0_TO1)
            .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        )

        if status.is_warning():
            wpilib.reportWarning(
                f"Creating elbow CANcoder on Arm generated {status.name} with description {status.description}"
            )
        elif status.is_error():
            wpilib.reportError(
                f"Creating elbow CANcoder on Arm generated {status.name} with description {status.description}"
            )

        # if you change any of these, you must fix the later code based on the changes. Only do this if you have a good reason
        status = self.wrist_absolute_encoder.configurator.apply(
            MagnetSensorConfigs()
            .with_absolute_sensor_range(AbsoluteSensorRangeValue.UNSIGNED_0_TO1)
            .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        )

        if status.is_warning():
            wpilib.reportWarning(
                f"Creating wrist CANcoder on Arm generated {status.name} with description {status.description}"
            )
        elif status.is_error():
            wpilib.reportError(
                f"Creating wrist CANcoder on Arm generated {status.name} with description {status.description}"
            )

        """Confirgure PID Controllers"""
        # we ignore these errors because they should only occur if there is not connection with the sparkmax.
        # this will already put the program into a state which can not be recovered. As a result, these are always OK.
        # the gains will be set to sensible defaults, though you almost certaintly need to tune them
        # you do need to set these in code after tuning with networktables
        _ = self.shoulder_pid.setP(0.1)
        _ = self.shoulder_pid.setI(0.0)
        _ = self.shoulder_pid.setD(0.01)

        _ = self.elbow_pid.setP(0.1)
        _ = self.elbow_pid.setI(0.0)
        _ = self.elbow_pid.setD(0.01)

        _ = self.wrist_pid.setP(0.1)
        _ = self.wrist_pid.setI(0.0)
        _ = self.wrist_pid.setD(0.01)

        def pid_listener(_nttable: NetworkTable, key: str, event: Event) -> None:
            # this is the listener for the pid controllers. It sets the gain values over network tables
            # this allows for easier tuning without redeploys
            if isinstance((ev := event.data), ValueEventData):
                *_, subtable, subkey = key.split("/")
                pid: SparkPIDController
                if subtable == "ShoulderPID":
                    pid = self.shoulder_pid
                elif subtable == "ElbowPID":
                    pid = self.elbow_pid
                elif subtable == "WristPID":
                    pid = self.wrist_pid
                else:
                    wpilib.reportError(
                        f"The networktable listener for the arm pid got a bad subtable: {subtable}"
                    )
                    return

                if ev.value.isFloat():
                    new_value = ev.value.getFloat()
                    if subkey == "P":
                        _ = pid.setP(new_value)
                    elif subkey == "I":
                        _ = pid.setI(new_value)
                    elif subkey == "D":
                        _ = pid.setI(new_value)
                    else:
                        wpilib.reportError(
                            f"The networktable listener for the arm pid got a bad subkey: {subkey}"
                        )

                else:
                    wpilib.reportWarning(
                        f"The networktable listener for the arm got data other than a float: {ev.value}"
                    )

        _ = self.nettable.addListener(
            "ShoulderPID/P", EventFlags.kValueAll, pid_listener
        )
        _ = self.nettable.addListener(
            "ShoulderPID/I", EventFlags.kValueAll, pid_listener
        )
        _ = self.nettable.addListener(
            "ShoulderPID/D", EventFlags.kValueAll, pid_listener
        )

        _ = self.nettable.addListener("ElbowPID/P", EventFlags.kValueAll, pid_listener)
        _ = self.nettable.addListener("ElbowPID/I", EventFlags.kValueAll, pid_listener)
        _ = self.nettable.addListener("ElbowPID/D", EventFlags.kValueAll, pid_listener)
        _ = self.nettable.addListener("WristPID/P", EventFlags.kValueAll, pid_listener)
        _ = self.nettable.addListener("WristPID/I", EventFlags.kValueAll, pid_listener)
        _ = self.nettable.addListener("WristPID/D", EventFlags.kValueAll, pid_listener)

        _ = self.nettable.setDefaultNumber("ShoulderPID/P", self.shoulder_pid.getP())
        _ = self.nettable.setDefaultNumber("ShoulderPID/I", self.shoulder_pid.getI())
        _ = self.nettable.setDefaultNumber("ShoulderPID/D", self.shoulder_pid.getD())

        _ = self.nettable.setDefaultNumber("ElbowPID/P", self.elbow_pid.getP())
        _ = self.nettable.setDefaultNumber("ElbowPID/I", self.elbow_pid.getI())
        _ = self.nettable.setDefaultNumber("ElbowPID/D", self.elbow_pid.getD())

        _ = self.nettable.setDefaultNumber("WristPID/P", self.wrist_pid.getP())
        _ = self.nettable.setDefaultNumber("WristPID/I", self.wrist_pid.getI())
        _ = self.nettable.setDefaultNumber("WristPID/D", self.wrist_pid.getD())
