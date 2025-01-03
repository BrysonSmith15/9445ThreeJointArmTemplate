# Description
This is supposed to be a simple tool that can be used by pgogrammers to create and control a subsystem that is an arm with three joints. The shoulder joint, elbow joint, and wrist joint.
The util/arm_kinematics.py file determines how the arm will move given a desired x and y position and wrist angle. This allows for simple control of the arm. It is tested in tests/kinematics_test.py.
The subsystems/arm.py file actually controls the arm. It should have 3 Spark Max controlled motors and 3 CANCoders to match the template perfectly. Ideally, only the constructor should be modified in this file unless you need to change the motor controllers or encoders. Read the comments in the file to understand what to change.
