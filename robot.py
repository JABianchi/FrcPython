#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# Imports
import wpilib           # Used to get the joysticks
import wpilib.drive     # Used for the DifferentialDrive class
import rev              # REV library


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.leftDrive = wpilib.PWMSparkMax(0)
        self.rightDrive = wpilib.PWMSparkMax(1)
        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.leftDrive, self.rightDrive
        )
        self.controller = wpilib.XboxController(0)
        self.timer = wpilib.Timer()

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightDrive.setInverted(True)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        if self.timer.get() < 2.0:
            # Drive forwards half speed, make sure to turn input squaring off
            self.robotDrive.arcadeDrive(0.5, 0, squareInputs=False)
        else:
            self.robotDrive.stopMotor()  # Stop robot

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        self.robotDrive.arcadeDrive(
            -self.controller.getLeftY(), -self.controller.getRightX()
        )

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""


if __name__ == "__main__":
    wpilib.run(MyRobot)




# Swerve Example code for Robot.py
#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# import wpilib
# import wpimath
# import wpilib.drive
# import wpimath.filter
# import wpimath.controller
# import drivetrain


# class MyRobot(wpilib.TimedRobot):
#     def robotInit(self) -> None:
#         """Robot initialization function"""
#         self.controller = wpilib.XboxController(0)
#         self.swerve = drivetrain.Drivetrain()

#         # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
#         self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
#         self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
#         self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

#     def autonomousPeriodic(self) -> None:
#         self.driveWithJoystick(False)
#         self.swerve.updateOdometry()

#     def teleopPeriodic(self) -> None:
#         self.driveWithJoystick(True)

#     def driveWithJoystick(self, fieldRelative: bool) -> None:
#         # Get the x speed. We are inverting this because Xbox controllers return
#         # negative values when we push forward.
#         xSpeed = (
#             -self.xspeedLimiter.calculate(
#                 wpimath.applyDeadband(self.controller.getLeftY(), 0.02)
#             )
#             * drivetrain.kMaxSpeed
#         )

#         # Get the y speed or sideways/strafe speed. We are inverting this because
#         # we want a positive value when we pull to the left. Xbox controllers
#         # return positive values when you pull to the right by default.
#         ySpeed = (
#             -self.yspeedLimiter.calculate(
#                 wpimath.applyDeadband(self.controller.getLeftX(), 0.02)
#             )
#             * drivetrain.kMaxSpeed
#         )

#         # Get the rate of angular rotation. We are inverting this because we want a
#         # positive value when we pull to the left (remember, CCW is positive in
#         # mathematics). Xbox controllers return positive values when you pull to
#         # the right by default.
#         rot = (
#             -self.rotLimiter.calculate(
#                 wpimath.applyDeadband(self.controller.getRightX(), 0.02)
#             )
#             * drivetrain.kMaxSpeed
#         )

#         self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())