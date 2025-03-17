#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine
import wpilib

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1) # Add a 10% deadband to the X and Y Axies
            .with_rotational_deadband(
                self._max_angular_rate * 0.15
            )  # Add a 15% deadband to the Z Axis
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self._joystick = wpilib.Joystick(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        self._servo = wpilib.Servo(0)

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._joystick.getRawAxis(1) * self._max_speed #/ 2
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.getRawAxis(0) * self._max_speed #/ 2
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.getRawAxis(2) * self._max_angular_rate #/ 2
                    )  # Drive counterclockwise with negative Z (left)
                )
            )
        )

        while self._joystick.getRawButton(1) == True:
            self.drivetrain.apply_request(lambda: self._brake())
            
        while self._joystick.getRawButton(2) == True:
            self.drivetrain.apply_request(
                lambda: self._point(
                    Rotation2d(-self._joystick.getRawAxis(1), -self._joystick.getRawAxis(0)
                    )
                )
            )
                

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        #(self._joystick.back() & self._joystick.y()).whileTrue(
        #    self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        #)
        #(self._joystick.back() & self._joystick.x()).whileTrue(
        #    self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        #)
        #(self._joystick.start() & self._joystick.y()).whileTrue(
        #    self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        #)
        #(self._joystick.start() & self._joystick.x()).whileTrue(
        #    self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        #)

        # reset the field-centric heading on left bumper press
        #if self._joystick.getRawButtonPressed(0):
        #    self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())

        #self.drivetrain.register_telemetry(
        #    lambda state: self._logger.telemeterize(state)
        #)

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return commands2.cmd.print_("No autonomous command configured")
