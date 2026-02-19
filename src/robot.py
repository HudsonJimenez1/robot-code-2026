#!/usr/bin/env python3

import wpilib
from subsystems import drive
from commands2 import CommandScheduler
from commands2 import Command
from utils.constants import OperatorConstants


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.scheduler = CommandScheduler.getInstance()
        self.driver_controller = wpilib.XboxController(OperatorConstants.DRIVER_XBOX_PORT)
        self.drive = drive.Drive(self.scheduler)
        self.autonomous_command: Command | None = None

    def robotPeriodic(self):
        self.scheduler.run()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def autonomousInit(self):
        self.autonomous_command = self.drive.get_autonomous_command()
        self.autonomous_command.schedule()

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        pass

    def teleopInit(self):
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()
            self.autonomous_command = None

    def teleopPeriodic(self):
        # Right stick controls translation; left X controls robot rotation.
        x_displacement = -self.driver_controller.getRightY()
        y_displacement = -self.driver_controller.getRightX()
        rotation = -self.driver_controller.getLeftX()
        self.drive.drive(
            x_displacement=x_displacement,
            y_displacement=y_displacement,
            rotation=rotation,
            field_oriented=False,
        )

    def teleopExit(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
