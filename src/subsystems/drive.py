from __future__ import annotations

import importlib
from typing import Callable

from commands2 import Command, InstantCommand
from commands2 import CommandScheduler
import wpilib

from utils.constants import DriveConstants

try:
    from phoenix6.controls import DutyCycleOut
    from phoenix6.hardware import TalonFX
except ModuleNotFoundError as exc:  # pragma: no cover - depends on local install
    DutyCycleOut = None  # type: ignore[assignment]
    TalonFX = None  # type: ignore[assignment]
    _PHOENIX_IMPORT_ERROR = exc
else:
    _PHOENIX_IMPORT_ERROR = None


class Drive:
    def __init__(self, scheduler: CommandScheduler | None = None):
        if TalonFX is None or DutyCycleOut is None:
            raise RuntimeError(
                "Phoenix 6 is required for swerve bring-up. "
                "Install project dependencies first."
            ) from _PHOENIX_IMPORT_ERROR

        self.scheduler = scheduler
        self.tuner_drivetrain: object | None = self._try_create_tuner_drivetrain()
        self._autonomous_factory: Callable[..., Command] | None = (
            self._try_create_tuner_auto_factory()
        )

        self.drive_motors: list[TalonFX] = []
        self.steer_motors: list[TalonFX] = []
        self._init_hardware()

    def _init_hardware(self) -> None:
        for module in DriveConstants.MODULES:
            self.drive_motors.append(
                TalonFX(module.drive_motor_id, DriveConstants.CANBUS_NAME)
            )
            self.steer_motors.append(
                TalonFX(module.steer_motor_id, DriveConstants.CANBUS_NAME)
            )

    def _try_create_tuner_drivetrain(self) -> object | None:
        # Tuner-generated projects often place this class in one of these modules.
        candidates = (
            ("generated.command_swerve_drivetrain", "CommandSwerveDrivetrain"),
            ("subsystems.command_swerve_drivetrain", "CommandSwerveDrivetrain"),
            ("command_swerve_drivetrain", "CommandSwerveDrivetrain"),
        )
        for module_name, class_name in candidates:
            try:
                module = importlib.import_module(module_name)
            except ModuleNotFoundError:
                continue

            drivetrain_type = getattr(module, class_name, None)
            if drivetrain_type is None:
                continue

            try:
                return drivetrain_type()
            except Exception as exc:  # pragma: no cover - depends on generated code
                wpilib.DriverStation.reportWarning(
                    f"Found {module_name}.{class_name} but failed to construct it: {exc}",
                    False,
                )
                return None
        return None

    def _try_create_tuner_auto_factory(self) -> Callable[..., Command] | None:
        candidates = (
            ("subsystems.tuner_autonomous", "build_autonomous_command"),
            ("generated.tuner_autonomous", "build_autonomous_command"),
            ("tuner_autonomous", "build_autonomous_command"),
        )
        for module_name, fn_name in candidates:
            try:
                module = importlib.import_module(module_name)
            except ModuleNotFoundError:
                continue

            factory = getattr(module, fn_name, None)
            if callable(factory):
                return factory
        return None

    def _apply_deadband(self, value: float, deadband: float) -> float:
        if abs(value) < deadband:
            return 0.0
        return value

    def _clamp(self, value: float) -> float:
        return max(-1.0, min(1.0, value))

    def drive(
        self,
        x_displacement: float,
        y_displacement: float,
        rotation: float,
        field_oriented: bool = False,
    ) -> None:
        if self.tuner_drivetrain is not None:
            # If your Tuner drivetrain exposes a drive(...) function, prefer it.
            tuner_drive = getattr(self.tuner_drivetrain, "drive", None)
            if callable(tuner_drive):
                try:
                    tuner_drive(x_displacement, y_displacement, rotation, field_oriented)
                    return
                except TypeError:
                    pass

        x = self._apply_deadband(x_displacement, DriveConstants.TRANSLATION_DEADBAND)
        y = self._apply_deadband(y_displacement, DriveConstants.TRANSLATION_DEADBAND)
        omega = self._apply_deadband(rotation, DriveConstants.ROTATION_DEADBAND)

        x *= DriveConstants.MAX_TRANSLATION_OUTPUT
        y *= DriveConstants.MAX_TRANSLATION_OUTPUT
        omega *= DriveConstants.MAX_ROTATION_OUTPUT

        # Open-loop bring-up mix while waiting on full swerve request wiring.
        drive_outputs = (
            self._clamp(x + y + omega),  # front left
            self._clamp(x - y - omega),  # front right
            self._clamp(x - y + omega),  # back left
            self._clamp(x + y - omega),  # back right
        )
        for motor, output in zip(self.drive_motors, drive_outputs):
            motor.set_control(DutyCycleOut(output))

        # Steer motors are initialized and controlled directly for basic validation.
        steer_output = self._clamp(omega)
        for motor in self.steer_motors:
            motor.set_control(DutyCycleOut(steer_output))

    def stop(self) -> None:
        for motor in self.drive_motors:
            motor.set_control(DutyCycleOut(0.0))
        for motor in self.steer_motors:
            motor.set_control(DutyCycleOut(0.0))

    def get_autonomous_command(self) -> Command:
        if self.tuner_drivetrain is not None:
            getter = getattr(self.tuner_drivetrain, "get_autonomous_command", None)
            if callable(getter):
                command = getter()
                if command is not None and hasattr(command, "schedule"):
                    return command

        if self._autonomous_factory is not None:
            try:
                command = self._autonomous_factory()
            except TypeError:
                command = self._autonomous_factory(self)  # type: ignore[misc]

            if command is not None and hasattr(command, "schedule"):
                return command

        wpilib.DriverStation.reportWarning(
            "No Phoenix 6 Tuner autonomous command source found; using stop command.",
            False,
        )
        return InstantCommand(self.stop)
