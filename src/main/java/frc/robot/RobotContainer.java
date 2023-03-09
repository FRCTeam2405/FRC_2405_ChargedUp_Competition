// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerve.AbsoluteDrive3Axis;
import frc.robot.settings.Constants;
import frc.robot.settings.DashboardConfig;
import frc.robot.settings.Constants.Intake;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class RobotContainer {

  private final DashboardConfig config;

  // Declare subsystems
  private final SwerveContainer swerveDrive;
  private final Compressor airCompressor = new Compressor(Intake.Ports.COMPRESSOR, PneumaticsModuleType.CTREPCM);

  // Declare controllers
  private Joystick driverStick = new Joystick(Constants.Controllers.DRIVER_JOYSTICK_PORT);
  private Joystick driverWheel = new Joystick(Constants.Controllers.DRIVER_WHEEL_PORT);

  public RobotContainer() {

    config = new DashboardConfig();

    swerveDrive = new SwerveContainer();

    configureBindings();

    // Set default commands
    swerveDrive.setDefaultCommand(new AbsoluteDrive3Axis(
      swerveDrive,
      axisDeadband(driverStick, Constants.Controllers.Axis.JOYSTICK_Y, Constants.Controllers.joystickDeadband, true),
      axisDeadband(driverStick, Constants.Controllers.Axis.JOYSTICK_X, Constants.Controllers.joystickDeadband, true),
      axisDeadband(driverWheel, Constants.Controllers.Axis.WHEEL_X, Constants.Controllers.wheelDeadband, true)
    ));
  }

  private DoubleSupplier axisDeadband(Joystick controller, int axis, double deadband, boolean inverted) {
    double invertedMultiplier = inverted ? -1.0 : 1.0;
    return () -> (
      Math.abs(controller.getRawAxis(axis)) > deadband
    ) ? controller.getRawAxis(axis) * invertedMultiplier : 0;
  }

  private void configureBindings() {
    // driverController = new XboxController(0); //TODO! convert to constant
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  // public Command getDifferentialCommand() {
  //   return new RunCommand(() -> differentialDrive.DriveTank(
  //       driverController.getLeftY(),
  //       driverController.getRightY()
  //     ),
  //     differentialDrive
  //   );
  // }
}
