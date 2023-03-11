// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.settings.Constants;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerve.AbsoluteDrive;
import frc.robot.commands.swerve.AbsoluteDrive3Axis;
import frc.robot.settings.Constants;
import frc.robot.settings.DashboardConfig;
import frc.robot.subsystems.drivetrains.SwerveContainer;
import java.util.function.DoubleSupplier;

public class RobotContainer {

  private final DashboardConfig config;

  // Declare subsystems
  private final SwerveContainer swerveDrive;
  final Lights lights = new Lights();

  // Declare controllers
  private XboxController driverController = new XboxController(Constants.Controllers.DRIVER_CONTROLLER_PORT);
  // private Joystick driverStick = new Joystick(Constants.Controllers.DRIVER_JOYSTICK_PORT);
  // private Joystick driverWheel = new Joystick(Constants.Controllers.DRIVER_WHEEL_PORT);

  public RobotContainer() {

    config = new DashboardConfig();

    swerveDrive = new SwerveContainer();

    configureBindings();

    // Set default commands
    swerveDrive.setDefaultCommand(new AbsoluteDrive(
      swerveDrive,
      axisDeadband(driverController, XboxController.Axis.kLeftY.value, Constants.Controllers.joystickDeadband, true),
      axisDeadband(driverController, XboxController.Axis.kLeftX.value, Constants.Controllers.joystickDeadband, true),
      axisDeadband(driverController, XboxController.Axis.kRightX.value, Constants.Controllers.wheelDeadband, true),
      axisDeadband(driverController, XboxController.Axis.kRightY.value, Constants.Controllers.wheelDeadband, true)
    ));
  }

  private DoubleSupplier axisDeadband(XboxController controller, int axis, double deadband, boolean inverted) {
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
}
