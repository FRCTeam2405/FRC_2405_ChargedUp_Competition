// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SetLEDLights;
import frc.robot.settings.Constants;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.IntakePiece;
import frc.robot.commands.intake.OutputPiece;
import frc.robot.commands.intake.ToggleGrip;
import frc.robot.commands.intake.arm.MoveArmHigh;
import frc.robot.commands.intake.arm.MoveArmLow;
import frc.robot.commands.intake.arm.MoveArmMed;
import frc.robot.commands.swerve.AbsoluteDrive3Axis;
import frc.robot.settings.Constants;
import frc.robot.settings.DashboardConfig;
import frc.robot.settings.Constants.Controllers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrains.SwerveContainer;
import java.util.function.DoubleSupplier;

public class RobotContainer {

  private final DashboardConfig config;

  // Declare subsystems
  private final SwerveContainer swerveDrive;
  private final Intake intake;
  final Lights lights = new Lights();
  
  private final Compressor airCompressor = new Compressor(Constants.Intake.Ports.COMPRESSOR, PneumaticsModuleType.CTREPCM);
  

  // Declare controllers
  private Joystick driverStick = new Joystick(Constants.Controllers.DRIVER_JOYSTICK_PORT);
  private Joystick driverWheel = new Joystick(Constants.Controllers.DRIVER_WHEEL_PORT);

  private CommandXboxController codriverController = new CommandXboxController(Controllers.CODRIVER_PORT);

  public RobotContainer() {

    config = new DashboardConfig();

    swerveDrive = new SwerveContainer();
    intake = new Intake();

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
    codriverController.rightTrigger().whileTrue(new IntakePiece(intake));
    codriverController.leftTrigger().whileTrue(new OutputPiece(intake));
    
    codriverController.x().onTrue(new ToggleGrip(intake));
    
    codriverController.a().onTrue(new MoveArmLow(intake, lights));
    codriverController.b().onTrue(new MoveArmMed(intake, lights));
    codriverController.y().onTrue(new MoveArmHigh(intake, lights));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
