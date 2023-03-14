// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.EdgeDetector;
import frc.robot.subsystems.drivetrains.Differential;
import frc.robot.commands.SetLEDLights;
import frc.robot.settings.Constants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private Differential differentialDrive;
  private EdgeDetector edgeDetector;
  private Limelight limelight;

  // Declare controllers
  // private Joystick driverStick = new Joystick(Constants.Controllers.DRIVER_JOYSTICK_PORT);
  // private Joystick driverWheel = new Joystick(Constants.Controllers.DRIVER_WHEEL_PORT);

  private CommandXboxController driverController = new CommandXboxController(0);
  private CommandXboxController codriverController = new CommandXboxController(1);

  public RobotContainer() {

    config = new DashboardConfig();

    swerveDrive = new SwerveContainer();
    limelight = new Limelight();

    configureBindings();
    edgeDetector = new EdgeDetector();

    // Set default commands
    
  }

  private DoubleSupplier axisDeadband(CommandXboxController controller, int axis, double deadband, boolean inverted) {
    double invertedMultiplier = inverted ? -1.0 : 1.0;
    return () -> (
      Math.abs(controller.getRawAxis(axis)) > deadband
    ) ? controller.getRawAxis(axis) * invertedMultiplier : 0;
  }

  private void configureBindings() {
    // driverController = new XboxController(0); //TODO! convert to constant

    // Driver Controls

    // Driving the robot: left stick for movement, right stick for turning
    swerveDrive.setDefaultCommand(new AbsoluteDrive3Axis(
      swerveDrive,
      axisDeadband(driverController, XboxController.Axis.kLeftY.value, Constants.Controllers.joystickDeadband, true),
      axisDeadband(driverController, XboxController.Axis.kLeftX.value, Constants.Controllers.joystickDeadband, true),
      axisDeadband(driverController, XboxController.Axis.kRightX.value, Constants.Controllers.wheelDeadband, true)
    ));

    // Codriver Controls
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
