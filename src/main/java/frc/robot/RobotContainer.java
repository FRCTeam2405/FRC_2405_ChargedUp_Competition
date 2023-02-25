// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.settings.Constants;
import frc.robot.settings.DashboardConfig;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class RobotContainer {

  private final DashboardConfig config;

  // Declare subsystems
  private final SwerveContainer swerveDrive;

  // Declare controllers
  private XboxController driverController = new XboxController(Constants.Controllers.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {

    config = new DashboardConfig();

    swerveDrive = new SwerveContainer();

    configureBindings();

    // Set default commands
    swerveDrive.setDefaultCommand(new DriveSwerve(swerveDrive, driverController));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
