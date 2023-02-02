// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.settings.Constants;
import frc.robot.subsystems.drivetrains.Swerve;

public class RobotContainer {

  // Declare subsystems
  private final Swerve swerveDrive;

  // Declare controllers
  private XboxController driverController = new XboxController(Constants.Controllers.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {

    swerveDrive = new Swerve();

    swerveDrive.navX.calibrate();

    configureBindings();

    // Set default commands
    swerveDrive.setDefaultCommand(getSwerveCommand());
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getSwerveCommand() {
    return new RunCommand(
      () ->
        swerveDrive.drive(
          // In WPILib, the front of the robot faces positive X
          // in the coordinate plane. However, the Xbox joystick's
          // X axis is horizontal (when held correctly), so to make
          // movement intuitive, we must swap the X and Y axes in the input.
          driverController.getLeftX(),
          driverController.getLeftY(),
          driverController.getRightX(),
          Constants.Drivetrains.Swerve.FIELD_RELATIVE
        ),
      swerveDrive
    );
  }
}
