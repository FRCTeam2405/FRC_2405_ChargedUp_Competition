// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class DriveSwerve extends CommandBase {

  SwerveContainer swerveDrive;
  XboxController controller;

  /** Creates a new DriveSwerve. */
  public DriveSwerve(SwerveContainer swerve, XboxController xboxController) {

    swerveDrive = swerve;
    controller = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(
      controller.getLeftY(),
      controller.getLeftX(),
      controller.getRightX()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
