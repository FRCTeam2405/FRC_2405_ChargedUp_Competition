// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class DriveConstant extends CommandBase {

  SwerveContainer swerve;

  double moveX;
  double moveY;
  double rotTheta;

  /** Creates a new DriveConstant. */
  public DriveConstant(SwerveContainer swerve, double x, double y, double theta) {
    this.swerve = swerve;

    moveX = x;
    moveY = y;
    rotTheta = theta;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(moveX, moveY, rotTheta);
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
