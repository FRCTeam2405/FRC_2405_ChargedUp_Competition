// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class MoveX extends CommandBase {

  SwerveContainer swerve;
  Pose2d startPos;
  double distance;
  boolean finished = false;
  boolean positiveDistance;

  /** Creates a new MoveX. */
  public MoveX(SwerveContainer swerve, double distance) {
    this.distance = distance;
    positiveDistance = this.distance >= 0;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = swerve.getPose();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDistance = 
      swerve.getPose().getX()
      - startPos.getX();
    
    if(positiveDistance) {
      if(currentDistance >= distance) {
        finish();
        return;
      }

      swerve.drive(0.15, 0, 0);
      return;
    }

    if(currentDistance <= distance) {
      finish();
      return;
    }

    swerve.drive(-0.15, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  private void finish() {
    swerve.drive(0, 0, 0);
    finished = true;
  }
}
