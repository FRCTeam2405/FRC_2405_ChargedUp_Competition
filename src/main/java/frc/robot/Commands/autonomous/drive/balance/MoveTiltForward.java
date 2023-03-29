// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive.balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class MoveTiltForward extends CommandBase {

  SwerveContainer swerve;
  boolean finished = false;
  double speed;

  /** Creates a new MoveTiltBack. */
  public MoveTiltForward(SwerveContainer swerve, double speed) {
    this.swerve = swerve;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((swerve.getPitch().getDegrees() - 360) < -7) {
      finished = true;
      return;
    }
    swerve.drive(speed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
