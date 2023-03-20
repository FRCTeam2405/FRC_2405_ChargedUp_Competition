// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class Autobalance2 extends CommandBase {

  PIDController pitchPID;
  PIDController rollPID;

  SwerveContainer swerve;

  /** Creates a new Autobalance2. */
  public Autobalance2(SwerveContainer drive) {
    swerve = drive;

    pitchPID = new PIDController(
      0.05,
      0.0001,
      0
    );
    pitchPID.setSetpoint(0);
    pitchPID.setTolerance(2.5);

    rollPID = new PIDController(
      0.05,
      0.0001,
      0
    );
    rollPID.setSetpoint(0);
    rollPID.setTolerance(2.5);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Rotation2d pitch = swerve.getPitch();
    Rotation2d roll = swerve.getRoll();

    double moveX = pitchPID.calculate(pitch.getDegrees());
    double moveY = rollPID.calculate(roll.getDegrees());

    if(pitchPID.atSetpoint()) {
      moveX = 0;
    }
    if(rollPID.atSetpoint()) {
      moveY = 0;
    }

    swerve.driveRaw(new Translation2d(moveX, moveY), 0, false, true);
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
