// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Limelight.Settings.Pipelines;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class PoleAlign extends CommandBase {

  SwerveContainer swerve;
  Limelight limelight;

  /** Creates a new PoleAlign. */
  public PoleAlign(SwerveContainer swerve, Limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve, this.limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the limelight to track reflective tape
    limelight.setVisionEnabled(true);
    limelight.setPipeline(Pipelines.REFLECTIVE_TAPE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The horizontal offset between the crosshair and the closest target
    double tx = limelight.getDouble("tx");

    // Sign is negative if tx is negative, vice versa
    double sign = tx < 0 ? -1 : 1;

    swerve.drive(0, (0.1 * sign), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Return to driver vision
    limelight.setVisionEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = Math.abs(limelight.getDouble("tx")) < Constants.Limelight.POLE_THRESHOLD;
    return finished;
  }
}
