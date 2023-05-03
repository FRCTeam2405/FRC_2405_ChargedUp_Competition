// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Limelight.Settings.Pipelines;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class ConeNodeAlign extends CommandBase {

  SwerveContainer swerve;
  Limelight limelight;

  boolean pipelineSwitched = false;

  /** Creates a new PoleAlign. */
  public ConeNodeAlign(SwerveContainer swerve, Limelight limelight) {
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

    if(limelight.getDouble("tx") != 0.0) {
      pipelineSwitched = true;
    }

    // The horizontal offset between the crosshair and the closest target
    double ty = limelight.getDouble("ty") + 10.5;

    swerve.driveRaw(
      new Translation2d(0, (ty / 25) * 0.15),
      0,
      false,
      true
    );
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

    return false;
  }
}
