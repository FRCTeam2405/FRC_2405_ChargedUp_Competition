// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Limelight.Settings.Pipelines;
import frc.robot.subsystems.Limelight;

public class EndOnConeAlign extends CommandBase {
  Limelight limelight;
  boolean isFinished;
  /** Creates a new EndOnConeAlign. */
  public EndOnConeAlign(Limelight limelight) {
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.limelight);
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
    double alignPoint = limelight.getDouble("ty") + 9.9;

    if(Math.abs(alignPoint) < Constants.Limelight.CONE_COLUMN_THRESHOLD) {
      isFinished = true;
    }
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
    return isFinished;
  }
}
