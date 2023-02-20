// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.subsystems.Intake;

public class OutputPiece extends CommandBase {

  private Intake intake;

  /** Creates a new IntakePiece. */
  public OutputPiece() {
    // Declare subsystem dependencies.
    addRequirements(intake);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.driveGrip(Constants.Intake.OUTPUT_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO!
    return false;
  }
}
