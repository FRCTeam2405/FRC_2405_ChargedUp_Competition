// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.grip;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.subsystems.intake.Grip;

public class IntakePiece extends CommandBase {

  private Grip intake;

  /** Creates a new IntakePiece. */
  public IntakePiece(Grip setIntake) {
    intake = setIntake;
    // Declare subsystem dependencies.
    addRequirements(intake);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.driveGrip(Constants.Intake.Speed.INTAKE);
  }

  @Override
  public void end(boolean interrupted) {
    intake.driveGrip(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The controller should handle cancelling the command via whileHeld
    return false;
  }
}
