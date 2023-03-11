// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.objects.vision.GamePiece;
import frc.robot.objects.vision.PieceType;
import frc.robot.subsystems.Vision;

public class Hunt extends CommandBase {

  Vision vision;

  /** Creates a new Hunt. */
  public Hunt(Vision vision) {
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GamePiece[] pieces = vision.getGamePieces(PieceType.CONE);
    
    
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
