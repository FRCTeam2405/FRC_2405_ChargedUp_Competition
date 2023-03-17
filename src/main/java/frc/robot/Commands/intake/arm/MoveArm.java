// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.subsystems.Intake;

public class MoveArm extends CommandBase {

  private Intake intake;
  private DoubleSupplier input;

  /** Creates a new MoveArm. */
  public MoveArm(Intake intake, DoubleSupplier move) {
    this.intake = intake;
    input = move;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.driveArm(input.getAsDouble() * Constants.Intake.Speed.ARM);
    intake.desiredArmPosition = intake.getArmPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.driveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
