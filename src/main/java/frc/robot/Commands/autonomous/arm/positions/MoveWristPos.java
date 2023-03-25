// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.arm.positions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveWristPos extends InstantCommand {

  Arm arm;
  double position;

  public MoveWristPos(Arm arm, double pos) {
    this.arm = arm;
    position = pos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.desiredWristPosition = position;
  }
}
