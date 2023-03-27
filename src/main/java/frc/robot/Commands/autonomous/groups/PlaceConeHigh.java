// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.arm.AutoOutputPiece;
import frc.robot.commands.autonomous.arm.positions.MoveArmPos;
import frc.robot.commands.autonomous.arm.positions.MoveWristPos;
import frc.robot.commands.intake.grip.CloseGrip;
import frc.robot.commands.intake.grip.IntakePiece;
import frc.robot.commands.intake.grip.OpenGrip;
import frc.robot.settings.Constants.Intake.Positions;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.intake.Arm;
import frc.robot.subsystems.intake.Grip;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeHigh extends SequentialCommandGroup {
  /** Creates a new PlacePieceHigh. */
  public PlaceConeHigh(Arm arm, Grip grip, Lights lights) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveArmPos(arm, Positions.Arm.HIGH),
      Commands.waitSeconds(1.0),
      new MoveWristPos(arm, Positions.Wrist.HIGH),
      Commands.waitSeconds(1.0),
      new OpenGrip(grip),
      new MoveArmPos(arm, Positions.Arm.COLLAPSED),
      Commands.waitSeconds(1.0),
      new MoveWristPos(arm, Positions.Wrist.COLLAPSED),
      Commands.waitSeconds(1.0)
    );
  }
}
