// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.intake.Arm;
import frc.robot.subsystems.intake.Grip;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlacePiece extends SequentialCommandGroup {
  /** Creates a new PlacePiece. */
  public PlacePiece(Arm arm, Grip grip, Lights lights) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoPlaceLow(arm, lights), new AutoOutputPiece(grip), new AutoCollapseArm(arm, lights));
  }
}
