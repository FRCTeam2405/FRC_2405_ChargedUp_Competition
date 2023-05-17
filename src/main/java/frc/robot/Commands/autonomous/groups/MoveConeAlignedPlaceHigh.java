// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.vision.EndOnConeAlign;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.intake.Arm;
import frc.robot.subsystems.intake.Grip;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveConeAlignedPlaceHigh extends SequentialCommandGroup {
  /** Creates a new LockMoveConeHigh. */
  public MoveConeAlignedPlaceHigh(Command driveCommand, Limelight limelight, Arm arm, Grip grip) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        driveCommand,
        new EndOnConeAlign(limelight)
      ),
      new PlaceConeHigh(arm, grip)
    );
  }
}
