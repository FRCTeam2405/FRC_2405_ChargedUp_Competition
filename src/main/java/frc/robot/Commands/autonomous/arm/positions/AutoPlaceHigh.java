// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.arm.positions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.arm.positions.placing.PlaceHigh;
import frc.robot.commands.intake.arm.positions.placing.PlaceLow;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.intake.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceHigh extends SequentialCommandGroup {
  /** Creates a new AutoPlaceLow. */
  public AutoPlaceHigh(Arm intake, Lights lights) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PlaceHigh(intake, lights),
      Commands.waitSeconds(1.0)
    );
  }
}
