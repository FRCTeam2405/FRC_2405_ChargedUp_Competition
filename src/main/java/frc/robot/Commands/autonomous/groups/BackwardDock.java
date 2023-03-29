// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetLights;
import frc.robot.commands.autonomous.drive.DriveConstant;
import frc.robot.commands.autonomous.drive.balance.MoveTiltBack;
import frc.robot.commands.autonomous.drive.balance.MoveTiltForward;
import frc.robot.settings.Constants.LEDs.Colors;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drivetrains.SwerveContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackwardDock extends SequentialCommandGroup {
  /** Creates a new ForwardDock. */
  public BackwardDock(SwerveContainer swerve, Lights lights) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new SetLights(lights, Colors.SOLID_RED),
      new ParallelRaceGroup(
        new DriveConstant(swerve, -0.15, 0, 0),
        Commands.waitSeconds(1.0)
      ),
      
      new MoveTiltForward(swerve, -0.2),

      new SetLights(lights, Colors.STROBE_RED),
      new ParallelRaceGroup(
        new DriveConstant(swerve, -0.07, 0, 0),
        Commands.waitSeconds(3)
      ),
      
      new SetLights(lights, Colors.GREEN),
      new ParallelRaceGroup(
        new DriveConstant(swerve, 0, 0.03, 0),
        Commands.waitSeconds(0.25)
      )
    );
  }
}
