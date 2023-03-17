// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.arm.positions.pickup;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.settings.Constants.Intake.Positions;
import frc.robot.settings.Constants.LEDs.Colors;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.intake.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupTipped extends InstantCommand {

  private Arm intake;
  private Lights lights;

  public PickupTipped(Arm intake, Lights lights) {
    this.intake = intake;
    this.lights = lights;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.desiredArmPosition = Positions.Arm.TIPPED;
    intake.desiredWristPosition = Positions.Wrist.TIPPED;

    lights.setColor(Colors.YELLOW);
  }
}
