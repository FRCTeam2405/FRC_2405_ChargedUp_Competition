// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.arm.positions.placing;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.LEDs.Colors;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceLow extends InstantCommand {

  private Intake intake;
  private Lights lights;

  public PlaceLow(Intake setIntake, Lights setLights) {
    intake = setIntake;
    lights = setLights;
    // Declare subsystem dependencies.
    addRequirements(intake, lights);
  }

  @Override
  public void initialize() {
    intake.desiredArmPosition = Constants.Intake.Positions.Arm.LOW;
    intake.desiredWristPosition = Constants.Intake.Positions.Wrist.LOW;
    lights.setColor(Colors.GREEN);
  }
}
