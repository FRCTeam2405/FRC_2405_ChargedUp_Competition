// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Lights;

//TODO! Test this
// More concise version of SetLEDLights, but WIP
public class SetLights extends InstantCommand {

  Lights lights;
  double color;

  public SetLights(Lights lights, double setColor) {

    this.lights = lights;
    this.color = setColor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lights.setColor(color);
  }
}
