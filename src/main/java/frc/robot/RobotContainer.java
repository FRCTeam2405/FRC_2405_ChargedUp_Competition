// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.SetLEDLights;
import frc.robot.settings.Constants;
import frc.robot.subsystems.Lights;

public class RobotContainer {
  final Lights m_Lights = new Lights();
  final SetLEDLights cmdLights = new SetLEDLights(m_Lights, SmartDashboard.getNumber("LEDSet", Constants.LEDs.Colors.LED_SETTING_DEFAULT));

  public RobotContainer() {
    configureBindings();
    m_Lights.setDefaultCommand(cmdLights);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
}
