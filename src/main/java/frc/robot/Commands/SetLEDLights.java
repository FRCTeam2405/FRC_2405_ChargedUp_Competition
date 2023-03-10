// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.subsystems.Lights;

public class SetLEDLights extends CommandBase {
  /** Creates a new SetLEDLights. */
    final Lights sysLights;
    final double LEDsetting;
    
    public SetLEDLights(Lights inSysLights, double inLEDsetting) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysLights = inSysLights;
    LEDsetting = inLEDsetting;
  
    // Use addRequirements() here to declare subsystem dependencies.  
    addRequirements(inSysLights);
    }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sysLights.rawLights.set(SmartDashboard.getNumber("LEDSet", Constants.LEDs.Colors.LED_SETTING_DEFAULT));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysLights.rawLights.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (sysLights.rawLights.get() == SmartDashboard.getNumber("LEDSet", Constants.LEDs.Colors.LED_SETTING_DEFAULT))
      return true; 
    else
      return false;
  }
}


