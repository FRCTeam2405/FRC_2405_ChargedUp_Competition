// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Lights extends SubsystemBase {

  public PWMSparkMax rawLights = new PWMSparkMax(Constants.LEDs.PORT);

  /** Creates a new Lights subsystem for managing the robot LEDs. */
  public Lights() {
    SmartDashboard.putNumber("LEDSet", Constants.LEDs.Colors.LED_SETTING_DEFAULT);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setColor(double color) {
    rawLights.set(color);
  }

  public double getColor() {
    return rawLights.get();
  }

}
