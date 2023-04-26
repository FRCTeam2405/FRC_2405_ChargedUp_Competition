// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Limelight.Settings.CameraMode;
import frc.robot.settings.Constants.Limelight.Settings.Pipelines;

public class Limelight extends SubsystemBase {

  NetworkTable limelight;

  /** Creates a new Limelight. */
  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    limelight.getEntry("pipeline").setNumber(Pipelines.LED_OFF);
    limelight.getEntry("camMode").setNumber(CameraMode.DRIVER_VISION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVisionEnabled(boolean enabled) {
    if(enabled) {
      limelight.getEntry("camMode").setNumber(CameraMode.AUTO_VISION);
    } else {
      limelight.getEntry("pipeline").setNumber(Pipelines.LED_OFF);
      limelight.getEntry("camMode").setNumber(CameraMode.DRIVER_VISION);
    }
  }

  public void setPipeline(int pipeline) {
    if(pipeline > 10 || pipeline < 0) {
      return;
    }

    limelight.getEntry("pipeline").setNumber(pipeline);
  }

  public double getDouble(String variable) {
    return limelight.getEntry(variable).getDouble(0);
  }
}
