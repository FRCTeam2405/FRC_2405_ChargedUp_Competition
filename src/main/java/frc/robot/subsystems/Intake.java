// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Intake extends SubsystemBase {

  public double desiredArmPosition;
  public double desiredWristPosition;

  /** Creates a new Intake. */
  public Intake() {
    desiredArmPosition = Constants.Intake.Positions.LOW_ARM;
    desiredWristPosition = Constants.Intake.Positions.COLLAPSED_WRIST;
  }

  @Override
  public void periodic() {
    //TODO! Drive motors towards desired positions
  }

  public void driveGrip(double speed) {
    //TODO! Drive motors
  }
}
