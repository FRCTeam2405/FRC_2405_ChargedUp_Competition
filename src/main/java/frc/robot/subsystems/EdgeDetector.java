// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class EdgeDetector extends SubsystemBase {

  Ultrasonic frontLeftSensor;
  Ultrasonic frontRightSensor;
  Ultrasonic backLeftSensor;
  Ultrasonic backRightSensor;

  boolean enabled = Constants.EdgeDetection.ENABLED_DEFAULT;

  /** Creates a new EdgeDetector. */
  public EdgeDetector() {

    frontLeftSensor = new Ultrasonic(
      Constants.EdgeDetection.FRONT_LEFT_PING,
      Constants.EdgeDetection.FRONT_LEFT_ECHO
    );
    frontRightSensor = new Ultrasonic(
      Constants.EdgeDetection.FRONT_RIGHT_PING,
      Constants.EdgeDetection.FRONT_RIGHT_ECHO
    );
    backLeftSensor = new Ultrasonic(
      Constants.EdgeDetection.BACK_LEFT_PING,
      Constants.EdgeDetection.BACK_LEFT_ECHO
    );
    backRightSensor = new Ultrasonic(
      Constants.EdgeDetection.BACK_RIGHT_PING,
      Constants.EdgeDetection.BACK_RIGHT_ECHO
    );
    Ultrasonic.setAutomaticMode(enabled);
  }

  public void setEnabled(boolean enable) {
    Ultrasonic.setAutomaticMode(enable);
    enabled = enable;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FrontLeftUltrasonic", frontLeftSensor.getRangeInches());
    SmartDashboard.putNumber("FrontRightUltrasonic", frontRightSensor.getRangeInches());
    SmartDashboard.putNumber("BackLeftUltrasonic", backLeftSensor.getRangeInches());
    SmartDashboard.putNumber("BackRightUltrasonic", backRightSensor.getRangeInches());

    if(enabled) {
      //TODO!
    }
  }
}
