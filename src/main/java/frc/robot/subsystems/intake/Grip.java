
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.settings.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grip extends SubsystemBase {

  CANSparkMax leftGripMotor;
  CANSparkMax rightGripMotor;

  Solenoid gripSolenoids;

  /** Creates a new Grip. */
  public Grip() {
    leftGripMotor = new CANSparkMax(Constants.Intake.Ports.LEFT_GRIP_MOTOR, MotorType.kBrushless);
    rightGripMotor = new CANSparkMax(Constants.Intake.Ports.RIGHT_GRIP_MOTOR, MotorType.kBrushless);

    gripSolenoids = new Solenoid(
      Constants.Intake.Ports.COMPRESSOR,
      PneumaticsModuleType.CTREPCM,
      Constants.Intake.Ports.SOLENOIDS
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveGrip(double speed) {
    leftGripMotor.set(speed);
    rightGripMotor.set(-speed);
  }

  public boolean getGripOpen() { return gripSolenoids.get(); }
  public void setGripOpen(boolean open) { gripSolenoids.set(open); }
}
