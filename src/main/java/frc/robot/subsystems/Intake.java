// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Intake extends SubsystemBase {

  public double desiredArmPosition;
  public double desiredWristPosition;

  WPI_TalonFX armMotor;

  CANSparkMax wristMotor;
  SparkMaxPIDController wristPID;

  CANSparkMax leftGripMotor;
  CANSparkMax rightGripMotor;

  Solenoid gripSolenoids;

  /** Creates a new Intake. */
  public Intake() {
    
    desiredArmPosition = Constants.Intake.Positions.LOW_ARM;
    desiredWristPosition = Constants.Intake.Positions.COLLAPSED_WRIST;
    armMotor = new WPI_TalonFX(Constants.Intake.Ports.ARM_MOTOR);

    wristMotor = new CANSparkMax(Constants.Intake.Ports.WRIST_MOTOR, MotorType.kBrushless);
    wristPID = wristMotor.getPIDController();

    //TODO! settings

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
    //TODO! do gear ratio and encoder res
    armMotor.set(ControlMode.Position, desiredArmPosition);
    wristPID.setReference((desiredWristPosition / 360), ControlType.kPosition);
  }

  public void driveGrip(double speed) {
    leftGripMotor.set(speed);
    rightGripMotor.set(speed);
  }

  public boolean getGripOpen() { return gripSolenoids.get(); }
  public void setGripOpen(boolean open) { gripSolenoids.set(open); }
}
