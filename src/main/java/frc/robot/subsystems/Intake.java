// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Intake extends SubsystemBase {

  public double desiredArmPosition;
  public double desiredWristPosition;

  WPI_TalonFX armMotor;
  PIDController armPID;

  CANSparkMax wristMotor;
  SparkMaxPIDController wristPID;

  CANSparkMax leftGripMotor;
  CANSparkMax rightGripMotor;

  Solenoid gripSolenoids;

  /** Creates a new Intake. */
  public Intake() {
    
    desiredArmPosition = Constants.Intake.Positions.Arm.COLLAPSED;
    desiredWristPosition = Constants.Intake.Positions.Wrist.COLLAPSED;

    armMotor = new WPI_TalonFX(Constants.Intake.Ports.ARM_MOTOR);
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    armPID = new PIDController(
      0.000005,
      0.0000000001,
      0
    );

    armMotor.config_kP(0, 0.000005);

    wristMotor = new CANSparkMax(Constants.Intake.Ports.WRIST_MOTOR, MotorType.kBrushless);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristPID = wristMotor.getPIDController();

    wristPID.setP(0.5);
    wristPID.setI(0.0001);
    wristPID.setD(0.0);
    wristPID.setFF(0);

    wristPID.setOutputRange(-0.2, 0.2);

    wristMotor.burnFlash();

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

    SmartDashboard.putNumber("wristPos", wristMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("armPosition", armMotor.getSelectedSensorPosition(0));

    armPID.setSetpoint(desiredArmPosition);
    double calculation = armPID.calculate(armMotor.getSelectedSensorPosition(), desiredArmPosition);
    armMotor.set(ControlMode.PercentOutput, MathUtil.clamp(calculation, -1, 1));

    wristPID.setReference(desiredWristPosition, ControlType.kPosition);
  }

  public void driveGrip(double speed) {
    leftGripMotor.set(speed);
    rightGripMotor.set(-speed);
  }

  public void driveArm(double speed) {
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  public void driveWrist(double speed) {
    wristMotor.set(speed);
  }

  public boolean getGripOpen() { return gripSolenoids.get(); }
  public void setGripOpen(boolean open) { gripSolenoids.set(open); }
}
