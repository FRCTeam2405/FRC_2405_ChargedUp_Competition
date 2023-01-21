// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Differential extends SubsystemBase {

  DifferentialDrive drive;

  /** Creates a new differential drive Subsystem. */
  public Differential() {

    MotorController frontLeftMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motor.Ports.FRONT_LEFT);
    MotorController frontRightMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motor.Ports.FRONT_RIGHT);
    MotorController backLeftMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motor.Ports.BACK_LEFT);
    MotorController backRightMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motor.Ports.BACK_RIGHT);

    MotorControllerGroup leftMotors = new MotorControllerGroup(
      frontLeftMotor,
      backLeftMotor
    );
    MotorControllerGroup rightMotors = new MotorControllerGroup(
      frontRightMotor,
      backRightMotor
    );

    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
