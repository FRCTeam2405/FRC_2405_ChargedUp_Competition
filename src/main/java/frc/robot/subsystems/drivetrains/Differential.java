// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Differential extends SubsystemBase {

  DifferentialDrive drive;

  /** Creates a new differential drive Subsystem. */
  public Differential() {

    WPI_TalonFX leftParentMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motor.Ports.FRONT_LEFT);
    WPI_TalonFX rightParentMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motor.Ports.FRONT_RIGHT);
    WPI_TalonFX leftChildMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motor.Ports.BACK_LEFT);
    WPI_TalonFX rightChildMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motor.Ports.BACK_RIGHT);

    leftParentMotor.setInverted(Constants.Drivetrains.Differential.Motor.FRONT_LEFT_REVERSED);
    rightParentMotor.setInverted(Constants.Drivetrains.Differential.Motor.FRONT_RIGHT_REVERSED);

    leftChildMotor.follow(leftParentMotor);
    rightChildMotor.follow(rightParentMotor);

    drive = new DifferentialDrive(leftParentMotor, rightParentMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public DifferentialDrive getDrive() { return drive; }
}
