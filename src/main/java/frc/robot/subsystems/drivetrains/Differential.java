// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Differential extends SubsystemBase {

  private WPI_TalonFX leftParentMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motors.Ports.FRONT_LEFT);
  private WPI_TalonFX rightParentMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motors.Ports.FRONT_RIGHT);
  private WPI_TalonFX leftChildMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motors.Ports.BACK_LEFT);
  private WPI_TalonFX rightChildMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motors.Ports.BACK_RIGHT);

  DifferentialDrive drive;

  /** Creates a new differential drive Subsystem. */
  public Differential() {
    
    leftParentMotor.setInverted(Constants.Drivetrains.Differential.Motors.FRONT_LEFT_REVERSED);
    rightParentMotor.setInverted(Constants.Drivetrains.Differential.Motors.FRONT_RIGHT_REVERSED);
    leftChildMotor.setInverted(Constants.Drivetrains.Differential.Motors.BACK_LEFT_REVERSED);
    rightChildMotor.setInverted(Constants.Drivetrains.Differential.Motors.BACK_RIGHT_REVERSED);

    leftChildMotor.follow(leftParentMotor);
    rightChildMotor.follow(rightParentMotor);

    drive = new DifferentialDrive(leftParentMotor, rightParentMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DriveArcade (double speedX, double rotationTheta) {
    speedX *= Constants.Drivetrains.Differential.SPEED_LIMIT;
    rotationTheta *= Constants.Drivetrains.Differential.SPEED_LIMIT;

    drive.arcadeDrive(speedX, rotationTheta);
  }
  public void DriveTank (double leftSpeed, double rightSpeed) {
    leftSpeed *= Constants.Drivetrains.Differential.SPEED_LIMIT;
    rightSpeed *= Constants.Drivetrains.Differential.SPEED_LIMIT;

    drive.tankDrive(leftSpeed, rightSpeed);
  } 

}
