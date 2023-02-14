// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.settings.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

public class SwerveModule {
  private final String moduleName;

  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final RelativeEncoder turningEncoder;

  private final CANCoder canCoder;

  private final SparkMaxPIDController drivingPIDController;
  private final SparkMaxPIDController turningPIDController;

  private double angularOffset;
  private SwerveModuleState desiredModuleState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(String name, int drivingCANId, int turningCANId, double chassisAngularOffset) {

    moduleName = name;

    angularOffset = chassisAngularOffset;

    drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    drivingEncoder = drivingSparkMax.getEncoder();
    drivingEncoder.setPositionConversionFactor(Constants.Drivetrains.Swerve.Module.DRIVING_ENCODER_POSITION_FACTOR);
    drivingEncoder.setVelocityConversionFactor(Constants.Drivetrains.Swerve.Module.DRIVING_ENCODER_VELOCITY_FACTOR);

    //TODO! make sure this is correct
    turningEncoder = turningSparkMax.getEncoder();
    turningEncoder.setPositionConversionFactor(Constants.Drivetrains.Swerve.Module.TURNING_ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(Constants.Drivetrains.Swerve.Module.TURNING_ENCODER_VELOCITY_FACTOR);

    canCoder = new CANCoder(turningCANId);

    drivingPIDController = drivingSparkMax.getPIDController();
    drivingPIDController.setFeedbackDevice(drivingEncoder);

    turningPIDController = turningSparkMax.getPIDController();
    turningPIDController.setFeedbackDevice(turningEncoder);

    //TODO! Tune these
    drivingPIDController.setP(Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_P);
    drivingPIDController.setI(Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_I);
    drivingPIDController.setD(Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_D);
    drivingPIDController.setFF(Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_FF);
    drivingPIDController.setOutputRange(
      Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_MINIMUM_OUTPUT,
      Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_MAXIMUM_OUTPUT
    );

    turningPIDController.setP(Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_P);
    turningPIDController.setI(Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_I);
    turningPIDController.setD(Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_D);
    turningPIDController.setFF(Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_FF);
    turningPIDController.setOutputRange(
      Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_MINIMUM_OUTPUT,
      Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_MAXIMUM_OUTPUT
    );

    drivingSparkMax.setIdleMode(Constants.Drivetrains.Swerve.Module.DRIVING_MOTOR_IDLE_MODE);  
    turningSparkMax.setIdleMode(Constants.Drivetrains.Swerve.Module.TURNING_MOTOR_IDLE_MODE);

    drivingSparkMax.setSmartCurrentLimit(Constants.Drivetrains.Swerve.Module.DRIVING_MOTOR_CURRENT_LIMIT);
    turningSparkMax.setSmartCurrentLimit(Constants.Drivetrains.Swerve.Module.TURNING_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    desiredModuleState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);

  }
  

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
      drivingEncoder.getVelocity(),
      new Rotation2d(turningEncoder.getPosition() - angularOffset)
    );
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
      drivingEncoder.getPosition(),
      new Rotation2d(turningEncoder.getPosition() - angularOffset)
    );
  }

  /**
   * Sets the desired state for the module.
   *
   * @param moduleState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState moduleState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = moduleState.speedMetersPerSecond;
    correctedDesiredState.angle = moduleState.angle.plus(Rotation2d.fromRadians(angularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
      correctedDesiredState,
      new Rotation2d(turningEncoder.getPosition())
    );

    desiredModuleState = optimizedDesiredState;

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    turningSparkMax.set(0.1);

    SmartDashboard.putNumber((moduleName + "EncoderPos"), turningEncoder.getPosition());
    SmartDashboard.putNumber((moduleName + "EncoderVel"), turningEncoder.getVelocity());
    SmartDashboard.putNumber((moduleName + "TurnMotorOutput"), turningSparkMax.getAppliedOutput());
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }
}
