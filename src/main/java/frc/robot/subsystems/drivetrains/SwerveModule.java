// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.settings.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final RelativeEncoder turningEncoder;

  private final SparkMaxPIDController drivingPIDController;
  private final PIDController turningPIDController;

  private double angularOffset;
  private SwerveModuleState desiredModuleState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, int canCoderID, double chassisAngularOffset) {

    angularOffset = chassisAngularOffset;

    drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = drivingSparkMax.getEncoder();
    turningEncoder = turningSparkMax.getEncoder();
    drivingPIDController = drivingSparkMax.getPIDController();
    turningPIDController = new PIDController(
      Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_P,
      Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_I,
      Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_D
    );
    drivingPIDController.setFeedbackDevice(drivingEncoder);
    turningPIDController.

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingEncoder.setPositionConversionFactor(Constants.Drivetrains.Swerve.Module.DRIVING_ENCODER_POSITION_FACTOR);
    drivingEncoder.setVelocityConversionFactor(Constants.Drivetrains.Swerve.Module.DRIVING_ENCODER_VELOCITY_FACTOR);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(Constants.Drivetrains.Swerve.Module.DRIVING_ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(Constants.Drivetrains.Swerve.Module.TURNING_ENCODER_VELOCITY_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    //TODO! is this true for our module?
    //turningEncoder.setInverted(Constants.Drivetrains.Swerve.Module.TURNING_ENCODER_INVERTED);


    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(Constants.Drivetrains.Swerve.Module.TURNING_ENCODER_POSITION_PID_MINIMUM_INPUT);
    turningPIDController.setPositionPIDWrappingMaxInput(Constants.Drivetrains.Swerve.Module.TURNING_ENCODER_POSITION_PID_MAXIMUM_INPUT);

    // Set the PID gains for the driving motor.
    //TODO! Tune these
    drivingPIDController.setP(Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_P);
    drivingPIDController.setI(Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_I);
    drivingPIDController.setD(Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_D);
    drivingPIDController.setFF(Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_FF);
    drivingPIDController.setOutputRange(
      Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_MINIMUM_OUTPUT,
      Constants.Drivetrains.Swerve.Module.PID.DRIVING_MOTOR_MAXIMUM_OUTPUT
    );

    // Set the PID gains for the turning motor.
    //TODO! Tune these
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
    correctedDesiredState.speedMetersPerSecond = desiredModuleState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredModuleState.angle.plus(Rotation2d.fromRadians(angularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
      correctedDesiredState,
      new Rotation2d(turningEncoder.getPosition())
    );

    desiredModuleState = moduleState;

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setSetpoint(optimizedDesiredState.angle.getRadians());
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  public void turnWheel() {
    turningSparkMax.set(turningPIDController.calculate(turningEncoder.getPosition()));
  }
}