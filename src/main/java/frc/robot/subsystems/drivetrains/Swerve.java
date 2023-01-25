// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Drivetrains.Swerve.Odometry;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve Drive subsystem. */
  public Swerve() {
    // Create MAXSwerveModules
    final SwerveModule frontLeftSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.FRONT_LEFT_DRIVE_MOTOR_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.FRONT_LEFT_TURNING_MOTOR_PORT,
      Constants.Drivetrains.Swerve.Odometry.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET    
    );
    final SwerveModule frontRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.FRONT_RIGHT_DRIVE_MOTOR_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.FRONT_RIGHT_TURNING_MOTOR_PORT,
      Constants.Drivetrains.Swerve.Odometry.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
    );
    final SwerveModule backLeftSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_LEFT_DRIVE_MOTOR_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_LEFT_TURNING_MOTOR_PORT,
      Constants.Drivetrains.Swerve.Odometry.BACK_LEFT_CHASSIS_ANGULAR_OFFSET
    );
    final SwerveModule backRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_RIGHT_DRIVE_MOTOR_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_RIGHT_TURNING_MOTOR_PORT,
      Constants.Drivetrains.Swerve.Odometry.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET
    );

    // The gyro sensor
    final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

    // Odometry class for tracking robot pose
    SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(
      Constants.Drivetrains.Swerve.Odometry.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
        frontLeftSwerveModule.getPosition(),
        frontRightSwerveModule.getPosition(),
        backLeftSwerveModule.getPosition(),
        backRightSwerveModule.getPosition()
      }
    );
  }
@Override
  public void periodic() {
    // Update the odometry in the periodic block
    SwerveOdometry.update(
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
        frontLeftSwerveModule.getPosition(),
        frontRightSwerveModule.getPosition(),
        backLeftSwerveModule.getPosition(),
        backRightSwerveModule.getPosition()
        }
      );
}

/**
 * Returns the currently-estimated pose of the robot.
 *
 * @return The pose.
 */
public Pose2d getPose() {
  return swerveOdometry.getPoseMeters();
}

/**
 * Resets the odometry to the specified pose.
 *
 * @param pose The pose to which to set the odometry.
 */
public void resetOdometry(Pose2d pose) {
  swerveOdometry.resetPosition(
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      pose);
}

/**
 * Method to drive the robot using joystick info.
 *
 * @param xSpeed        Speed of the robot in the x direction (forward).
 * @param ySpeed        Speed of the robot in the y direction (sideways).
 * @param rot           Angular rate of the robot.
 * @param fieldRelative Whether the provided x and y speeds are relative to the
 *                      field.
 */
public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
  // Adjust input based on max speed
  xSpeed *= Constants.Drivetrains.Swerve.Speed.MAX_SPEED_METERS_PER_SECONDS;
  ySpeed *= Constants.Drivetrains.Swerve.Speed.MAX_SPEED_METERS_PER_SECONDS;
  rot *= Constants.Drivetrains.Swerve.Speed.MAX_ANGULAR_SPEED;

  var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()))
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
  SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_frontRight.setDesiredState(swerveModuleStates[1]);
  m_rearLeft.setDesiredState(swerveModuleStates[2]);
  m_rearRight.setDesiredState(swerveModuleStates[3]);
}

/**
 * Sets the wheels into an X formation to prevent movement.
 */
public void setX() {
  m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
}

/**
 * Sets the swerve ModuleStates.
 *
 * @param desiredStates The desired SwerveModule states.
 */
public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
  m_frontLeft.setDesiredState(desiredStates[0]);
  m_frontRight.setDesiredState(desiredStates[1]);
  m_rearLeft.setDesiredState(desiredStates[2]);
  m_rearRight.setDesiredState(desiredStates[3]);
}

/** Resets the drive encoders to currently read a position of 0. */
public void resetEncoders() {
  m_frontLeft.resetEncoders();
  m_rearLeft.resetEncoders();
  m_frontRight.resetEncoders();
  m_rearRight.resetEncoders();
}

/** Zeroes the heading of the robot. */
public void zeroHeading() {
  m_gyro.reset();
}

/**
 * Returns the heading of the robot.
 *
 * @return the robot's heading in degrees, from -180 to 180
 */
public double getHeading() {
  return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
}

/**
 * Returns the turn rate of the robot.
 *
 * @return The turn rate of the robot, in degrees per second
 */
public double getTurnRate() {
  return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
}
  }
}
