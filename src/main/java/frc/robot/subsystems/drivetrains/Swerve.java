// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Drivetrains.Swerve.Odometry;

public class Swerve extends SubsystemBase {
  
  public static SwerveDriveOdometry swerveOdometry;
  public static SwerveModule frontLeftSwerveModule;
  public static SwerveModule frontRightSwerveModule;
  public static SwerveModule backLeftSwerveModule;
  public static SwerveModule backRightSwerveModule;
  //TODO! is this the right gyro?
  public static ADIS16470_IMU swerveGyro = new ADIS16470_IMU();

  /** Creates a new Swerve Drive subsystem. */
  public Swerve() {
    // Create MAXSwerveModules
    final SwerveModule frontLeftSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.FRONT_LEFT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.FRONT_LEFT_PORT,
      Constants.Drivetrains.Swerve.Odometry.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
    );
    final SwerveModule frontRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.FRONT_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.FRONT_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Odometry.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
    );
    final SwerveModule backLeftSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_LEFT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_LEFT_PORT,
      Constants.Drivetrains.Swerve.Odometry.BACK_LEFT_CHASSIS_ANGULAR_OFFSET
    );
    final SwerveModule backRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Odometry.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET
    );

    // Odometry class for tracking robot pose
    swerveOdometry = new SwerveDriveOdometry(
      Constants.Drivetrains.Swerve.Odometry.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(swerveGyro.getAngle()),
    
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
    swerveOdometry.update(
      Rotation2d.fromDegrees(swerveGyro.getAngle()),
    
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
      Rotation2d.fromDegrees(swerveGyro.getAngle()),
      
      new SwerveModulePosition[] {
        frontLeftSwerveModule.getPosition(),
        frontRightSwerveModule.getPosition(),
        backLeftSwerveModule.getPosition(),
        backRightSwerveModule.getPosition()
      },
      pose
    );
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

    var swerveModuleStates = Constants.Drivetrains.Swerve.Odometry.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(swerveGyro.getAngle())) : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    Constants.Drivetrains.Swerve.Odometry.DRIVE_KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, Constants.Drivetrains.Swerve.Speed.MAX_SPEED_METERS_PER_SECONDS);
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    backLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    backRightSwerveModule.setDesiredState(swerveModuleStates[3]);
  }

/**
 * Sets the wheels into an X formation to prevent movement.
*/
  public void setX() {
    frontLeftSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeftSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRightSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

/**
 * Sets the swerve ModuleStates.
 *
 * @param desiredStates The desired SwerveModule states.
 */

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    Constants.Drivetrains.Swerve.Odometry.DRIVE_KINEMATICS.desaturateWheelSpeeds(desiredStates, Constants.Drivetrains.Swerve.Speed.MAX_SPEED_METERS_PER_SECONDS);
    frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    frontRightSwerveModule.setDesiredState(desiredStates[1]);
    backLeftSwerveModule.setDesiredState(desiredStates[2]);
    backRightSwerveModule.setDesiredState(desiredStates[3]);
  }

/** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftSwerveModule.resetEncoders();
    backLeftSwerveModule.resetEncoders();
    frontRightSwerveModule.resetEncoders();
    backRightSwerveModule.resetEncoders();
  }

/** Zeroes the heading of the robot. */
  public void zeroHeading() {
    swerveGyro.reset();
  }

/**
 * Returns the heading of the robot.
 *
 * @return the robot's heading in degrees, from -180 to 180
 */
  public double getHeading() {
    return Rotation2d.fromDegrees(swerveGyro.getAngle()).getDegrees();
  }

/**
 * Returns the turn rate of the robot.
 *
 * @return The turn rate of the robot, in degrees per second
 */
  public double getTurnRate() {
    return swerveGyro.getRate() * (Constants.Drivetrains.Swerve.Odometry.GYRO_REVERSED ? -1.0 : 1.0);
  }
}

