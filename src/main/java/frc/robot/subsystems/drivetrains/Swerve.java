// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import edu.wpi.first.math.geometry.Translation2d;

public class Swerve extends SubsystemBase {

  public SwerveDriveOdometry swerveOdometry;
  public SwerveDriveKinematics swerveKinematics;

  public SwerveModule frontLeftSwerveModule;
  public SwerveModule frontRightSwerveModule;
  public SwerveModule backLeftSwerveModule;
  public SwerveModule backRightSwerveModule;

  public AHRS navX;

  /** Creates a new Swerve Drive subsystem. */
  public Swerve() {

    navX = new AHRS(Port.kMXP);

    // Create SwerveModules
    frontLeftSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.FRONT_LEFT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.FRONT_LEFT_PORT,
      Constants.Drivetrains.Swerve.Encoders.FRONT_LEFT_PORT,
      Constants.Drivetrains.Swerve.Odometry.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
    );
    frontRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.FRONT_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.FRONT_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Encoders.FRONT_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Odometry.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
    );
    backLeftSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_LEFT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_LEFT_PORT,
      Constants.Drivetrains.Swerve.Encoders.BACK_LEFT_PORT,
      Constants.Drivetrains.Swerve.Odometry.BACK_LEFT_CHASSIS_ANGULAR_OFFSET
    );
    backRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Encoders.BACK_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Odometry.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET
    );

    swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(
        Constants.Drivetrains.Swerve.Odometry.WHEEL_BASE / 2,
        Constants.Drivetrains.Swerve.Odometry.TRACK_WIDTH / 2
      ),
      new Translation2d(
        Constants.Drivetrains.Swerve.Odometry.WHEEL_BASE / 2,
        -Constants.Drivetrains.Swerve.Odometry.TRACK_WIDTH / 2
      ),
      new Translation2d(
        -Constants.Drivetrains.Swerve.Odometry.WHEEL_BASE / 2,
        Constants.Drivetrains.Swerve.Odometry.TRACK_WIDTH / 2
      ),
      new Translation2d(
        -Constants.Drivetrains.Swerve.Odometry.WHEEL_BASE / 2,
        -Constants.Drivetrains.Swerve.Odometry.TRACK_WIDTH / 2
      )
    );
    
    // Odometry class for tracking robot pose
    swerveOdometry = new SwerveDriveOdometry(
      swerveKinematics,
      Rotation2d.fromDegrees(navX.getAngle()),
    
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
    // Once all swerve modules have been initialized
    if(backRightSwerveModule != null) {
      // Update the odometry in the periodic block
      swerveOdometry.update(
        Rotation2d.fromDegrees(navX.getAngle()),
    
        new SwerveModulePosition[] {
          frontLeftSwerveModule.getPosition(),
          frontRightSwerveModule.getPosition(),
          backLeftSwerveModule.getPosition(),
          backRightSwerveModule.getPosition()
        }
      );

      // // Turn wheels towards the desired positions
      // frontLeftSwerveModule.turnWheel();
      // frontRightSwerveModule.turnWheel();
      // backLeftSwerveModule.turnWheel();
      // backRightSwerveModule.turnWheel();

      frontLeftSwerveModule.updateTurningPID();
      frontRightSwerveModule.updateTurningPID();
      backLeftSwerveModule.updateTurningPID();
      backRightSwerveModule.updateTurningPID();

      double turningP = SmartDashboard.getNumber("turningP", Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_P);
      double turningI = SmartDashboard.getNumber("turningI", Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_I);
      double turningD = SmartDashboard.getNumber("turningD", Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_D);
      frontLeftSwerveModule.setPID(turningP, turningI, turningD);
      frontRightSwerveModule.setPID(turningP, turningI, turningD);
      backLeftSwerveModule.setPID(turningP, turningI, turningD);
      backRightSwerveModule.setPID(turningP, turningI, turningD);
    }
    
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
      Rotation2d.fromDegrees(navX.getAngle()),
      
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

    var swerveModuleStates = swerveKinematics.toSwerveModuleStates(
      fieldRelative // ternary operator - runs ? if true, : if false
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(navX.getAngle()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drivetrains.Swerve.Speed.MAX_SPEED_METERS_PER_SECONDS);
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
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Drivetrains.Swerve.Speed.MAX_SPEED_METERS_PER_SECONDS);
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
    navX.reset();
  }

/**
 * Returns the heading of the robot.
 *
 * @return the robot's heading in degrees, from -180 to 180
 */
  public double getHeading() {
    return Rotation2d.fromDegrees(navX.getAngle()).getDegrees();
  }

/**
 * Returns the turn rate of the robot.
 *
 * @return The turn rate of the robot, in degrees per second
 */
  public double getTurnRate() {
    return navX.getRate() * (Constants.Drivetrains.Swerve.Odometry.GYRO_REVERSED ? -1.0 : 1.0);
  }
}

