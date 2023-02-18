// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
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

  private double slewRotation = 0.0;
  /** Translation Direction, part of a vector. */
  private double slewTranslationDir = 0.0;
  /** Translation Magnitude, part of a vector. */
  private double slewTranslationMag = 0.0;

  /** Translation magnitude limiter. */
  private SlewRateLimiter magLimiter;
  /** Rotation limiter. */
  private SlewRateLimiter rotLimiter;

  private double previousTime;

  /** Creates a new Swerve Drive subsystem. */
  public Swerve() {

    navX = new AHRS(Port.kMXP);

    // Create SwerveModules
    frontLeftSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Module.FRONT_LEFT_NAME,
      Constants.Drivetrains.Swerve.Motors.Drive.FRONT_LEFT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.FRONT_LEFT_PORT,
      Constants.Drivetrains.Swerve.Odometry.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
      Constants.Drivetrains.Swerve.Encoders.FRONT_LEFT_PORT
    );
    frontRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Module.FRONT_RIGHT_NAME,
      Constants.Drivetrains.Swerve.Motors.Drive.FRONT_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.FRONT_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Odometry.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
      Constants.Drivetrains.Swerve.Encoders.FRONT_RIGHT_PORT
    );
    backLeftSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Module.BACK_LEFT_NAME,
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_LEFT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_LEFT_PORT,
      Constants.Drivetrains.Swerve.Odometry.BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
      Constants.Drivetrains.Swerve.Encoders.BACK_LEFT_PORT
    );
    backRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Module.BACK_RIGHT_NAME,
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Odometry.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
      Constants.Drivetrains.Swerve.Encoders.BACK_RIGHT_PORT
    );

    swerveKinematics = new SwerveDriveKinematics(
      // Front Left
      new Translation2d(
        Constants.Drivetrains.Swerve.Odometry.WHEEL_BASE / 2,
        Constants.Drivetrains.Swerve.Odometry.TRACK_WIDTH / 2
      ),
      // Front Right
      new Translation2d(
        Constants.Drivetrains.Swerve.Odometry.WHEEL_BASE / 2,
        -Constants.Drivetrains.Swerve.Odometry.TRACK_WIDTH / 2
      ),
      // Back Left
      new Translation2d(
        -Constants.Drivetrains.Swerve.Odometry.WHEEL_BASE / 2,
        Constants.Drivetrains.Swerve.Odometry.TRACK_WIDTH / 2
      ),
      // Back Right
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

    magLimiter = new SlewRateLimiter(Constants.Drivetrains.Swerve.SlewRate.MAG_LIMIT);
    rotLimiter = new SlewRateLimiter(Constants.Drivetrains.Swerve.SlewRate.ROT_LIMIT);

    previousTime = WPIUtilJNI.now() * 1e-6;
  }

  private int periodicLoops = 0;

  @Override
  public void periodic() {

    periodicLoops += 1;
    periodicLoops %= 50;

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

      // Add offset, then wrap to 0-360 degrees
      SmartDashboard.putNumber("FLCANCoder", (frontLeftSwerveModule.canCoder.getAbsolutePosition() + Constants.Drivetrains.Swerve.Encoders.FRONT_LEFT_OFFSET) % 360);
      SmartDashboard.putNumber("FRCANCoder", (frontRightSwerveModule.canCoder.getAbsolutePosition() + Constants.Drivetrains.Swerve.Encoders.FRONT_RIGHT_OFFSET) % 360);
      SmartDashboard.putNumber("BRCANCoder", (backRightSwerveModule.canCoder.getAbsolutePosition() + Constants.Drivetrains.Swerve.Encoders.BACK_RIGHT_OFFSET) % 360);
      SmartDashboard.putNumber("BLCANCoder", (backLeftSwerveModule.canCoder.getAbsolutePosition() + Constants.Drivetrains.Swerve.Encoders.BACK_LEFT_OFFSET) % 360);

    }    

    if(periodicLoops == 1) {
      frontLeftSwerveModule.alignTurningEncoder(Constants.Drivetrains.Swerve.Encoders.FRONT_LEFT_OFFSET);
      frontRightSwerveModule.alignTurningEncoder(Constants.Drivetrains.Swerve.Encoders.FRONT_RIGHT_OFFSET);
      backLeftSwerveModule.alignTurningEncoder(Constants.Drivetrains.Swerve.Encoders.BACK_LEFT_OFFSET);
      backRightSwerveModule.alignTurningEncoder(Constants.Drivetrains.Swerve.Encoders.BACK_RIGHT_OFFSET);
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

    double[] ratedValues = rateLimit(xSpeed, ySpeed, rot);
    xSpeed = ratedValues[0];
    ySpeed = ratedValues[1];
    rot = ratedValues[2];

    // Adjust input based on max speed
    xSpeed *= Constants.Drivetrains.Swerve.Speed.MAX_SPEED_METERS_PER_SECONDS;
    ySpeed *= Constants.Drivetrains.Swerve.Speed.MAX_SPEED_METERS_PER_SECONDS;
    rot *= Constants.Drivetrains.Swerve.Speed.MAX_ANGULAR_SPEED;

    SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(
      fieldRelative // ternary operator - runs ? if true, : if false
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(navX.getAngle()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    setModuleStates(swerveModuleStates);
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

  private double[] rateLimit(double xSpeed, double ySpeed, double rot) {
    double[] output = new double[3];

    // Convert XY to polar for rate limiting
    double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
    double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

    // Calculate the direction slew rate based on an estimate of the lateral acceleration
    double directionSlewRate;
    if (slewTranslationMag != 0.0) {
      directionSlewRate = Math.abs(Constants.Drivetrains.Swerve.SlewRate.DIR_LIMIT / slewTranslationMag);
    } else {
      directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
    }
      

    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - previousTime;
    double angleDiff = AngleDifference(inputTranslationDir, slewTranslationDir);
    if (angleDiff < 0.45*Math.PI) {
      slewTranslationDir = StepTowardsCircular(slewTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      slewTranslationMag = magLimiter.calculate(inputTranslationMag);
    }
    else if (angleDiff > 0.85*Math.PI) {
      if (slewTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
        // keep currentTranslationDir unchanged
        slewTranslationMag = magLimiter.calculate(0.0);
      }
      else {
        slewTranslationDir = WrapAngle(slewTranslationDir + Math.PI);
        slewTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
    }
    else {
      slewTranslationDir = StepTowardsCircular(slewTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      slewTranslationMag = magLimiter.calculate(0.0);
    }
    previousTime = currentTime;
      
    output[0] = slewTranslationMag * Math.cos(slewTranslationDir);
    output[1] = slewTranslationMag * Math.sin(slewTranslationDir);
    output[2] = rotLimiter.calculate(rot);

    slewRotation = output[2];

    return output;
  }

  /**
  * Steps a value towards a target with a specified step size.
  * @param current The current or starting value.  Can be positive or negative.
  * @param target The target value the algorithm will step towards.  Can be positive or negative.
  * @param stepsize The maximum step size that can be taken.
  * @return The new value for {@code current} after performing the specified step towards the specified target.
  */
  public static double StepTowards(double current, double target, double stepsize) {
    if (Math.abs(current - target) <= stepsize) {
      return target;
    }
    else if (target < current) {
      return current - stepsize;
    }
    else {
      return current + stepsize;
    }
  }

  /**
  * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
  * @param current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
  * @param target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
  * @param stepSize The maximum step size that can be taken (in radians).
  * @return The new angle (in radians) for {@code current} after performing the specified step towards the specified target.
  * This value will always lie in the range 0 to 2*PI (exclusive).
  */
  public static double StepTowardsCircular(double current, double target, double stepSize) {
      current = WrapAngle(current);
      target = WrapAngle(target);

      double stepDirection = Math.signum(target - current);
      double difference = Math.abs(current - target);
      
      if (difference <= stepSize) {
          return target;
      }
      else if (difference > Math.PI) { // Does the system need to wrap over eventually?
          // Handle the special case where you can reach the target in one step while also wrapping
          if (current + 2*Math.PI - target < stepSize || target + 2*Math.PI - current < stepSize) {
              return target;
          }
          else {
              return WrapAngle(current - stepDirection * stepSize); // This will handle wrapping gracefully
          }

      }
      else {
          return current + stepDirection * stepSize;
      }
  }

  /**
  * Finds the (unsigned) minimum difference between two angles including calculating across 0.
  * @param angleA An angle (in radians).
  * @param angleB An angle (in radians).
  * @return The (unsigned) minimum difference between the two angles (in radians).
  */
  public static double AngleDifference(double angleA, double angleB) {
    double difference = Math.abs(angleA - angleB);
    return difference > Math.PI? (2 * Math.PI) - difference : difference;
  }

  /**
  * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
  * @param angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
  * @return An angle (in radians) from 0 and 2*PI (exclusive).
  */
  public static double WrapAngle(double angle) {
    double twoPi = 2*Math.PI;

    if (angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
      return 0.0;
    }
    else if (angle > twoPi) {
      return angle - twoPi*Math.floor(angle / twoPi);
    }
    else if (angle < 0.0) {
      return angle + twoPi*(Math.floor((-angle) / twoPi)+1);
    }
    else {
      return angle;
    }
  }
}

