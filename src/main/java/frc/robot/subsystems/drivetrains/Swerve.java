// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
      Constants.Drivetrains.Swerve.Odometry.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
    );
    frontRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Module.FRONT_RIGHT_NAME,
      Constants.Drivetrains.Swerve.Motors.Drive.FRONT_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.FRONT_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Odometry.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
    );
    backLeftSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Module.BACK_LEFT_NAME,
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_LEFT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_LEFT_PORT,
      Constants.Drivetrains.Swerve.Odometry.BACK_LEFT_CHASSIS_ANGULAR_OFFSET
    );
    backRightSwerveModule = new SwerveModule(
      Constants.Drivetrains.Swerve.Module.BACK_RIGHT_NAME,
      Constants.Drivetrains.Swerve.Motors.Drive.BACK_RIGHT_PORT,
      Constants.Drivetrains.Swerve.Motors.Turning.BACK_RIGHT_PORT,
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

    magLimiter = new SlewRateLimiter(0);
    rotLimiter = new SlewRateLimiter(0);

    previousTime = WPIUtilJNI.now() * 1e-6;
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

  private double[] rateLimit(double xSpeed, double ySpeed, double rot) {
    double[] output = new double[3];

    // Convert XY to polar for rate limiting
    double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
    double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

    // Calculate the direction slew rate based on an estimate of the lateral acceleration
    double directionSlewRate;
    if (slewTranslationMag != 0.0) {
      directionSlewRate = Math.abs(0 / slewTranslationMag);
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

    return output;
  }

  /**
     * Steps a value towards a target with a specified step size.
     * @param _current The current or starting value.  Can be positive or negative.
     * @param _target The target value the algorithm will step towards.  Can be positive or negative.
     * @param _stepsize The maximum step size that can be taken.
     * @return The new value for {@code _current} after performing the specified step towards the specified target.
     */
    public static double StepTowards(double _current, double _target, double _stepsize) {
        if (Math.abs(_current - _target) <= _stepsize) {
            return _target;
        }
        else if (_target < _current) {
            return _current - _stepsize;
        }
        else {
            return _current + _stepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param _current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param _target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = WrapAngle(_current);
        _target = WrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);
        
        if (difference <= _stepsize) {
            return _target;
        }
        else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2*Math.PI - _target < _stepsize || _target + 2*Math.PI - _current < _stepsize) {
                return _target;
            }
            else {
                return WrapAngle(_current - stepDirection * _stepsize); //this will handle wrapping gracefully
            }

        }
        else {
            return _current + stepDirection * _stepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double _angle) {
        double twoPi = 2*Math.PI;

        if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        }
        else if (_angle > twoPi) {
            return _angle - twoPi*Math.floor(_angle / twoPi);
        }
        else if (_angle < 0.0) {
            return _angle + twoPi*(Math.floor((-_angle) / twoPi)+1);
        }
        else {
            return _angle;
        }
    }
}

