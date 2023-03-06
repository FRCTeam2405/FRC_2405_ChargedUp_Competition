package frc.robot.subsystems.drivetrains;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class SwerveContainer extends SubsystemBase {

  SwerveDrive rawSwerveDrive;

  public SwerveContainer() {
    try {
      rawSwerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
    } catch(IOException exception) {
      SmartDashboard.putString("ERROR", "Swerve failed to init: " + exception.getMessage() + ". Did you remember to enter the swerve configuration?");
      return;
    }

    rawSwerveDrive.setMotorIdleMode(false);
  }

  @Override
  public void periodic() {
    rawSwerveDrive.updateOdometry();
  }

  /** 
   * Move the swerve drivetrain based on XY speed and rotational speed.
   * Inputs should be in a range from -1.0 to 1.0
   */
  public void drive(double moveX, double moveY, double rotTheta) {
    // Multiply the inputs (range -1 to 1) by the max speeds
    moveX *= Constants.Drivetrains.Swerve.Speed.MAX_TRANSLATION_MPS;
    moveY *= Constants.Drivetrains.Swerve.Speed.MAX_TRANSLATION_MPS;
    rotTheta *= Constants.Drivetrains.Swerve.Speed.MAX_ANGULAR_RPS;

    // Convert X, Y to Translation2D for the swerve library
    Translation2d translation = new Translation2d(
      moveX, 
      moveY
    );

    rawSwerveDrive.drive(
      translation,
      rotTheta,   
      Constants.Drivetrains.Swerve.FIELD_RELATIVE,
      Constants.Drivetrains.Swerve.OPEN_LOOP
    );
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return rawSwerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, rawSwerveDrive.getYaw().getRadians());
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return rawSwerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), rawSwerveDrive.getYaw().getRadians());
  }

  public void driveRaw(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    rawSwerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  public Pose2d getPose() {
    return rawSwerveDrive.getPose();
  }

  public ChassisSpeeds getFieldVelocity() {
    return rawSwerveDrive.getFieldVelocity();
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return rawSwerveDrive.swerveDriveConfiguration;
  }

  public Rotation2d getYaw() {
    return rawSwerveDrive.getYaw();
  }
}

