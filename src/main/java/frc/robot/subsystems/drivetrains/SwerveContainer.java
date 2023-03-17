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
import frc.robot.settings.Constants.Drivetrains.Swerve.Speed;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveModuleState2;
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

    SmartDashboard.putNumber("gyroPitch", rawSwerveDrive.getPitch().getDegrees());
    SmartDashboard.putNumber("gyroRoll", rawSwerveDrive.getRoll().getDegrees());
    SmartDashboard.putNumber("gyroYaw", rawSwerveDrive.getYaw().getDegrees());

    SmartDashboard.putNumber("FLEncoderPos", rawSwerveDrive.getModulePositions()[0].angle.getDegrees());
    SmartDashboard.putNumber("FREncoderPos", rawSwerveDrive.getModulePositions()[1].angle.getDegrees());
    SmartDashboard.putNumber("BLEncoderPos", rawSwerveDrive.getModulePositions()[2].angle.getDegrees());
    SmartDashboard.putNumber("BREncoderPos", rawSwerveDrive.getModulePositions()[3].angle.getDegrees());

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

  public void resetPose(Pose2d pose) {
    rawSwerveDrive.resetOdometry(pose);
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

  public SwerveController getController() {
    return rawSwerveDrive.swerveController;
  }

  public void setModuleStates(SwerveModuleState2[] desiredStates) {
    rawSwerveDrive.setModuleStates(desiredStates, false);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    speeds.vyMetersPerSecond *= -1.0;
    rawSwerveDrive.setChassisSpeeds(speeds);
  }

  public void setBrakes(boolean enabled) {
    rawSwerveDrive.setMotorIdleMode(enabled);
  }

  public void moveX() {

    SwerveModuleState2[] states = new SwerveModuleState2[4];

    states[0] = new SwerveModuleState2(0, new Rotation2d(0.25 * Math.PI), 0);
    states[1] = new SwerveModuleState2(0, new Rotation2d(0.75 * Math.PI), 0);
    states[2] = new SwerveModuleState2(0, new Rotation2d(1.25 * Math.PI), 0);
    states[3] = new SwerveModuleState2(0, new Rotation2d(1.75 * Math.PI), 0);

    rawSwerveDrive.setModuleStates(states, false);
  }

  public void resetYaw() {
    rawSwerveDrive.resetOdometry(new Pose2d(
      getPose().getTranslation(),
      new Rotation2d(0)
    ));
  }
}

