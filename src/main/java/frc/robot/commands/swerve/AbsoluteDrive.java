// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.subsystems.drivetrains.SwerveContainer;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AbsoluteDrive extends CommandBase {

  private SwerveContainer swerve;

  private DoubleSupplier moveX, moveY;
  private DoubleSupplier headingX, headingY;

  /** Creates a new AbsoluteDrive. */
  public AbsoluteDrive(SwerveContainer swerve,
    DoubleSupplier moveX, DoubleSupplier moveY, DoubleSupplier headingX, DoubleSupplier headingY
  ) {

    this.swerve = swerve;

    this.moveX = moveX;
    this.moveY = moveY;

    this.headingX = headingX;
    this.headingY = headingY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the desired chassis speeds based on a 2 joystick module.

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
      moveX.getAsDouble(),
      moveY.getAsDouble(),
      headingX.getAsDouble(),
      headingY.getAsDouble()
    );

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

    translation = SwerveMath.limitVelocity(
      translation,
      swerve.getFieldVelocity(),
      swerve.getPose(),
      Constants.Drivetrains.Swerve.Measurements.MESSAGE_LOOP_TIME,
      Constants.Drivetrains.Swerve.Measurements.ROBOT_MASS,
      List.of(Constants.Drivetrains.Swerve.Measurements.CHASSIS_CG),
      swerve.getSwerveDriveConfiguration()
    );


    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.driveRaw(
      translation,
      desiredSpeeds.omegaRadiansPerSecond,
      true,
      Constants.Drivetrains.Swerve.OPEN_LOOP
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
