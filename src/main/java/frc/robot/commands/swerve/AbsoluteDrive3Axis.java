// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Drivetrains.Swerve.Speed;
import frc.robot.subsystems.drivetrains.SwerveContainer;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AbsoluteDrive3Axis extends CommandBase {

  private SwerveContainer swerve;

  private DoubleSupplier moveX, moveY;
  private DoubleSupplier turnTheta;

  private Timer time = new Timer();
  private double previousTime = 0;

  private double desiredAngle = 0;

  /** Creates a new AbsoluteDrive. */
  public AbsoluteDrive3Axis(SwerveContainer swerve,
    DoubleSupplier moveX, DoubleSupplier moveY, DoubleSupplier turnTheta
  ) {

    this.swerve = swerve;

    this.moveX = moveX;
    this.moveY = moveY;

    this.turnTheta = turnTheta;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Cube the input values to make small inputs smaller, big inputs bigger
    double correctedMoveX = Math.pow(moveX.getAsDouble(), 3);
    double correctedMoveY = Math.pow(moveY.getAsDouble(), 3);
    double correctedTurnTheta = Math.pow(turnTheta.getAsDouble(), 3);

    if(correctedTurnTheta != 0) {
      // Use delta time to make this speed consistent over time
      double deltaTimeSeconds = time.get() - previousTime;
      desiredAngle += correctedTurnTheta * Speed.MAX_ANGULAR_RPS * deltaTimeSeconds;
      SmartDashboard.putNumber("deltaTime", deltaTimeSeconds);
    } else {
      desiredAngle = swerve.getYaw().getRadians();
    }
    
    
    SmartDashboard.putNumber("inputCubed", correctedTurnTheta);
    SmartDashboard.putNumber("desiredAng", desiredAngle);
    SmartDashboard.putNumber("configMaxAngularVelocity", swerve.getController().config.maxAngularVelocity);

    ChassisSpeeds desiredSpeeds = swerve.getController().getTargetSpeeds(
      correctedMoveX,
      correctedMoveY,
      desiredAngle,
      swerve.getYaw().getRadians()
    );

    // // Limit velocity to prevent tippy
    // Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

    // translation = SwerveMath.limitVelocity(
    //   translation,
    //   swerve.getFieldVelocity(),
    //   swerve.getPose(),
    //   Constants.Drivetrains.Swerve.Measurements.MESSAGE_LOOP_TIME,
    //   Constants.Drivetrains.Swerve.Measurements.ROBOT_MASS,
    //   List.of(Constants.Drivetrains.Swerve.Measurements.CHASSIS_CG),
    //   swerve.getSwerveDriveConfiguration()
    // );


    // SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    // SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.driveRaw(
      SwerveController.getTranslation2d(desiredSpeeds),
      desiredSpeeds.omegaRadiansPerSecond,
      Constants.Drivetrains.Swerve.FIELD_RELATIVE,
      Constants.Drivetrains.Swerve.OPEN_LOOP
    );

    previousTime = time.get();
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
