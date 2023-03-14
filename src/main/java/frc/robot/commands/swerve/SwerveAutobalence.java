// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class SwerveAutobalence extends CommandBase {

  SwerveContainer swerveDrive;
  XboxController controller;
  AHRS ahrs;


  /** Creates a new DriveSwerve. */
  public SwerveAutobalence(SwerveContainer swerve) {

    swerveDrive = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees  = 5;

  public void operatorControl() {
          //drive.setSafetyEnabled(true);

          double xAxisRate = 0;
          double yAxisRate = 0;
          double pitchAngleDegrees    = ahrs.getPitch();
          double rollAngleDegrees     = ahrs.getRoll();
          
          if ( !autoBalanceXMode && 
               (Math.abs(pitchAngleDegrees) >= 
                Math.abs(kOffBalanceAngleThresholdDegrees))) {
              autoBalanceXMode = true;
          }
          else if ( autoBalanceXMode && 
                    (Math.abs(pitchAngleDegrees) <= 
                     Math.abs(kOonBalanceAngleThresholdDegrees))) {
              autoBalanceXMode = false;
          }
          if ( !autoBalanceYMode && 
               (Math.abs(pitchAngleDegrees) >= 
                Math.abs(kOffBalanceAngleThresholdDegrees))) {
              autoBalanceYMode = true;
          }
          else if ( autoBalanceYMode && 
                    (Math.abs(pitchAngleDegrees) <= 
                     Math.abs(kOonBalanceAngleThresholdDegrees))) {
              autoBalanceYMode = false;
          }
          
          // Control drive system automatically, 
          // driving in reverse direction of pitch/roll angle,
          // with a magnitude based upon the angle
          
          if ( autoBalanceXMode ) {
              double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
              xAxisRate = Math.sin(pitchAngleRadians) * -1;
          }
          if ( autoBalanceYMode ) {
              double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
              yAxisRate = Math.sin(rollAngleRadians) * -1;
          }
          
          try {
              swerveDrive.drive(xAxisRate, yAxisRate, 0);
          } catch( RuntimeException ex ) {
              String err_string = "Drive system error:  " + ex.getMessage();
              DriverStation.reportError(err_string, true);
          }
      }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ahrs = new AHRS();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.operatorControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  
  // Should return false, we are relying on the 
  // controller to handle stopping the command
  @Override
  public boolean isFinished() {
    return false;
  }

  boolean autoBalanceXMode;
  boolean autoBalanceYMode;

}