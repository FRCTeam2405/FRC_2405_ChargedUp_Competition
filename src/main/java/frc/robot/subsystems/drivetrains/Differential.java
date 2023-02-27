// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Differential extends SubsystemBase {


  private WPI_TalonFX leftParentMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motors.Ports.FRONT_LEFT);
  private WPI_TalonFX rightParentMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motors.Ports.FRONT_RIGHT);
  private WPI_TalonFX leftChildMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motors.Ports.BACK_LEFT);
  private WPI_TalonFX rightChildMotor = new WPI_TalonFX(Constants.Drivetrains.Differential.Motors.Ports.BACK_RIGHT);

  AHRS ahrs;
  DifferentialDrive drive;
  Joystick stick;

  /** Creates a new differential drive Subsystem. */
  public Differential() {
    
    leftParentMotor.setInverted(Constants.Drivetrains.Differential.Motors.FRONT_LEFT_REVERSED);
    rightParentMotor.setInverted(Constants.Drivetrains.Differential.Motors.FRONT_RIGHT_REVERSED);
    leftChildMotor.setInverted(Constants.Drivetrains.Differential.Motors.BACK_LEFT_REVERSED);
    rightChildMotor.setInverted(Constants.Drivetrains.Differential.Motors.BACK_RIGHT_REVERSED);

    leftChildMotor.follow(leftParentMotor);
    rightChildMotor.follow(rightParentMotor);

    drive = new DifferentialDrive(leftParentMotor, rightParentMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DriveArcade (double speedX, double rotationTheta) {
    speedX *= Constants.Drivetrains.Differential.SPEED_LIMIT;
    rotationTheta *= Constants.Drivetrains.Differential.SPEED_LIMIT;

    drive.arcadeDrive(speedX, rotationTheta);
  }
  public void DriveTank (double leftSpeed, double rightSpeed) {
    leftSpeed *= Constants.Drivetrains.Differential.SPEED_LIMIT;
    rightSpeed *= Constants.Drivetrains.Differential.SPEED_LIMIT;

    drive.tankDrive(leftSpeed, rightSpeed);
  } 
// new stuff \/

  //TODO! constants
  boolean autoBalanceXMode;
  boolean autoBalanceYMode;

  public void Balance () {
    drive = new DifferentialDrive(leftParentMotor, rightParentMotor);
         drive.setExpiration(0.1);
      stick = new Joystick(0);
      try {
    /***********************************************************************
     * navX-MXP:
     * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
     * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro:
     * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
     * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP); 
      } catch (RuntimeException ex ) {
          DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
      }
  }
  /**
   * Drive left & right motors for 2 seconds then stop
   */
  public void autonomous() {
    drive.setSafetyEnabled(false);
    drive.arcadeDrive(0.0, 0.5);	// drive forwards half speed					//    for 2 seconds
    drive.arcadeDrive(0.0, 0.0);	// stop robot
  }

  /**
   * Runs the motors with arcade steering.
   */

  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees  = 5;

  
  public void operatorControl() {
    drive.setSafetyEnabled(true);
      while (isOperatorControl() && isEnabled()) {

          double xAxisRate            = stick.getX();
          double yAxisRate            = stick.getY();
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
            drive.arcadeDrive(-stick.getY(), stick.getX());
          } catch( RuntimeException ex ) {
              String err_string = "Drive system error:  " + ex.getMessage();
              DriverStation.reportError(err_string, true);
          }
      }
  }

}
