// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DashboardConfig {

    public DashboardConfig() {
        SmartDashboard.putNumber("TurningP", Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_P);
        SmartDashboard.putNumber("TurningI", Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_I);
        SmartDashboard.putNumber("TurningD", Constants.Drivetrains.Swerve.Module.PID.TURNING_MOTOR_D);
    }

}
