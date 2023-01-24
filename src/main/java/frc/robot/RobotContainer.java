// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drivetrains.Differential;

public class RobotContainer {

  private Differential differentialDrive;
  private XboxController driverController;


  public RobotContainer() {
    configureBindings();
    differentialDrive = new Differential();
    differentialDrive.setDefaultCommand(getDifferentialCommand());
  }

  private void configureBindings() {
    driverController = new XboxController(0); //TODO! convert to constant
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getDifferentialCommand() {
    return new RunCommand(() -> differentialDrive.DriveTank(
      driverController.getLeftY(),
      driverController.getRightY()
    ));
  }
}
