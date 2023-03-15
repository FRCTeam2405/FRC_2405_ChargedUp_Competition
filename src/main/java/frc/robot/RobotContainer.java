// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.swerve.AbsoluteDrive;
import frc.robot.commands.swerve.SwerveAutobalence;
import frc.robot.settings.Constants;
import frc.robot.settings.DashboardConfig;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class RobotContainer {

  private final DashboardConfig config;

  // Declare subsystems
  private final SwerveContainer swerveDrive;

  //Declare commands
  private final SwerveAutobalence commandBalence;

  // Declare controllers
  private Joystick driverLeftStick = new Joystick(Constants.Controllers.DRIVER_CONTROLLER_PORT);
  private Joystick driverRightStick = new Joystick(Constants.Controllers.DRIVER_CONTROLLER_PORT);

  private CommandXboxController driverController = new CommandXboxController(Constants.Controllers.DRIVER_CONTROLLER_PORT);
  // private final JoystickButton driverMainButtonX = new JoystickButton(driverController, Constants.Controllers.DriverController.DRIVER_CONTROLLER_BUTTON_X);
  // private final JoystickButton driverMainButtonB = new JoystickButton(driverController, Constants.Controllers.DriverController.DRIVER_CONTROLLER_BUTTON_B);
  // private final JoystickButton driverMainButtonY = new JoystickButton(driverController, Constants.Controllers.DriverController.DRIVER_CONTROLLER_BUTTON_Y);
  // private final JoystickButton driverMainButtonA = new JoystickButton(driverController, Constants.Controllers.DriverController.DRIVER_CONTROLLER_BUTTON_A);

  public RobotContainer() {

    config = new DashboardConfig();

    swerveDrive = new SwerveContainer();

    commandBalence = new SwerveAutobalence(swerveDrive);

    configureBindings();

    // Set default commands
    // swerveDrive.setDefaultCommand(new AbsoluteDrive(
    //   swerveDrive,
    //   axisDeadband(driverController, XboxController.Axis.kLeftY.value),
    //   axisDeadband(driverController, XboxController.Axis.kLeftX.value),
    //   axisRaw(driverController, XboxController.Axis.kRightX.value),
    //   axisRaw(driverController, XboxController.Axis.kRightY.value)
    // ));
  }

  private DoubleSupplier axisDeadband(XboxController controller, int axis) {
    return () -> (
      Math.abs(controller.getRawAxis(axis)) > Constants.Controllers.DriverController.joystickDeadband
    ) ? controller.getRawAxis(axis) : 0;
  }

  private DoubleSupplier axisRaw(XboxController controller, int axis) {
    return () -> controller.getRawAxis(axis);
  }

  private void configureBindings() {
    //driverMainButtonX.whenHeld();
    //driverMainButtonB.whenHeld();
    //driverMainButtonY.whenHeld();
    driverController.a().whileTrue(commandBalence);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
