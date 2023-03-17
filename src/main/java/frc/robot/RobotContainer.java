// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.swerve.AbsoluteDrive;
import frc.robot.commands.swerve.SwerveAutobalance;
import frc.robot.commands.swerve.SwerveBrake;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.EdgeDetector;
import frc.robot.subsystems.drivetrains.Differential;
import frc.robot.commands.SetLEDLights;
import frc.robot.commands.autonomous.AutoRoutineType;
import frc.robot.settings.Constants;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.intake.arm.MoveArm;
import frc.robot.commands.intake.arm.MoveWrist;
import frc.robot.commands.intake.arm.positions.CollapseArm;
import frc.robot.commands.intake.arm.positions.pickup.PickupChute;
import frc.robot.commands.intake.arm.positions.pickup.PickupShelf;
import frc.robot.commands.intake.arm.positions.pickup.PickupTipped;
import frc.robot.commands.intake.arm.positions.placing.PlaceHigh;
import frc.robot.commands.intake.arm.positions.placing.PlaceLow;
import frc.robot.commands.intake.arm.positions.placing.PlaceMed;
import frc.robot.commands.intake.grip.CloseGrip;
import frc.robot.commands.intake.grip.IntakePiece;
import frc.robot.commands.intake.grip.OpenGrip;
import frc.robot.commands.intake.grip.OutputPiece;
import frc.robot.commands.swerve.AbsoluteDrive3Axis;
import frc.robot.settings.Constants;
import frc.robot.settings.DashboardConfig;
import frc.robot.settings.Constants.Controllers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrains.SwerveContainer;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class RobotContainer {

  private final DashboardConfig config;

  private AutoRoutineType chosenRoutine = AutoRoutineType.IN_AND_OUT;

  // Declare subsystems
  private final SwerveContainer swerveDrive;
  private final Intake intake;
  final Lights lights = new Lights();
  private EdgeDetector edgeDetector;
  private Limelight limelight;
  
  private final Compressor airCompressor = new Compressor(Constants.Intake.Ports.COMPRESSOR, PneumaticsModuleType.CTREPCM);

  //Declare commands
  private final SwerveAutobalance commandBalance;

  private SendableChooser<Command> autonomousDropDown;

  // Declare controllers
  // private Joystick driverStick = new Joystick(Constants.Controllers.DRIVER_JOYSTICK_PORT);
  // private Joystick driverWheel = new Joystick(Constants.Controllers.DRIVER_WHEEL_PORT);

  private CommandXboxController driverController = new CommandXboxController(0);
  private CommandXboxController codriverController = new CommandXboxController(1);

  // Globally loaded path planner stuff
  private SwerveAutoBuilder pathBuilder;
  private HashMap<String, Command> commandMap;

  private PathPlannerTrajectory inAndOut;


  public RobotContainer() {

    config = new DashboardConfig();

    swerveDrive = new SwerveContainer();
    intake = new Intake();
    limelight = new Limelight();
    edgeDetector = new EdgeDetector();

    commandBalance = new SwerveAutobalance(swerveDrive);

    configureBindings();
    configureCommands();

    // Set default commands
  }

  private DoubleSupplier axisDeadband(CommandXboxController controller, int axis, double deadband, boolean inverted) {
    double invertedMultiplier = inverted ? -1.0 : 1.0;
    return () -> (
      Math.abs(controller.getRawAxis(axis)) > deadband
    ) ? controller.getRawAxis(axis) * invertedMultiplier : 0;
  }

  private void configureBindings() {
    // DRIVER CONTROLS

    // Driving the robot: left stick for movement, right stick for turning
    swerveDrive.setDefaultCommand(new AbsoluteDrive3Axis(
      swerveDrive,
      axisDeadband(driverController, XboxController.Axis.kLeftY.value, Constants.Controllers.joystickDeadband, true),
      axisDeadband(driverController, XboxController.Axis.kLeftX.value, Constants.Controllers.joystickDeadband, true),
      axisDeadband(driverController, XboxController.Axis.kRightX.value, Constants.Controllers.wheelDeadband, true)
    ));

    // Manipulating the claw: A to open, B to close;
    // RT to drive forward, LT to drive backward
    driverController.a().onTrue(new OpenGrip(intake));
    driverController.b().onTrue(new CloseGrip(intake));
    driverController.x().onTrue(new SwerveAutobalance(swerveDrive));
    driverController.rightBumper().whileTrue(new SwerveBrake(swerveDrive));
    driverController.rightTrigger().whileTrue(new IntakePiece(intake));
    driverController.leftTrigger().whileTrue(new OutputPiece(intake));


    // CODRIVER CONTROLS

    // Triggers if any dpad position is hit, except for middle (untouched dpad)
    Trigger hitPov = codriverController.povUp()
      .or(codriverController.povUpRight())
      .or(codriverController.povRight())
      .or(codriverController.povDownRight())
      .or(codriverController.povDown())
      .or(codriverController.povDownLeft())
      .or(codriverController.povLeft())
      .or(codriverController.povUpLeft());

    // Auto moving the arm: DPad to collapse it, A/X/Y to move the arm Low/Med/High 
    hitPov.onTrue(new CollapseArm(intake, lights));
    codriverController.a().onTrue(new PlaceLow(intake, lights));
    codriverController.x().onTrue(new PlaceMed(intake, lights));
    codriverController.y().onTrue(new PlaceHigh(intake, lights));

    // Pickup Moves: B to tip the arm down, LB to pick up from the shelf,
    // RB to pick up from the chute
    codriverController.b().onTrue(new PickupTipped(intake, lights));
    codriverController.leftBumper().onTrue(new PickupChute(intake, lights));
    codriverController.rightBumper().onTrue(new PickupShelf(intake, lights));

    // Manually moving the arm: Left Stick Y for moving the arm, Right Stick Y
    // for moving the wrist
    codriverController.rightTrigger().whileTrue(new MoveArm(
      intake,
      axisDeadband(
        codriverController,
        XboxController.Axis.kLeftY.value,
        0.1,
        false
      )
    ));
    codriverController.leftTrigger().whileTrue(new MoveWrist(
      intake,
      axisDeadband(
        codriverController,
        XboxController.Axis.kRightY.value,
        0.1,
        false
      )
    ));
    
  }

  private void configureCommands() {

    commandMap = new HashMap<>();

    pathBuilder = new SwerveAutoBuilder(
      swerveDrive::getPose,
      swerveDrive::resetPose,
      new PIDConstants(0, 0, 0),
      new PIDConstants(0, 0, 0),
      swerveDrive::setChassisSpeeds,
      commandMap,
      swerveDrive
    );

    inAndOut = PathPlanner.loadPath("In and Out", 3, 4);

    autonomousDropDown = new SendableChooser<>();

    autonomousDropDown.addOption("In and Out", pathBuilder.followPath(inAndOut));
    autonomousDropDown.addOption("None", null);
  }

  public Command getAutonomousCommand() {
    Command command = autonomousDropDown.getSelected();
    if(command == null) {
      return Commands.print("No autonomous command selected.");
    }
    return command;
  }
}
