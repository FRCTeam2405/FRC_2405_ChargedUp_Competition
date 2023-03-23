// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.BackwardDock;
import frc.robot.commands.autonomous.ForwardDock;
import frc.robot.commands.autonomous.PlacePiece;
import frc.robot.commands.autonomous.YawTest;
import frc.robot.commands.autonomous.arm.AutoOutputPiece;
import frc.robot.commands.autonomous.arm.positions.AutoCollapseArm;
import frc.robot.commands.autonomous.arm.positions.AutoPlaceLow;
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
import frc.robot.commands.swerve.RecenterRotation;
import frc.robot.commands.swerve.SnapRotation;
import frc.robot.commands.swerve.SwerveAutobalance;
import frc.robot.commands.swerve.SwerveBrake;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Drivetrains.Swerve.Paths;
import frc.robot.settings.DashboardConfig;
import frc.robot.subsystems.EdgeDetector;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrains.SwerveContainer;
import frc.robot.subsystems.intake.Arm;
import frc.robot.subsystems.intake.Grip;

public class RobotContainer {

  private final DashboardConfig config;

  // Declare subsystems
  private final SwerveContainer swerveDrive;
  private final Arm arm;
  private final Grip grip;
  final Lights lights = new Lights();
  private EdgeDetector edgeDetector;
  private Limelight limelight;
  
  private final Compressor airCompressor = new Compressor(Constants.Intake.Ports.COMPRESSOR, PneumaticsModuleType.CTREPCM);

  private SendableChooser<Command> autonomousDropDown;

  // Declare controllers
  // private Joystick driverStick = new Joystick(Constants.Controllers.DRIVER_JOYSTICK_PORT);
  // private Joystick driverWheel = new Joystick(Constants.Controllers.DRIVER_WHEEL_PORT);

  private CommandXboxController driverController = new CommandXboxController(0);
  private CommandXboxController codriverController = new CommandXboxController(1);

  // Globally loaded path planner stuff
  private SwerveAutoBuilder pathBuilder;
  private HashMap<String, Command> commandMap;

  public RobotContainer() {

    config = new DashboardConfig();

    swerveDrive = new SwerveContainer();
    arm = new Arm();
    grip = new Grip();
    limelight = new Limelight();
    edgeDetector = new EdgeDetector();

    configureBindings();
    configureCommands();

    // Set default commands
  }

  private DoubleSupplier axisDeadband(CommandXboxController controller, int axis, double deadband, boolean inverted, double multiplier) {
    double invertedMultiplier = inverted ? -multiplier : multiplier;
    return () -> (
      Math.abs(controller.getRawAxis(axis)) > deadband
    ) ? controller.getRawAxis(axis) * invertedMultiplier : 0;
  }

  private void configureBindings() {
    // DRIVER CONTROLS

    // Driving the robot: left stick for movement, right stick for turning, LB to quarter speed
    swerveDrive.setDefaultCommand(new AbsoluteDrive3Axis(
      swerveDrive,
      axisDeadband(driverController, XboxController.Axis.kLeftY.value, Constants.Controllers.joystickDeadband, true, 1.0),
      axisDeadband(driverController, XboxController.Axis.kLeftX.value, Constants.Controllers.joystickDeadband, true, 1.0),
      axisDeadband(driverController, XboxController.Axis.kRightX.value, Constants.Controllers.wheelDeadband, false, 1.0)
    ));
    driverController.leftBumper().whileTrue(new AbsoluteDrive3Axis(
      swerveDrive,
      axisDeadband(driverController, XboxController.Axis.kLeftY.value, Constants.Controllers.joystickDeadband, true, 0.25),
      axisDeadband(driverController, XboxController.Axis.kLeftX.value, Constants.Controllers.joystickDeadband, true, 0.25),
      axisDeadband(driverController, XboxController.Axis.kRightX.value, Constants.Controllers.wheelDeadband, false, 0.25)
    ));

    // Manipulating the claw: A to open, B to close;
    // RT to drive forward, LT to drive backward
    driverController.a().onTrue(new OpenGrip(grip));
    driverController.b().onTrue(new CloseGrip(grip));
    driverController.x().onTrue(new SwerveAutobalance(swerveDrive));
    driverController.y().onTrue(new RecenterRotation(swerveDrive));
    driverController.rightBumper().whileTrue(new SwerveBrake(swerveDrive));
    driverController.rightTrigger().whileTrue(new IntakePiece(grip));
    driverController.leftTrigger().whileTrue(new OutputPiece(grip));

    driverController.povUp().onTrue(new SnapRotation(swerveDrive, 0));
    driverController.povRight().onTrue(new SnapRotation(swerveDrive, (0.5 * Math.PI)));
    driverController.povDown().onTrue(new SnapRotation(swerveDrive, (1 * Math.PI)));
    driverController.povLeft().onTrue(new SnapRotation(swerveDrive, (1.5 *  Math.PI)));


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
    hitPov.onTrue(new CollapseArm(arm, lights));
    codriverController.a().onTrue(new PlaceLow(arm, lights));
    codriverController.x().onTrue(new PlaceMed(arm, lights));
    codriverController.y().onTrue(new PlaceHigh(arm, lights));
  
    // Pickup Moves: B to tip the arm down, LB to pick up from the shelf,
    // RB to pick up from the chute
    codriverController.b().onTrue(new PickupTipped(arm, lights));
    codriverController.leftBumper().onTrue(new PickupChute(arm, lights));
    codriverController.rightBumper().onTrue(new PickupShelf(arm, lights));

    // Manually moving the arm: Left Stick Y for moving the arm, Right Stick Y
    // for moving the wrist
    codriverController.rightTrigger().whileTrue(new MoveArm(
      arm,
      axisDeadband(
        codriverController,
        XboxController.Axis.kLeftY.value,
        0.1,
        false,
        1.0
      )
    ));
    codriverController.leftTrigger().whileTrue(new MoveWrist(
      arm,
      axisDeadband(
        codriverController,
        XboxController.Axis.kRightY.value,
        0.1,
        false,
        1.0
      )
    ));
    
  }

  private void configureCommands() {

    commandMap = new HashMap<>();

    // Connect markers in the path file to our auton commands
    commandMap.put("collapseArm", new AutoCollapseArm(arm, lights));
    commandMap.put("placeLow", new AutoPlaceLow(arm, lights));
    commandMap.put("outputPiece", new AutoOutputPiece(grip));
    commandMap.put("autobalance", new SwerveAutobalance(swerveDrive));
    commandMap.put("lazyAutobalance", new ForwardDock(swerveDrive));
    commandMap.put("backAutobalance", new BackwardDock(swerveDrive));

    pathBuilder = new SwerveAutoBuilder(
      swerveDrive::getPose,
      swerveDrive::resetPose,
      new PIDConstants(0.7, 0, 0),
      new PIDConstants(0.4, 0, 0.01),
      swerveDrive::setChassisSpeeds,
      commandMap,
      true,
      swerveDrive
    );

    PathPlannerTrajectory pio = PathPlanner.loadPath(Paths.PIO, 1, 1);
    PathPlannerTrajectory wio = PathPlanner.loadPath(Paths.WIO, 1, 1);
    PathPlannerTrajectory cioda = PathPlanner.loadPath(Paths.CIODA, 1, 1);
    PathPlannerTrajectory ciodn = PathPlanner.loadPath(Paths.CIODN, 1, 1);
    PathPlannerTrajectory out = PathPlanner.loadPath("[Either Side] Out", 1, 1);
    PathPlannerTrajectory blueCenterDock = PathPlanner.loadPath("[Center] Dock", 1, 1);

    // PathPlannerTrajectory bpio = PathPlanner.loadPath("[Blue Pickup Side] In, Out", 1, 1);
    // PathPlannerTrajectory bwio = PathPlanner.loadPath("[Blue Wire Side] In, Out", 1, 1);

    autonomousDropDown = new SendableChooser<>();

    autonomousDropDown.setDefaultOption(Paths.PIO, pathBuilder.fullAuto(pio));
    autonomousDropDown.addOption(Paths.WIO, pathBuilder.fullAuto(wio));
    autonomousDropDown.addOption(Paths.CIODA, pathBuilder.fullAuto(cioda));
    autonomousDropDown.addOption(Paths.CIODN, pathBuilder.fullAuto(ciodn));
    autonomousDropDown.addOption("Place Piece, no movement", new PlacePiece(arm, grip, lights));
    autonomousDropDown.addOption("[Either Side] Out", pathBuilder.fullAuto(out));
    // autonomousDropDown.addOption("[Blue Pickup Side] In, Out", pathBuilder.fullAuto(bpio));
    // autonomousDropDown.addOption("[Blue Wire Side] In, Out", pathBuilder.fullAuto(bwio));
    autonomousDropDown.addOption("Yaw Test", new YawTest(swerveDrive));
    autonomousDropDown.addOption("[Center] Dock", pathBuilder.followPath(blueCenterDock));
    autonomousDropDown.addOption("[Center] Dock", new BackwardDock(swerveDrive));

    SmartDashboard.putData("Auton Routine", autonomousDropDown);
  }

  public Command getAutonomousCommand() {
    Command command = autonomousDropDown.getSelected();
    if(command == null) {
      return Commands.print("No autonomous command selected.");
    }
    return command;
  }

  public void unbrake() {
    swerveDrive.setBrakes(false);
  }
}
