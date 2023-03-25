// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrains.SwerveContainer;

public class ForwardDock extends CommandBase {

  private SwerveContainer swerve;
  private boolean onChargeStation = false;
  private boolean finished = false;
  private Translation2d startPosition;

  /** Creates a new ForwardDock. */
  public ForwardDock(SwerveContainer swervee) {
    swerve = swervee;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(swerve.getPitch().getDegrees() - 360) > 7) {
      onChargeStation = true;
      startPosition = swerve.getPose().getTranslation();
    }

    if(!onChargeStation) {
      swerve.drive(0.25, 0, 0);
      return;
    }

    double distance =
      swerve.getPose().getTranslation().getX() -
      startPosition.getX();

    SmartDashboard.putNumber("autoDistance", distance);

    if(Math.abs(distance) > 0.06) {
      finished = true;
      return;
    }

    swerve.drive(0.1, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(-0.05, 0, 0);
    swerve.setBrakes(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
