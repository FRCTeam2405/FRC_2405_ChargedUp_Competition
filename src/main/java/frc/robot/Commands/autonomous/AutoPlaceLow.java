// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.LEDs.Colors;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class AutoPlaceLow extends CommandBase {

  Timer timer;
  double startTime;
  
  private Intake intake;
  private Lights lights;

  public AutoPlaceLow(Intake setIntake, Lights setLights) {

    timer = new Timer();

    intake = setIntake;
    lights = setLights;
    // Declare subsystem dependencies.
    addRequirements(intake, lights);
  }

  @Override
  public void initialize() {

    timer.start();
    startTime = timer.get();

    intake.desiredArmPosition = Constants.Intake.Positions.Arm.LOW;
    intake.desiredWristPosition = Constants.Intake.Positions.Wrist.LOW;
    lights.setColor(Colors.GREEN);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if((timer.get() - startTime) > 0.5) {
      return true;
    }

    return false;
  }
}
