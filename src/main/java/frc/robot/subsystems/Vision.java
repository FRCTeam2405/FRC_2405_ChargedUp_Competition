// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  NetworkTable jetson;

  DoubleArraySubscriber distances;
  DoubleArraySubscriber bearings;
  IntegerArraySubscriber categories;

  /** Creates a new Vision. */
  public Vision() {
    jetson = NetworkTableInstance.getDefault().getTable("Jetson");

    distances = jetson.getDoubleArrayTopic("Distance").subscribe(new double[] {});
    bearings = jetson.getDoubleArrayTopic("Bearing").subscribe(new double[] {});
    categories = jetson.getIntegerArrayTopic("Category").subscribe(new long[] {});
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("distance0", distances.get()[0]);
    SmartDashboard.putNumber("bearing0", bearings.get()[0]);
    SmartDashboard.putNumber("category0", categories.get()[0]);
  }
}
