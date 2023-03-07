// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.vision.GamePiece;
import frc.robot.objects.vision.PieceType;

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

  public double[] getDistances() {
    return distances.get();
  }

  public double[] getBearings() {
    return bearings.get();
  }

  public long[] getCategories() {
    return categories.get();
  }

  public GamePiece[] getGamePieces(PieceType type) {
    double[] currentDistances = getDistances();
    double[] currentBearings = getBearings();
    long[] currentCategories = getCategories();

    int robots = 0;
    int dataIndex = 0;
    List<GamePiece> pieces = new ArrayList<>();

    for(int i = 0; i < currentCategories.length; i++) {
      long category = currentCategories[i];

      if(category == PieceType.ROBOT.typeInt) {
        robots++;
      } 
      if(category != type.typeInt) {
        continue;
      }

      dataIndex = i + robots;

      if(type == PieceType.ROBOT) {
        double leftDistance = currentDistances[dataIndex];
        double rightDistance = currentDistances[dataIndex + 1];

        double leftBearing = currentBearings[dataIndex];
        double rightBearing = currentBearings[dataIndex + 1];

        pieces.add(
          new GamePiece(
            type,
            leftDistance,
            rightDistance,
            leftBearing,
            rightBearing
          )
        );

      } else {
        double distance = currentDistances[dataIndex];
        double bearing = currentBearings[dataIndex];

        pieces.add(
          new GamePiece(type, distance, bearing)
        );
      }

      
    }

    GamePiece[] piecesArray = new GamePiece[pieces.size()];
    piecesArray = pieces.toArray(piecesArray);
    return piecesArray;
  }
}
