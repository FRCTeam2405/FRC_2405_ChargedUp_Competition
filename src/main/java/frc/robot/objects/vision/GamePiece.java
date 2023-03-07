// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects.vision;

/** Add your docs here. */
public class GamePiece {

    PieceType category;
    double distance1;
    double bearing1;

    double distance2;
    double bearing2;

    public GamePiece(PieceType category, double distance, double bearing) {
        this.category = category;
        this.distance1 = distance;
        this.bearing1 = bearing;
    }

    public GamePiece(PieceType category, double leftDistance, double rightDistance, double leftBearing, double rightBearing) {
        this.category = category;

        this.distance1 = leftDistance;
        this.bearing1 = leftBearing;

        this.distance2 = rightDistance;
        this.bearing2 = rightBearing;
    }
}
