// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects.vision;

/** Add your docs here. */
public enum PieceType {
    CONE(0),
    CUBE(1),
    ROBOT(2);

    public final int typeInt;
    private PieceType(int type) { this.typeInt = type; }
}
