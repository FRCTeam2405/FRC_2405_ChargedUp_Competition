// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

/** Program-wide constants for the robot.
 *  For settings that should be changed before
 *  building and deploying the code.
 */
public final class Constants {
    /** This year we have to support multiple drivetrains, so each drivetrain is a subclass. */
    public static final class Drivetrains {
        public static final class Differential {
            public static final class Motor {
                public static final class Ports {
                    //TODO! Init these values
                    public static final int FRONT_LEFT = 10;
                    public static final int FRONT_RIGHT = 11;
                    public static final int BACK_LEFT = 12;
                    public static final int BACK_RIGHT = 13;
                }

                public static final boolean FRONT_LEFT_REVERSED = true;
                public static final boolean FRONT_RIGHT_REVERSED = false;
                public static final boolean BACK_LEFT_REVERSED = true;
                public static final boolean BACK_RIGHT_REVERSED = false;

            }

            public static final double SPEED_LIMIT = 0.4;
        }
    }
}
