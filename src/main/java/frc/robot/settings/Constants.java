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
                    public static final int FRONT_LEFT = 0;
                    public static final int FRONT_RIGHT = 0;
                    public static final int BACK_LEFT = 0;
                    public static final int BACK_RIGHT = 0;
                }
            }
        }
    }

    public static final class GrabbingArm {
        
        /** Lowest arm position, for intake
         * and output to the bottom row.
         * Measured in degrees of the driving motor.
        */
        public static final int LOW_ARM_POSITION = 0;
        /** Medium arm position, for output
         * to the middle row.
         * Measured in degrees of the driving motor.
        */
        public static final int MED_ARM_POSITION = 0;
        /** Highest arm position, for output
         * to the top row.
         * Measured in degrees of the driving motor.
        */
        public static final int HIGH_ARM_POSITION = 0;

    }
}
