// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

/** Program-wide constants for the robot.
 *  For settings that should be changed before
 *  building and deploying the code.
 */
//TODO! Init values
public final class Constants {
    /** This year we have to support multiple drivetrains, so each drivetrain is a subclass. */
    public static final class Drivetrains {
        public static final class Differential {
            public static final class Motors {
                public static final class Ports {
                    //TODO! Init these values
                    public static final int FRONT_LEFT = 10;
                    public static final int FRONT_RIGHT = 11;
                    public static final int BACK_LEFT = 12;
                    public static final int BACK_RIGHT = 13;
                }
                
                public static final class Encoders {
                    public static final int FRONT_LEFT_PORT = 0;
                    public static final int FRONT_RIGHT_PORT = 0;
                    public static final int BACK_LEFT_PORT = 0;
                    public static final int BACK_RIGHT_PORT = 0;

                }

                public static final boolean FRONT_LEFT_REVERSED = true;
                public static final boolean FRONT_RIGHT_REVERSED = false;
                public static final boolean BACK_LEFT_REVERSED = true;
                public static final boolean BACK_RIGHT_REVERSED = false;

            }

            public static final double SPEED_LIMIT = 0.4;
        }

        public static final class Swerve {
            public static final class Motors {
                public static final class Drive {
                    public static final int FRONT_LEFT_PORT = 0;
                    public static final int FRONT_RIGHT_PORT = 0;
                    public static final int BACK_LEFT_PORT = 0;
                    public static final int BACK_RIGHT_PORT = 0;
                }

                public static final class Turning {
                    public static final int FRONT_LEFT_PORT = 0;
                    public static final int FRONT_RIGHT_PORT = 0;
                    public static final int BACK_LEFT_PORT = 0;
                    public static final int BACK_RIGHT_PORT = 0;
                }
            }

            public static final class EncoderShaft {
                public static final int FRONT_LEFT_PORT = 0;
                public static final int FRONT_RIGHT_PORT = 0;
                public static final int BACK_LEFT_PORT = 0;
                public static final int BACK_RIGHT_PORT = 0;
            }
        }
    }

    public static final class Intake {

        public static final double INTAKE_SPEED = 0;
        public static final double OUTPUT_SPEED = 0;
        
        public static final class Positions {
            /**
             * Lowest arm position, for intake
             * and output to the bottom row.
             * Measured in degrees.
             */
            public static final double LOW_ARM = 0;
            /**
             * Medium arm position, for output
             * to the middle row. Measured in degrees.
             */
            public static final double MED_ARM = 0;
            /**
             * Highest arm position, for output
             * to the top row. Measured in degrees.
             */
            public static final double HIGH_ARM = 0;


            /**
             * Inner wrist position, for storage
             * and protection. Measured in degrees.
             */
            public static final double COLLAPSED_WRIST = 0;
            /** 
             * Outer wrist position, for intake
             * and output. Measured in degrees.
            */
            public static final double EXTENDED_WRIST = 0;

        }

        public static final class Ports {
            public static final int ARM_MOTOR = 0;
            public static final int WRIST_MOTOR = 0;
            public static final int LEFT_GRIP_MOTOR = 0;
            public static final int RIGHT_GRIP_MOTOR = 0;
        }

    }
}
