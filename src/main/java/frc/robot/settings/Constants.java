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

    public static final class EdgeDetection {
        public static final int FRONT_LEFT_PING = 0;
        public static final int FRONT_RIGHT_PING = 2;
        public static final int BACK_LEFT_PING = 4;
        public static final int BACK_RIGHT_PING = 6;

        public static final int FRONT_LEFT_ECHO = 1;
        public static final int FRONT_RIGHT_ECHO = 3;
        public static final int BACK_LEFT_ECHO = 5;
        public static final int BACK_RIGHT_ECHO = 7;
    }
}
