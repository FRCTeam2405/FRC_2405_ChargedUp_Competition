// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
                public static final int FRONT_LEFT_PORT = 0;
                public static final int FRONT_RIGHT_PORT = 0;
                public static final int BACK_LEFT_PORT = 0;
                public static final int BACK_RIGHT_PORT = 0;
                
                public static final class Encoders {
                    public static final int FRONT_LEFT_PORT = 0;
                    public static final int FRONT_RIGHT_PORT = 0;
                    public static final int BACK_LEFT_PORT = 0;
                    public static final int BACK_RIGHT_PORT = 0;
                }
            }
        }

        public static final class Swerve {
            public static final class Odometry {
            //TODO! FIX ALL
                // Chassis configuration
                public static final double kTrackWidth = Units.inchesToMeters(26.5);
                // Distance between centers of right and left wheels on robot
                public static final double kWheelBase = Units.inchesToMeters(26.5);
                // Distance between front and back wheels on robot
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                // Angular offsets of the modules relative to the chassis in radians
                public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
                public static final double kFrontRightChassisAngularOffset = 0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2;
            }

            public static final class Motors {
                public static final class Drive {
                    //TODO! fix later
                    public static final int FRONT_LEFT_PORT = 0;
                    public static final int FRONT_RIGHT_PORT = 0;
                    public static final int BACK_LEFT_PORT = 0;
                    public static final int BACK_RIGHT_PORT = 0;
                }

                public static final class Turning {
                    //TODO! fix later
                    public static final int FRONT_LEFT_PORT = 0;
                    public static final int FRONT_RIGHT_PORT = 0;
                    public static final int BACK_LEFT_PORT = 0;
                    public static final int BACK_RIGHT_PORT = 0;
                }
            }

            public static final class Encoders {
                //TODO! fix later
                public final int QUAD_COUNTS_PER_ROTATION = 4096;
                public static final int FRONT_LEFT_PORT = 0;
                public static final int FRONT_RIGHT_PORT = 0;
                public static final int BACK_LEFT_PORT = 0;
                public static final int BACK_RIGHT_PORT = 0;
            }
        }
    }
}
