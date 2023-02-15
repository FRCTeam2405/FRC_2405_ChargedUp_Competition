// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import com.revrobotics.CANSparkMax.IdleMode;

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

            public static final boolean FIELD_RELATIVE = false;

            public static final class Speed {
                public static final double MAX_SPEED_METERS_PER_SECONDS = 0.5;
                public static final double MAX_ANGULAR_SPEED = 0.25 * Math.PI; // radians per second
            }
            public static final class Odometry {
                //TODO! FIX ALL
                // Chassis configuration
                
                // Distance between centers of right and left wheels on robot
                public static final double TRACK_WIDTH = Units.inchesToMeters(26.5);
                // Distance between front and back wheels on robot
                public static final double WHEEL_BASE = Units.inchesToMeters(26.5);

                // Angular offsets of the modules relative to the chassis in radians
                public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
                public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
                public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
                public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

                public static final boolean GYRO_REVERSED = false;
            }

            public static final class Motors {
                public static final class Drive {
                    //TODO! fix later
                    public static final int FRONT_LEFT_PORT = 20;
                    public static final int FRONT_RIGHT_PORT = 21;
                    public static final int BACK_LEFT_PORT = 22;
                    public static final int BACK_RIGHT_PORT = 23;
                }

                public static final class Turning {
                    //TODO! fix later
                    public static final int FRONT_LEFT_PORT = 30;
                    public static final int FRONT_RIGHT_PORT = 31;
                    public static final int BACK_LEFT_PORT = 32;
                    public static final int BACK_RIGHT_PORT = 33;
                }
            }

            public static final class Encoders {
                //TODO! fix later
                public final int QUAD_COUNTS_PER_ROTATION = 4096;
                public static final int FRONT_LEFT_PORT = 40;
                public static final int FRONT_RIGHT_PORT = 41;
                public static final int BACK_LEFT_PORT = 42;
                public static final int BACK_RIGHT_PORT = 43;

                public static final double ERROR_TOLERANCE = 3.0;
                public static final double DERIVATIVE_TOLERANCE = 1.0;
            }

            public static final class Module {

                public static final String FRONT_LEFT_NAME = "FrontLeft";
                public static final String FRONT_RIGHT_NAME = "FrontRight";
                public static final String BACK_LEFT_NAME = "BackLeft";
                public static final String BACK_RIGHT_NAME = "BackRight";


                //TODO! ALL #'s
                //TODO! Organize these please
            
                // Invert the turning encoder, since the output shaft rotates in the opposite direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean TURNING_ENCODER_INVERTED = true;
                
                // Calculations required for driving motor conversion factors and feed forward
                public static final double FREE_SPEED_RPM = 5676;

                public static final double FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
                public static final double WHEEL_DIAMETER_METERS = 0.096012;
                public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion ex. public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
                public static final double DRIVING_MOTOR_REDUCTION = 7.13;
                public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
                    / DRIVING_MOTOR_REDUCTION;
                
                public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
                    / DRIVING_MOTOR_REDUCTION; // meters
                public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
                    / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second
                
                public static final double TURNING_GEAR_RATIO = 15.42857;
                public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI) / TURNING_GEAR_RATIO; // radians
                public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / TURNING_GEAR_RATIO / 60.0; // radians per second
                
                public static final double TURNING_ENCODER_POSITION_PID_MINIMUM_INPUT = 0; // radians
                public static final double TURNING_ENCODER_POSITION_PID_MAXIMUM_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians
            
                public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
                public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;
                
                public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
                public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps

                public static final double DIR_SLEW_RATE_LIMIT = 1.2;
                public static final double MAG_SLEW_RATE_LIMIT = 1.8;
                public static final double ROT_SLEW_RATE_LIMIT = 2.0;
            
                public static final class PID {
                    public static final double DRIVING_MOTOR_P = 0.04;
                    public static final double DRIVING_MOTOR_I = 0;
                    public static final double DRIVING_MOTOR_D = 0;
                    public static final double DRIVING_MOTOR_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
                    public static final double DRIVING_MOTOR_MINIMUM_OUTPUT = -0.4;
                    public static final double DRIVING_MOTOR_MAXIMUM_OUTPUT = 0.4;

                    public static final double TURNING_MOTOR_P = 0.25;
                    public static final double TURNING_MOTOR_I = 0.0005;
                    public static final double TURNING_MOTOR_D = 0.0001;
                    public static final double TURNING_MOTOR_FF = 0 / DRIVE_WHEEL_FREE_SPEED_RPS;
                    public static final double TURNING_MOTOR_MINIMUM_OUTPUT = -0.4;
                    public static final double TURNING_MOTOR_MAXIMUM_OUTPUT = 0.4;
                
                }
            }
        }
    }

    public static final class Controllers {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
