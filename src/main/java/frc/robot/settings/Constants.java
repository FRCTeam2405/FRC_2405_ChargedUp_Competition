// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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

            public static final boolean FIELD_RELATIVE = true;
            public static final boolean OPEN_LOOP = false;

            public static final class Speed {
                public static final double MAX_TRANSLATION_MPS = 4.0;
                public static final double MAX_ANGULAR_RPS = 0.4 * (2 * Math.PI); // radians per second
            }

            public static final class Odometry {
                //TODO! FIX ALL
                // Chassis configuration
                
                /** Distance between centers of right and left wheels on robot */
                public static final double TRACK_WIDTH = 0.5461;
                /** Distance between front and back wheels on robot */
                public static final double WHEEL_BASE = 0.5715;

                // Angular offsets of the modules relative to the chassis in radians
                public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
                public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
                public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
                public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

                public static final boolean GYRO_REVERSED = false;
            }

            public static final class Measurements {
                // Calculations required for driving motor conversion factors and feed forward
                public static final double FREE_SPEED_RPM = 5676;
                public static final double FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
                public static final double WHEEL_DIAMETER_METERS = 0.1524;
                public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion ex. public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
                public static final double DRIVING_GEAR_RATIO = 7.13;
                public static final double TURNING_GEAR_RATIO = 15.42857;
                
                public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_GEAR_RATIO;

                public static final double ROBOT_MASS = 0;
                public static final double CHASSIS_MASS = 0;
                public static final double MESSAGE_LOOP_TIME = 0.13;
                public static final Matter CHASSIS_CG = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), CHASSIS_MASS);
            }

            public static final class Motors {
                public static final class Driving {
                    //TODO! fix later
                    public static final int FRONT_LEFT_PORT = 20;
                    public static final int FRONT_RIGHT_PORT = 21;
                    public static final int BACK_LEFT_PORT = 22;
                    public static final int BACK_RIGHT_PORT = 23;

                    public static final double MINIMUM_OUTPUT = -0.4;
                    public static final double MAXIMUM_OUTPUT = 0.4;
                    
                    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

                    public static final int CURRENT_LIMIT = 50; // amps
                }

                public static final class Turning {
                    //TODO! fix later
                    public static final int FRONT_LEFT_PORT = 30;
                    public static final int FRONT_RIGHT_PORT = 31;
                    public static final int BACK_LEFT_PORT = 32;
                    public static final int BACK_RIGHT_PORT = 33;

                    public static final double MINIMUM_OUTPUT = -0.6;
                    public static final double MAXIMUM_OUTPUT = 0.6;

                    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

                    public static final int CURRENT_LIMIT = 20; // amps
                }
            }

            public static final class Encoders {
                //TODO! fix later
                public final int QUAD_COUNTS_PER_ROTATION = 4096;

                public static final int FRONT_LEFT_PORT = 40;
                public static final int FRONT_RIGHT_PORT = 41;
                public static final int BACK_LEFT_PORT = 42;
                public static final int BACK_RIGHT_PORT = 43;

                public static final double FRONT_LEFT_OFFSET = 58.7;
                public static final double FRONT_RIGHT_OFFSET = 69.6;
                public static final double BACK_LEFT_OFFSET = 168.0;
                public static final double BACK_RIGHT_OFFSET = 3.7;

                public static final class Driving {                  
                    public static final double POSITION_FACTOR = Measurements.WHEEL_CIRCUMFERENCE_METERS / Measurements.DRIVING_GEAR_RATIO; // meters
                    public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0;
                }

                public static final class Turning {
                    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
                    // the steering motor in the MAXSwerve Module.
                    public static final boolean TURNING_ENCODER_INVERTED = true;

                    public static final double POSITION_FACTOR = (2 * Math.PI) / Measurements.TURNING_GEAR_RATIO; // radians
                    public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0; // radians per second
                }
                
            }

            public static final class PID {
                public static final class Driving {
                    public static final double P = 0.04;
                    public static final double I = 0;
                    public static final double D = 0;
                    public static final double FF = 1 / Measurements.DRIVE_WHEEL_FREE_SPEED_RPS;
                }
                
                public static final class Turning {
                    public static final double P = 0.9;
                    public static final double I = 0.005;
                    public static final double D = 0.001;
                    public static final double FF = 0 / Measurements.DRIVE_WHEEL_FREE_SPEED_RPS;

                    public static final double ENCODER_MINIMUM_INPUT = 0;
                    public static final double ENCODER_MAXIMUM_INPUT = Encoders.Turning.POSITION_FACTOR;
                }

            }

            public static final class Module {

                public static final String FRONT_LEFT_NAME = "FrontLeft";
                public static final String FRONT_RIGHT_NAME = "FrontRight";
                public static final String BACK_LEFT_NAME = "BackLeft";
                public static final String BACK_RIGHT_NAME = "BackRight";

            }
            
            public static final class SlewRate {
                public static final double DIR_LIMIT = 1.2;
                public static final double MAG_LIMIT = 1.8;
                public static final double ROT_LIMIT = 2.0;
            }
        }
    }

    public static final class Controllers {
        public static final int DRIVER_JOYSTICK_PORT = 0;
        public static final int DRIVER_WHEEL_PORT = 1;

        
        public static final class Axis {
            public static final int JOYSTICK_X = 0;
            public static final int JOYSTICK_Y = 1;

            public static final int WHEEL_X = 0;
        }
        
        public static final double joystickDeadband = 0.05;
        public static final double wheelDeadband = 0.2;
        

    }

    public static final class LEDs {
        public static final int PORT = 0;
        public static final class Colors {
            public static final double LED_SETTING_DEFAULT = -0.95;
            public static final double STROBE_RED = -0.11;
            public static final double HEARTBEAT_RED = -0.25;
            public static final double SOLID_RED = 0.61;

            public static final double SHOT_BLUE = -0.83;

            public static final double GREEN = 0.77;
            public static final double AQUA = 0.83;
            public static final double BLUE = 0.87;
        }
    }
}
