// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.Color;
import frc.lib.Gains;

import static java.lang.Math.PI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class SwerveConstants {
        public enum Modules {
            // drive ID, spin ID, abs encoder channel, offset angle, drive reversed, angle reversed
            FL(18, 17, 5, 0.842, false, true),
            FR(12, 11, 9, 0.122, false, true),
            BL(16, 15, 4, 0.55, false, true),
            BR(14, 13, 8, 0.3339, false, true);


            public int DRIVE_MOTOR_ID;
            public int SPIN_MOTOR_ID;
            public int ABS_ENCODER_CHANNEL;
            public double OFFSET_ANGLE;
            public boolean DRIVE_MOTOR_REVERSED;
            public boolean SPIN_MOTOR_REVERSED;

            public static final int FRONT_LEFT = 0;
            public static final int FRONT_RIGHT = 1;
            public static final int BACK_LEFT = 2;
            public static final int BACK_RIGHT = 3;
            Modules(int DRIVE_MOTOR_ID,
                    int SPIN_MOTOR_ID,
                    int ABS_ENCODER_CHANNEL,
                    double OFFSET_ANGLE,
                    boolean DRIVE_MOTOR_REVERSED,
                    boolean SPIN_MOTOR_REVERSED) {
                this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
                this.SPIN_MOTOR_ID = SPIN_MOTOR_ID;
                this.ABS_ENCODER_CHANNEL = ABS_ENCODER_CHANNEL;
                this.OFFSET_ANGLE = OFFSET_ANGLE;
                this.DRIVE_MOTOR_REVERSED = DRIVE_MOTOR_REVERSED;
                this.SPIN_MOTOR_REVERSED = SPIN_MOTOR_REVERSED;
            }

        }

        public static final double TRACK_WIDTH = 0.56665; // m
        public static final double WHEEL_BASE = 0.56665; // m
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        public static final double MAX_VELOCITY_METER_PER_SECOND = Units.feetToMeters(12);
        public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SECOND = 2 * 2 * PI;
        public static final double MAX_ACCCEL_METER_PER_SECOND = 3;

        public static final double RAMP_BALANCE_KP = 0.008;
        public static final double RAMP_BALANCE_KD = 0.003;

        // intentional limitations
        public static final double DRIVE_SPEED_PERCENTAGE = 20; // %
        public static final double MAX_DRIVING_SPEED = MAX_VELOCITY_METER_PER_SECOND / 100 * DRIVE_SPEED_PERCENTAGE; // m/s
        public static final double MAX_TURNING_SPEED = MAX_ANGULAR_VELOCITY_RAD_PER_SECOND / 100 * DRIVE_SPEED_PERCENTAGE;// rad/s
        public static final double MAX_TURNING_ACCEL = PI / 100 * DRIVE_SPEED_PERCENTAGE; // rad/s^2
        public static final double MINIMUM_SWERVE_SPEED = 0.25; // even when the decelerator is fully pressed, the swerve wouldn't drive below this speed
        public static final double MAX_DRIVING_ACCEL_PER_SECOND = 3;
        public static final double MAX_ANGULAR_ACCEL_PER_SECOND = 5;

        // autonomous constants
        public static final Gains ANGLE_GAINS = new Gains(0.005733, 0, 3.51E-4);
        public static final Gains TRANSLATION_GAINS = new Gains(0, 0, 0);
    }
    public static final class ModuleConstants {

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.14;
        public static final double kDriveEncoderRotationsToMeters = kDriveMotorGearRatio * PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPMToMeterPerSec = kDriveEncoderRotationsToMeters / 60;
        public static final double kTurningMotorGearRatio = 1 / 21.4285714;
        public static final double kTurningEncoderRotationsToRadians = kTurningMotorGearRatio * 2 * PI;
        public static final double kTurningEncoderRPMToRadiansPerSec = kTurningEncoderRotationsToRadians / 60;

        public static final Gains MODULE_ANGLE_GAINS = new Gains(0.75, 0, 0);
        public static final double TOLORANCE = 0.05;

        public static final int DRIVE_CURRENT_LIMIT = 65;
        public static final int ANGLE_CURRENT_LIMIT = 25;
    }
    public static class LedsConstants {

        public static final int LEDS_PORT = 0; // pwm
        public static final int LENGTH = 139;
        public enum GamePiece {
            CONE(Color.Colors.PURPLE),
            CUBE(Color.Colors.ORANGE);


            public Color color;

            GamePiece(Color.Colors color){
                this.color = color.color;
            }
        }
    }
}
