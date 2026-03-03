package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final boolean kEnableVision = true;

    public static final class DriveConstants {
        public static final int kXboxControllerID = 0;
        public static final int kTurnStickID = 1;
        public static final int kThrottleStickID = 2;

        public static final double kControllerMovementDeadband = 0.1;
        public static final double kControllerRotationDeadband = 0.1;

        public static final double kTimeBeforeBrakeDisabled = 1;

        public static final int pigeonID = 1;
        public static final String kCanivoreBusId = "canivore";

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i
                .KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        // TODO: adjust with real robot dimensions
        public static final double trackWidth = 0.57785; // meters
        public static final double wheelBase = 0.66675; // meters
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double kRobotWidth = 0.884; // meters
        public static final double kRobotLength = 0.973; // meters

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 40;
        public static final int angleCurrentThreshold = 50;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 60;
        public static final int driveCurrentThreshold = 70;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = 100;
        public static final double angleKI = 0;
        public static final double angleKD = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 1;
        public static final double driveKI = 0;
        public static final double driveKD = 0;

        /* Drive Motor Characterization Values From SYSID */
        // TODO: retune
        public static final double driveKS = 0.12498;
        public static final double driveKV = 2.4;
        public static final double driveKA = 0.0353;

        public static final double kXYMovementP = 5;
        public static final double kXYMovementI = 0;
        public static final double kXYMovementD = 0;

        public static final double kRotationMovementP = 5;
        public static final double kRotationMovementI = 0;
        public static final double kRotationMovementD = 0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double kMaxTeleopSpeed = 5.25;
        /** Radians per Second */
        public static final double kMaxTeleopRotateSpeed = 2 * Math.PI;

        public static final double kGeneralSpeedMulti = .8;
        public static final double kDriverMultiSlow = 0.15;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(29.75); // Anciennement
            // -152.95
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-59.2); // Anciennement
            // 138.65
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-52.95); // Anciennement
            // -53.65
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-132.25); // Anciennement
            // -149.55
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double driveKP = 6.75;
        public static final double driveKI = 0;
        public static final double driveKD = 0;

        public static final double turnKP = 5;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
    }

    public static final double kPeriodicInterval = .02;

    public static final int kCANTimeout = 50;
    public static final int kPeriodicFrameTimeout = 500;
}
