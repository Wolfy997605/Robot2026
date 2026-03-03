package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {
    private final Pigeon2 m_gyro;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveModule[] m_swerveMods;
    final CANBus canbus = new CANBus(DriveConstants.kCanivoreBusId);

    private Alliance m_allianceColor = Alliance.Red;

    private final Field2d m_robotField;

    /** radians */
    private double m_gyroOffset = 0.0;
    private boolean m_drivingInFieldRelative = true;
    private boolean m_currentlyPathfinding = false;

    public Base() {
        m_gyro = new Pigeon2(Constants.DriveConstants.pigeonID, canbus);

        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        m_gyro.setYaw(0);

        m_robotField = new Field2d();

        Optional<RobotConfig> robotConfig;

        try {  
            // TODO: set config in pathplanner GUI
            robotConfig = Optional.of(RobotConfig.fromGUISettings());
        } catch (Exception e) {
            robotConfig = Optional.empty();
            e.printStackTrace();
        }

        m_swerveMods = new SwerveModule[] {
                new SwerveModule(0, DriveConstants.Mod0.constants),
                new SwerveModule(1, DriveConstants.Mod1.constants),
                new SwerveModule(2, DriveConstants.Mod2.constants),
                new SwerveModule(3, DriveConstants.Mod3.constants)
        };

        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.swerveKinematics, m_gyro.getRotation2d(),
                getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

        SmartDashboard.putData("Robot Measurement", m_robotField);

        AutoBuilder.configure(this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) ->

                driveRobotRelativeChassisSpeeds(speeds), // Method that will drive the robot
                                                         // given ROBOT RELATIVE
                                                         // ChassisSpeeds. Also optionally
                                                         // outputs individual module
                                                         // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(AutoConstants.driveKP, AutoConstants.driveKI, AutoConstants.driveKD), // Translation
                                                                                                               // PID
                                                                                                               // constants
                        new PIDConstants(AutoConstants.turnKP, AutoConstants.turnKI, AutoConstants.turnKD) // Rotation
                                                                                                           // PID
                                                                                                           // constants
                ),
                robotConfig.get(), // The robot configuration
                this::shouldPathsFlip,
                this // Reference to this subsystem to set requirements
        );
        m_drivingInFieldRelative = true;
        m_gyro.reset();
    }

    public boolean shouldPathsFlip() {
        if (m_currentlyPathfinding) {
            return false;
        }
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void setPathfindingFlag(boolean newValue) {
        m_currentlyPathfinding = newValue;
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.DriveConstants.swerveKinematics.toSwerveModuleStates(
                m_drivingInFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        Rotation2d.fromRadians(m_gyro.getRotation2d().getRadians() - m_gyroOffset))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxTeleopSpeed);

        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxTeleopSpeed);

        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_swerveMods) {
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_swerveMods) {
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        var swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);

        setModuleStates(swerveModuleStates);
    }

    public void setPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : m_swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void setModulesFacingForward() {
        for (var module : m_swerveMods) {
            module.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false);
        }
    }

    public void stopWheels() {
        for (var module : m_swerveMods) {
            module.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false);
        }
    }

    public void changeFieldDrivingMode() {
        m_drivingInFieldRelative = !m_drivingInFieldRelative;
    }

    public void switchToRobotRelative() {
        m_drivingInFieldRelative = false;
    }

    public void switchToFieldRelative() {
        m_drivingInFieldRelative = true;
    }

    public void resetGyroOffset(boolean usePoseEstimator) {
        if (!usePoseEstimator) {
            m_gyroOffset = m_gyro.getRotation2d().getRadians();
        } else {
            double currentHeading = m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
            double currentGyroAngle = m_gyro.getRotation2d().getRadians();
            double targetHeading = (m_allianceColor == DriverStation.Alliance.Blue) ? 0
                    : Math.toRadians(180);

            double error = targetHeading - currentHeading;

            m_gyroOffset = currentGyroAngle + error;
        }
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        for (SwerveModule mod : m_swerveMods) {
            mod.setNeutralMode(neutralMode);
        }
    }
    

    @Override
    public void periodic() {
        // update pose estimator with gyro and swerve
        m_poseEstimator.update(getGyroYaw(), getModulePositions());
        {
            var all = DriverStation.getAlliance();
            if (all.isPresent()) {
                m_allianceColor = all.get();
            }
        }

        for (SwerveModule mod : m_swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        m_robotField.setRobotPose(m_poseEstimator.getEstimatedPosition());

        SmartDashboard.putNumber("RobotOrientation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("RobotHeading", getHeading().getRadians());

        SmartDashboard.putNumber("CANivore Usage", canbus.getStatus().BusUtilization);
    }
}