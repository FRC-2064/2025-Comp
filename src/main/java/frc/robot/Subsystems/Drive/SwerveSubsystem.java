package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Constants.Limelight1Constants;
import frc.robot.Utils.Constants.Limelight2Constants;
import frc.robot.Utils.Constants.Limelight3Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private DriveState driveState = DriveState.USER_CONTROLLED;
    private Pose2d otfStartPose;
    private Pose2d otfEndPose;

    public SwerveSubsystem(File directory) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                    Constants.MAX_SPEED,
                    new Pose2d(
                            new Translation2d(
                                    Meter.of(1),
                                    Meter.of(2)),
                            Rotation2d.fromDegrees(0)));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(true);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(
                true,
                true,
                0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(
                true,
                1);
        swerveDrive.stopOdometryThread();

        LimelightHelpers.setCameraPose_RobotSpace(
                Limelight1Constants.LIMELIGHT_NAME,
                Limelight1Constants.FORWARD_OFFSET, // Forward offset (meters)
                Limelight1Constants.SIDE_OFFSET, // Side offset (meters)
                Limelight1Constants.HEIGHT_OFFSET, // Height offset (meters)
                Limelight1Constants.ROLL_OFFSET, // Roll (degrees)
                Limelight1Constants.PITCH_OFFSET, // Pitch (degrees)
                Limelight1Constants.YAW_OFFSET // Yaw (degrees)
        );

        LimelightHelpers.setCameraPose_RobotSpace(
                Limelight2Constants.LIMELIGHT_NAME,
                Limelight2Constants.FORWARD_OFFSET, // Forward offset (meters)
                Limelight2Constants.SIDE_OFFSET, // Side offset (meters)
                Limelight2Constants.HEIGHT_OFFSET, // Height offset (meters)
                Limelight2Constants.ROLL_OFFSET, // Roll (degrees)
                Limelight2Constants.PITCH_OFFSET, // Pitch (degrees)
                Limelight2Constants.YAW_OFFSET // Yaw (degrees)
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            Limelight3Constants.LIMELIGHT_NAME,
            Limelight3Constants.FORWARD_OFFSET, // Forward offset (meters)
            Limelight3Constants.SIDE_OFFSET, // Side offset (meters)
            Limelight3Constants.HEIGHT_OFFSET, // Height offset (meters)
            Limelight3Constants.ROLL_OFFSET, // Roll (degrees)
            Limelight3Constants.PITCH_OFFSET, // Pitch (degrees)
            Limelight3Constants.YAW_OFFSET // Yaw (degrees)
    );

        setupPathPlanner();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Logging/Drive/State", getDriveState().toString());
        SmartDashboard.putNumber("Logging/Drive/Heading", getHeading().getDegrees());

        manageDriveState();

        swerveDrive.updateOdometry();
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        if (Robot.isReal()) {
            // LIMELIGHT 1 : BACK
            // boolean doRejectUpdate = false;
            double degrees = swerveDrive.getOdometryHeading().getDegrees();
            LimelightHelpers.SetRobotOrientation(
                    Limelight1Constants.LIMELIGHT_NAME,
                    degrees,
                    0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                    Limelight1Constants.LIMELIGHT_NAME);
            if (mt2 != null && mt2.tagCount != 0) {
                swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }

            // LIMELIGHT 2 : FRONT
            // doRejectUpdate = false;
            LimelightHelpers.SetRobotOrientation(
                    Limelight2Constants.LIMELIGHT_NAME,
                    degrees,
                    0, 0, 0, 0, 0);

            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                    Limelight2Constants.LIMELIGHT_NAME);
            if (mt2 != null && mt2.tagCount != 0) {
                swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }

            // LIMELIGHT 3 : BACK
            // doRejectUpdate = false;
            LimelightHelpers.SetRobotOrientation(
                    Limelight3Constants.LIMELIGHT_NAME,
                    degrees,
                    0, 0, 0, 0, 0);

            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                    Limelight3Constants.LIMELIGHT_NAME);
            if (mt2 != null && mt2.tagCount != 0) {
                swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    public Command driveDirectAngle(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        swerveDrive.setHeadingCorrection(true);
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(
                    new Translation2d(
                            translationX.getAsDouble(),
                            translationY.getAsDouble()),
                    0.8);

            driveFieldOriented(
                    swerveDrive.swerveController.getTargetSpeeds(
                            scaledInputs.getX(),
                            scaledInputs.getY(),
                            headingX.getAsDouble(),
                            headingY.getAsDouble(),
                            swerveDrive.getOdometryHeading().getRadians(),
                            swerveDrive.getMaximumChassisVelocity()));
        });
    }

    public void setupPathPlanner() {
        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedForward = false;
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedForward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(3.0, 0.0, 0.0),
                            new PIDConstants(2.0, 0.0, 0.0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        } else {
                            return false;
                        }
                    },
                    this);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity(),
                4.0,
                swerveDrive.getMaximumChassisAngularVelocity(),
                Units.degreesToRadians(720));

        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0));
    }

    public Command centerModules() {
        return run(
                () -> Arrays.asList(
                        swerveDrive.getModules()).forEach(
                                it -> it.setAngle(0)));
    }

    public void replaceSwerveModuleFeedForwad(double kS, double kV, double kA) {
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(
                translation,
                rotation,
                fieldRelative,
                false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(
                () -> {
                    swerveDrive.driveFieldOriented(velocity.get());
                });
    }

    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            resetOdometry(
                    new Pose2d(
                            getPose().getTranslation(),
                            Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(
                new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(
                scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                Constants.MAX_SPEED);
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(
                new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(
                scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                Constants.MAX_SPEED);
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void setPredefinedOdom() {
        Pose2d resetPose = new Pose2d(2.0, 2.0, new Rotation2d());
        swerveDrive.resetOdometry(resetPose);
    }

    public Command pathfindToPath(PathPlannerPath currPath) {
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity() * 0.4,//.286,
                4.0,
                swerveDrive.getMaximumChassisAngularVelocity(),
                Units.degreesToRadians(720));

        return AutoBuilder.pathfindThenFollowPath(currPath, constraints);
    }

    public Command pathfindToOTFPath(List<Pose2d> pathPoses) {
        if (pathPoses == null || pathPoses.size() < 2) {
            return new Command() {
            };
        }
        driveState = DriveState.PATHFINDING;
        otfStartPose = pathPoses.get(0);
        otfEndPose = pathPoses.get(1);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathPoses);

        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity() * 0.25,
                0.5,
                swerveDrive.getMaximumChassisAngularVelocity(),
                Units.degreesToRadians(720));

        PathPlannerPath generatedPath = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, pathPoses.get(pathPoses.size() - 1).getRotation()));

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                generatedPath.flipPath();
            }
        }
        return pathfindToPath(generatedPath);
    }

    private void manageDriveState() {
        switch (driveState) {
            case PATHFINDING:
                pathfinding();
                break;

            case FOLLOWING_PATH:
                followingPath();
                break;

            default:
                break;
        }
    }

    private Pose2d getRelativePose() {
        Pose2d absPos = getPose();
        double absX = absPos.getX();
        double absY = absPos.getY();

        double field_length = Units.inchesToMeters(649);
        double field_width = Units.inchesToMeters(319);

        double center_x = field_length / 2;
        double center_y = field_width / 2;

        double x_centered = absX - center_x;
        double y_centered = absY - center_y;

        double x_rotated = -x_centered;
        double y_rotated = -y_centered;

        double x_final = x_rotated + center_x;
        double y_final = y_rotated + center_y;

        return new Pose2d(x_final, y_final, new Rotation2d());

    }

    private void pathfinding() {
        if (otfStartPose == null) {
            return;
        }

        var alliance = DriverStation.getAlliance();
        Pose2d currentPose = getPose();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                currentPose = getRelativePose();
            }
        }

        Translation2d diffTranslation = currentPose.getTranslation().minus(otfStartPose.getTranslation());
        double positionError = diffTranslation.getNorm();

        SmartDashboard.putNumber("Logging/Drive/PositionError", positionError);

        if (positionError < Constants.DRIVESTATE_ALLOWED_ERROR) {
            driveState = DriveState.FOLLOWING_PATH;
        }

    }

    private void followingPath() {
        Pose2d currentPose = getRelativePose();
        SmartDashboard.putNumber("Logging/Drive/absX", getPose().getX());
        SmartDashboard.putNumber("Logging/Drive/absY", getPose().getY());
        SmartDashboard.putNumber("Logging/Drive/PositionX", currentPose.getX());
        SmartDashboard.putNumber("Logging/Drive/PositionY", currentPose.getY());
        SmartDashboard.putNumber("Logging/Drive/otfEndX", otfEndPose.getX());
        SmartDashboard.putNumber("Logging/Drive/otfEndY", otfEndPose.getY());
        Translation2d diffTranslation = currentPose.getTranslation().minus(otfEndPose.getTranslation());
        double positionError = diffTranslation.getNorm();

        if (positionError < Constants.DRIVESTATE_ALLOWED_ERROR) {
            driveState = DriveState.USER_CONTROLLED;
        }
    }

    public void setState(DriveState state) {
        driveState = state;
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public enum DriveState {
        PATHFINDING,
        FOLLOWING_PATH,
        USER_CONTROLLED
    }

}
