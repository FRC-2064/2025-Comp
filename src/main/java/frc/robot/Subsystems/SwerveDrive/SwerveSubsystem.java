package frc.robot.Subsystems.SwerveDrive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.text.ParseException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final boolean visionDriveTest = false;
    public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                Constants.MAX_SPEED, 
                new Pose2d(
                    new Translation2d(
                        Meter.of(1), 
                        Meter.of(2)), 
                    Rotation2d.fromDegrees(0)
                )
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(
            true,
            true,
            0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(
            true,
            1);
        swerveDrive.pushOffsetsToEncoders();
        if (visionDriveTest) {
            // add vision stuff here
        }
        
    }

    public void setupPathPlanner() {
        RobotConfig config;
        
        try {
            config = RobotConfig.fromGUISettings();
            
            final boolean enableFeedForward = true;
            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                (speedsRobotRelative, moduleFeedForwards) -> {
                  if (enableFeedForward)
                  {
                    swerveDrive.drive(
                        speedsRobotRelative,
                        swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        moduleFeedForwards.linearForces()
                                     );
                  } else
                  {
                    swerveDrive.setChassisSpeeds(speedsRobotRelative);
                  }
                }, 
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
                    ),
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

        PathfindingCommand.warmupCommand().schedule();
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
            edu.wpi.first.units.Units.MetersPerSecond.of(0)
        );
    }

    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
    throws IOException, ParseException, org.json.simple.parser.ParseException {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
            RobotConfig.fromGUISettings(), 
            swerveDrive.getMaximumChassisAngularVelocity()
        );
        
        AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
            new SwerveSetpoint(
                swerveDrive.getRobotVelocity(),
                swerveDrive.getStates(),
                DriveFeedforwards.zeros(swerveDrive.getModules().length)
            )
        );
        
        AtomicReference<Double> previousTime = new AtomicReference<>();

        return startRun(
            () -> previousTime.set(Timer.getFPGATimestamp()),
            () -> {
                double newTime = Timer.getFPGATimestamp();
                SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
                    prevSetpoint.get(),
                    robotRelativeChassisSpeed.get(),
                    newTime - previousTime.get()
                );
                swerveDrive.drive(
                    newSetpoint.robotRelativeSpeeds(),
                    newSetpoint.moduleStates(),
                    newSetpoint.feedforwards().linearForces()
                );
                prevSetpoint.set(newSetpoint);
                previousTime.set(newTime);
            }
        );
    }

    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(
                () -> {
                    return ChassisSpeeds.fromFieldRelativeSpeeds(
                        fieldRelativeSpeeds.get(),
                        getHeading()
                    );
                }
            );
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }
        return Commands.none();
    }

    public Command centerModules() {
        return run(
            () -> Arrays.asList(
                swerveDrive.getModules()).forEach(
                    it -> it.setAngle(0)
                )
            );
    }

    public void replaceSwerveModuleFeedForwad(double kS, double kV, double kA) {
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(
          () -> {
            swerveDrive.drive(
                SwerveMath.scaleTranslation(
                    new Translation2d(
                        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
                    ), 
                    0.8
                ),
                Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                true,
                false
            );
          }  
        );
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(
            () -> {
                Translation2d scaledInputs = SwerveMath.scaleTranslation(
                    new Translation2d(
                        translationX.getAsDouble(),
                        translationY.getAsDouble()
                    ),
                    0.8
                );

                driveFieldOriented(
                    swerveDrive.swerveController.getTargetSpeeds(
                        scaledInputs.getX(),
                        scaledInputs.getY(),
                        headingX.getAsDouble(),
                        headingY.getAsDouble(),
                        swerveDrive.getOdometryHeading().getRadians(),
                        swerveDrive.getMaximumChassisVelocity()
                    )
                );
            }
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(
            translation,
            rotation,
            fieldRelative,
            false
        );
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(
            () -> {
                swerveDrive.driveFieldOriented(velocity.get());
            }
        );
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
                    Rotation2d.fromDegrees(180)
                )
            );
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
            new Translation2d(xInput, yInput)
        );
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
            new Translation2d(xInput, yInput)
        );
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

}
