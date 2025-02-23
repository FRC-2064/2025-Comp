package frc.robot.Utils.ControlBoard;

import static edu.wpi.first.units.Units.Degrees;

import java.io.Console;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem.EndEffectorState;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.ControlBoardConstants;
import frc.robot.Utils.Constants.OTFPaths;
import frc.robot.Utils.Constants.WristConstants;
import frc.robot.Utils.ControlBoard.ReefLookup.AlgaeHeight;
import frc.robot.Utils.ControlBoard.ReefLookup.AlgaePair;

public class RobotConfigProvider {

    public static RobotConfiguration getGamePieceConfiguration(double currHeading, double coralOffset) {
        try {

            if (ControlBoardHelpers.getScoreLocation().equals("TEST")) {
                return new RobotConfiguration(
                    OTFPaths.TEST_LOCATION,
                    computeStartPose(OTFPaths.TEST_LOCATION, true),
                    ArmConstants.ARM_TROUGH_FRONT_ANGLE,
                     ArmConstants.ARM_TROUGH_FRONT_ANGLE,
                     WristConstants.WRIST_TROUGH_FRONT_ANGLE,
                     WristConstants.WRIST_TROUGH_FRONT_ANGLE,
                     EndEffectorState.STOPPED);
            }

            // SCORE ALGAE IN PROCESSOR
            if (ControlBoardHelpers.getScoreLocation().equals(ControlBoardConstants.SCORE_PROCESSOR)) {
                Pose2d desiredEndPose = new Pose2d(
                        OTFPaths.PROCESSOR.getX(),
                        OTFPaths.PROCESSOR.getY(),
                        rotateBasedOnAlliance(OTFPaths.PROCESSOR.getRotation()));
                return new RobotConfiguration(
                        desiredEndPose,
                        computeStartPose(desiredEndPose, true),
                        ArmConstants.ARM_ALGAE_CARRY_ANGLE,
                        ArmConstants.ARM_ALGAE_CARRY_ANGLE,
                        WristConstants.WRIST_ALGAE_CARRY_ANGLE,
                        WristConstants.WRIST_ALGAE_CARRY_ANGLE,
                        EndEffectorState.OUTTAKING_ALGAE);
            }

            String reefLocation = ControlBoardHelpers.getReefLocation();
            int reefLevel = (int) ControlBoardHelpers.getLevel();

            // REMOVE ALGAE FROM REEF
            if (reefLevel == ControlBoardConstants.REEF_LEVEL_ALGAE) {
                AlgaePair pair = ReefLookup.algaePoses.get(reefLocation);
                boolean isHighAlgae = pair.algaeHeight == AlgaeHeight.HIGH;
                Pose2d desiredEndPose = new Pose2d(
                        pair.endPose.getX(),
                        pair.endPose.getY(),
                        rotateBasedOnAlliance(pair.endPose.getRotation()));
                return new RobotConfiguration(
                        desiredEndPose,
                        computeStartPose(desiredEndPose, !isHighAlgae),
                        (isHighAlgae) ? ArmConstants.ARM_HIGH_ALGAE_REMOVAL_ANGLE
                                : ArmConstants.ARM_LOW_ALGAE_REMOVAL_ANGLE,
                        ArmConstants.ARM_HOME_ANGLE,
                        (isHighAlgae) ? WristConstants.WRIST_HIGH_ALGAE_REMOVAL_ANGLE
                                : WristConstants.WRIST_LOW_ALGAE_REMOVAL_ANGLE,
                        WristConstants.WRIST_HOME_ANGLE,
                        (isHighAlgae) ? EndEffectorState.REMOVING_HIGH_ALGAE : EndEffectorState.REMOVING_LOW_ALGAE);
            }

            // SCORE IN TROUGH
            if (reefLevel == ControlBoardConstants.REEF_LEVEL_TROUGH) {
                Pose2d endPose = ReefLookup.coralPoses.get(reefLocation);
                if (endPose != null) {
                    Pose2d desiredEndPose = new Pose2d(
                            endPose.getX(),
                            endPose.getY(),
                            rotateBasedOnAlliance(endPose.getRotation()));
                    double closestHeading = getClosestHeading(currHeading,
                            desiredEndPose.getRotation().getDegrees());
                    boolean usingFront = closestHeading == desiredEndPose.getRotation().getDegrees();
                    Pose2d adjustedPose = new Pose2d(
                            desiredEndPose.getX(),
                            desiredEndPose.getY(),
                            Rotation2d.fromDegrees(closestHeading));
                    return new RobotConfiguration(
                            adjustedPose,
                            computeStartPose(adjustedPose, usingFront),
                            (usingFront) ? ArmConstants.ARM_TROUGH_FRONT_ANGLE
                                    : ArmConstants.ARM_TROUGH_BACK_ANGLE,
                            ArmConstants.ARM_HOME_ANGLE,
                            (usingFront) ? WristConstants.WRIST_TROUGH_FRONT_ANGLE
                                    : WristConstants.WRIST_TROUGH_BACK_ANGLE,
                            WristConstants.WRIST_HOME_ANGLE,
                            EndEffectorState.OUTTAKING_CORAL);
                }
            }

            // SCORE IN LEVEL 2
            if (reefLevel == ControlBoardConstants.REEF_LEVEL_2) {
                Pose2d endPose = ReefLookup.coralPoses.get(reefLocation);
                if (endPose != null) {
                    Pose2d desiredEndPose = new Pose2d(
                            endPose.getX(),
                            endPose.getY(),
                            rotateBasedOnAlliance(endPose.getRotation()));
                    double closestHeading = getClosestHeading(currHeading,
                            desiredEndPose.getRotation().getDegrees());
                    boolean usingFront = closestHeading == desiredEndPose.getRotation().getDegrees();
                    Pose2d basePose = new Pose2d(desiredEndPose.getTranslation(),
                            Rotation2d.fromDegrees(closestHeading));
                    double lateralOffset = usingFront ? coralOffset : -coralOffset;

                    Rotation2d heading = basePose.getRotation();
                    double offsetX = -lateralOffset * heading.getSin();
                    double offsetY = lateralOffset * heading.getCos();

                    Pose2d adjustedPose = new Pose2d(
                            basePose.getX() + offsetX,
                            basePose.getY() + offsetY,
                            basePose.getRotation());

                    SmartDashboard.putNumber("Logging/Heading/targetHeading", endPose.getRotation().getDegrees());
                    SmartDashboard.putNumber("Logging/Heading/closestHeading", closestHeading);

                    return new RobotConfiguration(
                            adjustedPose,
                            computeStartPose(adjustedPose, usingFront),
                            (usingFront) ? ArmConstants.ARM_L2_FRONT_ANGLE
                                    : ArmConstants.ARM_L2_BACK_ANGLE,
                            ArmConstants.ARM_HOME_ANGLE,
                            (usingFront) ? WristConstants.WRIST_L2_FRONT_ANGLE
                                    : WristConstants.WRIST_L2_BACK_ANGLE,
                            WristConstants.WRIST_HOME_ANGLE,
                            EndEffectorState.OUTTAKING_CORAL);
                }
            }

            // SCORE IN LEVEL 3
            if (reefLevel == ControlBoardConstants.REEF_LEVEL_3) {
                Pose2d endPose = ReefLookup.coralPoses.get(reefLocation);
                if (endPose != null) {
                    Pose2d desiredEndPose = new Pose2d(
                            endPose.getX(),
                            endPose.getY(),
                            rotateBasedOnAlliance(endPose.getRotation()));
                    Pose2d basePose = desiredEndPose.rotateBy(Rotation2d.fromDegrees(180));
                    double lateralOffset = -coralOffset;

                    Rotation2d heading = basePose.getRotation();
                    double offsetX = -lateralOffset * heading.getSin();
                    double offsetY = lateralOffset * heading.getCos();

                    Pose2d adjustedPose = new Pose2d(
                            basePose.getX() + offsetX,
                            basePose.getY() + offsetY,
                            basePose.getRotation());

                    return new RobotConfiguration(
                            adjustedPose,
                            computeStartPose(adjustedPose, false),
                            ArmConstants.ARM_L3_BACK_ANGLE,
                            ArmConstants.ARM_HOME_ANGLE,
                            WristConstants.WRIST_L3_BACK_ANGLE,
                            WristConstants.WRIST_HOME_ANGLE,
                            EndEffectorState.OUTTAKING_CORAL);
                }
            }

        } catch (Exception e) {
            System.out.println("Error in getScoreConfiguration");
            e.printStackTrace();
        }
        return null;
    }

    public static RobotConfiguration getFeederConfiguration(double currHeading) {
        try {
            String feeder = ControlBoardHelpers.getFeeder();

            if (feeder.equals(ControlBoardConstants.FEEDER_LEFT)) {
                Pose2d endPose = new Pose2d(
                        OTFPaths.FEEDER_LOCATION_LEFT.getX(),
                        OTFPaths.CAGE_LEFT.getY(),
                        rotateBasedOnAlliance(OTFPaths.FEEDER_LOCATION_LEFT.getRotation()));
                double closestHeading = getClosestHeading(currHeading,
                        endPose.getRotation().getDegrees());
                Boolean usingFront = closestHeading == endPose.getRotation().getDegrees();
                Pose2d adjustedPose = new Pose2d(endPose.getX(), endPose.getY(),
                        Rotation2d.fromDegrees(closestHeading));
                return new RobotConfiguration(
                        adjustedPose,
                        computeStartPose(adjustedPose, usingFront),
                        (usingFront) ? ArmConstants.ARM_FRONT_INTAKE_ANGLE : ArmConstants.ARM_BACK_INTAKE_ANGLE,
                        ArmConstants.ARM_HOME_ANGLE,
                        (usingFront) ? WristConstants.WRIST_FRONT_INTAKE_ANGLE
                                : WristConstants.WRIST_BACK_INTAKE_ANGLE,
                        WristConstants.WRIST_HOME_ANGLE,
                        EndEffectorState.INTAKING_CORAL);
            } else if (feeder.equals(ControlBoardConstants.FEEDER_RIGHT)) {
                Pose2d endPose = new Pose2d(
                        OTFPaths.FEEDER_LOCATION_RIGHT.getX(),
                        OTFPaths.FEEDER_LOCATION_RIGHT.getY(),
                        rotateBasedOnAlliance(OTFPaths.FEEDER_LOCATION_RIGHT.getRotation()));
                double closestHeading = getClosestHeading(currHeading,
                        endPose.getRotation().getDegrees());
                Boolean usingFront = closestHeading == endPose.getRotation().getDegrees();
                Pose2d adjustedPose = new Pose2d(endPose.getX(), endPose.getY(),
                        Rotation2d.fromDegrees(closestHeading));
                return new RobotConfiguration(
                        adjustedPose,
                        computeStartPose(adjustedPose, usingFront),
                        (usingFront) ? ArmConstants.ARM_FRONT_INTAKE_ANGLE : ArmConstants.ARM_BACK_INTAKE_ANGLE,
                        ArmConstants.ARM_HOME_ANGLE,
                        (usingFront) ? WristConstants.WRIST_FRONT_INTAKE_ANGLE
                                : WristConstants.WRIST_BACK_INTAKE_ANGLE,
                        WristConstants.WRIST_HOME_ANGLE,
                        EndEffectorState.INTAKING_CORAL);
            }

        } catch (Exception e) {
            System.out.println("Error in getFeederConfiguration");
            e.printStackTrace();
        }
        return null;
    }

    public static RobotConfiguration getCageConfiguration() {
        try {
            String cage = ControlBoardHelpers.getCage();
            if (cage.equals(ControlBoardConstants.CAGE_LEFT)) {
                return new RobotConfiguration(
                        new Pose2d(
                                OTFPaths.CAGE_LEFT.getX(),
                                OTFPaths.CAGE_LEFT.getY(),
                                rotateBasedOnAlliance(OTFPaths.CAGE_LEFT.getRotation())),
                        computeStartPose(OTFPaths.CAGE_LEFT, false),
                        ArmConstants.ARM_CLIMB_ANGLE,
                        ArmConstants.ARM_HOME_ANGLE,
                        WristConstants.WRIST_CLIMB_ANGLE,
                        WristConstants.WRIST_HOME_ANGLE,
                        EndEffectorState.STOPPED);
            } else if (cage.equals(ControlBoardConstants.CAGE_CENTER)) {
                return new RobotConfiguration(
                        new Pose2d(
                                OTFPaths.CAGE_CENTER.getX(),
                                OTFPaths.CAGE_CENTER.getY(),
                                rotateBasedOnAlliance(OTFPaths.CAGE_CENTER.getRotation())),
                        computeStartPose(OTFPaths.CAGE_CENTER, false),
                        ArmConstants.ARM_CLIMB_ANGLE,
                        ArmConstants.ARM_HOME_ANGLE,
                        WristConstants.WRIST_CLIMB_ANGLE,
                        WristConstants.WRIST_HOME_ANGLE,
                        EndEffectorState.STOPPED);
            } else if (cage.equals(ControlBoardConstants.CAGE_RIGHT)) {
                return new RobotConfiguration(
                        new Pose2d(
                                OTFPaths.CAGE_RIGHT.getX(),
                                OTFPaths.CAGE_RIGHT.getY(),
                                rotateBasedOnAlliance(OTFPaths.CAGE_RIGHT.getRotation())),
                        computeStartPose(OTFPaths.CAGE_RIGHT, false),
                        ArmConstants.ARM_CLIMB_ANGLE,
                        ArmConstants.ARM_HOME_ANGLE,
                        WristConstants.WRIST_CLIMB_ANGLE,
                        WristConstants.WRIST_HOME_ANGLE,
                        EndEffectorState.STOPPED);
            }
        } catch (Exception e) {
            System.out.println("Error in getCageConfiguration");
            e.printStackTrace();
        }
        return null;
    }

    private static Pose2d computeStartPose(Pose2d endPose, Boolean usingFront) {
        double theta = endPose.getRotation().getRadians();
        double dx = 0.5 * Math.cos(theta);
        double dy = 0.5 * Math.sin(theta);
        dx = (usingFront) ? dx : -dx;
        dy = (usingFront) ? dy : -dy;
        return new Pose2d(endPose.getX() - dx, endPose.getY() - dy, Rotation2d.fromDegrees(0.0));
    }

    private static double getClosestHeading(double currHeading, double targetHeading) {
        double newheading = currHeading;
            try {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    newheading = currHeading + 180;
                }
            }

        } catch (Exception e) {
            System.out.println("rotate error " + e);
        }
        double inverseHeading = (targetHeading + 180) % 360;

        newheading = normalizeAngle(newheading);
        targetHeading = normalizeAngle(targetHeading);
        inverseHeading = normalizeAngle(inverseHeading);

        double diffToTarget = Math.abs(newheading - targetHeading);
        double diffToInverse = Math.abs(newheading - inverseHeading);

        // Choose the closest heading (targetHeading or inverseHeading)
        double closestHeading = (diffToTarget <= diffToInverse) ? targetHeading : inverseHeading;

        // Ensure the result is in the range 0 to 360
        return (closestHeading + 360) % 360;
    }

    private static double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    private static Rotation2d rotateBasedOnAlliance(Rotation2d rot) {
        return rot;
        // try {
        //     var alliance = DriverStation.getAlliance();
        //     if (alliance.isPresent()) {
        //         if (alliance.get() == DriverStation.Alliance.Red) {
        //             return rot.rotateBy(Rotation2d.fromDegrees(180));
        //         } else {
        //             return rot;
        //         }
        //     }

        // } catch (Exception e) {
        //     System.out.println("rotate error " + e);
        // }
        // return null;
    }

}
