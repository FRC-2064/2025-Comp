package frc.robot.ControlBoard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControlBoardConstants;
import frc.robot.Constants.OTFPaths;
import frc.robot.Constants.WristConstants;
import frc.robot.ControlBoard.ReefLookup.AlgaeHeight;
import frc.robot.ControlBoard.ReefLookup.AlgaePair;
import frc.robot.Subsystems.EndEffectorSubsystem.EndEffectorState;

public class ScoreConfigProvider {

    public static ScoreConfiguration getGamePieceConfiguration(double currHeading) {
        try {
            // SCORE ALGAE IN PROCESSOR
            if (ControlBoardHelpers.getScoreLocation().equals(ControlBoardConstants.SCORE_PROCESSOR)) {
                return new ScoreConfiguration(
                        OTFPaths.PROCESSOR,
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
                return new ScoreConfiguration(
                        pair.endPose,
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
                    double closestHeading = getClosestHeading(currHeading,
                            endPose.getRotation().getDegrees());
                    boolean usingFront = closestHeading == endPose.getRotation().getDegrees();
                    return new ScoreConfiguration(
                            new Pose2d(endPose.getX(), endPose.getY(), Rotation2d.fromDegrees(closestHeading)),
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
                    double closestHeading = getClosestHeading(currHeading,
                            endPose.getRotation().getDegrees());
                    boolean usingFront = closestHeading == endPose.getRotation().getDegrees();
                    return new ScoreConfiguration(
                            new Pose2d(endPose.getX(), endPose.getY(), Rotation2d.fromDegrees(closestHeading)),
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
                    return new ScoreConfiguration(
                            endPose.rotateBy(Rotation2d.fromDegrees(180)),
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

    public static ScoreConfiguration getFeederConfiguration(double currHeading) {
        try {
            String feeder = ControlBoardHelpers.getFeeder();

            if (feeder.equals(ControlBoardConstants.FEEDER_LEFT)) {
                Pose2d endPose = OTFPaths.FEEDER_LOCATION_LEFT;
                double closestHeading = getClosestHeading(currHeading,
                        endPose.getRotation().getDegrees());
                Boolean usingFront = closestHeading == endPose.getRotation().getDegrees();
                return new ScoreConfiguration(
                        new Pose2d(endPose.getX(), endPose.getY(), Rotation2d.fromDegrees(closestHeading)),
                        (usingFront) ? ArmConstants.ARM_FRONT_INTAKE_ANGLE : ArmConstants.ARM_BACK_INTAKE_ANGLE,
                        ArmConstants.ARM_HOME_ANGLE,
                        (usingFront) ? WristConstants.WRIST_FRONT_INTAKE_ANGLE
                                : WristConstants.WRIST_BACK_INTAKE_ANGLE,
                        WristConstants.WRIST_HOME_ANGLE,
                        EndEffectorState.INTAKING_CORAL);
            } else if (feeder.equals(ControlBoardConstants.FEEDER_RIGHT)) {
                Pose2d endPose = OTFPaths.FEEDER_LOCATION_RIGHT;
                double closestHeading = getClosestHeading(currHeading,
                        endPose.getRotation().getDegrees());
                Boolean usingFront = closestHeading == endPose.getRotation().getDegrees();
                return new ScoreConfiguration(
                        new Pose2d(endPose.getX(), endPose.getY(), Rotation2d.fromDegrees(closestHeading)),
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

    public static ScoreConfiguration getCageConfiguration() {
        try {
            String cage = ControlBoardHelpers.getCage();
            if (cage.equals(ControlBoardConstants.CAGE_LEFT)) {
                return new ScoreConfiguration(
                        OTFPaths.CAGE_LEFT,
                        ArmConstants.ARM_CLIMB_ANGLE,
                        ArmConstants.ARM_HOME_ANGLE,
                        WristConstants.WRIST_CLIMB_ANGLE,
                        WristConstants.WRIST_HOME_ANGLE,
                        EndEffectorState.STOPPED);
            } else if (cage.equals(ControlBoardConstants.CAGE_CENTER)) {
                return new ScoreConfiguration(
                        OTFPaths.CAGE_CENTER,
                        ArmConstants.ARM_CLIMB_ANGLE,
                        ArmConstants.ARM_HOME_ANGLE,
                        WristConstants.WRIST_CLIMB_ANGLE,
                        WristConstants.WRIST_HOME_ANGLE,
                        EndEffectorState.STOPPED);
            } else if (cage.equals(ControlBoardConstants.CAGE_RIGHT)) {
                return new ScoreConfiguration(
                        OTFPaths.CAGE_RIGHT,
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

    private static double getClosestHeading(double currHeading, double targetHeading) {
        double inverseHeading = (targetHeading + 180) % 360;

        currHeading = normalizeAngle(currHeading);
        targetHeading = normalizeAngle(targetHeading);
        inverseHeading = normalizeAngle(inverseHeading);

        double diffToTarget = Math.abs(currHeading - targetHeading);
        double diffToInverse = Math.abs(currHeading - inverseHeading);

        return (diffToTarget <= diffToInverse) ? targetHeading : inverseHeading;
    }

    private static double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

}
