package frc.robot.ControlBoard;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoHeadings;
import frc.robot.Constants.ControlBoardConstants;
import frc.robot.Constants.NamedPaths;
import frc.robot.Constants.WristConstants;
import frc.robot.ControlBoard.ReefPathLookup.AlgaeHeight;
import frc.robot.ControlBoard.ReefPathLookup.AlgaePair;
import frc.robot.ControlBoard.ReefPathLookup.ReefPathPair;

public class ControlBoardUtils {

    public static ScoreOutput getScorePath(double currHeading) {
        try {
            if (ControlBoardHelpers.getScoreLocation().equals(ControlBoardConstants.SCORE_PROCESSOR)) {
                return new ScoreOutput(
                        PathPlannerPath.fromPathFile(NamedPaths.ALGAE_LOCATION_PROCESSOR),
                        ArmConstants.ARM_ALGAE_CARRY_ANGLE,
                        WristConstants.WRIST_ALGAE_CARRY_ANGLE);
            }

            String reefLocation = ControlBoardHelpers.getReefLocation();
            int reefLevel = (int) ControlBoardHelpers.getLevel();

            if (reefLevel == ControlBoardConstants.REEF_LEVEL_ALGAE) {
                AlgaePair pair = ReefPathLookup.algaePaths.get(reefLocation);
                boolean isHighAlgae = pair.algaeHeight == AlgaeHeight.HIGH;
                return new ScoreOutput(
                        PathPlannerPath.fromPathFile(pair.path),
                        (isHighAlgae) ? ArmConstants.ARM_HIGH_ALGAE_REMOVAL_ANGLE
                                : ArmConstants.ARM_LOW_ALGAE_REMOVAL_ANGLE,
                        (isHighAlgae) ? WristConstants.WRIST_HIGH_ALGAE_REMOVAL_ANGLE
                                : WristConstants.WRIST_LOW_ALGAE_REMOVAL_ANGLE);
            }

            if (reefLevel == ControlBoardConstants.REEF_LEVEL_TROUGH) {
                ReefPathPair pair = ReefPathLookup.coralPaths.get(reefLocation);
                if (pair != null) {
                    double closestHeading = getClosestHeading(currHeading, pair.targetHeading);
                    boolean usingFront = closestHeading == pair.targetHeading;
                    return new ScoreOutput(
                            PathPlannerPath.fromPathFile((usingFront) ? pair.frontPath : pair.backPath),
                            (usingFront) ? ArmConstants.ARM_TROUGH_FRONT_ANGLE : ArmConstants.ARM_TROUGH_BACK_ANGLE,
                            (usingFront) ? WristConstants.WRIST_TROUGH_FRONT_ANGLE
                                    : WristConstants.WRIST_TROUGH_BACK_ANGLE);
                }
            }

            if (reefLevel == ControlBoardConstants.REEF_LEVEL_2) {
                ReefPathPair pair = ReefPathLookup.coralPaths.get(reefLocation);
                if (pair != null) {
                    double closestHeading = getClosestHeading(currHeading, pair.targetHeading);
                    boolean usingFront = closestHeading == pair.targetHeading;
                    return new ScoreOutput(
                            PathPlannerPath.fromPathFile((usingFront) ? pair.frontPath : pair.backPath),
                            (usingFront) ? ArmConstants.ARM_L2_FRONT_ANGLE : ArmConstants.ARM_L2_BACK_ANGLE,
                            (usingFront) ? WristConstants.WRIST_L2_FRONT_ANGLE : WristConstants.WRIST_L2_BACK_ANGLE);
                }
            }

            if (reefLevel == ControlBoardConstants.REEF_LEVEL_3) {
                ReefPathLookup.ReefPathPair pair = ReefPathLookup.coralPaths.get(reefLocation);
                if (pair != null) {
                    return new ScoreOutput(
                            PathPlannerPath.fromPathFile(pair.backPath),
                            ArmConstants.ARM_L3_BACK_ANGLE,
                            WristConstants.WRIST_L3_BACK_ANGLE);
                }
            }

        } catch (Exception e) {
            System.err.println("Error in getScorePath: " + e.getMessage());
            e.printStackTrace();
        }

        return null;
    }

    public static ScoreOutput getFeederPath(double currHeading) {
        String feeder = ControlBoardHelpers.getFeeder();
        try {
            if (feeder.equals(ControlBoardConstants.FEEDER_LEFT)) {
                double closestHeading = getClosestHeading(currHeading, AutoHeadings.FEEDER_LEFT);
                Boolean usingFront = closestHeading == AutoHeadings.FEEDER_LEFT;
                SmartDashboard.putString("FeederAttemptedPath",
                        (closestHeading == AutoHeadings.FEEDER_LEFT)
                                ? NamedPaths.FEEDER_LOCATION_FRONT_LEFT
                                : NamedPaths.FEEDER_LOCATION_BACK_LEFT);
                return new ScoreOutput(
                        PathPlannerPath.fromPathFile((usingFront)
                                ? NamedPaths.FEEDER_LOCATION_FRONT_LEFT
                                : NamedPaths.FEEDER_LOCATION_BACK_LEFT),
                        (usingFront) ? ArmConstants.ARM_FRONT_INTAKE_ANGLE : ArmConstants.ARM_BACK_INTAKE_ANGLE,
                        (usingFront) ? WristConstants.WRIST_FRONT_INTAKE_ANGLE
                                : WristConstants.WRIST_BACK_INTAKE_ANGLE);

            } else if (feeder.equals(ControlBoardConstants.FEEDER_RIGHT)) {
                double closestHeading = getClosestHeading(currHeading, AutoHeadings.FEEDER_RIGHT);
                Boolean usingFront = closestHeading == AutoHeadings.FEEDER_RIGHT;
                SmartDashboard.putString("FeederAttemptedPath",
                        (closestHeading == AutoHeadings.FEEDER_RIGHT)
                                ? NamedPaths.FEEDER_LOCATION_FRONT_RIGHT
                                : NamedPaths.FEEDER_LOCATION_BACK_RIGHT);
                return new ScoreOutput(
                        PathPlannerPath.fromPathFile((usingFront)
                                ? NamedPaths.FEEDER_LOCATION_FRONT_RIGHT
                                : NamedPaths.FEEDER_LOCATION_BACK_RIGHT),
                        (usingFront) ? ArmConstants.ARM_FRONT_INTAKE_ANGLE : ArmConstants.ARM_BACK_INTAKE_ANGLE,
                        (usingFront) ? WristConstants.WRIST_FRONT_INTAKE_ANGLE
                                : WristConstants.WRIST_BACK_INTAKE_ANGLE);
            }

        } catch (Exception e) {
            System.err.println("Error in getFeederPath: " + e.getMessage());
            e.printStackTrace();
        }

        return null;
    }

    public static PathPlannerPath getCagePath() {
        try {
            switch (ControlBoardHelpers.getCage()) {
                case ControlBoardConstants.CAGE_LEFT:
                    return PathPlannerPath.fromPathFile(NamedPaths.CAGE_LOCATION_LEFT);

                case ControlBoardConstants.CAGE_CENTER:
                    return PathPlannerPath.fromPathFile(NamedPaths.CAGE_LOCATION_CENTER);

                case ControlBoardConstants.CAGE_RIGHT:
                    return PathPlannerPath.fromPathFile(NamedPaths.CAGE_LOCATION_RIGHT);
            }

        } catch (Exception e) {
            System.err.println("Error in getCagePath: " + e.getMessage());
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
