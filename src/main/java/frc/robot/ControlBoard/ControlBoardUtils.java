package frc.robot.ControlBoard;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoHeadings;
import frc.robot.Constants.ControlBoardConstants;
import frc.robot.Constants.NamedPaths;

public class ControlBoardUtils {

    public static PathPlannerPath getScorePath(double currHeading) {
        try {
            if (ControlBoardHelpers.getScoreLocation().equals(ControlBoardConstants.SCORE_PROCESSOR)) {
                System.out.println("Scoring Algae");
                return PathPlannerPath.fromPathFile(NamedPaths.ALGAE_LOCATION_PROCESSOR);
            }

            String reefLocation = ControlBoardHelpers.getReefLocation();
            int reefLevel = (int) ControlBoardHelpers.getLevel();

            if (reefLevel == ControlBoardConstants.REEF_LEVEL_ALGAE) {
                String path = ReefPathLookup.algaePaths.get(reefLocation);
                return PathPlannerPath.fromPathFile(path);
            }

            if (reefLevel == ControlBoardConstants.REEF_LEVEL_TROUGH ||
                    reefLevel == ControlBoardConstants.REEF_LEVEL_2) {
                ReefPathLookup.ReefPathPair pair = ReefPathLookup.coralPaths.get(reefLocation);
                if (pair != null) {
                    double closestHeading = getClosestHeading(currHeading, pair.targetHeading);
                    String pathName = (closestHeading == pair.targetHeading) ? pair.frontPath : pair.backPath;
                    return PathPlannerPath.fromPathFile(pathName);
                }
            }

            if (reefLevel == ControlBoardConstants.REEF_LEVEL_3 ||
                    reefLevel == ControlBoardConstants.REEF_LEVEL_4) {
                ReefPathLookup.ReefPathPair pair = ReefPathLookup.coralPaths.get(reefLocation);
                if (pair != null) {
                    return PathPlannerPath.fromPathFile(pair.backPath);
                }
            }

        } catch (Exception e) {
            System.err.println("Error in getScorePath: " + e.getMessage());
            e.printStackTrace();
        }

        return null;
    }

    public static PathPlannerPath getFeederPath(double currHeading) {
        String feeder = ControlBoardHelpers.getFeeder();
        try {
            if (feeder.equals(ControlBoardConstants.FEEDER_LEFT)) {
                double closestHeading = getClosestHeading(currHeading, AutoHeadings.FEEDER_LEFT);
                SmartDashboard.putString("FeederAttemptedPath", 
                (closestHeading == AutoHeadings.FEEDER_LEFT)
                        ? NamedPaths.FEEDER_LOCATION_FRONT_LEFT
                        : NamedPaths.FEEDER_LOCATION_BACK_LEFT);
                return PathPlannerPath.fromPathFile(
                        (closestHeading == AutoHeadings.FEEDER_LEFT)
                                ? NamedPaths.FEEDER_LOCATION_FRONT_LEFT
                                : NamedPaths.FEEDER_LOCATION_BACK_LEFT);

            } else if (feeder.equals(ControlBoardConstants.FEEDER_RIGHT)) {
                double closestHeading = getClosestHeading(currHeading, AutoHeadings.FEEDER_RIGHT);
                SmartDashboard.putString("FeederAttemptedPath", 
                (closestHeading == AutoHeadings.FEEDER_RIGHT)
                        ? NamedPaths.FEEDER_LOCATION_FRONT_RIGHT
                        : NamedPaths.FEEDER_LOCATION_BACK_RIGHT);
                return PathPlannerPath.fromPathFile(
                        (closestHeading == AutoHeadings.FEEDER_RIGHT)
                                ? NamedPaths.FEEDER_LOCATION_FRONT_RIGHT
                                : NamedPaths.FEEDER_LOCATION_BACK_RIGHT);
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
