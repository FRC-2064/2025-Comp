package frc.robot.Utils.ControlBoard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.ControlBoardConstants;
import frc.robot.Utils.Constants.OTFPaths;
import frc.robot.Utils.Constants.WristConstants;
import frc.robot.Utils.Enums.EndEffectorState;
import frc.robot.Utils.ControlBoard.ReefLookup.AlgaeHeight;
import frc.robot.Utils.ControlBoard.ReefLookup.AlgaePair;

public class RobotConfigProvider {

        /**
         * Returns the robot configuration for scoring a game piece.
         * <p>
         * This method selects the appropriate configuration based on the score location
         * and reef level.
         * It uses the current heading and a robot-relative translation offset (as a
         * Translation2d) to compute adjustments.
         *
         * @param currHeading The current heading of the robot in degrees.
         * @param offset      The robot-relative translation offset (assuming the robot
         *                    is facing forward).
         * @return A RobotConfiguration object representing the desired configuration,
         *         or null if an error occurs.
         */
        public static RobotConfiguration getGameScoreConfiguration(double currHeading) {
                try {
                        String scoreLocation = ControlBoardHelpers.getScoreLocation();

                        if (scoreLocation.equals(ControlBoardConstants.SCORE_PROCESSOR)) {
                                return createProcessorConfiguration();
                        }

                        if (scoreLocation.equals(ControlBoardConstants.SCORE_CAGE)) {
                                return getCageConfiguration();
                        }

                        String reefLocation = ControlBoardHelpers.getReefLocation();
                        int reefLevel = (int) ControlBoardHelpers.getLevel();

                        if (reefLocation.equals("")) {
                                return null;
                        }

                        return switch (reefLevel) {
                                case ControlBoardConstants.REEF_LEVEL_ALGAE ->
                                        createAlgaeRemovalConfiguration(reefLocation);
                                case ControlBoardConstants.REEF_LEVEL_TROUGH ->
                                        createTroughScoringConfiguration(reefLocation);
                                case ControlBoardConstants.REEF_LEVEL_2 ->
                                        createLevel2ScoringConfiguration(reefLocation, currHeading);
                                case ControlBoardConstants.REEF_LEVEL_3 ->
                                        createLevel3ScoringConfiguration(reefLocation);
                                default -> null;
                        };

                } catch (Exception e) {
                        System.out.println("Error in getScoreConfiguration");
                        e.printStackTrace();
                        return null;
                }
        }

        /**
         * Creates a configuration for processor scoring.
         *
         * @return A RobotConfiguration object for the processor location.
         */
        private static RobotConfiguration createProcessorConfiguration() {
                return new RobotConfiguration(
                                OTFPaths.PROCESSOR,
                                computeStartPose(OTFPaths.PROCESSOR, true),
                                ArmConstants.ARM_ALGAE_CARRY_ANGLE,
                                ArmConstants.ARM_ALGAE_CARRY_ANGLE,
                                WristConstants.WRIST_ALGAE_ANGLE,
                                WristConstants.WRIST_ALGAE_ANGLE,
                                EndEffectorState.INTAKING_ALGAE,
                                EndEffectorState.OUTTAKING_ALGAE);
        }

        /**
         * Creates a configuration for algae removal based on the reef location.
         *
         * @param reefLocation The reef location identifier.
         * @return A RobotConfiguration object for algae removal.
         */
        private static RobotConfiguration createAlgaeRemovalConfiguration(String reefLocation) {
                AlgaePair pair = ReefLookup.algaePoses.get(reefLocation);
                boolean isHighAlgae = (pair.algaeHeight == AlgaeHeight.HIGH);

                return new RobotConfiguration(
                                pair.endPose,
                                computeStartPose(pair.endPose, !isHighAlgae),
                                isHighAlgae ? ArmConstants.ARM_HIGH_ALGAE_REMOVAL_ANGLE
                                                : ArmConstants.ARM_LOW_ALGAE_REMOVAL_ANGLE,
                                ArmConstants.ARM_HOME_ANGLE,
                                isHighAlgae ? WristConstants.WRIST_HIGH_ALGAE_REMOVAL_ANGLE
                                                : WristConstants.WRIST_LOW_ALGAE_REMOVAL_ANGLE,
                                WristConstants.WRIST_HOME_ANGLE,
                                isHighAlgae ? EndEffectorState.REMOVING_HIGH_ALGAE
                                                : EndEffectorState.REMOVING_LOW_ALGAE,
                                isHighAlgae ? EndEffectorState.REMOVING_HIGH_ALGAE
                                                : EndEffectorState.REMOVING_LOW_ALGAE);
        }

        /**
         * Creates a configuration for scoring in the trough.
         *
         * @param reefLocation The reef location identifier.
         * @return A RobotConfiguration object for trough scoring, or null if the
         *         location is not defined.
         */
        private static RobotConfiguration createTroughScoringConfiguration(String reefLocation) {
                Pose2d endPose = ReefLookup.coralPoses.get(reefLocation);
                return (endPose != null) ? new RobotConfiguration(
                                endPose,
                                computeStartPose(endPose, true),
                                ArmConstants.ARM_TROUGH_FRONT_ANGLE,
                                ArmConstants.ARM_HOME_ANGLE,
                                WristConstants.WRIST_TROUGH_FRONT_ANGLE,
                                WristConstants.WRIST_HOME_ANGLE,
                                EndEffectorState.INTAKING_CORAL,
                                EndEffectorState.OUTTAKING_CORAL) : null;
        }

        /**
         * Creates a configuration for scoring in level 2.
         *
         * @param reefLocation The reef location identifier.
         * @param currHeading  The current heading of the robot in degrees.
         * @param coralOffset  The robot-relative translation offset (assuming the robot
         *                     is facing forward).
         * @return A RobotConfiguration object for level 2 scoring, or null if the reef
         *         location is not defined.
         */
        private static RobotConfiguration createLevel2ScoringConfiguration(String reefLocation, double currHeading) {
                Pose2d endPose = ReefLookup.coralPoses.get(reefLocation);
                if (endPose == null)
                        return null;
        
                SmartDashboard.putNumber("Logging/Heading/targetHeading", endPose.getRotation().getDegrees());

                return new RobotConfiguration(
                                endPose,
                                computeStartPose(endPose, true),
                                ArmConstants.ARM_L2_FRONT_ANGLE,
                                ArmConstants.ARM_HOME_ANGLE,
                                WristConstants.WRIST_L2_FRONT_ANGLE,
                                WristConstants.WRIST_HOME_ANGLE,
                                EndEffectorState.INTAKING_CORAL,
                                EndEffectorState.OUTTAKING_PEG);
        }

        /**
         * Creates a configuration for scoring in level 3.
         *
         * @param reefLocation The reef location identifier.
         * @param coralOffset  The robot-relative translation offset (assuming the robot
         *                     is facing forward).
         * @return A RobotConfiguration object for level 3 scoring, or null if the reef
         *         location is not defined.
         */
        private static RobotConfiguration createLevel3ScoringConfiguration(String reefLocation) {
                Pose2d endPose = ReefLookup.coralPoses.get(reefLocation);
                if (endPose == null)
                        return null;

                Pose2d basePose = new Pose2d(endPose.getTranslation(),
                                endPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));

                return new RobotConfiguration(
                                basePose,
                                computeStartPose(basePose, false),
                                ArmConstants.ARM_L3_BACK_ANGLE,
                                ArmConstants.ARM_HOME_ANGLE,
                                WristConstants.WRIST_L3_BACK_ANGLE,
                                WristConstants.WRIST_HOME_ANGLE,
                                EndEffectorState.INTAKING_CORAL,
                                EndEffectorState.OUTTAKING_PEG);
        }

        /**
         * Returns the feeder configuration based on ControlBoard and the current
         * heading.
         *
         * @param currHeading The current heading of the robot in degrees.
         * @return A RobotConfiguration object representing the feeder configuration, or
         *         null if an error occurs.
         */
        public static RobotConfiguration getFeederConfiguration() {
                try {
                        String feeder = ControlBoardHelpers.getFeeder();
                        Pose2d endPose = feeder.equals(ControlBoardConstants.FEEDER_LEFT)
                                        ? OTFPaths.FEEDER_LOCATION_LEFT
                                        : OTFPaths.FEEDER_LOCATION_RIGHT;

                        return new RobotConfiguration(
                                        endPose,
                                        computeStartPose(endPose, true),
                                        ArmConstants.ARM_FRONT_INTAKE_ANGLE,
                                        ArmConstants.ARM_HOME_ANGLE,
                                        WristConstants.WRIST_FRONT_INTAKE_ANGLE,
                                        WristConstants.WRIST_HOME_ANGLE,
                                        EndEffectorState.INTAKING_CORAL,
                                        EndEffectorState.OUTTAKING_CORAL);

                } catch (Exception e) {
                        System.out.println("Error in getFeederConfiguration");
                        e.printStackTrace();
                }
                return null;
        }

        /**
         * Returns the cage configuration based on ControlBoard.
         *
         * @return A RobotConfiguration object representing the cage configuration, or
         *         null if an error occurs.
         */
        public static RobotConfiguration getCageConfiguration() {
                try {
                        String cage = ControlBoardHelpers.getCage();
                        Pose2d cagePose = switch (cage) {
                                case ControlBoardConstants.CAGE_LEFT -> OTFPaths.CAGE_LEFT;
                                case ControlBoardConstants.CAGE_CENTER -> OTFPaths.CAGE_CENTER;
                                case ControlBoardConstants.CAGE_RIGHT -> OTFPaths.CAGE_RIGHT;
                                default -> null;
                        };

                        return (cagePose != null) ? new RobotConfiguration(
                                        cagePose,
                                        computeStartPose(cagePose, false),
                                        ArmConstants.ARM_CLIMB_DOWN_ANGLE,
                                        ArmConstants.ARM_CLIMB_DOWN_ANGLE,
                                        WristConstants.WRIST_CLIMB_ANGLE,
                                        WristConstants.WRIST_CLIMB_ANGLE,
                                        EndEffectorState.STOPPED,
                                        EndEffectorState.STOPPED)
                                        : null;
                } catch (Exception e) {
                        System.out.println("Error in getCageConfiguration");
                        e.printStackTrace();
                }
                return null;
        }

        /**
         * Adjusts a base pose by applying a robot-relative translation offset.
         * <p>
         * The offset is specified relative to the robot's forward-facing direction. If
         * the robot is not using its front,
         * the offset is flipped.
         * The offset is then rotated by the base pose's rotation to convert it into
         * field coordinates.
         *
         * @param basePose   The base Pose2d before applying the offset.
         * @param offset     The robot-relative translation offset (assuming the robot
         *                   is facing forward).
         * @param usingFront True if the robot is using its front; false if using its
         *                   back.
         * @return A new Pose2d with the adjusted position and the same rotation as the
         *         base pose.
         */
        private static Pose2d adjustPoseForOffset(Pose2d basePose, Translation2d offset, Boolean usingFront) {
                Translation2d effectiveOffset = usingFront ? offset : new Translation2d(-offset.getX(), -offset.getY());

                double theta = basePose.getRotation().getRadians();
                double fieldOffsetX = effectiveOffset.getX() * Math.cos(theta)
                                - effectiveOffset.getY() * Math.sin(theta);
                double fieldOffsetY = effectiveOffset.getX() * Math.sin(theta)
                                + effectiveOffset.getY() * Math.cos(theta);

                return new Pose2d(
                                basePose.getX() + fieldOffsetX,
                                basePose.getY() + fieldOffsetY,
                                basePose.getRotation());
        }

        /**
         * Computes a starting pose based on an end pose.
         * <p>
         * The start pose is offset from the end pose by a fixed distance.
         * The direction of the offset is reversed if the robot is not using its front.
         *
         * @param endPose      The end Pose2d.
         * @param isUsingFront True if the robot is using its front; false if using its
         *                     back.
         * @return A new Pose2d representing the computed start pose.
         */
        private static Pose2d computeStartPose(Pose2d endPose, boolean isUsingFront) {
                double theta = endPose.getRotation().getRadians();
                double dx = 0.75 * Math.cos(theta) * (isUsingFront ? 1 : -1);
                double dy = 0.75 * Math.sin(theta) * (isUsingFront ? 1 : -1);
                return new Pose2d(endPose.getX() - dx, endPose.getY() - dy, endPose.getRotation());
        }

        /**
         * Computes the closest heading to a target heading from the current heading.
         * <p>
         * If the robot is on the Red alliance, the current heading is adjusted by 180
         * degrees.
         * The method then normalizes the angles and chooses the heading (target or its
         * inverse) that minimizes the difference.
         *
         * @param currHeading   The current heading of the robot in degrees.
         * @param targetHeading The target heading in degrees.
         * @return The closest heading in degrees (normalized to the range of [0, 360)).
         */
        private static double getClosestHeading(double currHeading, double targetHeading) {
                try {
                        if (DriverStation.getAlliance()
                                        .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                                currHeading += 180;
                        }
                } catch (Exception e) {
                        System.out.println("rotate error " + e);
                }

                double inverseHeading = (targetHeading + 180) % 360;
                currHeading = normalizeAngle(currHeading);
                targetHeading = normalizeAngle(targetHeading);
                inverseHeading = normalizeAngle(inverseHeading);

                double diffTarget = Math.abs(normalizeAngle(currHeading - targetHeading));
                double diffInverse = Math.abs(normalizeAngle(currHeading - inverseHeading));

                return (diffTarget <= diffInverse) ? targetHeading : inverseHeading;

        }

        /**
         * Normalizes an angle to be within the range [-180, 180) degrees.
         *
         * @param angle The angle in degrees.
         * @return The normalized angle in degrees.
         */
        private static double normalizeAngle(double angle) {
                angle = ((angle % 360) + 360) % 360;
                return (angle >= 180) ? angle - 360 : angle;
        }
}
