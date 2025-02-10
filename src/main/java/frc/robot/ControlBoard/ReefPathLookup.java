package frc.robot.ControlBoard;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoHeadings;
import frc.robot.Constants.ControlBoardConstants;
import frc.robot.Constants.NamedPaths;
import frc.robot.Constants.OTFPaths;

import java.util.HashMap;

public class ReefPathLookup {

        // A IS HIGH
        public static class AlgaePair {
                public final Pose2d endPose;
                public final AlgaeHeight algaeHeight;

                public AlgaePair(Pose2d endPose, AlgaeHeight algaeHeight) {
                        this.endPose = endPose;
                        this.algaeHeight = algaeHeight;
                }

        }

        public static final Map<String, AlgaePair> algaePoses = new HashMap<>();
        static {
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_A, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_AB,
                                AlgaeHeight.HIGH));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_B, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_AB,
                                AlgaeHeight.HIGH));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_C, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_CD,
                                AlgaeHeight.LOW));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_D, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_CD,
                                AlgaeHeight.LOW));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_E, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_EF,
                                AlgaeHeight.HIGH));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_F, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_EF,
                                AlgaeHeight.HIGH));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_G, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_GH,
                                AlgaeHeight.LOW));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_H, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_GH,
                                AlgaeHeight.LOW));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_I, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_IJ,
                                AlgaeHeight.HIGH));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_J, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_IJ,
                                AlgaeHeight.HIGH));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_K, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_KL,
                                AlgaeHeight.LOW));
                algaePoses.put(ControlBoardConstants.REEF_LOCATION_L, new AlgaePair(
                                OTFPaths.ALGAE_LOCATION_KL,
                                AlgaeHeight.LOW));
        }

        public static Map<String, Pose2d> coralPoses = new HashMap<>();
        static {
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_A, OTFPaths.CORAL_LOCATION_A);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_B, OTFPaths.CORAL_LOCATION_B);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_C, OTFPaths.CORAL_LOCATION_C);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_D, OTFPaths.CORAL_LOCATION_D);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_E, OTFPaths.CORAL_LOCATION_E);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_F, OTFPaths.CORAL_LOCATION_F);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_G, OTFPaths.CORAL_LOCATION_G);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_H, OTFPaths.CORAL_LOCATION_H);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_I, OTFPaths.CORAL_LOCATION_I);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_J, OTFPaths.CORAL_LOCATION_J);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_K, OTFPaths.CORAL_LOCATION_K);
                coralPoses.put(ControlBoardConstants.REEF_LOCATION_L, OTFPaths.CORAL_LOCATION_L);      
        }

        public enum AlgaeHeight {
                HIGH,
                LOW
        }
}
