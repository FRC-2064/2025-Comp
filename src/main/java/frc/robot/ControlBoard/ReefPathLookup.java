package frc.robot.ControlBoard;

import java.util.Map;

import frc.robot.Constants.AutoHeadings;
import frc.robot.Constants.ControlBoardConstants;
import frc.robot.Constants.NamedPaths;

import java.util.HashMap;

public class ReefPathLookup {

    public static class ReefPathPair {
        public final double targetHeading;
        public final String frontPath;
        public final String backPath;
        
        public ReefPathPair(double targetHeading, String frontPath, String backPath) {
            this.targetHeading = targetHeading;
            this.frontPath = frontPath;
            this.backPath = backPath;
        }
    }

    // A IS HIGH
    public static class AlgaePair {
        public final String path;
        public final AlgaeHeight algaeHeight;

        public AlgaePair(String path, AlgaeHeight algaeHeight) {
                this.path = path;
                this.algaeHeight = algaeHeight;
        }

    }

    
    public static final Map<String, AlgaePair> algaePaths = new HashMap<>();
    static {
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_A, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_AL, 
                AlgaeHeight.HIGH));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_L, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_AL, 
                AlgaeHeight.HIGH));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_B, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_BC, 
                AlgaeHeight.LOW));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_C, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_BC, 
                AlgaeHeight.LOW));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_D, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_DE, 
                AlgaeHeight.HIGH));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_E, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_DE, 
                AlgaeHeight.HIGH));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_F, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_FG, 
                AlgaeHeight.LOW));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_G, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_FG, 
                AlgaeHeight.LOW));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_H, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_HI, 
                AlgaeHeight.HIGH));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_I, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_HI, 
                AlgaeHeight.HIGH));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_J, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_JK, 
                AlgaeHeight.LOW));
        algaePaths.put(ControlBoardConstants.REEF_LOCATION_K, new AlgaePair(
                NamedPaths.ALGAE_LOCATION_JK, 
                AlgaeHeight.LOW));
    }
    

    public static final Map<String, ReefPathPair> coralPaths = new HashMap<>();
    static {
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_A, new ReefPathPair(
                AutoHeadings.REEF_HEADING_AL,
                NamedPaths.CORAL_LOCATION_FRONT_A,
                NamedPaths.CORAL_LOCATION_BACK_A));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_B, new ReefPathPair(
                AutoHeadings.REEF_HEADING_BC,
                NamedPaths.CORAL_LOCATION_FRONT_B,
                NamedPaths.CORAL_LOCATION_BACK_B));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_C, new ReefPathPair(
                AutoHeadings.REEF_HEADING_BC,
                NamedPaths.CORAL_LOCATION_FRONT_C,
                NamedPaths.CORAL_LOCATION_BACK_C));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_D, new ReefPathPair(
                AutoHeadings.REEF_HEADING_DE,
                NamedPaths.CORAL_LOCATION_FRONT_D,
                NamedPaths.CORAL_LOCATION_BACK_D));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_E, new ReefPathPair(
                AutoHeadings.REEF_HEADING_DE,
                NamedPaths.CORAL_LOCATION_FRONT_E,
                NamedPaths.CORAL_LOCATION_BACK_E));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_F, new ReefPathPair(
                AutoHeadings.REEF_HEADING_FG,
                NamedPaths.CORAL_LOCATION_FRONT_F,
                NamedPaths.CORAL_LOCATION_BACK_F));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_G, new ReefPathPair(
                AutoHeadings.REEF_HEADING_FG,
                NamedPaths.CORAL_LOCATION_FRONT_G,
                NamedPaths.CORAL_LOCATION_BACK_G));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_H, new ReefPathPair(
                AutoHeadings.REEF_HEADING_HI,
                NamedPaths.CORAL_LOCATION_FRONT_H,
                NamedPaths.CORAL_LOCATION_BACK_H));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_I, new ReefPathPair(
                AutoHeadings.REEF_HEADING_HI,
                NamedPaths.CORAL_LOCATION_FRONT_I,
                NamedPaths.CORAL_LOCATION_BACK_I));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_J, new ReefPathPair(
                AutoHeadings.REEF_HEADING_JK,
                NamedPaths.CORAL_LOCATION_FRONT_J,
                NamedPaths.CORAL_LOCATION_BACK_J));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_K, new ReefPathPair(
                AutoHeadings.REEF_HEADING_JK,
                NamedPaths.CORAL_LOCATION_FRONT_K,
                NamedPaths.CORAL_LOCATION_BACK_K));
        coralPaths.put(ControlBoardConstants.REEF_LOCATION_L, new ReefPathPair(
                AutoHeadings.REEF_HEADING_AL,
                NamedPaths.CORAL_LOCATION_FRONT_L,
                NamedPaths.CORAL_LOCATION_BACK_L));
    }

    public enum AlgaeHeight {
        HIGH,
        LOW
    }
}
