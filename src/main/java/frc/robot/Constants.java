package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {

    public static final double ROBOT_MASS = 100;
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13;
    public static final double MAX_SPEED = Units.feetToMeters(14.5);

    public class OperatorConstants {
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
        
    }

    public class Limelight1Constants {
        public static final String LIMELIGHT_NAME = "limelight-one";
        public static final double FORWARD_OFFSET = Units.inchesToMeters(14.5);
        public static final double SIDE_OFFSET = Units.inchesToMeters(-9.5);
        public static final double HEIGHT_OFFSET = Units.inchesToMeters(8.0);
        public static final double ROLL_OFFSET = 0.0;
        public static final double PITCH_OFFSET = 14.15;
        public static final double YAW_OFFSET = 0.0;
    }

    public class Limelight2Constants {
        public static final String LIMELIGHT_NAME = "limelight-two";
        public static final double FORWARD_OFFSET = Units.inchesToMeters(14.5);
        public static final double SIDE_OFFSET = Units.inchesToMeters(-9.5);
        public static final double HEIGHT_OFFSET = Units.inchesToMeters(8.0);
        public static final double ROLL_OFFSET = 0.0;
        public static final double PITCH_OFFSET = 14.15;
        public static final double YAW_OFFSET = 0.0;
    }

    public class ArmConstants {
        //ARM
        public static final int ARM_LEADER_ID = 26;
        public static final int ARM_FOLLOWER_ID = 27;
        
        public static final double ARM_HOME_ANGLE = 8.5;

        public static final double ARM_FRONT_INTAKE_ANGLE = 40.7; //72
        public static final double ARM_BACK_INTAKE_ANGLE = 102.2;
        
        public static final double ARM_TROUGH_FRONT_ANGLE = 32.6;
        public static final double ARM_TROUGH_BACK_ANGLE = 110.3;

        public static final double ARM_L2_FRONT_ANGLE = 47.4;
        public static final double ARM_L2_BACK_ANGLE = 95.2;

        public static final double ARM_L3_BACK_ANGLE = 92.4;

        public static final double ARM_ALGAE_CARRY_ANGLE = 18.97;
        // public static final double ARM_ALGAE_INTAKE_ANGLE = 0.0;

        public static final double ARM_LOW_ALGAE_REMOVAL_ANGLE = 55.0;
        public static final double ARM_HIGH_ALGAE_REMOVAL_ANGLE = 106.0;

        public static final double ARM_CLIMB_ANGLE = 68.0;

        //CLIMB
        public static final int CLIMB_ID = 21;
        public static final double CLIMB_CLAMP_VAL = 0.32;
        public static final double HOME_CLAMP_VAL = 0.137;
    }

    public class WristConstants {
        //WRIST
        public static final int WRIST_ID = 47;

        public static final double WRIST_HOME_ANGLE = 60.5;

        public static final double WRIST_FRONT_INTAKE_ANGLE = 133.1;
        public static final double WRIST_BACK_INTAKE_ANGLE = 174.9;

        public static final double WRIST_TROUGH_BACK_ANGLE = 227.6;
        public static final double WRIST_TROUGH_FRONT_ANGLE = 96.2;
        
        public static final double WRIST_L2_FRONT_ANGLE = 81.8;
        public static final double WRIST_L2_BACK_ANGLE = 226.5;
        
        public static final double WRIST_L3_BACK_ANGLE = 156.6;
                      
        public static final double WRIST_ALGAE_CARRY_ANGLE = 0.0;
        public static final double WRIST_ALGAE_INTAKE_ANGLE = 109.22;
        
        public static final double WRIST_HIGH_ALGAE_REMOVAL_ANGLE = 95.15;
        public static final double WRIST_LOW_ALGAE_REMOVAL_ANGLE = 0;
        
        //INTAKE
        public static final int INTAKE_TOP_ID = 28;
        public static final int INTAKE_BOTTOM_ID = 29;
        public static final int INTAKE_LIMIT_ID = 1;



    }

    public class NamedPaths {
        // CORAL PATHS
        public static final String CORAL_LOCATION_FRONT_A = "CORAL_LOCATION_FRONT_A";
        public static final String CORAL_LOCATION_FRONT_B = "CORAL_LOCATION_FRONT_B";
        public static final String CORAL_LOCATION_FRONT_C = "CORAL_LOCATION_FRONT_C";
        public static final String CORAL_LOCATION_FRONT_D = "CORAL_LOCATION_FRONT_D";
        public static final String CORAL_LOCATION_FRONT_E = "CORAL_LOCATION_FRONT_E";
        public static final String CORAL_LOCATION_FRONT_F = "CORAL_LOCATION_FRONT_F";
        public static final String CORAL_LOCATION_FRONT_G = "CORAL_LOCATION_FRONT_G";
        public static final String CORAL_LOCATION_FRONT_H = "CORAL_LOCATION_FRONT_H";
        public static final String CORAL_LOCATION_FRONT_I = "CORAL_LOCATION_FRONT_I";
        public static final String CORAL_LOCATION_FRONT_J = "CORAL_LOCATION_FRONT_J";
        public static final String CORAL_LOCATION_FRONT_K = "CORAL_LOCATION_FRONT_K";
        public static final String CORAL_LOCATION_FRONT_L = "CORAL_LOCATION_FRONT_L";

        public static final String CORAL_LOCATION_BACK_A = "CORAL_LOCATION_BACK_A";
        public static final String CORAL_LOCATION_BACK_B = "CORAL_LOCATION_BACK_B";
        public static final String CORAL_LOCATION_BACK_C = "CORAL_LOCATION_BACK_C";
        public static final String CORAL_LOCATION_BACK_D = "CORAL_LOCATION_BACK_D";
        public static final String CORAL_LOCATION_BACK_E = "CORAL_LOCATION_BACK_E";
        public static final String CORAL_LOCATION_BACK_F = "CORAL_LOCATION_BACK_F";
        public static final String CORAL_LOCATION_BACK_G = "CORAL_LOCATION_BACK_G";
        public static final String CORAL_LOCATION_BACK_H = "CORAL_LOCATION_BACK_H";
        public static final String CORAL_LOCATION_BACK_I = "CORAL_LOCATION_BACK_I";
        public static final String CORAL_LOCATION_BACK_J = "CORAL_LOCATION_BACK_J";
        public static final String CORAL_LOCATION_BACK_K = "CORAL_LOCATION_BACK_K";
        public static final String CORAL_LOCATION_BACK_L = "CORAL_LOCATION_BACK_L";

        // ALGAE PATHS
        public static final String ALGAE_LOCATION_AL = "ALGAE_LOCATION_AL";
        public static final String ALGAE_LOCATION_BC = "ALGAE_LOCATION_BC";
        public static final String ALGAE_LOCATION_DE = "ALGAE_LOCATION_DE";
        public static final String ALGAE_LOCATION_FG = "ALGAE_LOCATION_FG";
        public static final String ALGAE_LOCATION_HI = "ALGAE_LOCATION_HI";
        public static final String ALGAE_LOCATION_JK = "ALGAE_LOCATION_JK";
        public static final String ALGAE_LOCATION_PROCESSOR = "ALGAE_LOCATION_PROCESSOR"; // DONE

        // FEEDER PATHS
        public static final String FEEDER_LOCATION_FRONT_LEFT = "FEEDER_LOCATION_FRONT_LEFT"; // DONE
        public static final String FEEDER_LOCATION_FRONT_RIGHT = "FEEDER_LOCATION_FRONT_RIGHT"; // DONE
        public static final String FEEDER_LOCATION_BACK_LEFT = "FEEDER_LOCATION_BACK_LEFT"; // DONE
        public static final String FEEDER_LOCATION_BACK_RIGHT = "FEEDER_LOCATION_BACK_RIGHT"; // DONE

        // CAGE PATHS
        public static final String CAGE_LOCATION_LEFT = "CAGE_LOCATION_LEFT"; // DONE
        public static final String CAGE_LOCATION_CENTER = "CAGE_LOCATION_CENTER"; // DONE
        public static final String CAGE_LOCATION_RIGHT = "CAGE_LOCATION_RIGHT"; // DONE
    }

    public class AutoHeadings {
        // REEF
        public static final double REEF_HEADING_AL = 0.0;
        public static final double REEF_HEADING_FG = 180.0;
        public static final double REEF_HEADING_BC = -60.0;
        public static final double REEF_HEADING_HI = 120.0;
        public static final double REEF_HEADING_DE = -120.0;
        public static final double REEF_HEADING_JK = 60.0;

        // FEEDER
        public static final double FEEDER_LEFT = 126.0;
        public static final double FEEDER_RIGHT = -126.0;
    }

    public class ControlBoardConstants {
        // REEF LOCATION
        public static final String REEF_LOCATION_A = "A";
        public static final String REEF_LOCATION_B = "B";
        public static final String REEF_LOCATION_C = "C";
        public static final String REEF_LOCATION_D = "D";
        public static final String REEF_LOCATION_E = "E";
        public static final String REEF_LOCATION_F = "F";
        public static final String REEF_LOCATION_G = "G";
        public static final String REEF_LOCATION_H = "H";
        public static final String REEF_LOCATION_I = "I";
        public static final String REEF_LOCATION_J = "J";
        public static final String REEF_LOCATION_K = "K";
        public static final String REEF_LOCATION_L = "L";

        // REEF LEVEL
        public static final int REEF_LEVEL_ALGAE = 0;
        public static final int REEF_LEVEL_TROUGH = 1;
        public static final int REEF_LEVEL_2 = 2;
        public static final int REEF_LEVEL_3 = 3;
        public static final int REEF_LEVEL_4 = 4;

        // CAGE LOCATION
        public static final String CAGE_LEFT = "LEFT";
        public static final String CAGE_CENTER = "CENTER";
        public static final String CAGE_RIGHT = "RIGHT";

        // SCORE LOCATION
        public static final String SCORE_REEF = "REEF";
        public static final String SCORE_PROCESSOR = "PROCESSOR";

        // FEEDER LOCATION
        public static final String FEEDER_LEFT = "LEFT";
        public static final String FEEDER_RIGHT = "RIGHT";

    }
}
