package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    public static final double DRIVESTATE_ALLOWED_ERROR = 1.0;

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
        public static final double FORWARD_OFFSET = Units.inchesToMeters(-14.5);
        public static final double SIDE_OFFSET = Units.inchesToMeters(9.5);
        public static final double HEIGHT_OFFSET = Units.inchesToMeters(8.0);
        public static final double ROLL_OFFSET = 0.0;
        public static final double PITCH_OFFSET = 14.15;
        public static final double YAW_OFFSET = 180.0;
    }

    public class ClampConstants {
        //CLIMB
        public static final int CLAMP_ID = 21;
        public static final double CLAMP_CLOSED_VAL = 0.32;
        public static final double CLAMP_OPEN_VAL = 0.143;

        public static final int WINCH_ID = 48;
        
    }

    public class ArmConstants {
        //ARM
        // this is for PHASE 4 BOT, PHASE 3 bot is flipped
        public static final int ARM_LEADER_ID = 27;
        public static final int ARM_FOLLOWER_ID = 26;
        
        public static final double ARM_HOME_ANGLE = 45;

        // FOR PHASE 4
        public static final double ARM_FRONT_INTAKE_ANGLE = 56.9; //72
        public static final double ARM_BACK_INTAKE_ANGLE = 123.05;
        
        public static final double ARM_TROUGH_FRONT_ANGLE = 43.2;
        public static final double ARM_TROUGH_BACK_ANGLE = 120.3;

        public static final double ARM_L2_FRONT_ANGLE = 62.5;
        public static final double ARM_L2_BACK_ANGLE = 83.25;

        public static final double ARM_L3_BACK_ANGLE = 83.5;

        public static final double ARM_ALGAE_CARRY_ANGLE = 22.5;

        public static final double ARM_LOW_ALGAE_REMOVAL_ANGLE = 52.7;
        public static final double ARM_HIGH_ALGAE_REMOVAL_ANGLE = 95.8;

        public static final double ARM_CLIMB_UP_ANGLE = 84.32;
        public static final double ARM_CLIMB_DOWN_ANGLE = 30.8;
        
        public static final double ARM_GROUND_INTAKE = 3.88;

        // FOR PHASE 3
        // public static final double ARM_FRONT_INTAKE_ANGLE = 45.7; //72
        // public static final double ARM_BACK_INTAKE_ANGLE = 102.2;
        
        // public static final double ARM_TROUGH_FRONT_ANGLE = 32.6;
        // public static final double ARM_TROUGH_BACK_ANGLE = 120.3;

        // public static final double ARM_L2_FRONT_ANGLE = 47.4;
        // public static final double ARM_L2_BACK_ANGLE = 95.2;

        // public static final double ARM_L3_BACK_ANGLE = 92.4;

        // public static final double ARM_ALGAE_CARRY_ANGLE = 18.97;

        // public static final double ARM_LOW_ALGAE_REMOVAL_ANGLE = 55.0;
        // public static final double ARM_HIGH_ALGAE_REMOVAL_ANGLE = 106.0;

        // public static final double ARM_CLIMB_ANGLE = 68.0;

        public static final double DEGREES_PER_ROTATION = 360.0;
        public static final double ALLOWED_ERROR_DEGREES = 1.0;

        public static final double ARM_SAFE_MIN_ANGLE = 10;
        public static final double ARM_SAFE_MAX_ANGLE = 90;
    }

    public class EndEffectorConstants {
        //INTAKE
        public static final int EE_TOP_ID = 28;
        public static final int EE_BOTTOM_ID = 29;
        public static final int EE_LIMIT_ID = 1;
        public static final double TOF_PORT = 0;
        public static final double EE_BASE_OFFSET = 0;
    }

    public class WristConstants {
        //WRIST
        public static final int WRIST_ID = 47;

        public static final double WRIST_HOME_ANGLE = 60.5;

        // FOR PHASE 4
        public static final double WRIST_FRONT_INTAKE_ANGLE = 102.2;
        public static final double WRIST_BACK_INTAKE_ANGLE = 219.85;

        public static final double WRIST_TROUGH_FRONT_ANGLE = 43.2;
        public static final double WRIST_TROUGH_BACK_ANGLE = 227.6;
        
        public static final double WRIST_L2_FRONT_ANGLE = 48.5;
        public static final double WRIST_L2_BACK_ANGLE = 257.2;
        
        public static final double WRIST_L3_BACK_ANGLE = 220.18;
                      
        public static final double WRIST_ALGAE_CARRY_ANGLE = 88.8;
        public static final double WRIST_ALGAE_INTAKE_ANGLE = 88.8;
        
        public static final double WRIST_LOW_ALGAE_REMOVAL_ANGLE = 50.98;
        public static final double WRIST_HIGH_ALGAE_REMOVAL_ANGLE = 205.78;

        public static final double WRIST_GROUND_INTAKE = 67.3;

        // FOR PHASE 3
        // public static final double WRIST_FRONT_INTAKE_ANGLE = 133.1;
        // public static final double WRIST_BACK_INTAKE_ANGLE = 174.9;

        // public static final double WRIST_TROUGH_BACK_ANGLE = 227.6;
        // public static final double WRIST_TROUGH_FRONT_ANGLE = 96.2;
        
        // public static final double WRIST_L2_FRONT_ANGLE = 81.8;
        // public static final double WRIST_L2_BACK_ANGLE = 226.5;
        
        // public static final double WRIST_L3_BACK_ANGLE = 156.6;
                      
        // public static final double WRIST_ALGAE_CARRY_ANGLE = 0.0;
        // public static final double WRIST_ALGAE_INTAKE_ANGLE = 109.22;
        
        // public static final double WRIST_HIGH_ALGAE_REMOVAL_ANGLE = 95.15;
        // public static final double WRIST_LOW_ALGAE_REMOVAL_ANGLE = 60;

        public static final double WRIST_CLIMB_ANGLE = 10;

        public static final double DEGREES_PER_ROTATION = 360.0;
        public static final double ALLOWED_ERROR_DEGREES = 1.0;

        public static final double WRIST_SAFE_MIN_ANGLE = 10;
        public static final double WRIST_SAFE_MAX_ANGLE = 200;
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

    public class OTFPaths {
        // CAGE
        public static final Pose2d CAGE_LEFT = new Pose2d(8.715, 7.25, Rotation2d.fromDegrees(180.0)); // Done
        public static final Pose2d CAGE_CENTER = new Pose2d(8.715, 6.15, Rotation2d.fromDegrees(180.0)); // Done
        public static final Pose2d CAGE_RIGHT = new Pose2d(8.715, 5.1, Rotation2d.fromDegrees(180.0)); // Done

        // ALGAE REMOVAL
        public static final Pose2d ALGAE_LOCATION_AB = new Pose2d(3.19405, 4.0259, Rotation2d.fromDegrees(0.0)); // tag 18
        public static final Pose2d ALGAE_LOCATION_CD = new Pose2d(3.842131, 2.904871924, Rotation2d.fromDegrees(60.0)); // tag 17
        public static final Pose2d ALGAE_LOCATION_EF = new Pose2d(5.136515, 2.904871924, Rotation2d.fromDegrees(120.0)); // tag 22
        public static final Pose2d ALGAE_LOCATION_GH = new Pose2d(5.784596, 4.0259, Rotation2d.fromDegrees(180.0)); // tag 21
        public static final Pose2d ALGAE_LOCATION_IJ = new Pose2d(5.136515, 5.146928076, Rotation2d.fromDegrees(240.0)); // tag 20
        public static final Pose2d ALGAE_LOCATION_KL = new Pose2d(3.842131, 5.146928076, Rotation2d.fromDegrees(300.0)); // tag 19
        
        // SCORE
        public static final Pose2d PROCESSOR = new Pose2d(5.987542, 0.45974, Rotation2d.fromDegrees(270)); // tag 16
        public static final Pose2d CORAL_LOCATION_A = new Pose2d(0,0, Rotation2d.fromDegrees(0.0)); // tag 18
        public static final Pose2d CORAL_LOCATION_B = new Pose2d(0,0, Rotation2d.fromDegrees(0.0)); // tag 18
        public static final Pose2d CORAL_LOCATION_C = new Pose2d(0,0, Rotation2d.fromDegrees(60.0)); // tag 17
        public static final Pose2d CORAL_LOCATION_D = new Pose2d(0,0, Rotation2d.fromDegrees(60.0)); // tag 17
        public static final Pose2d CORAL_LOCATION_E = new Pose2d(0,0, Rotation2d.fromDegrees(120.0)); // tag 22
        public static final Pose2d CORAL_LOCATION_F = new Pose2d(0,0, Rotation2d.fromDegrees(120.0)); // tag 22
        public static final Pose2d CORAL_LOCATION_G = new Pose2d(0,0, Rotation2d.fromDegrees(180.0)); // tag 21
        public static final Pose2d CORAL_LOCATION_H = new Pose2d(0,0, Rotation2d.fromDegrees(180.0)); // tag 21
        public static final Pose2d CORAL_LOCATION_I = new Pose2d(0,0, Rotation2d.fromDegrees(240.0)); // tag 20
        public static final Pose2d CORAL_LOCATION_J = new Pose2d(0,0, Rotation2d.fromDegrees(240.0)); // tag 20
        public static final Pose2d CORAL_LOCATION_K = new Pose2d(3.842131, 5.146928076, Rotation2d.fromDegrees(300.0)); // tag 19
        public static final Pose2d CORAL_LOCATION_L = new Pose2d(3.842131,5.146928076, Rotation2d.fromDegrees(300.0)); // tag 19

        // FEEDER
        public static final Pose2d FEEDER_LOCATION_LEFT = new Pose2d(1.123621854, 7.021460172, Rotation2d.fromDegrees(126.0)); // tag 13
        public static final Pose2d FEEDER_LOCATION_RIGHT = new Pose2d(1.123621854, 1.030339828, Rotation2d.fromDegrees(234.0)); // tag 12

        // TEST POSE
        public static final Pose2d TEST_LOCATION = new Pose2d(12.57, 1.75, Rotation2d.fromDegrees(180));

        
    }
}
