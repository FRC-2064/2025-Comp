package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    public static final double DRIVESTATE_ALLOWED_ERROR = 2.0;

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
        public static final double CLAMP_CLOSED_VAL = 0.255;
        public static final double CLAMP_OPEN_VAL = 0.1;

        public static final int WINCH_ID = 48;
        
    }

    public class ArmConstants {
        //ARM
        public static final int ARM_LEADER_ID = 27;
        public static final int ARM_FOLLOWER_ID = 26;
        
        public static final double ARM_HOME_ANGLE = 45;
        public static final double ARM_FRONT_INTAKE_ANGLE = 57;
        public static final double ARM_BACK_INTAKE_ANGLE = 123;
        
        public static final double ARM_TROUGH_FRONT_ANGLE = 43;
        public static final double ARM_TROUGH_BACK_ANGLE = 120;

        public static final double ARM_L2_FRONT_ANGLE = 63;
        public static final double ARM_L2_BACK_ANGLE = 83;

        public static final double ARM_L3_BACK_ANGLE = 84;

        public static final double ARM_ALGAE_CARRY_ANGLE = 23;

        public static final double ARM_LOW_ALGAE_REMOVAL_ANGLE = 53;
        public static final double ARM_HIGH_ALGAE_REMOVAL_ANGLE = 95;

        public static final double ARM_CLIMB_UP_ANGLE = 85;
        public static final double ARM_CLIMB_DOWN_ANGLE = 31;
        
        public static final double ARM_GROUND_INTAKE = 4;

        public static final double ARM_SAFE_MIN_ANGLE = 10;
        public static final double ARM_SAFE_MAX_ANGLE = 90;

        public static final double DEGREES_PER_ROTATION = 360.0;
        public static final double ALLOWED_ERROR_DEGREES = 1.0;

    }
    
    public class WristConstants {
        //WRIST
        public static final int WRIST_ID = 47;
        
        public static final double WRIST_HOME_ANGLE = 60;
        
        public static final double WRIST_FRONT_INTAKE_ANGLE = 102;
        public static final double WRIST_BACK_INTAKE_ANGLE = 220;
        
        public static final double WRIST_TROUGH_FRONT_ANGLE = 43;
        public static final double WRIST_TROUGH_BACK_ANGLE = 228;
        
        public static final double WRIST_L2_FRONT_ANGLE = 54;
        public static final double WRIST_L2_BACK_ANGLE = 250;
        
        public static final double WRIST_L3_BACK_ANGLE = 211;
        
        public static final double WRIST_ALGAE_ANGLE = 93;
        
        public static final double WRIST_LOW_ALGAE_REMOVAL_ANGLE = 50;
        public static final double WRIST_HIGH_ALGAE_REMOVAL_ANGLE = 205;
        
        public static final double WRIST_GROUND_INTAKE = 80;
        
        public static final double WRIST_CLIMB_ANGLE = 10;
        
        public static final double WRIST_SAFE_MIN_ANGLE = 10;
        public static final double WRIST_SAFE_MAX_ANGLE = 200;
        
        public static final double DEGREES_PER_ROTATION = 360.0;
        public static final double ALLOWED_ERROR_DEGREES = 1.0;
    }
    
        public class EndEffectorConstants {
            //INTAKE
            public static final int EE_TOP_ID = 28;
            public static final int EE_BOTTOM_ID = 29;
            public static final int EE_LIMIT_ID = 1;
            public static final double TOF_PORT = 0;
            public static final double EE_BASE_OFFSET = 77;
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
        public static final String SCORE_CAGE = "CAGE";

        // FEEDER LOCATION
        public static final String FEEDER_LEFT = "LEFT";
        public static final String FEEDER_RIGHT = "RIGHT";

    }

    public class OTFPaths {
        // CAGE
        public static final Pose2d CAGE_LEFT = new Pose2d(8.715, 7.25, Rotation2d.fromDegrees(180.0)); 
        public static final Pose2d CAGE_CENTER = new Pose2d(8.715, 6.15, Rotation2d.fromDegrees(180.0)); 
        public static final Pose2d CAGE_RIGHT = new Pose2d(8.715, 5.1, Rotation2d.fromDegrees(180.0));

        // ALGAE REMOVAL
        public static final Pose2d ALGAE_LOCATION_AB = new Pose2d(3.19405, 4.0259, Rotation2d.fromDegrees(180.0)); // tag 18
        public static final Pose2d ALGAE_LOCATION_CD = new Pose2d(3.842131, 2.904871924, Rotation2d.fromDegrees(60.0)); // tag 17
        public static final Pose2d ALGAE_LOCATION_EF = new Pose2d(5.136515, 2.904871924, Rotation2d.fromDegrees(300)); // tag 22
        public static final Pose2d ALGAE_LOCATION_GH = new Pose2d(5.784596, 4.0259, Rotation2d.fromDegrees(180.0)); // tag 21
        public static final Pose2d ALGAE_LOCATION_IJ = new Pose2d(5.136515, 5.146928076, Rotation2d.fromDegrees(60.0)); // tag 20
        public static final Pose2d ALGAE_LOCATION_KL = new Pose2d(3.842131, 5.146928076, Rotation2d.fromDegrees(300.0)); // tag 19
        
        // SCORE
        public static final Pose2d PROCESSOR = new Pose2d(5.987542, 0.45974, Rotation2d.fromDegrees(270)); // tag 16
        public static final Pose2d CORAL_LOCATION_A = new Pose2d(3.19405,4.1909, Rotation2d.fromDegrees(0.0)); // tag 18
        public static final Pose2d CORAL_LOCATION_B = new Pose2d(3.19405,3.8609, Rotation2d.fromDegrees(0.0)); // tag 18
        public static final Pose2d CORAL_LOCATION_C = new Pose2d(3.699236808,2.987371924, Rotation2d.fromDegrees(60.0)); // tag 17
        public static final Pose2d CORAL_LOCATION_D = new Pose2d(3.985025192,2.822371924, Rotation2d.fromDegrees(60.0)); // tag 17
        public static final Pose2d CORAL_LOCATION_E = new Pose2d(4.993620808,2.822371924, Rotation2d.fromDegrees(120.0)); // tag 22
        public static final Pose2d CORAL_LOCATION_F = new Pose2d(5.279409192,2.987371924, Rotation2d.fromDegrees(120.0)); // tag 22
        public static final Pose2d CORAL_LOCATION_G = new Pose2d(5.784596,3.8609, Rotation2d.fromDegrees(180.0)); // tag 21
        public static final Pose2d CORAL_LOCATION_H = new Pose2d(5.784596,4.1909, Rotation2d.fromDegrees(180.0)); // tag 21
        public static final Pose2d CORAL_LOCATION_I = new Pose2d(5.279409192,5.064428076, Rotation2d.fromDegrees(240.0)); // tag 20
        public static final Pose2d CORAL_LOCATION_J = new Pose2d(4.993620808,5.229428076, Rotation2d.fromDegrees(240.0)); // tag 20
        public static final Pose2d CORAL_LOCATION_K = new Pose2d(3.985025192, 5.229428076, Rotation2d.fromDegrees(300.0)); // tag 19
        public static final Pose2d CORAL_LOCATION_L = new Pose2d(3.699236808, 5.064428076, Rotation2d.fromDegrees(300.0)); // tag 19

        // FEEDER
        public static final Pose2d FEEDER_LOCATION_LEFT = new Pose2d(1.123621854, 7.021460172, Rotation2d.fromDegrees(126.0)); // tag 13
        public static final Pose2d FEEDER_LOCATION_RIGHT = new Pose2d(1.123621854, 1.030339828, Rotation2d.fromDegrees(234.0)); // tag 12

        
    }
}
