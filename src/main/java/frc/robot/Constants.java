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
        public static final double FORWARD_OFFSET = Units.inchesToMeters(0.0);
        public static final double SIDE_OFFSET = Units.inchesToMeters(0.0);
        public static final double HEIGHT_OFFSET = Units.inchesToMeters(0.0);
        public static final double ROLL_OFFSET = 0.0;
        public static final double PITCH_OFFSET = 0.0;
        public static final double YAW_OFFSET = 0.0;
    }

    public class ArmConstants {
        //ARM
        public static final int ARM_LEADER_ID = 26;
        public static final int ARM_FOLLOWER_ID = 27;
        
        public static final double HOME_ANGLE = 8.5;
        public static final double TROUGH_ANGLE = 44;
        public static final double FEEDER_ANGLE = 49; //72
        public static final double LOW_ALGAE_REMOVAL_ANGLE = 55.0;
        public static final double CLIMB_ANGLE = 68.0;
        public static final double HIGH_ALGAE_REMOVAL_ANGLE = 124.0;
        public static final double ALGEA_CARRY_ANGLE = 32.0;

        //CLIMB
        public static final int CLIMB_ID = 21;

        public static final double CLIMB_CLAMP_VAL = 0.32;
        public static final double HOME_CLAMP_VAL = 0.137;
    }

    public class WristConstants {
        //WRIST
        public static final int WRIST_ID = 47;
        public static final double WRIST_HOME_ANGLE = 10;
        public static final double WRIST_INTAKE_ANGLE = 125;
        public static final double WRIST_L2_ANGLE = 80;

        //INTAKE
        public static final int INTAKE_TOP_ID = 28;
        public static final int INTAKE_BOTTOM_ID = 29;
        public static final int INTAKE_LIMIT_ID = 1;


    } 
}
