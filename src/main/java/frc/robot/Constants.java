package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {

    public static final double ROBOT_MASS = 100;
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13;
    public static final double MAX_SPEED = Units.feetToMeters(1);

    public class OperatorConstants {
        public static final double DEADBAND        = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT    = 6;
        
    }

    public class Limelight1Constants {
        public static final String LIMELIGHT_NAME = "limelight1";
        public static final double FORWARD_OFFSET = 0.0;
        public static final double SIDE_OFFSET = 0.0;
        public static final double HEIGHT_OFFSET = 0.0;
        public static final double ROLL_OFFSET = 0.0;
        public static final double PITCH_OFFSET = 0.0;
        public static final double YAW_OFFSET = 0.0;
    }

    public class Limelight2Constants {
        public static final String LIMELIGHT_NAME = "limelight2";
        public static final double FORWARD_OFFSET = 0.0;
        public static final double SIDE_OFFSET = 0.0;
        public static final double HEIGHT_OFFSET = 0.0;
        public static final double ROLL_OFFSET = 0.0;
        public static final double PITCH_OFFSET = 0.0;
        public static final double YAW_OFFSET = 0.0;
    }

    public class ShovelConstants {
        public static final int shovelRotatorID = 25;
        public static final double INTAKE_ANGLE = 15;
        public static final double DUMP_ANGLE = -15;
    }

    public class ArmConstants {
        //ARM
        public static final int ARM_LEADER_ID = 26;
        public static final int ARM_FOLLOWER_ID = 27;
        
        public static final double HOME_ANGLE = 10.0;
        public static final double TROUGH_ANGLE = 40.0;
        public static final double FEEDER_ANGLE = 40.0;
        public static final double LOW_ALGAE_REMOVAL_ANGLE = 40.0;
        public static final double HIGH_ALGAE_REMOVAL_ANGLE = 40.0;
        public static final double ALGEA_CARRY_ANGLE = 40.0;

        //INTAKE
        public static final int INTAKE_TOP_ID = 28;
        public static final int INTAKE_BOTTOM_ID = 29;
        public static final int INTAKE_LIMIT_ID = 1;

        //CLIMB
        public static final int CLIMB_ID = 35;

        public static final double CLIMB_ANGLE = 40.0;
    }
}
