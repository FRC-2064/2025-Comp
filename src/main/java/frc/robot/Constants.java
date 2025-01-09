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
        public static final double DEADBAND        = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT    = 6;
        
    }

    public class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight";
        public static final double FORWARD_OFFSET = 0.0;
        public static final double SIDE_OFFSET = 0.0;
        public static final double HEIGHT_OFFSET = 0.0;
        public static final double ROLL_OFFSET = 0.0;
        public static final double PITCH_OFFSET = 0.0;
        public static final double YAW_OFFSET = 0.0;

    }
}
