package frc.robot.Utils.ControlBoard;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Utils.Enums.EndEffectorState;

public class RobotConfiguration {
    public final List<Pose2d> pathPoses;
    public final double finalArmAngle;
    public final double travelArmAngle;
    public final double finalWristAngle;
    public final double travelWristAngle;
    public final EndEffectorState startEndEffectorState;
    public EndEffectorState finalEndEffectorState;

    
    public RobotConfiguration(
        List<Pose2d> pathPoses,
        double finalArmAngle, 
        double travelArmAngle, 
        double finalWristAngle,  
        double travelWristAngle,
        EndEffectorState startEndEffectorState,
        EndEffectorState finalEndEffectorState) {

        this.pathPoses = pathPoses;
        this.finalArmAngle = finalArmAngle;
        this.travelArmAngle = travelArmAngle;
        this.finalWristAngle = finalWristAngle;
        this.travelWristAngle = travelWristAngle;
        this.startEndEffectorState = startEndEffectorState;
        this.finalEndEffectorState = finalEndEffectorState;
    }

    public void setFinalEndEffectorState(EndEffectorState eeState){
        finalEndEffectorState = eeState;
    }

}
