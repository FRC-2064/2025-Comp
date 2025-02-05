package frc.robot.ControlBoard;

import com.pathplanner.lib.path.PathPlannerPath;

public class ScoreOutput {
    public final PathPlannerPath path;
    public final double armAngle;
    public final double wristAngle;
    
    public ScoreOutput(PathPlannerPath path, double armAngle, double wristAngle) {
        this.path = path;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
    }

}
