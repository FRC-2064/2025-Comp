package frc.robot.ControlBoard;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;

public class ControlBoardHelpers {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("ControlBoard");
    
    public static StringEntry feeder;
    public static StringEntry scoreLocation;
    public static StringEntry reefLocation;
    public static IntegerEntry reefLevel;
    public static StringEntry bargeCage;
    public static BooleanEntry hasScored;
    public static StringEntry selectedAuto;
    public static BooleanEntry hasCoral;
    public static BooleanEntry hasAlgae;
    public static BooleanEntry Clamped;
    public static BooleanEntry isClimbed;

    public ControlBoardHelpers() {
        feeder = table.getStringTopic("Feeder").getEntry("LEFT");
        scoreLocation = table.getStringTopic("ScoreLocation").getEntry("REEF");
        reefLocation = table.getStringTopic("Reef/Location").getEntry("A");
        reefLevel = table.getIntegerTopic("Reef/Level").getEntry(0);
        bargeCage = table.getStringTopic("Barge/Cage").getEntry("LEFT");
        hasScored = table.getBooleanTopic("Robot/HasScored").getEntry(false);
        selectedAuto = table.getStringTopic("Robot/SelectedAuto").getEntry("");
        hasCoral = table.getBooleanTopic("Robot/HasCoral").getEntry(false);
        hasAlgae = table.getBooleanTopic("Robot/HasAlgae").getEntry(false);
        Clamped = table.getBooleanTopic("Robot/Clamped").getEntry(false);
        isClimbed = table.getBooleanTopic("Robot/IsClimbed").getEntry(false); 
    }
}
