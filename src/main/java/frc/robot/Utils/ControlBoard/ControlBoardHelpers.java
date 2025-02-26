package frc.robot.Utils.ControlBoard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class ControlBoardHelpers {
    private static final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

    private static NetworkTableEntry getEntry(String key) {
        return ntInstance.getEntry(key);
    }

    public static void updateRobotStatus() {
        getEntry("/ControlBoard/Robot/GameTime").setDouble(DriverStation.getMatchTime());
        getEntry("/ControlBoard/Robot/Reef/Location").setString(getReefLocation());
        getEntry("/ControlBoard/Robot/Reef/Level").setNumber(getLevel());
        getEntry("/ControlBoard/Robot/Feeder").setString(getFeeder());
        getEntry("/ControlBoard/Robot/Barge/Cage").setString(getCage());
        getEntry("/ControlBoard/Robot/ScoreLocation").setString(getScoreLocation());
    }

    public static int getLevel() {
        return (int) getEntry("/ControlBoard/Reef/Level").getDouble(0);
    }

    public static void setLevel(int value) {
        getEntry("/ControlBoard/Reef/Level").setDouble(value);
    }

    public static String getAuto() {
        return getEntry("/ControlBoard/Robot/SelectedAuto").getString("");
    }

    public static void setAuto(String value) {
        getEntry("/ControlBoard/Robot/SelectedAuto").setString(value);
    }

    public static String getCage() {
        return getEntry("/ControlBoard/Barge/Cage").getString("");
    }

    public static void setCage(String value) {
        getEntry("/ControlBoard/Barge/Cage").setString(value);
    }

    public static String getFeeder() {
        return getEntry("/ControlBoard/Feeder").getString("");
    }

    public static void setFeeder(String value) {
        getEntry("/ControlBoard/Feeder").setString(value);
    }

    public static String getScoreLocation() {
        return getEntry("/ControlBoard/ScoreLocation").getString("");
    }

    public static void setScoreLocation(String value) {
        getEntry("/ControlBoard/ScoreLocation").setString(value);
    }

    public static String getReefLocation() {
        return getEntry("/ControlBoard/Reef/Location").getString("");
    }

    public static void setReefLocation(String value) {
        getEntry("/ControlBoard/Reef/Location").setString(value);
    }

    public static boolean getHasScored() {
        return getEntry("/ControlBoard/Robot/HasScored").getBoolean(false);
    }

    public static void setHasScored(boolean value) {
        getEntry("/ControlBoard/Robot/HasScored").setBoolean(value);
    }

    public static boolean getHasAlgae() {
        return getEntry("/ControlBoard/Robot/HasAlgae").getBoolean(false);
    }

    public static void setHasAlgae(boolean value) {
        getEntry("/ControlBoard/Robot/HasAlgae").setBoolean(value);
    }

    public static boolean getHasCoral() {
        return getEntry("/ControlBoard/Robot/HasCoral").getBoolean(false);
    }

    public static void setHasCoral(boolean value) {
        getEntry("/ControlBoard/Robot/HasCoral").setBoolean(value);
    }

    public static boolean getClamped() {
        return getEntry("/ControlBoard/Robot/Clamped").getBoolean(false);
    }

    public static void setClamped(boolean value) {
        getEntry("/ControlBoard/Robot/Clamped").setBoolean(value);
    }

    public static boolean getClimbed() {
        return getEntry("/ControlBoard/Robot/IsClimbed").getBoolean(false);
    }

    public static void setClimbed(boolean value) {
        getEntry("/ControlBoard/Robot/IsClimbed").setBoolean(value);
    }
}
